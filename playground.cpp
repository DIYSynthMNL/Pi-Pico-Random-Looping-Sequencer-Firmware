// playground.cpp
// Native macOS simulator for the generative sequencer firmware.
//
// Phase A: rewired to the polyphonic-ready architecture —
// seq::Sequencer owns N voices (currently 1) and a ViewStack drives
// hierarchical OLED navigation.
//
// Two ImGui windows:
//   - "OLED 128x64"        : pixel-for-pixel render of what the firmware
//                            shows. Encoder keyboard-only: ←/→ to
//                            rotate, Space to press, Backspace (or hold
//                            Space) to long-press / cancel.
//   - "Sequencer Controls" : sliders bidirectionally synced with the
//                            menu items. Internal BPM clock, engine
//                            inspector with mini step grid.
//
// Keymap:
//   ← / →     encoder rotate
//   Space     encoder click (short press)
//   Backspace long-press / cancel-out-of-submenu
//   G         manual clock pulse
//   S         start/stop internal clock
//   R         reset transport
//   Esc / Q   quit

#include "Voice.h"
#include "Sequencer.h"
#include "Scales.h"
#include "FakeOled.h"
#include "Menu.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <portaudio.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace {

// ============================================================
//  Shared state
// ============================================================
std::mutex             g_mutex;          // guards Sequencer + Menu items + scale buffer

seq::Sequencer         g_sequencer;
seq::VoiceParams       g_vparams;        // mirror of voice(0) params
seq::SequencerParams   g_sparams;

// Quantizer scale buffer the host owns.
uint16_t               g_scale[seq::kMaxScaleNotes] = {0};
int                    g_scale_count    = 0;
int                    g_scale_idx      = -1;
int                    g_starting_note  = 12;
int                    g_octaves        = 1;

// Internal clock
std::atomic<bool>      g_clock_running{false};
std::atomic<int>       g_bpm{60};
std::atomic<bool>      g_manual_pulse_request{false};
std::atomic<bool>      g_reset_request{false};
std::atomic<bool>      g_app_quitting{false};

// Display-only mirrors for ImGui rendering (read without holding the mutex).
std::atomic<int>       g_disp_step{0};
std::atomic<uint16_t>  g_disp_dac{0};
std::atomic<bool>      g_disp_trig{false};
uint16_t               g_disp_cv[seq::kMaxSteps]      = {0};
uint8_t                g_disp_trig_grid[seq::kMaxSteps] = {0};

seq::FakeOled          g_oled;

// ============================================================
//  Audition — desktop-only synth voice
// ============================================================
// Reads voice(0).last_dac() + trig_active() via the existing display
// atomics, drives a sine oscillator gated by an AR envelope. Lets you
// *hear* what the engine is emitting without patching any external
// hardware. Audio output via PortAudio at 48 kHz stereo.
constexpr int   kAudioSampleRate = 48000;
constexpr float kTwoPi           = 6.28318530717958647692f;

std::atomic<bool>  g_audition_enabled{false};
std::atomic<float> g_audition_volume{0.25f};
std::atomic<float> g_audition_attack_ms{8.0f};
std::atomic<float> g_audition_release_ms{300.0f};
std::atomic<float> g_audition_pitch_semis{0.0f};  // global transpose, ±48 semitones

// Audio-thread-only state (the audio thread is the only writer).
float g_audio_phase     = 0.0f;
float g_audio_env       = 0.0f;
bool  g_audio_prev_trig = false;
enum class EnvStage : uint8_t { Idle, Attack, Sustain, Release };
EnvStage g_audio_env_stage = EnvStage::Idle;

// CV source labels — index = enum value.
constexpr const char*  kCvSourceNames[]   = { "Normal", "Test", "Tuning" };
constexpr int          kCvSourceCount     = 3;

// DigitalInMode labels — index = enum value (matches seq::DigitalInMode).
constexpr const char*  kDigInModeNames[]  = { "None", "Reset", "RunStop", "Freeze" };
constexpr int          kDigInModeCount    = 4;

// Menu items + view stack
std::vector<const char*>  g_scale_names;
seq::ViewStack            g_views;
seq::MenuListView         g_menu_view;       // pushed on top of playback when user clicks

seq::SingleSelectItem*    g_scale_item        = nullptr;
seq::NumericalItem*       g_cv_prob_item      = nullptr;
seq::NumericalItem*       g_trig_prob_item    = nullptr;
seq::NumericalItem*       g_trig_length_item  = nullptr;
seq::NumericalItem*       g_steps_item        = nullptr;
seq::NumericalItem*       g_octaves_item      = nullptr;
seq::NumericalItem*       g_start_note_item   = nullptr;
seq::NumericalItem*       g_clk_div_item      = nullptr;  // audit 2d
seq::NumericalItem*       g_clk_mult_item     = nullptr;  // task A8
seq::ActionItem*          g_clear_cv_item     = nullptr;  // audit 2a
seq::ActionItem*          g_clear_trig_item   = nullptr;  // audit 2a
seq::ToggleItem*          g_run_item          = nullptr;  // audit 2f
seq::SingleSelectItem*    g_cv_source_item    = nullptr;
seq::SingleSelectItem*    g_dig_in_item       = nullptr;  // audit 2e
std::vector<seq::MenuItem*> g_items;

// Sim-only: fire a single digital-in rising/falling edge on the
// Sequencer when the user presses D. Matches the manual-pulse pattern.
std::atomic<bool>      g_digital_pulse_request{false};

}  // namespace

// ============================================================
//  Scale rebuild
// ============================================================
static void RebuildScale_locked() {
  if (g_scale_idx < 0 || g_scale_idx >= seq::kNumScales) g_scale_idx = 0;
  g_scale_count = seq::BuildScale(seq::kScales[g_scale_idx],
                                  g_starting_note + 12,
                                  g_octaves, g_scale);
  g_sparams.scale        = g_scale;
  g_sparams.scale_length = g_scale_count;
  g_sequencer.SetParams(g_sparams);
}

// ============================================================
//  One-shot actions (audit 2a)
// ============================================================
// Called by the corresponding ActionItem when pressed in the menu. The
// caller already holds g_mutex (the menu OnPress path runs under lock),
// so these touch engine state directly.
static void ActionClearCv(void* /*user*/) {
  g_sequencer.voice(0).ClearCv(g_scale, g_scale_count);
}
static void ActionClearTriggers(void* /*user*/) {
  g_sequencer.voice(0).ClearTriggers();
}

// ============================================================
//  Note-name helper — DAC → "C3" / "D#4" etc.
// ============================================================
// Matches the audition's pitch model: DAC 0 = MIDI 24 = C1, every 68
// DAC units = 1 semitone. Used by the OLED PlaybackView, the engine
// inspector's CV-bar labels, and the active-scale-notes panel.
namespace {
constexpr const char* kNoteNames[12] = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};
}  // namespace

static void DacToNoteName(uint16_t dac, char* out, int cap) {
  const int midi      = 24 + static_cast<int>(dac) / 68;
  const int idx       = ((midi % 12) + 12) % 12;
  const int octave    = (midi / 12) - 1;   // MIDI 24 = C1
  std::snprintf(out, cap, "%s%d", kNoteNames[idx], octave);
}

// ============================================================
//  PlaybackView — OLED home screen showing the engine in motion
// ============================================================
// Sits at the bottom of the view stack. Encoder click pushes the menu
// list on top of it; long-press inside the menu pops back here.
class PlaybackView : public seq::View {
 public:
  void Draw(seq::FakeOled& oled) const override {
    oled.Clear();

    // Header: PLAY/STOP, scale (truncated), step n/total
    const int  step  = g_disp_step.load();
    const int  steps = g_steps_item ? g_steps_item->value() : 16;
    const bool run   = g_run_item  ? g_run_item->value()    : false;
    const char* scale = (g_scale_idx >= 0 && g_scale_idx < seq::kNumScales)
                        ? seq::kScales[g_scale_idx].name : "";
    char scale_short[8];
    int sl = 0;
    for (; scale[sl] && sl < 7; ++sl) scale_short[sl] = scale[sl];
    scale_short[sl] = 0;

    char header[24];
    std::snprintf(header, sizeof(header), "%s %s %02d/%02d",
                  run ? "PLAY" : "STOP",
                  scale_short, step + 1, steps);
    oled.Text(2, 2, header);
    oled.Rect(0, 0, 128, 12);

    // Step grid — 8 cells per row, 2 rows.
    constexpr int kTopY  = 16;
    constexpr int kCellW = 14;
    constexpr int kCellH = 14;
    constexpr int kGapX  = 2;
    constexpr int kGapY  = 2;
    for (int i = 0; i < seq::kMaxSteps; ++i) {
      const int col = i % 8;
      const int row = i / 8;
      const int x   = col * (kCellW + kGapX);
      const int y   = kTopY + row * (kCellH + kGapY);
      const bool active = (i < steps);
      const bool trig   = g_disp_trig_grid[i] != 0;
      const bool is_cur = (i == step);
      if (!active) {
        oled.Px(x + kCellW / 2, y + kCellH / 2, true);   // dim dot
      } else if (is_cur) {
        oled.FillRect(x, y, kCellW, kCellH);             // filled = now playing
      } else if (trig) {
        oled.Rect(x, y, kCellW, kCellH);                 // outlined = trig on
      } else {
        // four corners (a "trig off" tag)
        oled.Px(x, y, true);
        oled.Px(x + kCellW - 1, y, true);
        oled.Px(x, y + kCellH - 1, true);
        oled.Px(x + kCellW - 1, y + kCellH - 1, true);
      }
    }

    // Footer: current note name + DAC value + trig indicator
    char note[8];
    DacToNoteName(g_disp_dac.load(), note, sizeof(note));
    char footer[28];
    std::snprintf(footer, sizeof(footer), "%-4s %4u %s",
                  note,
                  static_cast<unsigned>(g_disp_dac.load()),
                  g_disp_trig.load() ? "TRIG" : "    ");
    oled.Text(2, 53, footer);
  }

  // Click → enter the menu.
  void OnPress() override {
    if (stack()) stack()->Push(&g_menu_view);
  }
  // Long-press at the root → no-op (default View::OnLongPress doesn't pop
  // when depth is 1, so this is implicit).
};

PlaybackView g_playback_view;

// ============================================================
//  Menu commit callback — sync menu items into engine params
// ============================================================
static void OnMenuCommit(void* /*user*/) {
  g_scale_idx                  = g_scale_item->selected_index();
  g_vparams.cv_change_pct      = g_cv_prob_item->value();
  g_vparams.trig_change_pct    = g_trig_prob_item->value();
  g_vparams.trig_length_pct    = g_trig_length_item->value();
  g_vparams.number_of_steps    = g_steps_item->value();
  g_octaves                    = g_octaves_item->value();
  g_starting_note              = g_start_note_item->value();
  g_vparams.clock_divider      = g_clk_div_item->value();
  g_vparams.clock_multiplier   = g_clk_mult_item->value();
  g_vparams.cv_source          = static_cast<seq::CvSource>(
      g_cv_source_item->selected_index());

  g_sparams.enabled            = g_run_item->value();
  g_sparams.digital_in_mode    = static_cast<seq::DigitalInMode>(
      g_dig_in_item->selected_index());

  g_sequencer.voice(0).SetParams(g_vparams);
  RebuildScale_locked();   // also re-applies g_sparams via Sequencer::SetParams
}

// ============================================================
//  Menu setup
// ============================================================
static void BuildMenu() {
  g_scale_idx = seq::FindScaleIndex("major");
  if (g_scale_idx < 0) g_scale_idx = 0;

  g_scale_names.reserve(seq::kNumScales);
  for (int i = 0; i < seq::kNumScales; ++i)
    g_scale_names.push_back(seq::kScales[i].name);

  g_run_item          = new seq::ToggleItem("Run", true);
  g_scale_item        = new seq::SingleSelectItem(
      "Scale", g_scale_names.data(), seq::kNumScales, g_scale_idx);
  g_cv_prob_item      = new seq::NumericalItem("CVProb",    0,  0, 100, 5);
  g_trig_prob_item    = new seq::NumericalItem("TrigProb",  0,  0, 100, 5);
  g_trig_length_item  = new seq::NumericalItem("TrgLngth%",50,  0, 100, 10);
  g_steps_item        = new seq::NumericalItem("Steps",    16,
                                               seq::kMinSteps,
                                               seq::kMaxSteps, 1);
  g_octaves_item      = new seq::NumericalItem("Octaves",   1,
                                               seq::kMinOctaves,
                                               seq::kMaxOctaves, 1);
  g_start_note_item   = new seq::NumericalItem("Start note",0,  0, 36, 1);
  g_clk_div_item      = new seq::NumericalItem("ClkDiv",   1,  1, 16, 1);
  g_clk_mult_item     = new seq::NumericalItem("ClkMult",  1,  1, 16, 1);
  g_clear_cv_item     = new seq::ActionItem("Clear CV",   &ActionClearCv,       nullptr);
  g_clear_trig_item   = new seq::ActionItem("Clear Trig", &ActionClearTriggers, nullptr);
  g_cv_source_item    = new seq::SingleSelectItem(
      "CV Source", kCvSourceNames, kCvSourceCount, 0);
  g_dig_in_item       = new seq::SingleSelectItem(
      "DigIn", kDigInModeNames, kDigInModeCount,
      static_cast<int>(seq::DigitalInMode::Reset));

  // Order: transport-y stuff first, then params, then modes/actions.
  g_items = {
    g_run_item, g_scale_item,
    g_cv_prob_item, g_trig_prob_item, g_trig_length_item,
    g_steps_item, g_octaves_item, g_start_note_item,
    g_clk_div_item, g_clk_mult_item,
    g_cv_source_item, g_dig_in_item,
    g_clear_cv_item, g_clear_trig_item,
  };

  g_menu_view.SetTitle("Main Menu");
  g_menu_view.SetItems(g_items.data(), static_cast<int>(g_items.size()));
  g_menu_view.SetCommitCallback(&OnMenuCommit, nullptr);

  // PlaybackView is the bottom of the view stack — the OLED "home"
  // screen. User clicks the encoder to push the menu on top of it,
  // long-presses inside the menu to pop back here.
  g_views.Push(&g_playback_view);

  OnMenuCommit(nullptr);
  g_sequencer.Init();
}

// ============================================================
//  Clock thread
// ============================================================
static void ClockThreadMain() {
  // 1ms tick loop. Each iteration:
  //   1) Service async requests (reset, digital-in pulse, manual clock).
  //   2) If the internal clock is running and now >= next_edge_time,
  //      emit one clock edge and advance the next-edge timestamp.
  //   3) Always call Sequencer::Tick() so scheduled sub-edges fire on
  //      time and the trigger-off timer runs while paused.
  //   4) Refresh the display atomics.
  //
  // The bug this replaces: previously Tick was only called at clock
  // edges (every half-period). Sub-edges scheduled by clock-multiplier
  // > 1 between edges accumulated and fired all at once at the next
  // Tick — "burst then gap" instead of even sub-step spacing.
  using clock_t = std::chrono::steady_clock;
  auto next_edge_time = clock_t::now();
  bool clock_high     = false;
  bool was_running    = false;

  while (!g_app_quitting.load()) {
    if (g_reset_request.exchange(false)) {
      std::lock_guard<std::mutex> lk(g_mutex);
      g_sequencer.Reset();
    }
    if (g_digital_pulse_request.exchange(false)) {
      const auto t0 = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              clock_t::now().time_since_epoch()).count());
      std::lock_guard<std::mutex> lk(g_mutex);
      g_sequencer.OnDigitalEdge(true,  t0);
      g_sequencer.OnDigitalEdge(false, t0 + 1);
    }
    if (g_manual_pulse_request.exchange(false)) {
      const auto t0 = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              clock_t::now().time_since_epoch()).count());
      constexpr uint32_t kManualGateMs = 100;
      {
        std::lock_guard<std::mutex> lk(g_mutex);
        g_sequencer.OnClockEdge(true, t0);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kManualGateMs));
      const auto t1 = t0 + kManualGateMs;
      {
        std::lock_guard<std::mutex> lk(g_mutex);
        g_sequencer.OnClockEdge(false, t1);
      }
    }

    const auto now    = clock_t::now();
    const auto now_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count());
    const bool running = g_clock_running.load();

    // Edge-trigger when the internal clock flips on: schedule first
    // rising edge a half-period out so it starts cleanly.
    if (running && !was_running) {
      const int bpm       = g_bpm.load();
      const int period_ms = (bpm > 0) ? (60000 / bpm) : 1000;
      next_edge_time = now + std::chrono::milliseconds(period_ms / 2);
      clock_high     = false;
    }
    was_running = running;

    {
      std::lock_guard<std::mutex> lk(g_mutex);

      // Emit clock edges based on absolute time (allows catching up if
      // we ever sleep a bit long).
      if (running) {
        while (now >= next_edge_time) {
          clock_high = !clock_high;
          g_sequencer.OnClockEdge(clock_high, now_ms);
          const int bpm       = g_bpm.load();
          const int period_ms = (bpm > 0) ? (60000 / bpm) : 1000;
          next_edge_time += std::chrono::milliseconds(period_ms / 2);
        }
      }

      g_sequencer.Tick(now_ms);

      const seq::Voice& v = g_sequencer.voice(0);
      g_disp_step.store(v.last_played_step());
      g_disp_dac.store(v.last_dac());
      g_disp_trig.store(v.trig_active());
      std::memcpy(g_disp_cv,        v.cv_sequence(),      sizeof(g_disp_cv));
      std::memcpy(g_disp_trig_grid, v.trigger_sequence(), sizeof(g_disp_trig_grid));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// ============================================================
//  OLED widget
// ============================================================
static void RenderOledWidget() {
  // Render whatever view is on top of the stack into the framebuffer.
  {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_views.Draw(g_oled);
  }
  ImGui::Begin("OLED 128x64");
  // Tiny status banner — what view are we looking at?
  const int depth = g_views.depth();
  const char* view_name = (depth <= 1) ? "Playback"
                         : (depth == 2) ? "Menu"
                                       : "Editing";
  ImGui::TextColored(ImVec4(0.65f, 0.80f, 1.00f, 1.0f), "View: %s", view_name);
  ImGui::TextColored(ImVec4(0.55f, 0.55f, 0.55f, 1.0f),
                     "Space click  ← → rotate  Backspace cancel");
  ImGui::Separator();

  ImDrawList* dl = ImGui::GetWindowDrawList();
  // Bezel — a slightly larger rounded rect framing the OLED.
  const float scale  = 3.0f;
  const float bezel  = 8.0f;
  ImVec2 cursor      = ImGui::GetCursorScreenPos();
  ImVec2 bz_p0(cursor.x, cursor.y);
  ImVec2 bz_p1(bz_p0.x + 128 * scale + bezel * 2,
               bz_p0.y + 64  * scale + bezel * 2);
  dl->AddRectFilled(bz_p0, bz_p1, IM_COL32(28, 32, 38, 255), 6.0f);
  dl->AddRect      (bz_p0, bz_p1, IM_COL32(80, 90, 110, 255), 6.0f, 0, 1.5f);

  // Inner panel
  ImVec2 p0(bz_p0.x + bezel, bz_p0.y + bezel);
  ImVec2 p1(p0.x + 128 * scale, p0.y + 64 * scale);
  dl->AddRectFilled(p0, p1, IM_COL32(8, 12, 20, 255));
  for (int y = 0; y < 64; ++y) {
    for (int x = 0; x < 128; ++x) {
      if (g_oled.buf[y * 128 + x]) {
        ImVec2 a(p0.x + x * scale, p0.y + y * scale);
        ImVec2 b(a.x + scale, a.y + scale);
        dl->AddRectFilled(a, b, IM_COL32(140, 220, 255, 255));
      }
    }
  }
  ImGui::Dummy(ImVec2(128 * scale + bezel * 2, 64 * scale + bezel * 2));
  ImGui::End();
}

// ============================================================
//  Sequencer Controls window
// ============================================================
static void RenderControlsWindow() {
  ImGui::Begin("Sequencer Controls");

  // ---- Status banner at the top ----
  {
    char note[8];
    DacToNoteName(g_disp_dac.load(), note, sizeof(note));
    const int  step  = g_disp_step.load();
    const int  steps = g_steps_item ? g_steps_item->value() : 16;
    const bool run   = g_run_item  ? g_run_item->value()    : false;
    const bool trig  = g_disp_trig.load();
    const char* scale = (g_scale_idx >= 0 && g_scale_idx < seq::kNumScales)
                        ? seq::kScales[g_scale_idx].name : "";
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 bnr_p0 = ImGui::GetCursorScreenPos();
    const ImU32 bg     = run ? IM_COL32(28, 38, 48, 255) : IM_COL32(40, 28, 30, 255);
    const ImU32 accent = run ? IM_COL32(120, 200, 255, 255) : IM_COL32(220, 120, 120, 255);
    ImVec2 bnr_p1(bnr_p0.x + ImGui::GetContentRegionAvail().x, bnr_p0.y + 36);
    dl->AddRectFilled(bnr_p0, bnr_p1, bg, 4.0f);
    dl->AddRect      (bnr_p0, bnr_p1, accent, 4.0f, 0, 1.5f);
    char status[40];
    std::snprintf(status, sizeof(status), "%s   STEP %02d/%02d   %-4s   %s",
                  run ? "PLAY" : "STOP",
                  step + 1, steps, note,
                  trig ? "TRIG" : "    ");
    dl->AddText(ImVec2(bnr_p0.x + 8, bnr_p0.y + 5),
                IM_COL32(220, 230, 240, 255), status);
    char meta[48];
    std::snprintf(meta, sizeof(meta), "%s   %d BPM   DAC %u",
                  scale, g_bpm.load(),
                  static_cast<unsigned>(g_disp_dac.load()));
    dl->AddText(ImVec2(bnr_p0.x + 8, bnr_p0.y + 20),
                IM_COL32(140, 160, 180, 255), meta);
    ImGui::Dummy(ImVec2(0, 40));
  }

  ImGui::SeparatorText("Internal clock");
  bool running = g_clock_running.load();
  if (ImGui::Checkbox("Clock running", &running)) g_clock_running.store(running);
  int bpm = g_bpm.load();
  if (ImGui::SliderInt("BPM", &bpm, 20, 240)) g_bpm.store(bpm);

  // Audition (desktop-only) — sine + AR envelope driven by voice(0).
  ImGui::SeparatorText("Audition (sim only — hear the engine)");
  bool aud = g_audition_enabled.load();
  if (ImGui::Checkbox("Audio on", &aud)) g_audition_enabled.store(aud);
  float vol = g_audition_volume.load();
  if (ImGui::SliderFloat("Volume", &vol, 0.0f, 1.0f, "%.2f")) g_audition_volume.store(vol);
  float pitch = g_audition_pitch_semis.load();
  if (ImGui::SliderFloat("Pitch (semis)", &pitch, -48.0f, 48.0f, "%+.1f")) g_audition_pitch_semis.store(pitch);
  ImGui::SameLine();
  if (ImGui::Button("0##pitch_reset")) g_audition_pitch_semis.store(0.0f);
  float atk = g_audition_attack_ms.load();
  if (ImGui::SliderFloat("Attack ms", &atk, 1.0f, 500.0f, "%.0f")) g_audition_attack_ms.store(atk);
  float rel = g_audition_release_ms.load();
  if (ImGui::SliderFloat("Release ms", &rel, 10.0f, 3000.0f, "%.0f")) g_audition_release_ms.store(rel);

  ImGui::SeparatorText("Direct param edit (syncs to menu)");
  bool changed = false;
  int  cv_prob, trig_prob, trig_len, steps, octs, start_note, clk_div, clk_mult;
  bool run;
  int  cv_source_idx, dig_in_idx, scale_idx;
  {
    std::lock_guard<std::mutex> lk(g_mutex);
    run           = g_run_item->value();
    cv_prob       = g_cv_prob_item->value();
    trig_prob     = g_trig_prob_item->value();
    trig_len      = g_trig_length_item->value();
    steps         = g_steps_item->value();
    octs          = g_octaves_item->value();
    start_note    = g_start_note_item->value();
    clk_div       = g_clk_div_item->value();
    clk_mult      = g_clk_mult_item->value();
    cv_source_idx = g_cv_source_item->selected_index();
    dig_in_idx    = g_dig_in_item->selected_index();
    scale_idx     = g_scale_item->selected_index();
  }
  changed |= ImGui::Checkbox("Run (transport)", &run);

  if (ImGui::BeginCombo("Scale", seq::kScales[scale_idx].name)) {
    for (int i = 0; i < seq::kNumScales; ++i) {
      bool sel = (i == scale_idx);
      if (ImGui::Selectable(seq::kScales[i].name, sel)) {
        scale_idx = i;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  changed |= ImGui::SliderInt("Steps",         &steps,      seq::kMinSteps,   seq::kMaxSteps);
  changed |= ImGui::SliderInt("Octaves",       &octs,       seq::kMinOctaves, seq::kMaxOctaves);
  changed |= ImGui::SliderInt("Start note",    &start_note, 0, 36);
  changed |= ImGui::SliderInt("Clock divider",    &clk_div,  1, 16);
  changed |= ImGui::SliderInt("Clock multiplier", &clk_mult, 1, 16);
  changed |= ImGui::SliderInt("CV change %",   &cv_prob,    0, 100);
  changed |= ImGui::SliderInt("Trig change %", &trig_prob,  0, 100);
  changed |= ImGui::SliderInt("Trig length %", &trig_len,   0, 100);
  // One-shot actions (replace the v0.3 destructive toggles)
  if (ImGui::Button("Clear CV (action)")) {
    std::lock_guard<std::mutex> lk(g_mutex);
    ActionClearCv(nullptr);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Trig (action)")) {
    std::lock_guard<std::mutex> lk(g_mutex);
    ActionClearTriggers(nullptr);
  }
  if (ImGui::BeginCombo("CV source", kCvSourceNames[cv_source_idx])) {
    for (int i = 0; i < kCvSourceCount; ++i) {
      bool sel = (i == cv_source_idx);
      if (ImGui::Selectable(kCvSourceNames[i], sel)) {
        cv_source_idx = i;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  if (ImGui::BeginCombo("Digital-in mode", kDigInModeNames[dig_in_idx])) {
    for (int i = 0; i < kDigInModeCount; ++i) {
      bool sel = (i == dig_in_idx);
      if (ImGui::Selectable(kDigInModeNames[i], sel)) {
        dig_in_idx = i;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  ImGui::SameLine();
  if (ImGui::Button("Fire digital-in pulse (D)")) {
    g_digital_pulse_request.store(true);
  }

  if (changed) {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_run_item->set_value(run);
    g_scale_item->set_selected_index(scale_idx);
    g_cv_prob_item->set_value(cv_prob);
    g_trig_prob_item->set_value(trig_prob);
    g_trig_length_item->set_value(trig_len);
    g_steps_item->set_value(steps);
    g_octaves_item->set_value(octs);
    g_start_note_item->set_value(start_note);
    g_clk_div_item->set_value(clk_div);
    g_clk_mult_item->set_value(clk_mult);
    g_cv_source_item->set_selected_index(cv_source_idx);
    g_dig_in_item->set_selected_index(dig_in_idx);
    OnMenuCommit(nullptr);
  }

  ImGui::SeparatorText("Engine inspector");
  {
    char note[8];
    DacToNoteName(g_disp_dac.load(), note, sizeof(note));
    ImGui::Text("Step %2d   Note %-4s  DAC %4u   TRIG %s",
                g_disp_step.load(),
                note,
                static_cast<unsigned>(g_disp_dac.load()),
                g_disp_trig.load() ? "ON" : "off");
  }
  ImGui::Text("Scale: %d notes,  Views in stack: %d",
              g_scale_count, g_views.depth());

  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImVec2 grid_p0 = ImGui::GetCursorScreenPos();
  const float cell  = 18.0f;
  const float gap   = 3.0f;
  const int   step  = g_disp_step.load();
  for (int i = 0; i < seq::kMaxSteps; ++i) {
    const int   col = i % 8;
    const int   row = i / 8;
    const ImVec2 a(grid_p0.x + col * (cell + gap),
                   grid_p0.y + row * (cell + gap));
    const ImVec2 b(a.x + cell, a.y + cell);
    const bool active_seq = (i < steps);
    const bool has_trig   = g_disp_trig_grid[i] != 0;
    const bool is_cur     = (i == step);
    if (!active_seq) {
      dl->AddRect(a, b, IM_COL32(60, 60, 60, 255));
    } else if (is_cur) {
      dl->AddRectFilled(a, b, IM_COL32(255, 200, 80, 255));
    } else if (has_trig) {
      dl->AddRectFilled(a, b, IM_COL32(80, 100, 130, 255));
    } else {
      dl->AddRect(a, b, IM_COL32(120, 120, 120, 255));
    }
  }
  ImGui::Dummy(ImVec2(8 * (cell + gap), 2 * (cell + gap) + 6));

  // CV-per-step bar chart. Each bar height = cv_sequence[i]/4095, labels
  // below it show the note name (C3, D#4, etc.) — so you can read the
  // pattern at a glance instead of decoding 12-bit DAC numbers.
  ImGui::SeparatorText("CV per step");
  ImVec2 bar_p0 = ImGui::GetCursorScreenPos();
  const float bar_w     = 22.0f;
  const float bar_gap   = 2.0f;
  const float bar_max_h = 56.0f;
  const float label_h   = 14.0f;
  for (int i = 0; i < seq::kMaxSteps; ++i) {
    const float x  = bar_p0.x + i * (bar_w + bar_gap);
    const float frac = static_cast<float>(g_disp_cv[i]) / 4095.0f;
    const float bh = bar_max_h * (frac < 0.0f ? 0.0f : (frac > 1.0f ? 1.0f : frac));
    const ImVec2 a(x, bar_p0.y + (bar_max_h - bh));
    const ImVec2 b(x + bar_w, bar_p0.y + bar_max_h);
    const bool   is_cur = (i == step);
    const bool   active = (i < steps);
    ImU32 fill_col = active ? (is_cur ? IM_COL32(255, 200, 80, 255)
                                      : IM_COL32(120, 160, 220, 255))
                            : IM_COL32(60, 60, 60, 255);
    dl->AddRectFilled(a, b, fill_col);
    dl->AddRect(ImVec2(x, bar_p0.y),
                ImVec2(x + bar_w, bar_p0.y + bar_max_h),
                IM_COL32(80, 80, 80, 255));

    // Note name + step number labels under each bar
    if (active) {
      char note[8];
      DacToNoteName(g_disp_cv[i], note, sizeof(note));
      const ImVec2 tsz   = ImGui::CalcTextSize(note);
      const float  label_x = x + (bar_w - tsz.x) / 2.0f;
      dl->AddText(ImVec2(label_x, bar_p0.y + bar_max_h + 1),
                  is_cur ? IM_COL32(255, 220, 120, 255)
                         : IM_COL32(180, 200, 220, 255),
                  note);
    }
  }
  ImGui::Dummy(ImVec2(seq::kMaxSteps * (bar_w + bar_gap),
                      bar_max_h + label_h + 6));

  // Active scale notes — what notes is the engine actually choosing
  // from? Renders as note names rather than DAC numbers ("C3 D3 E3 …"
  // vs "1632 1768 1904 …"). Verifies that the scale + starting note +
  // octave combination is doing what you expect.
  ImGui::SeparatorText("Active scale notes");
  {
    std::lock_guard<std::mutex> lk(g_mutex);
    char buf[256];
    int  w = 0;
    for (int i = 0; i < g_scale_count && w < (int)sizeof(buf) - 8; ++i) {
      char note[8];
      DacToNoteName(g_scale[i], note, sizeof(note));
      w += std::snprintf(buf + w, sizeof(buf) - w,
                         (i == 0) ? "%s" : "  %s", note);
    }
    ImGui::TextWrapped("%s", buf);
  }

  ImGui::End();
}

// ============================================================
//  Keyboard
// ============================================================
static void OnKey(GLFWwindow* win, int key, int /*sc*/, int action, int /*mods*/) {
  if (action != GLFW_PRESS && action != GLFW_REPEAT) return;
  switch (key) {
    case GLFW_KEY_LEFT: {
      std::lock_guard<std::mutex> lk(g_mutex);
      g_views.OnRotate(-1);
      break;
    }
    case GLFW_KEY_RIGHT: {
      std::lock_guard<std::mutex> lk(g_mutex);
      g_views.OnRotate(+1);
      break;
    }
    case GLFW_KEY_SPACE:
      if (action == GLFW_PRESS) {
        std::lock_guard<std::mutex> lk(g_mutex);
        g_views.OnPress();
      }
      break;
    case GLFW_KEY_BACKSPACE:
      if (action == GLFW_PRESS) {
        std::lock_guard<std::mutex> lk(g_mutex);
        g_views.OnLongPress();
      }
      break;
    // Number keys 1-9: jump directly to that submenu, but only when the
    // root list is on top (avoids weird jumps from inside an editor).
    case GLFW_KEY_1: case GLFW_KEY_2: case GLFW_KEY_3:
    case GLFW_KEY_4: case GLFW_KEY_5: case GLFW_KEY_6:
    case GLFW_KEY_7: case GLFW_KEY_8: case GLFW_KEY_9:
      if (action == GLFW_PRESS) {
        std::lock_guard<std::mutex> lk(g_mutex);
        // Number-keys jump only when the menu list is on top — not from
        // the PlaybackView home screen, and not from inside an editor.
        if (g_views.top() == &g_menu_view) {
          g_menu_view.JumpTo(key - GLFW_KEY_1);
        }
      }
      break;
    case GLFW_KEY_G:
      if (action == GLFW_PRESS) g_manual_pulse_request.store(true);
      break;
    case GLFW_KEY_D:
      if (action == GLFW_PRESS) g_digital_pulse_request.store(true);
      break;
    case GLFW_KEY_S:
      if (action == GLFW_PRESS) g_clock_running.store(!g_clock_running.load());
      break;
    case GLFW_KEY_R:
      if (action == GLFW_PRESS) g_reset_request.store(true);
      break;
    case GLFW_KEY_ESCAPE:
    case GLFW_KEY_Q:
      if (action == GLFW_PRESS) glfwSetWindowShouldClose(win, GLFW_TRUE);
      break;
    default: break;
  }
}

// ============================================================
//  Audition audio callback
// ============================================================
static int AuditionCb(const void* /*input*/, void* output,
                      unsigned long frames,
                      const PaStreamCallbackTimeInfo* /*time*/,
                      PaStreamCallbackFlags /*flags*/,
                      void* /*user*/) {
  auto* out = static_cast<float*>(output);

  if (!g_audition_enabled.load(std::memory_order_relaxed)) {
    std::memset(output, 0, sizeof(float) * frames * 2);
    return paContinue;
  }

  const uint16_t dac          = g_disp_dac.load(std::memory_order_relaxed);
  const bool     trig         = g_disp_trig.load(std::memory_order_relaxed);
  const float    volume       = g_audition_volume.load(std::memory_order_relaxed);
  const float    attack_ms    = g_audition_attack_ms.load(std::memory_order_relaxed);
  const float    release_ms   = g_audition_release_ms.load(std::memory_order_relaxed);
  const float    pitch_offset = g_audition_pitch_semis.load(std::memory_order_relaxed);

  // Edge-detect trig → env state transitions
  if (trig && !g_audio_prev_trig)        g_audio_env_stage = EnvStage::Attack;
  else if (!trig && g_audio_prev_trig)   g_audio_env_stage = EnvStage::Release;
  g_audio_prev_trig = trig;

  // DAC → MIDI → Hz. Matches main.py's note*68 convention: 68 DAC units
  // per semitone. Base offset of 24 puts the lowest reachable note at
  // C1 (~32.7 Hz), giving headroom for the full 5-octave scale span.
  const float midi      = 24.0f + static_cast<float>(dac) / 68.0f + pitch_offset;
  const float freq_hz   = 440.0f * std::pow(2.0f, (midi - 69.0f) / 12.0f);
  const float phase_inc = kTwoPi * freq_hz / static_cast<float>(kAudioSampleRate);

  const float atk_per_s = 1.0f / (attack_ms  * 0.001f * kAudioSampleRate);
  const float rel_per_s = 1.0f / (release_ms * 0.001f * kAudioSampleRate);

  for (unsigned long i = 0; i < frames; ++i) {
    switch (g_audio_env_stage) {
      case EnvStage::Attack:
        g_audio_env += atk_per_s;
        if (g_audio_env >= 1.0f) {
          g_audio_env       = 1.0f;
          g_audio_env_stage = EnvStage::Sustain;
        }
        break;
      case EnvStage::Sustain:
        g_audio_env = 1.0f;
        break;
      case EnvStage::Release:
        g_audio_env -= rel_per_s;
        if (g_audio_env <= 0.0f) {
          g_audio_env       = 0.0f;
          g_audio_env_stage = EnvStage::Idle;
        }
        break;
      case EnvStage::Idle:
      default:
        g_audio_env = 0.0f;
        break;
    }

    const float s = std::sin(g_audio_phase) * g_audio_env * volume;
    out[2 * i + 0] = s;
    out[2 * i + 1] = s;
    g_audio_phase += phase_inc;
    if (g_audio_phase >  kTwoPi) g_audio_phase -= kTwoPi;
    if (g_audio_phase < -kTwoPi) g_audio_phase += kTwoPi;
  }
  return paContinue;
}

// ============================================================
//  Entry
// ============================================================
int main() {
  BuildMenu();

  if (!glfwInit()) { std::fprintf(stderr, "glfwInit failed\n"); return 1; }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

  GLFWwindow* win = glfwCreateWindow(
      1100, 720, "Generative Sequencer — Playground", nullptr, nullptr);
  if (!win) { glfwTerminate(); return 1; }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);
  glfwSetKeyCallback(win, [](GLFWwindow* w, int k, int sc, int a, int m) {
    OnKey(w, k, sc, a, m);
  });

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  // PortAudio for audition. Stream always runs (callback emits silence
  // when audition is disabled) — simpler than opening/closing on demand.
  bool audio_ok = false;
  PaStream* audio_stream = nullptr;
  if (Pa_Initialize() == paNoError) {
    PaError err = Pa_OpenDefaultStream(&audio_stream,
                                       0, 2, paFloat32,
                                       kAudioSampleRate, 256,
                                       AuditionCb, nullptr);
    if (err == paNoError && Pa_StartStream(audio_stream) == paNoError) {
      audio_ok = true;
    } else {
      std::fprintf(stderr, "PortAudio failed: %s\n", Pa_GetErrorText(err));
    }
  } else {
    std::fprintf(stderr, "PortAudio Init failed\n");
  }

  std::thread clock_thread(ClockThreadMain);

  while (!glfwWindowShouldClose(win)) {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    RenderOledWidget();
    RenderControlsWindow();
    ImGui::Render();

    int w, h; glfwGetFramebufferSize(win, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(win);
  }

  g_app_quitting.store(true);
  clock_thread.join();

  if (audio_ok && audio_stream) {
    Pa_StopStream(audio_stream);
    Pa_CloseStream(audio_stream);
  }
  Pa_Terminate();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(win);
  glfwTerminate();

  delete g_scale_item;       delete g_cv_prob_item;
  delete g_trig_prob_item;   delete g_trig_length_item;
  delete g_steps_item;       delete g_octaves_item;
  delete g_start_note_item;  delete g_clk_div_item;
  delete g_clk_mult_item;    delete g_clear_cv_item;
  delete g_clear_trig_item;
  delete g_run_item;         delete g_cv_source_item;
  delete g_dig_in_item;
  return 0;
}
