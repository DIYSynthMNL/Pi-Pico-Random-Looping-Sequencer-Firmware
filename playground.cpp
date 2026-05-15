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

// Menu items + view stack — hierarchical now.
//
//   PlaybackView  (root, depth 1)
//     └─ g_main_menu      (depth 2)
//         ├─ Run                       (toggle, inline)
//         ├─ CLOCK >  → g_clock_menu   (depth 3)
//         ├─ PITCH >  → g_pitch_menu   (depth 3)
//         ├─ PROBS >  → g_probs_menu   (depth 3)
//         └─ ACTIONS> → g_actions_menu (depth 3)
//
// Long-press pops at every level (PlaybackView at depth 1 cycles its
// layout instead). The encoder's number-key shortcuts (1-9) jump within
// whichever MenuListView happens to be on top.
std::vector<const char*>  g_scale_names;
seq::ViewStack            g_views;
seq::MenuListView         g_main_menu;
seq::MenuListView         g_clock_menu;
seq::MenuListView         g_pitch_menu;
seq::MenuListView         g_probs_menu;
seq::MenuListView         g_actions_menu;

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

// Submenu-entry items on the main menu (each pushes its child menu).
seq::SubmenuItem*         g_clock_sub_item    = nullptr;
seq::SubmenuItem*         g_pitch_sub_item    = nullptr;
seq::SubmenuItem*         g_probs_sub_item    = nullptr;
seq::SubmenuItem*         g_actions_sub_item  = nullptr;

// Item arrays per menu (must outlive the MenuListViews).
std::vector<seq::MenuItem*> g_main_items;
std::vector<seq::MenuItem*> g_clock_items;
std::vector<seq::MenuItem*> g_pitch_items;
std::vector<seq::MenuItem*> g_probs_items;
std::vector<seq::MenuItem*> g_actions_items;

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
//  Icons — 6x8 pixel bitmaps drawn directly into the OLED buffer
// ============================================================
// Each icon is 6 columns × 8 rows, bit 0 = top row. Same encoding as
// the FakeOled font glyphs. Drawn by DrawIcon, which respects the
// `on` flag (so icons over an inverted background render correctly).
namespace icons {

struct Icon6x8 { uint8_t cols[6]; };

constexpr Icon6x8 kPlay    = {{0xFF, 0x7E, 0x3C, 0x18, 0x00, 0x00}};  // right-pointing triangle
constexpr Icon6x8 kStop    = {{0x00, 0x7E, 0x7E, 0x7E, 0x7E, 0x00}};  // filled square
[[maybe_unused]] constexpr Icon6x8 kPause   = {{0x00, 0x7E, 0x7E, 0x00, 0x7E, 0x7E}};
[[maybe_unused]] constexpr Icon6x8 kNote    = {{0x00, 0x30, 0xFF, 0x00, 0x18, 0x1C}};
constexpr Icon6x8 kTrig    = {{0xFF, 0x03, 0x03, 0xFF, 0x80, 0x80}};  // pulse shape
[[maybe_unused]] constexpr Icon6x8 kDice    = {{0x7E, 0x49, 0x65, 0x49, 0x7E, 0x00}};
[[maybe_unused]] constexpr Icon6x8 kReset   = {{0x3C, 0x42, 0x81, 0x81, 0x42, 0x18}};
[[maybe_unused]] constexpr Icon6x8 kLoop    = {{0x3C, 0x42, 0x81, 0x91, 0x42, 0x3C}};
[[maybe_unused]] constexpr Icon6x8 kArrowUp = {{0x08, 0x0C, 0xFE, 0xFE, 0x0C, 0x08}};
[[maybe_unused]] constexpr Icon6x8 kArrowDn = {{0x10, 0x30, 0x7F, 0x7F, 0x30, 0x10}};

static void Draw(seq::FakeOled& oled, const Icon6x8& icon,
                 int x, int y, bool on = true) {
  for (int col = 0; col < 6; ++col) {
    const uint8_t bits = icon.cols[col];
    for (int row = 0; row < 8; ++row) {
      if (bits & (1u << row)) oled.Px(x + col, y + row, on);
    }
  }
}

}  // namespace icons

// ============================================================
//  PlaybackView — OLED home screen showing the engine in motion
// ============================================================
// Sits at the bottom of the view stack. Encoder click pushes the menu
// list on top of it; long-press cycles between playback layouts.
class PlaybackView : public seq::View {
 public:
  enum class Layout : uint8_t { Grid = 0, PianoRoll = 1, Count = 2 };

  void Draw(seq::FakeOled& oled) const override {
    oled.Clear();   // critical — without this, previous frames' pixels
                    // (especially the menu's, after popping back) bleed
                    // through and ghost the playback view.
    DrawHeader(oled);
    switch (layout_) {
      case Layout::PianoRoll: DrawPianoRoll(oled); break;
      case Layout::Grid:
      default:                DrawGrid(oled);      break;
    }
    DrawLayoutHint(oled);
  }

  // Click → enter the menu.
  void OnPress() override {
    if (stack()) stack()->Push(&g_main_menu);
  }

  // Long-press at the root cycles through playback layouts (instead of
  // the default View::OnLongPress, which pops the stack — a no-op at
  // depth 1 anyway).
  void OnLongPress() override { CycleLayout(); }

  // Exposed so the keyboard handler / Encoder & Inputs window can also
  // cycle layouts without going through the long-press path.
  void CycleLayout() {
    layout_ = static_cast<Layout>(
        (static_cast<uint8_t>(layout_) + 1u) %
        static_cast<uint8_t>(Layout::Count));
    hint_frames_remaining_ = 60;   // ~1 s at 60 fps
  }
  Layout layout() const { return layout_; }

 private:
  Layout      layout_                = Layout::Grid;
  mutable int hint_frames_remaining_ = 0;

  void DrawHeader(seq::FakeOled& oled) const {
    const int  step  = g_disp_step.load();
    const int  steps = g_steps_item ? g_steps_item->value() : 16;
    const bool run   = g_run_item  ? g_run_item->value()    : false;
    const char* scale = (g_scale_idx >= 0 && g_scale_idx < seq::kNumScales)
                        ? seq::kScales[g_scale_idx].name : "";
    char scale_short[8];
    int sl = 0;
    for (; scale[sl] && sl < 7; ++sl) scale_short[sl] = scale[sl];
    scale_short[sl] = 0;

    char now_note[8];
    DacToNoteName(g_disp_dac.load(), now_note, sizeof(now_note));

    // Play/Stop icon at the far left, then text.
    icons::Draw(oled, run ? icons::kPlay : icons::kStop, 1, 1);
    char header[24];
    std::snprintf(header, sizeof(header), " %-3s %-7s %02d/%02d",
                  now_note, scale_short, step + 1, steps);
    oled.Text(8, 1, header);

    // Live trig indicator on the right edge if the gate is high.
    if (g_disp_trig.load()) {
      icons::Draw(oled, icons::kTrig, 121, 1);
    }
  }

  void DrawLayoutHint(seq::FakeOled& oled) const {
    if (hint_frames_remaining_ <= 0) return;
    --hint_frames_remaining_;
    // Small "n/N" badge at top-right just after a layout switch.
    const int total = static_cast<int>(Layout::Count);
    const int idx   = static_cast<int>(layout_) + 1;
    char b[8];
    std::snprintf(b, sizeof(b), "%d/%d", idx, total);
    const int w = static_cast<int>(std::strlen(b)) * 6;
    oled.FillRect(128 - w - 2, 0, w + 2, 9, true);   // white badge
    oled.Text   (128 - w - 1, 1, b, false);
  }

  // ---- Layout 0: 8x2 grid, note letter + trig per cell ----
  void DrawGrid(seq::FakeOled& oled) const {
    const int  step  = g_disp_step.load();
    const int  steps = g_steps_item ? g_steps_item->value() : 16;

    constexpr int kCellW   = 16;
    constexpr int kCellH   = 26;
    constexpr int kGridTop = 11;

    for (int i = 0; i < seq::kMaxSteps; ++i) {
      const int col = i % 8;
      const int row = i / 8;
      const int x   = col * kCellW;
      const int y   = kGridTop + row * kCellH;
      const bool active = (i < steps);
      const bool trig   = g_disp_trig_grid[i] != 0;
      const bool is_cur = (i == step);

      if (!active) {
        oled.Px(x + kCellW / 2, y + kCellH / 2, true);
        continue;
      }
      if (is_cur) oled.FillRect(x, y, kCellW, kCellH, true);

      char note_full[8];
      DacToNoteName(g_disp_cv[i], note_full, sizeof(note_full));
      char note_letters[4] = {0};
      int  nlen = 0;
      for (int k = 0; note_full[k] && nlen < 3; ++k) {
        const char c = note_full[k];
        if ((c >= 'A' && c <= 'G') || c == '#') note_letters[nlen++] = c;
      }
      const int note_w = nlen * 6;
      const int note_x = x + (kCellW - note_w) / 2;
      oled.Text(note_x, y + 3, note_letters, !is_cur);

      // Trig icon below the note (centred). Filled icon if trig on,
      // dim dot if off.
      const int trig_x = x + (kCellW - 6) / 2;
      const int trig_y = y + 14;
      if (trig) {
        icons::Draw(oled, icons::kTrig, trig_x, trig_y, !is_cur);
      } else {
        // single centre dot — "step exists, trig off"
        oled.Px(trig_x + 3, trig_y + 4, !is_cur);
      }
    }
  }

  // ---- Layout 1: piano roll — pitch contour ----
  void DrawPianoRoll(seq::FakeOled& oled) const {
    const int  step  = g_disp_step.load();
    const int  steps = g_steps_item ? g_steps_item->value() : 16;
    if (steps <= 0) return;

    // Auto-scale Y to the actual pitch range in the sequence (otherwise
    // a tight chromatic cluster wastes most of the screen).
    uint16_t cv_min = 4095, cv_max = 0;
    for (int i = 0; i < steps; ++i) {
      if (g_disp_cv[i] < cv_min) cv_min = g_disp_cv[i];
      if (g_disp_cv[i] > cv_max) cv_max = g_disp_cv[i];
    }
    if (cv_max <= cv_min) cv_max = cv_min + 1;

    constexpr int kGraphTop = 12;
    constexpr int kGraphBot = 48;
    constexpr int kGraphH   = kGraphBot - kGraphTop;
    constexpr int kTrigY    = 52;
    constexpr int kTrigSize = 6;

    const int step_w = 128 / steps;
    auto x_for = [&](int i) { return i * step_w + step_w / 2; };
    auto y_for = [&](uint16_t cv) {
      return kGraphBot -
             static_cast<int>((static_cast<int32_t>(cv - cv_min) * kGraphH) /
                              (cv_max - cv_min));
    };

    // Current-step vertical playhead (dashed)
    const int cx = x_for(step);
    for (int y = kGraphTop; y < kTrigY + kTrigSize; y += 2) oled.Px(cx, y, true);

    // Pitch contour line — connect consecutive steps.
    for (int i = 0; i < steps; ++i) {
      const int x = x_for(i);
      const int y = y_for(g_disp_cv[i]);
      if (i > 0) {
        const int px = x_for(i - 1);
        const int py = y_for(g_disp_cv[i - 1]);
        oled.Line(px, py, x, y);
      }
      // Step dot (filled square at the point)
      oled.FillRect(x - 1, y - 1, 3, 3, true);
    }

    // Trig row beneath the graph
    for (int i = 0; i < steps; ++i) {
      const int x = i * step_w + (step_w - kTrigSize) / 2;
      const bool is_cur = (i == step);
      if (g_disp_trig_grid[i]) {
        oled.FillRect(x, kTrigY, kTrigSize, kTrigSize, true);
        if (is_cur) {
          // Punch out a dot so the current trig reads "selected".
          oled.Px(x + kTrigSize / 2, kTrigY + kTrigSize / 2, false);
        }
      } else {
        oled.Rect(x, kTrigY, kTrigSize, kTrigSize);
      }
    }

    // Min/Max note labels on the right edge so you know the range.
    char nmin[8], nmax[8];
    DacToNoteName(cv_min, nmin, sizeof(nmin));
    DacToNoteName(cv_max, nmax, sizeof(nmax));
    // (omitted — limited horizontal space; piano roll's strength is
    // the contour itself, and the header already shows the current note.)
    (void)nmin; (void)nmax;
  }
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

  // ---- Category submenus ----
  g_clock_items = {
    g_steps_item, g_clk_div_item, g_clk_mult_item, g_dig_in_item,
  };
  g_pitch_items = {
    g_scale_item, g_octaves_item, g_start_note_item, g_cv_source_item,
  };
  g_probs_items = {
    g_cv_prob_item, g_trig_prob_item, g_trig_length_item,
  };
  g_actions_items = {
    g_clear_cv_item, g_clear_trig_item,
  };
  g_clock_menu  .SetTitle("Clock");
  g_pitch_menu  .SetTitle("Pitch");
  g_probs_menu  .SetTitle("Probs");
  g_actions_menu.SetTitle("Actions");
  g_clock_menu  .SetItems(g_clock_items.data(),   static_cast<int>(g_clock_items.size()));
  g_pitch_menu  .SetItems(g_pitch_items.data(),   static_cast<int>(g_pitch_items.size()));
  g_probs_menu  .SetItems(g_probs_items.data(),   static_cast<int>(g_probs_items.size()));
  g_actions_menu.SetItems(g_actions_items.data(), static_cast<int>(g_actions_items.size()));
  // Same commit callback on every menu — any commit anywhere syncs all
  // params into the engine.
  g_clock_menu  .SetCommitCallback(&OnMenuCommit, nullptr);
  g_pitch_menu  .SetCommitCallback(&OnMenuCommit, nullptr);
  g_probs_menu  .SetCommitCallback(&OnMenuCommit, nullptr);
  g_actions_menu.SetCommitCallback(&OnMenuCommit, nullptr);

  // ---- Main menu (Run toggle + 4 category entries) ----
  g_clock_sub_item   = new seq::SubmenuItem("Clock",   &g_clock_menu);
  g_pitch_sub_item   = new seq::SubmenuItem("Pitch",   &g_pitch_menu);
  g_probs_sub_item   = new seq::SubmenuItem("Probs",   &g_probs_menu);
  g_actions_sub_item = new seq::SubmenuItem("Actions", &g_actions_menu);

  g_main_items = {
    g_run_item,
    g_clock_sub_item, g_pitch_sub_item, g_probs_sub_item, g_actions_sub_item,
  };
  g_main_menu.SetTitle("Main Menu");
  g_main_menu.SetItems(g_main_items.data(), static_cast<int>(g_main_items.size()));
  g_main_menu.SetCommitCallback(&OnMenuCommit, nullptr);

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
// ============================================================
//  Sim window — macOS-only conveniences (internal clock + audition)
// ============================================================
// Anything in here would NOT exist on real hardware. The OLED menu is
// the canonical edit surface for engine params; this window only hosts
// sim-side helpers so the user can drive the engine without an
// external clock source.
static void RenderSimWindow() {
  ImGui::Begin("Sim");

  ImGui::SeparatorText("Internal clock (sim only)");
  bool running = g_clock_running.load();
  if (ImGui::Checkbox("Clock running (S)", &running)) g_clock_running.store(running);
  int bpm = g_bpm.load();
  if (ImGui::SliderInt("BPM", &bpm, 20, 240)) g_bpm.store(bpm);

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

  ImGui::End();
}

// ============================================================
//  Inspector window — read-only engine state visualization
// ============================================================
// All state shown here also exists on the OLED in some form. The point
// is a bigger, more debuggable rendering for development. No writeable
// controls — change params via the OLED menu (or the Encoder & Inputs
// window's button mirror).
static void RenderInspectorWindow() {
  ImGui::Begin("Inspector");

  // ---- Status banner ----
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

  // ---- Engine state line ----
  {
    char note[8];
    DacToNoteName(g_disp_dac.load(), note, sizeof(note));
    ImGui::Text("Step %2d   Note %-4s   DAC %4u   TRIG %s",
                g_disp_step.load(),
                note,
                static_cast<unsigned>(g_disp_dac.load()),
                g_disp_trig.load() ? "ON" : "off");
  }
  ImGui::Text("Scale: %d notes   Views in stack: %d",
              g_scale_count, g_views.depth());

  // ---- Step grid ----
  ImGui::SeparatorText("Step grid");
  const int   step  = g_disp_step.load();
  const int   steps = g_steps_item ? g_steps_item->value() : 16;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImVec2 grid_p0 = ImGui::GetCursorScreenPos();
  const float cell  = 18.0f;
  const float gap   = 3.0f;
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
//  Input action helpers — shared by keyboard handler and the
//  "Encoder & Inputs" ImGui window's clickable buttons.
// ============================================================
static void DoEncoderRotate(int delta) {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_views.OnRotate(delta);
}
static void DoEncoderPress() {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_views.OnPress();
}
static void DoEncoderLongPress() {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_views.OnLongPress();
}
// Returns the MenuListView currently on top, or nullptr if the top view
// isn't a menu list (PlaybackView, an editor view, etc.). Used by the
// number-key / Encoder-window-button shortcuts which should only jump
// when we're actually inside a list.
static seq::MenuListView* TopMenuList() {
  seq::View* top = g_views.top();
  seq::MenuListView* candidates[] = {
    &g_main_menu, &g_clock_menu, &g_pitch_menu, &g_probs_menu, &g_actions_menu
  };
  for (auto* m : candidates) if (m == top) return m;
  return nullptr;
}
static void DoMenuJump(int index) {
  std::lock_guard<std::mutex> lk(g_mutex);
  if (auto* m = TopMenuList()) m->JumpTo(index);
}
static void DoCyclePlaybackLayout() {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_playback_view.CycleLayout();
}

// Direct navigation — the sim doesn't have to play the "encoder dance"
// to get into the menu. These helpers manipulate the view stack
// directly so a mouse-driven user can jump anywhere in one click.
static void DoGoHome() {
  std::lock_guard<std::mutex> lk(g_mutex);
  while (g_views.can_pop()) g_views.Pop();   // pop to PlaybackView (depth 1)
}
static void DoOpenMainMenu() {
  std::lock_guard<std::mutex> lk(g_mutex);
  while (g_views.can_pop()) g_views.Pop();
  g_views.Push(&g_main_menu);
}
static void DoOpenSubmenu(seq::MenuListView* sub) {
  std::lock_guard<std::mutex> lk(g_mutex);
  while (g_views.can_pop()) g_views.Pop();
  g_views.Push(&g_main_menu);
  g_views.Push(sub);
}
static void DoBack() {
  // Pop one level. Already what long-press does — wrapper for clarity.
  DoEncoderLongPress();
}

// ============================================================
//  Controls window — direct navigation + hardware-encoder mirror
// ============================================================
static void RenderEncoderInputsWindow() {
  ImGui::Begin("Controls");

  // ---- Direct navigation (sim-only — no hardware-encoder dance) ----
  ImGui::SeparatorText("Navigation");
  if (ImGui::Button("MENU (M)", ImVec2(120, 30))) DoOpenMainMenu();
  ImGui::SameLine();
  if (ImGui::Button("BACK",     ImVec2(80, 30)))  DoBack();
  ImGui::SameLine();
  if (ImGui::Button("HOME (H)", ImVec2(120, 30))) DoGoHome();

  ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "Jump straight to a category:");
  if (ImGui::Button("Clock",   ImVec2(80, 0))) DoOpenSubmenu(&g_clock_menu);
  ImGui::SameLine();
  if (ImGui::Button("Pitch",   ImVec2(80, 0))) DoOpenSubmenu(&g_pitch_menu);
  ImGui::SameLine();
  if (ImGui::Button("Probs",   ImVec2(80, 0))) DoOpenSubmenu(&g_probs_menu);
  ImGui::SameLine();
  if (ImGui::Button("Actions", ImVec2(80, 0))) DoOpenSubmenu(&g_actions_menu);

  // ---- Encoder (hardware mirror) ----
  ImGui::SeparatorText("Encoder (mirrors hardware)");
  if (ImGui::Button("<<  rotate left",  ImVec2(140, 0))) DoEncoderRotate(-1);
  ImGui::SameLine();
  if (ImGui::Button("CLICK",            ImVec2(80, 0)))  DoEncoderPress();
  ImGui::SameLine();
  if (ImGui::Button("rotate right  >>", ImVec2(140, 0))) DoEncoderRotate(+1);
  if (ImGui::Button("LONG PRESS (back / cancel)", ImVec2(0, 0))) DoEncoderLongPress();
  if (ImGui::Button("Cycle playback layout (V)", ImVec2(0, 0))) DoCyclePlaybackLayout();

  // ---- Jacks (mirror what's physically wired on the panel) ----
  ImGui::SeparatorText("Jacks");
  if (ImGui::Button("Manual clock pulse"))  g_manual_pulse_request.store(true);
  ImGui::SameLine();
  if (ImGui::Button("Digital-in pulse"))    g_digital_pulse_request.store(true);
  ImGui::SameLine();
  if (ImGui::Button("RESET"))               g_reset_request.store(true);

  // ---- Menu jumps (1..9) ----
  ImGui::SeparatorText("Menu shortcuts (only when a menu is on top)");
  const bool in_menu = (TopMenuList() != nullptr);
  if (!in_menu) ImGui::BeginDisabled();
  for (int i = 1; i <= 9; ++i) {
    char lbl[3] = { static_cast<char>('0' + i), 0, 0 };
    if (i > 1) ImGui::SameLine();
    if (ImGui::Button(lbl, ImVec2(28, 0))) DoMenuJump(i - 1);
  }
  if (!in_menu) ImGui::EndDisabled();

  // ---- Keyboard reference ----
  ImGui::SeparatorText("Keyboard reference");
  ImGui::BeginTable("keys", 2, ImGuiTableFlags_SizingFixedFit);
  auto row = [](const char* k, const char* v) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn(); ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "%s", k);
    ImGui::TableNextColumn(); ImGui::Text("%s", v);
  };
  row("Left / Right", "Encoder rotate");
  row("Space",        "Encoder click");
  row("Backspace",    "Long-press / cancel / back");
  row("G",            "Manual clock pulse");
  row("D",            "Digital-in pulse");
  row("S",            "Toggle internal clock");
  row("R",            "Reset transport");
  row("V",            "Cycle playback layout (Grid / Piano Roll)");
  row("M",            "Open main menu");
  row("H",            "Home — return to playback view");
  row("1 .. 9",       "Jump to menu item N");
  row("Esc / Q",      "Quit");
  ImGui::EndTable();

  ImGui::End();
}

// ============================================================
//  Keyboard
// ============================================================
static void OnKey(GLFWwindow* win, int key, int /*sc*/, int action, int /*mods*/) {
  if (action != GLFW_PRESS && action != GLFW_REPEAT) return;
  switch (key) {
    case GLFW_KEY_LEFT:  DoEncoderRotate(-1); break;
    case GLFW_KEY_RIGHT: DoEncoderRotate(+1); break;
    case GLFW_KEY_SPACE:
      if (action == GLFW_PRESS) DoEncoderPress();
      break;
    case GLFW_KEY_BACKSPACE:
      if (action == GLFW_PRESS) DoEncoderLongPress();
      break;
    // Number keys 1-9: jump directly to that submenu, but only when the
    // root list is on top (avoids weird jumps from inside an editor).
    case GLFW_KEY_1: case GLFW_KEY_2: case GLFW_KEY_3:
    case GLFW_KEY_4: case GLFW_KEY_5: case GLFW_KEY_6:
    case GLFW_KEY_7: case GLFW_KEY_8: case GLFW_KEY_9:
      if (action == GLFW_PRESS) DoMenuJump(key - GLFW_KEY_1);
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
    case GLFW_KEY_V:
      if (action == GLFW_PRESS) DoCyclePlaybackLayout();
      break;
    case GLFW_KEY_M:
      if (action == GLFW_PRESS) DoOpenMainMenu();
      break;
    case GLFW_KEY_H:
      if (action == GLFW_PRESS) DoGoHome();
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
    RenderEncoderInputsWindow();
    RenderSimWindow();
    RenderInspectorWindow();
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
