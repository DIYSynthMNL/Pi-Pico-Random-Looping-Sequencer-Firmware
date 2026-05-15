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

// StepDirection labels — index = enum value (matches seq::StepDirection).
constexpr const char*  kStepDirNames[]    = { "Fwd", "Rev", "Pend", "Rand" };
constexpr int          kStepDirCount      = 4;

// Step rate options — how long each step is relative to the incoming
// clock. Replaces the v0.8 "ClkDiv" / "ClkMult" pair. Internally maps
// to (divider, multiplier) — the engine still uses those fields but
// users never see them as separate numbers.
constexpr const char*  kStepRateNames[]   = {
  "/8", "/4", "/2", "x1", "x2", "x3", "x4", "x6", "x8"
};
constexpr int          kStepRateCount     = 9;
constexpr int          kStepRateDefault   = 3;   // "x1"
struct StepRateMapping { int div; int mult; };
constexpr StepRateMapping kStepRateMap[kStepRateCount] = {
  {8, 1}, {4, 1}, {2, 1},        // /8 /4 /2
  {1, 1},                         // x1 (default)
  {1, 2}, {1, 3}, {1, 4},        // x2 x3 x4
  {1, 6}, {1, 8},                // x6 x8 (triplet 16 / 32nd)
};

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
// Replaced ClkDiv + ClkMult with a single Step rate, rendered as a
// 3x3 grid on the OLED.
seq::GridSelectItem*      g_step_rate_item    = nullptr;
seq::ActionItem*          g_clear_cv_item     = nullptr;  // audit 2a
seq::ActionItem*          g_clear_trig_item   = nullptr;  // audit 2a
seq::ToggleItem*          g_run_item          = nullptr;  // audit 2f
seq::GridSelectItem*      g_cv_source_item    = nullptr;
seq::GridSelectItem*      g_dig_in_item       = nullptr;  // audit 2e
seq::GridSelectItem*      g_step_dir_item     = nullptr;

// Stateless — one shared instance for every submenu's "< Back" row.
seq::BackItem             g_back_item;

// Item arrays per menu (must outlive the MenuListViews).
std::vector<seq::MenuItem*> g_clock_items;
std::vector<seq::MenuItem*> g_pitch_items;
std::vector<seq::MenuItem*> g_probs_items;
std::vector<seq::MenuItem*> g_actions_items;

// Sim-only: fire a single digital-in rising/falling edge on the
// Sequencer when the user presses D. Matches the manual-pulse pattern.
std::atomic<bool>      g_digital_pulse_request{false};

// Per-window visibility — toggled from the View menu at the top.
bool g_show_oled       = true;
bool g_show_hardware   = true;
bool g_show_quicknav   = true;
bool g_show_params     = true;
bool g_show_sim        = true;
bool g_show_inspector  = true;

// Tap-tempo feedback timestamp. NowMs() < g_tap_flash_until_ms_ means
// the BPM slider in the Sim window should flash. Written by DoTapTempo,
// read by RenderSimWindow.
std::atomic<uint32_t> g_tap_flash_until_ms_{0};
std::atomic<uint32_t> g_last_tap_ms_{0};

}  // namespace

// ============================================================
//  Forward decls for input helpers used by the Sim window before
//  their definitions appear below.
// ============================================================
static void DoCyclePlaybackLayout();
static void DoTogglePlay();
static void DoTapTempo();

// MainMenuView is defined later (it needs Play toggle and submenu
// list views which exist in this file lower down). PlaybackView's
// OnPress wants to push the main view — go through this trampoline.
static seq::View* MainView();

// Milliseconds since the steady-clock epoch. Used by views that want
// to animate (step pulse halo, trig flash, etc.).
static uint32_t NowMs() {
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}
// MainMenuView::OnPress toggles g_run_item directly (it's already
// under the mutex from DoEncoderPress) and then fires OnMenuCommit
// to sync into engine params — declared below, used above.
static void OnMenuCommit(void*);

// Persistence helpers — defined further down so they can reference
// PlaybackView / Voice / etc., but forward-declared here because
// OnMenuCommit calls SaveStateLocked.
static void SaveStateLocked();
static void LoadStateLocked();

// Suppressed during BuildMenu's startup sequence so the first
// OnMenuCommit doesn't write a half-initialised state to disk.
static bool g_state_loading = false;


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
    if (stack()) stack()->Push(MainView());
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
  // Animation state — driven by NowMs() comparisons in Draw.
  mutable int      last_step_seen_       = -1;
  mutable uint32_t step_pulse_started_ms_ = 0;
  mutable bool     last_trig_seen_        = false;
  mutable uint32_t trig_pulse_started_ms_ = 0;

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
    // On the rising edge (transition off → on) we kick off a brief
    // outline flash around the icon so a fast gate is still visible.
    const bool trig = g_disp_trig.load();
    if (trig && !last_trig_seen_) {
      trig_pulse_started_ms_ = NowMs();
    }
    last_trig_seen_ = trig;
    if (trig) {
      icons::Draw(oled, icons::kTrig, 121, 1);
      const uint32_t age = NowMs() - trig_pulse_started_ms_;
      if (age < 80) {
        // Outline 1 px outside the icon, fades after ~80 ms.
        oled.Rect(119, -1, 9, 11);
      }
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

    // Detect step advance — used for the pulse halo below.
    if (step != last_step_seen_) {
      step_pulse_started_ms_ = NowMs();
      last_step_seen_ = step;
    }
    const uint32_t pulse_age = NowMs() - step_pulse_started_ms_;

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
      if (is_cur) {
        oled.FillRect(x, y, kCellW, kCellH, true);
        // Pulse halo — expanding outline that fades in 150 ms after a
        // step change. Gives the playhead an "alive" feel.
        if (pulse_age < 150) {
          const int outset = (pulse_age < 50) ? 2 :
                             (pulse_age < 100) ? 1 : 0;
          if (outset > 0) {
            oled.Rect(x - outset, y - outset,
                      kCellW + outset * 2, kCellH + outset * 2);
          }
        }
      }

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

    // Tiny min/max note labels in the right margin so you can read
    // the y-axis range of the contour. Drawn over the rightmost step
    // — at 16 steps × 8 px each = 128 px there's no clean margin, so
    // we paint a thin dark strip first.
    char nmin[8], nmax[8];
    DacToNoteName(cv_min, nmin, sizeof(nmin));
    DacToNoteName(cv_max, nmax, sizeof(nmax));
    const int min_w = static_cast<int>(std::strlen(nmin)) * 6;
    const int max_w = static_cast<int>(std::strlen(nmax)) * 6;
    // Top-right = high range
    oled.FillRect(128 - max_w - 1, kGraphTop, max_w + 1, 8, false);
    oled.Text   (128 - max_w,     kGraphTop, nmax, true);
    // Bottom-right = low range, just above the trig row
    oled.FillRect(128 - min_w - 1, kGraphBot - 8, min_w + 1, 8, false);
    oled.Text   (128 - min_w,     kGraphBot - 8, nmin, true);
  }
};

PlaybackView g_playback_view;
// ============================================================
//  Persistence — save/reload sim state across launches
// ============================================================
// All edits (menu commits, Sim window slider changes, layout toggles)
// are saved to ./playground.state. Atomic write via .tmp + rename.
// On launch, LoadState restores values into the menu items + sim
// atomics + the live cv/trig sequences. If the file is missing or a
// key is unknown, the corresponding default applies.
//
// Both Save and Load assume the caller holds g_mutex.

static constexpr const char* kStatePath    = "./playground.state";
static constexpr const char* kStateTmpPath = "./playground.state.tmp";

static void SaveStateLocked() {
  if (g_state_loading) return;   // skip during BuildMenu's setup
  std::FILE* f = std::fopen(kStateTmpPath, "w");
  if (!f) return;
  std::fprintf(f, "# seq playground state — auto-saved\n");
  // Menu items
  std::fprintf(f, "play=%d\n",            g_run_item        ? (g_run_item->value() ? 1 : 0) : 0);
  std::fprintf(f, "scale_idx=%d\n",       g_scale_item      ? g_scale_item->selected_index() : 0);
  std::fprintf(f, "octaves=%d\n",         g_octaves_item    ? g_octaves_item->value() : 1);
  std::fprintf(f, "start_note=%d\n",      g_start_note_item ? g_start_note_item->value() : 0);
  std::fprintf(f, "steps=%d\n",           g_steps_item      ? g_steps_item->value() : 16);
  std::fprintf(f, "step_rate_idx=%d\n",   g_step_rate_item  ? g_step_rate_item->selected_index() : kStepRateDefault);
  std::fprintf(f, "cv_change_pct=%d\n",   g_cv_prob_item    ? g_cv_prob_item->value() : 0);
  std::fprintf(f, "trig_change_pct=%d\n", g_trig_prob_item  ? g_trig_prob_item->value() : 0);
  std::fprintf(f, "trig_length_pct=%d\n", g_trig_length_item? g_trig_length_item->value() : 50);
  std::fprintf(f, "cv_source_idx=%d\n",   g_cv_source_item  ? g_cv_source_item->selected_index() : 0);
  std::fprintf(f, "dig_in_idx=%d\n",      g_dig_in_item     ? g_dig_in_item->selected_index() : 1);
  std::fprintf(f, "step_dir_idx=%d\n",    g_step_dir_item   ? g_step_dir_item->selected_index() : 0);
  // Sim
  std::fprintf(f, "clock_running=%d\n",        g_clock_running.load() ? 1 : 0);
  std::fprintf(f, "bpm=%d\n",                  g_bpm.load());
  std::fprintf(f, "audition_enabled=%d\n",     g_audition_enabled.load() ? 1 : 0);
  std::fprintf(f, "audition_volume=%f\n",      g_audition_volume.load());
  std::fprintf(f, "audition_pitch_semis=%f\n", g_audition_pitch_semis.load());
  std::fprintf(f, "audition_attack_ms=%f\n",   g_audition_attack_ms.load());
  std::fprintf(f, "audition_release_ms=%f\n",  g_audition_release_ms.load());
  std::fprintf(f, "playback_layout=%d\n",
               static_cast<int>(g_playback_view.layout()));

  // Live sequences (the user's mutating pattern)
  const seq::Voice& v = g_sequencer.voice(0);
  const uint16_t* cv = v.cv_sequence();
  const uint8_t*  tr = v.trigger_sequence();
  std::fprintf(f, "cv_sequence=");
  for (int i = 0; i < seq::kMaxSteps; ++i)
    std::fprintf(f, "%s%u", i ? "," : "", static_cast<unsigned>(cv[i]));
  std::fprintf(f, "\n");
  std::fprintf(f, "trigger_sequence=");
  for (int i = 0; i < seq::kMaxSteps; ++i)
    std::fprintf(f, "%s%u", i ? "," : "", static_cast<unsigned>(tr[i]));
  std::fprintf(f, "\n");

  std::fclose(f);
  std::rename(kStateTmpPath, kStatePath);
}

static void LoadStateLocked() {
  std::FILE* f = std::fopen(kStatePath, "r");
  if (!f) return;
  char line[2048];
  uint16_t cv_buf[seq::kMaxSteps]  = {0};
  uint8_t  tr_buf[seq::kMaxSteps]  = {0};
  bool     have_cv = false, have_tr = false;
  while (std::fgets(line, sizeof(line), f)) {
    if (line[0] == '#' || line[0] == '\n' || line[0] == 0) continue;
    char* eq = std::strchr(line, '=');
    if (!eq) continue;
    *eq = 0;
    const char* key = line;
    char* val = eq + 1;
    char* nl = std::strchr(val, '\n');
    if (nl) *nl = 0;

    auto i = [&]() { return std::atoi(val); };
    auto fl = [&]() { return static_cast<float>(std::atof(val)); };

    if      (!std::strcmp(key, "play"))            { if (g_run_item)        g_run_item->set_value(i() != 0); }
    else if (!std::strcmp(key, "scale_idx"))       { if (g_scale_item)      g_scale_item->set_selected_index(i()); }
    else if (!std::strcmp(key, "octaves"))         { if (g_octaves_item)    g_octaves_item->set_value(i()); }
    else if (!std::strcmp(key, "start_note"))      { if (g_start_note_item) g_start_note_item->set_value(i()); }
    else if (!std::strcmp(key, "steps"))           { if (g_steps_item)      g_steps_item->set_value(i()); }
    else if (!std::strcmp(key, "step_rate_idx"))   { if (g_step_rate_item)  g_step_rate_item->set_selected_index(i()); }
    else if (!std::strcmp(key, "cv_change_pct"))   { if (g_cv_prob_item)    g_cv_prob_item->set_value(i()); }
    else if (!std::strcmp(key, "trig_change_pct")) { if (g_trig_prob_item)  g_trig_prob_item->set_value(i()); }
    else if (!std::strcmp(key, "trig_length_pct")) { if (g_trig_length_item)g_trig_length_item->set_value(i()); }
    else if (!std::strcmp(key, "cv_source_idx"))   { if (g_cv_source_item)  g_cv_source_item->set_selected_index(i()); }
    else if (!std::strcmp(key, "dig_in_idx"))      { if (g_dig_in_item)     g_dig_in_item->set_selected_index(i()); }
    else if (!std::strcmp(key, "step_dir_idx"))    { if (g_step_dir_item)   g_step_dir_item->set_selected_index(i()); }
    else if (!std::strcmp(key, "clock_running"))   { g_clock_running.store(i() != 0); }
    else if (!std::strcmp(key, "bpm"))             { g_bpm.store(i()); }
    else if (!std::strcmp(key, "audition_enabled")) { g_audition_enabled.store(i() != 0); }
    else if (!std::strcmp(key, "audition_volume"))      { g_audition_volume.store(fl()); }
    else if (!std::strcmp(key, "audition_pitch_semis")) { g_audition_pitch_semis.store(fl()); }
    else if (!std::strcmp(key, "audition_attack_ms"))   { g_audition_attack_ms.store(fl()); }
    else if (!std::strcmp(key, "audition_release_ms"))  { g_audition_release_ms.store(fl()); }
    else if (!std::strcmp(key, "playback_layout"))  {
      // We don't have a public setter; cycle the layout until it matches
      const int target = i();
      for (int n = 0; n < 8 && static_cast<int>(g_playback_view.layout()) != target; ++n) {
        g_playback_view.CycleLayout();
      }
    }
    else if (!std::strcmp(key, "cv_sequence")) {
      int n = 0;
      for (char* tok = std::strtok(val, ","); tok && n < seq::kMaxSteps;
           tok = std::strtok(nullptr, ",")) {
        cv_buf[n++] = static_cast<uint16_t>(std::atoi(tok));
      }
      if (n == seq::kMaxSteps) have_cv = true;
    }
    else if (!std::strcmp(key, "trigger_sequence")) {
      int n = 0;
      for (char* tok = std::strtok(val, ","); tok && n < seq::kMaxSteps;
           tok = std::strtok(nullptr, ",")) {
        tr_buf[n++] = static_cast<uint8_t>(std::atoi(tok) != 0);
      }
      if (n == seq::kMaxSteps) have_tr = true;
    }
  }
  std::fclose(f);
  if (have_cv || have_tr) {
    g_sequencer.voice(0).SetSequences(have_cv ? cv_buf : nullptr,
                                      have_tr ? tr_buf : nullptr);
  }
}

// ============================================================
//  MainMenuView — the OLED "home menu" rendered graphically
// ============================================================
// Sits between PlaybackView (depth 1) and the per-category MenuListViews
// (depth 3). Layout:
//
//   y= 0..21 : big PLAY/STOP banner spanning full width
//   y=24..63 : 2x2 grid of category tiles (Clock, Pitch, Probs, Actions)
//
// highlighted_ ∈ [0..4] where 0 = Play banner, 1..4 = grid tiles row-
// major. Encoder rotation cycles; click activates.
class MainMenuView : public seq::View {
 public:
  void Draw(seq::FakeOled& oled) const override {
    oled.Clear();

    const bool playing = g_run_item ? g_run_item->value() : false;
    DrawBanner(oled, playing, highlighted_ == 0);
    DrawTiles (oled);
  }

  void OnRotate(int delta) override {
    highlighted_ += delta;
    if (highlighted_ < 0) highlighted_ = 0;
    if (highlighted_ > 4) highlighted_ = 4;
  }

  void OnPress() override {
    // NOTE: OnPress runs *under* g_mutex (acquired by DoEncoderPress).
    // We must not call DoTogglePlay() or DoOpenSubmenu() here — they
    // try to re-lock the same non-recursive mutex on the same thread,
    // which is UB on std::mutex and locks up the app on macOS.
    if (highlighted_ == 0) {
      if (g_run_item) {
        g_run_item->set_value(!g_run_item->value());
        OnMenuCommit(nullptr);
      }
      return;
    }
    seq::MenuListView* targets[4] = {
      &g_clock_menu, &g_pitch_menu, &g_probs_menu, &g_actions_menu
    };
    if (stack()) stack()->Push(targets[highlighted_ - 1]);
  }

  // 1..9 jump for number keys (1=play, 2-5 = category tiles).
  void JumpTo(int idx) {
    if (idx < 0 || idx > 4) return;
    highlighted_ = idx;
  }

 private:
  static constexpr int kBannerH    = 22;
  static constexpr int kTileTop    = 24;
  static constexpr int kTileH      = 20;
  static constexpr int kTileW      = 64;

  static void DrawBanner(seq::FakeOled& oled, bool playing, bool highlighted) {
    if (highlighted) {
      oled.FillRect(0, 0, 128, kBannerH, true);
    } else {
      oled.Rect(0, 0, 128, kBannerH);
    }
    const bool ink_on = !highlighted;
    icons::Draw(oled, playing ? icons::kStop : icons::kPlay, 8, 7, ink_on);
    const char* label = playing ? "STOP" : "PLAY";
    const int label_w = static_cast<int>(std::strlen(label)) * 6;
    oled.Text((128 - label_w) / 2, 7, label, ink_on);
  }

  void DrawTiles(seq::FakeOled& oled) const {
    static const char* kNames[4] = { "CLOCK", "PITCH", "PROBS", "ACTIONS" };
    const icons::Icon6x8* tile_icons[4] = {
      &icons::kReset,   // Clock — circle (closest to clock face)
      &icons::kNote,    // Pitch — music note
      &icons::kDice,    // Probs — dice
      &icons::kTrig,    // Actions — pulse / exclamation flavour
    };
    // Breathing halo phase (single shared phase so all selected tiles
    // pulse in unison — looks more intentional than per-tile timing).
    const uint32_t phase     = NowMs() % 1500;
    const int      halo_out  = (phase < 200) ? 1 : (phase < 400) ? 2 : 0;
    for (int i = 0; i < 4; ++i) {
      const int col = i % 2;
      const int row = i / 2;
      const int x   = col * kTileW;
      const int y   = kTileTop + row * kTileH;
      const bool sel = (highlighted_ == i + 1);
      if (sel) {
        oled.FillRect(x, y, kTileW, kTileH, true);
        // Outer breathing halo — pulses every 1.5 s.
        if (halo_out > 0) {
          oled.Rect(x - halo_out, y - halo_out,
                    kTileW + halo_out * 2, kTileH + halo_out * 2);
        }
      } else {
        oled.Rect(x, y, kTileW, kTileH);
      }
      const bool ink_on = !sel;
      // Icon on the left side of the tile, text to its right.
      const int icon_x = x + 6;
      const int icon_y = y + (kTileH - 8) / 2;
      icons::Draw(oled, *tile_icons[i], icon_x, icon_y, ink_on);
      const char* name = kNames[i];
      const int name_w = static_cast<int>(std::strlen(name)) * 6;
      // Right of the icon, then visually centred in the remaining space.
      const int region_start = icon_x + 8;
      const int region_w     = (x + kTileW) - region_start;
      const int tx = region_start + (region_w - name_w) / 2;
      const int ty = y + (kTileH - 8) / 2;
      oled.Text(tx, ty, name, ink_on);
    }
  }

  int highlighted_ = 0;
};

MainMenuView g_main_view;
static seq::View* MainView() { return &g_main_view; }

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
  {
    const int rate_idx = g_step_rate_item->selected_index();
    const auto& m = kStepRateMap[
        (rate_idx >= 0 && rate_idx < kStepRateCount) ? rate_idx : kStepRateDefault];
    g_vparams.clock_divider    = m.div;
    g_vparams.clock_multiplier = m.mult;
  }
  g_vparams.cv_source          = static_cast<seq::CvSource>(
      g_cv_source_item->selected_index());
  g_vparams.step_direction     = static_cast<seq::StepDirection>(
      g_step_dir_item->selected_index());

  g_sparams.enabled            = g_run_item->value();
  g_sparams.digital_in_mode    = static_cast<seq::DigitalInMode>(
      g_dig_in_item->selected_index());

  g_sequencer.voice(0).SetParams(g_vparams);
  RebuildScale_locked();   // also re-applies g_sparams via Sequencer::SetParams
  SaveStateLocked();        // persist after every menu commit
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

  g_run_item          = new seq::ToggleItem("Play", true);
  g_scale_item        = new seq::SingleSelectItem(
      "Scale", g_scale_names.data(), seq::kNumScales, g_scale_idx);
  // Wire the scale section info — Scales.h provides ScaleSectionForIndex.
  g_scale_item->SetSectionForIndex(&seq::ScaleSectionForIndex);
  // Wire a per-scale preview that paints the 12 semitones of the scale
  // as a tiny piano-roll bar — lit cell means "the scale contains this
  // semitone." Helps the user "see" the scale's shape while browsing.
  g_scale_item->SetPreviewFn([](int idx, seq::FakeOled& oled,
                                int x, int y, int w, int h) {
    if (idx < 0 || idx >= seq::kNumScales) return;
    const seq::ScaleInterval& s = seq::kScales[idx];
    bool in_scale[12] = {true};   // root is always present
    int note = 0;
    for (int i = 0; i < s.step_count; ++i) {
      note = (note + s.steps[i]) % 12;
      if (note >= 0 && note < 12) in_scale[note] = true;
    }
    // 12 columns spread evenly across the rect.
    const int cell_w = w / 12;
    for (int i = 0; i < 12; ++i) {
      const int cx = x + i * cell_w;
      if (in_scale[i]) {
        oled.FillRect(cx + 1, y + 1, cell_w - 2, h - 2);
      } else {
        // single dim dot — slot exists but is not in scale
        oled.Px(cx + cell_w / 2, y + h / 2, true);
      }
    }
  });
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
  g_step_rate_item    = new seq::GridSelectItem(
      "Step rate", kStepRateNames, kStepRateCount, kStepRateDefault,
      /*cols=*/3);
  g_clear_cv_item     = new seq::ActionItem(
      "Clear CV",   &ActionClearCv,       nullptr,
      /*confirm=*/true, "Clear all CVs?");
  g_clear_trig_item   = new seq::ActionItem(
      "Clear Trig", &ActionClearTriggers, nullptr,
      /*confirm=*/true, "Clear all trigs?");
  g_cv_source_item    = new seq::GridSelectItem(
      "CV Source", kCvSourceNames, kCvSourceCount, 0,
      /*cols=*/3);
  g_dig_in_item       = new seq::GridSelectItem(
      "DigIn", kDigInModeNames, kDigInModeCount,
      static_cast<int>(seq::DigitalInMode::Reset),
      /*cols=*/2);
  g_step_dir_item     = new seq::GridSelectItem(
      "StepDir", kStepDirNames, kStepDirCount,
      static_cast<int>(seq::StepDirection::Forward),
      /*cols=*/2);

  // ---- Category submenus ----
  // Each submenu opens with a "< Back" row so the navigation is
  // discoverable (long-press still works as the shortcut).
  g_clock_items = {
    &g_back_item, g_steps_item, g_step_rate_item, g_step_dir_item, g_dig_in_item,
  };
  g_pitch_items = {
    &g_back_item, g_scale_item, g_octaves_item, g_start_note_item, g_cv_source_item,
  };
  g_probs_items = {
    &g_back_item, g_cv_prob_item, g_trig_prob_item, g_trig_length_item,
  };
  g_actions_items = {
    &g_back_item, g_clear_cv_item, g_clear_trig_item,
  };
  // Breadcrumb titles so the user knows where they are in the hierarchy.
  g_clock_menu  .SetTitle("Main > Clock");
  g_pitch_menu  .SetTitle("Main > Pitch");
  g_probs_menu  .SetTitle("Main > Probs");
  g_actions_menu.SetTitle("Main > Actions");
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

  // Main menu doesn't use a MenuListView any more — it's rendered as
  // a graphical MainMenuView (defined below, with a PLAY banner + 2x2
  // category tiles). The MainMenuView pushes the per-category
  // MenuListViews on tile press.

  // PlaybackView is the bottom of the view stack — the OLED "home"
  // screen. User clicks the encoder to push the menu on top of it,
  // long-presses inside the menu to pop back here.
  g_views.Push(&g_playback_view);

  // BuildMenu does a few rounds of sync that we don't want to persist:
  // the first OnMenuCommit runs before engine.Init populates voice
  // sequences, and LoadStateLocked may overwrite values. Suppress
  // SaveStateLocked while this dance happens; turn saves back on at
  // the end so the next user edit / periodic tick captures everything.
  g_state_loading = true;
  OnMenuCommit(nullptr);
  g_sequencer.Init();
  // Restore any persisted state from a previous session.
  LoadStateLocked();
  // Re-commit so the engine sees the loaded menu values.
  OnMenuCommit(nullptr);
  g_state_loading = false;
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
  ImGui::SetNextWindowPos (ImVec2(10, 30),  ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(440, 350), ImGuiCond_FirstUseEver);
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
  ImGui::SetNextWindowPos (ImVec2(470, 30),  ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(400, 350), ImGuiCond_FirstUseEver);
  ImGui::Begin("Sim");

  ImGui::SeparatorText("Internal clock generator");
  bool running = g_clock_running.load();
  if (ImGui::Checkbox("Generator running (S)", &running)) g_clock_running.store(running);
  int bpm = g_bpm.load();
  // Flash the BPM slider green for ~250 ms when DoTapTempo accepts a
  // new tempo — gives an unmistakeable visual confirmation that the
  // tap registered.
  const bool tap_flash = NowMs() < g_tap_flash_until_ms_.load();
  if (tap_flash) {
    ImGui::PushStyleColor(ImGuiCol_FrameBg,        IM_COL32(60, 150, 80, 255));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, IM_COL32(70, 170, 95, 255));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive,  IM_COL32(50, 130, 70, 255));
  }
  if (ImGui::SliderInt("BPM", &bpm, 20, 240)) g_bpm.store(bpm);
  if (tap_flash) ImGui::PopStyleColor(3);
  if (ImGui::Button("Tap tempo (T)", ImVec2(160, 0))) DoTapTempo();
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                     "tap 2+ times to set BPM from interval");

  ImGui::SeparatorText("Audition — hear the engine");
  bool aud = g_audition_enabled.load();
  if (ImGui::Checkbox("Audio on (A)", &aud)) g_audition_enabled.store(aud);
  float vol = g_audition_volume.load();
  if (ImGui::SliderFloat("Volume",        &vol,   0.0f, 1.0f,    "%.2f")) g_audition_volume.store(vol);
  float pitch = g_audition_pitch_semis.load();
  if (ImGui::SliderFloat("Pitch (semis)", &pitch, -48.0f, 48.0f, "%+.1f")) g_audition_pitch_semis.store(pitch);
  ImGui::SameLine();
  if (ImGui::Button("0##pitch_reset")) g_audition_pitch_semis.store(0.0f);
  float atk = g_audition_attack_ms.load();
  if (ImGui::SliderFloat("Attack ms",     &atk,   1.0f,  500.0f,  "%.0f")) g_audition_attack_ms.store(atk);
  float rel = g_audition_release_ms.load();
  if (ImGui::SliderFloat("Release ms",    &rel,  10.0f, 3000.0f,  "%.0f")) g_audition_release_ms.store(rel);

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
  ImGui::SetNextWindowPos (ImVec2(880, 30),  ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(510, 510), ImGuiCond_FirstUseEver);
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
    &g_clock_menu, &g_pitch_menu, &g_probs_menu, &g_actions_menu
  };
  for (auto* m : candidates) if (m == top) return m;
  return nullptr;
}
static void DoMenuJump(int index) {
  std::lock_guard<std::mutex> lk(g_mutex);
  // Number keys jump within whichever menu surface is on top.
  if (auto* m = TopMenuList()) {
    m->JumpTo(index);
    return;
  }
  if (g_views.top() == &g_main_view) {
    g_main_view.JumpTo(index);
  }
}
static void DoCyclePlaybackLayout() {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_playback_view.CycleLayout();
}

// Transport toggle — flips engine "Play" param via the menu item, which
// also fires OnMenuCommit so the engine and all other surfaces see the
// change. Bound to P key + big PLAY/STOP banner button.
static void DoTogglePlay() {
  std::lock_guard<std::mutex> lk(g_mutex);
  if (g_run_item) {
    g_run_item->set_value(!g_run_item->value());
    OnMenuCommit(nullptr);
  }
}

// Tap-tempo (sim only — drives the internal-clock BPM). On each tap, if
// the interval since the previous tap is in a musical range (200..3000 ms
// → 20..300 BPM), set g_bpm. First tap of a session just records the
// timestamp. On a successful tap, g_tap_flash_until_ms_ kicks the BPM
// slider into a brief green flash so the user gets visible feedback.
static void DoTapTempo() {
  const auto now_ms = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
  const uint32_t last = g_last_tap_ms_.exchange(now_ms);
  if (last == 0) return;
  const uint32_t interval = now_ms - last;
  if (interval < 200 || interval > 3000) return;     // outside musical range
  const int new_bpm = 60000 / static_cast<int>(interval);
  if (new_bpm >= 20 && new_bpm <= 240) {
    g_bpm.store(new_bpm);
    g_tap_flash_until_ms_.store(now_ms + 250);
  }
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
  g_views.Push(&g_main_view);
}
static void DoOpenSubmenu(seq::MenuListView* sub) {
  std::lock_guard<std::mutex> lk(g_mutex);
  while (g_views.can_pop()) g_views.Pop();
  g_views.Push(&g_main_view);
  g_views.Push(sub);
}
static void DoBack() {
  // Pop one level. Already what long-press does — wrapper for clarity.
  DoEncoderLongPress();
}

// ============================================================
//  Hardware window — buttons that mirror real-panel controls
// ============================================================
// This is the "would exist on the eventual Eurorack module" surface:
// encoder rotate/click/long-press, manual clock-in, digital-in,
// reset jack. Use this view to feel out the hardware UX. Anything
// here will match the physical module.
static void RenderHardwareWindow() {
  ImGui::SetNextWindowPos (ImVec2(470, 390), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(400, 350), ImGuiCond_FirstUseEver);
  ImGui::Begin("Hardware (mirror)");

  ImGui::SeparatorText("Encoder");
  if (ImGui::Button("Rotate -",     ImVec2(100, 0))) DoEncoderRotate(-1);
  ImGui::SameLine();
  if (ImGui::Button("Click",        ImVec2(100, 0))) DoEncoderPress();
  ImGui::SameLine();
  if (ImGui::Button("Rotate +",     ImVec2(100, 0))) DoEncoderRotate(+1);
  if (ImGui::Button("Long press (back / cancel)", ImVec2(-FLT_MIN, 0))) DoEncoderLongPress();

  ImGui::SeparatorText("Jacks");
  if (ImGui::Button("Manual clock (G)", ImVec2(140, 0))) g_manual_pulse_request.store(true);
  ImGui::SameLine();
  if (ImGui::Button("Digital in (D)",   ImVec2(140, 0))) g_digital_pulse_request.store(true);
  if (ImGui::Button("Reset (R)",        ImVec2(140, 0))) g_reset_request.store(true);

  ImGui::SeparatorText("Hardware keyboard reference");
  ImGui::BeginTable("hwkeys", 2, ImGuiTableFlags_SizingFixedFit);
  auto row = [](const char* k, const char* v) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn(); ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "%s", k);
    ImGui::TableNextColumn(); ImGui::Text("%s", v);
  };
  row("Left / Right", "Encoder rotate");
  row("Space",        "Encoder click");
  row("Backspace",    "Long-press / back / cancel");
  row("G",            "Manual clock pulse");
  row("D",            "Digital-in pulse");
  row("R",            "Reset transport");
  ImGui::EndTable();

  ImGui::End();
}

// ============================================================
//  Params window — sim-only direct edit of every engine param
// ============================================================
// Bidirectionally synced with the OLED menu: change a slider here and
// the menu reflects it next frame; commit a menu edit and the sliders
// update. Lives in its own window because on hardware there are no
// sliders — the encoder + menu is the only path. Sim users can use
// whichever they prefer.
static void RenderParamsWindow() {
  ImGui::SetNextWindowPos (ImVec2(10, 390),  ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(440, 510), ImGuiCond_FirstUseEver);
  ImGui::Begin("Params (sim direct edit)");

  bool changed = false;
  int  cv_prob, trig_prob, trig_len, steps, octs, start_note, step_rate_idx;
  bool play_on;
  int  cv_source_idx, dig_in_idx, scale_idx;
  {
    std::lock_guard<std::mutex> lk(g_mutex);
    play_on       = g_run_item->value();
    cv_prob       = g_cv_prob_item->value();
    trig_prob     = g_trig_prob_item->value();
    trig_len      = g_trig_length_item->value();
    steps         = g_steps_item->value();
    octs          = g_octaves_item->value();
    start_note    = g_start_note_item->value();
    step_rate_idx = g_step_rate_item->selected_index();
    cv_source_idx = g_cv_source_item->selected_index();
    dig_in_idx    = g_dig_in_item->selected_index();
    scale_idx     = g_scale_item->selected_index();
  }

  // ---- Big PLAY/STOP banner at the top ----
  // Green when stopped (press to play), red when playing (press to stop).
  // Bound to the P keyboard shortcut as well.
  {
    const ImU32 play_color    = IM_COL32( 70, 170,  90, 255);
    const ImU32 play_hovered  = IM_COL32( 90, 200, 110, 255);
    const ImU32 play_active   = IM_COL32( 50, 140,  70, 255);
    const ImU32 stop_color    = IM_COL32(200,  80,  70, 255);
    const ImU32 stop_hovered  = IM_COL32(220, 100,  90, 255);
    const ImU32 stop_active   = IM_COL32(170,  60,  50, 255);
    const bool  on = play_on;
    ImGui::PushStyleColor(ImGuiCol_Button,        on ? stop_color   : play_color);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, on ? stop_hovered : play_hovered);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  on ? stop_active  : play_active);
    if (ImGui::Button(on ? "STOP  (P)" : "PLAY  (P)", ImVec2(-FLT_MIN, 40))) {
      DoTogglePlay();
    }
    ImGui::PopStyleColor(3);
  }
  ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                     "Direct sliders — syncs both ways with the OLED menu.");
  ImGui::Separator();

  ImGui::SeparatorText("Pitch");
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
  changed |= ImGui::SliderInt("Octaves",       &octs,       seq::kMinOctaves, seq::kMaxOctaves);
  changed |= ImGui::SliderInt("Start note",    &start_note, 0, 36);
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

  ImGui::SeparatorText("Clock");
  changed |= ImGui::SliderInt("Steps", &steps, seq::kMinSteps, seq::kMaxSteps);
  if (ImGui::BeginCombo("Step rate", kStepRateNames[step_rate_idx])) {
    for (int i = 0; i < kStepRateCount; ++i) {
      bool sel = (i == step_rate_idx);
      if (ImGui::Selectable(kStepRateNames[i], sel)) {
        step_rate_idx = i;
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

  ImGui::SeparatorText("Probability");
  changed |= ImGui::SliderInt("CV change %",   &cv_prob,    0, 100);
  changed |= ImGui::SliderInt("Trig change %", &trig_prob,  0, 100);
  changed |= ImGui::SliderInt("Trig length %", &trig_len,   0, 100);

  ImGui::SeparatorText("Actions");
  ImGui::PushStyleColor(ImGuiCol_Button,        IM_COL32(140, 50, 50, 255));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(180, 70, 70, 255));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive,  IM_COL32(110, 40, 40, 255));
  if (ImGui::Button("Clear CV", ImVec2(140, 0))) {
    std::lock_guard<std::mutex> lk(g_mutex);
    ActionClearCv(nullptr);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Trig", ImVec2(140, 0))) {
    std::lock_guard<std::mutex> lk(g_mutex);
    ActionClearTriggers(nullptr);
  }
  ImGui::PopStyleColor(3);

  if (changed) {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_run_item       ->set_value(play_on);
    g_scale_item     ->set_selected_index(scale_idx);
    g_cv_prob_item   ->set_value(cv_prob);
    g_trig_prob_item ->set_value(trig_prob);
    g_trig_length_item->set_value(trig_len);
    g_steps_item     ->set_value(steps);
    g_octaves_item   ->set_value(octs);
    g_start_note_item->set_value(start_note);
    g_step_rate_item ->set_selected_index(step_rate_idx);
    g_cv_source_item ->set_selected_index(cv_source_idx);
    g_dig_in_item    ->set_selected_index(dig_in_idx);
    OnMenuCommit(nullptr);
  }

  ImGui::End();
}

// ============================================================
//  Quick Nav window — sim-only navigation shortcuts
// ============================================================
// Direct buttons for things you'd otherwise have to encoder-dance for.
// None of these will exist on the physical module — they're here so
// the sim is comfortable to operate with a mouse. The OLED menu (driven
// by the Hardware window's encoder mirror) is still the canonical edit
// surface, and the same actions are reachable that way.
static void RenderQuickNavWindow() {
  ImGui::SetNextWindowPos (ImVec2(880, 550), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(510, 350), ImGuiCond_FirstUseEver);
  ImGui::Begin("Quick Nav (sim only)");

  ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                     "Sim shortcuts — not on hardware.");
  ImGui::Separator();

  ImGui::SeparatorText("Navigation");
  if (ImGui::Button("MENU  (M)", ImVec2(160, 36))) DoOpenMainMenu();
  ImGui::SameLine();
  if (ImGui::Button("BACK",      ImVec2(100, 36))) DoBack();
  ImGui::SameLine();
  if (ImGui::Button("HOME  (H)", ImVec2(160, 36))) DoGoHome();

  ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "Jump straight to a category:");
  if (ImGui::Button("Clock",   ImVec2(100, 0))) DoOpenSubmenu(&g_clock_menu);
  ImGui::SameLine();
  if (ImGui::Button("Pitch",   ImVec2(100, 0))) DoOpenSubmenu(&g_pitch_menu);
  ImGui::SameLine();
  if (ImGui::Button("Probs",   ImVec2(100, 0))) DoOpenSubmenu(&g_probs_menu);
  ImGui::SameLine();
  if (ImGui::Button("Actions", ImVec2(100, 0))) DoOpenSubmenu(&g_actions_menu);

  ImGui::SeparatorText("Menu shortcuts (only when a menu is on top)");
  const bool in_menu = (TopMenuList() != nullptr);
  if (!in_menu) ImGui::BeginDisabled();
  for (int i = 1; i <= 9; ++i) {
    char lbl[3] = { static_cast<char>('0' + i), 0, 0 };
    if (i > 1) ImGui::SameLine();
    if (ImGui::Button(lbl, ImVec2(28, 0))) DoMenuJump(i - 1);
  }
  if (!in_menu) ImGui::EndDisabled();

  ImGui::SeparatorText("Sim-only keyboard reference");
  ImGui::BeginTable("simkeys", 2, ImGuiTableFlags_SizingFixedFit);
  auto row = [](const char* k, const char* v) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn(); ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "%s", k);
    ImGui::TableNextColumn(); ImGui::Text("%s", v);
  };
  row("P",       "Play / Stop transport");
  row("S",       "Toggle internal clock generator");
  row("T",       "Tap tempo");
  row("V",       "Cycle playback layout (Grid / Piano Roll)");
  row("M",       "Open main menu");
  row("H",       "Home — return to playback view");
  row("A",       "Toggle audition audio");
  row("1 .. 9",  "Jump to menu item N");
  row("Esc / Q", "Quit");
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
    case GLFW_KEY_P:
      if (action == GLFW_PRESS) DoTogglePlay();
      break;
    case GLFW_KEY_T:
      if (action == GLFW_PRESS) DoTapTempo();
      break;
    case GLFW_KEY_A:
      if (action == GLFW_PRESS) {
        g_audition_enabled.store(!g_audition_enabled.load());
      }
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
      1400, 920, "Generative Sequencer — Playground", nullptr, nullptr);
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

  // Periodic state-save tick — every 2 seconds, persist sim state.
  // Covers Sim-window slider changes + live cv/trig mutations that
  // OnMenuCommit doesn't get fired for.
  auto last_save = std::chrono::steady_clock::now();

  while (!glfwWindowShouldClose(win)) {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    // ---- Main menu bar — view toggles, key hints ----
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("View")) {
        ImGui::MenuItem("OLED 128x64",         nullptr, &g_show_oled);
        ImGui::MenuItem("Hardware (mirror)",   nullptr, &g_show_hardware);
        ImGui::MenuItem("Quick Nav",           nullptr, &g_show_quicknav);
        ImGui::MenuItem("Params",              nullptr, &g_show_params);
        ImGui::MenuItem("Sim",                 nullptr, &g_show_sim);
        ImGui::MenuItem("Inspector",           nullptr, &g_show_inspector);
        ImGui::Separator();
        if (ImGui::MenuItem("Show all"))    {
          g_show_oled = g_show_hardware = g_show_quicknav =
          g_show_params = g_show_sim = g_show_inspector = true;
        }
        if (ImGui::MenuItem("OLED + Hardware only")) {
          g_show_oled = g_show_hardware = true;
          g_show_quicknav = g_show_params = g_show_sim = g_show_inspector = false;
        }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Transport")) {
        bool playing = false;
        {
          std::lock_guard<std::mutex> lk(g_mutex);
          if (g_run_item) playing = g_run_item->value();
        }
        if (ImGui::MenuItem(playing ? "Stop  (P)" : "Play  (P)")) DoTogglePlay();
        if (ImGui::MenuItem("Reset transport (R)")) g_reset_request.store(true);
        if (ImGui::MenuItem("Tap tempo (T)"))      DoTapTempo();
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }

    if (g_show_oled)      RenderOledWidget();
    if (g_show_hardware)  RenderHardwareWindow();
    if (g_show_quicknav)  RenderQuickNavWindow();
    if (g_show_params)    RenderParamsWindow();
    if (g_show_sim)       RenderSimWindow();
    if (g_show_inspector) RenderInspectorWindow();
    ImGui::Render();

    int w, h; glfwGetFramebufferSize(win, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(win);

    // Periodic save — every 2 s, capture Sim-window changes and live
    // cv/trig mutations that don't fire OnMenuCommit.
    const auto now = std::chrono::steady_clock::now();
    if (now - last_save > std::chrono::seconds(2)) {
      std::lock_guard<std::mutex> lk(g_mutex);
      SaveStateLocked();
      last_save = now;
    }
  }

  g_app_quitting.store(true);
  clock_thread.join();

  // Final save before tearing down — covers last-second edits.
  {
    std::lock_guard<std::mutex> lk(g_mutex);
    SaveStateLocked();
  }

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
  delete g_start_note_item;  delete g_step_rate_item;
  delete g_clear_cv_item;    delete g_clear_trig_item;
  delete g_run_item;         delete g_cv_source_item;
  delete g_dig_in_item;
  return 0;
}
