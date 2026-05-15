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

#include <atomic>
#include <chrono>
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

// CV source labels — index = enum value.
constexpr const char*  kCvSourceNames[]   = { "Normal", "Test", "Tuning" };
constexpr int          kCvSourceCount     = 3;

// DigitalInMode labels — index = enum value (matches seq::DigitalInMode).
constexpr const char*  kDigInModeNames[]  = { "None", "Reset", "RunStop", "Freeze" };
constexpr int          kDigInModeCount    = 4;

// Menu items + view stack
std::vector<const char*>  g_scale_names;
seq::ViewStack            g_views;
seq::MenuListView         g_root_view;

seq::SingleSelectItem*    g_scale_item        = nullptr;
seq::NumericalItem*       g_cv_prob_item      = nullptr;
seq::NumericalItem*       g_trig_prob_item    = nullptr;
seq::NumericalItem*       g_trig_length_item  = nullptr;
seq::NumericalItem*       g_steps_item        = nullptr;
seq::NumericalItem*       g_octaves_item      = nullptr;
seq::NumericalItem*       g_start_note_item   = nullptr;
seq::NumericalItem*       g_clk_div_item      = nullptr;  // audit 2d
seq::ToggleItem*          g_cv_erase_item     = nullptr;
seq::ToggleItem*          g_trig_erase_item   = nullptr;
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
  g_vparams.is_cv_erase        = g_cv_erase_item->value();
  g_vparams.is_trig_erase      = g_trig_erase_item->value();
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
  g_cv_erase_item     = new seq::ToggleItem("CvErase",   false);
  g_trig_erase_item   = new seq::ToggleItem("TrigErase", false);
  g_cv_source_item    = new seq::SingleSelectItem(
      "CV Source", kCvSourceNames, kCvSourceCount, 0);
  g_dig_in_item       = new seq::SingleSelectItem(
      "DigIn", kDigInModeNames, kDigInModeCount,
      static_cast<int>(seq::DigitalInMode::Reset));

  // Order: transport-y stuff first, then params, then modes.
  g_items = {
    g_run_item, g_scale_item,
    g_cv_prob_item, g_trig_prob_item, g_trig_length_item,
    g_steps_item, g_octaves_item, g_start_note_item, g_clk_div_item,
    g_cv_erase_item, g_trig_erase_item,
    g_cv_source_item, g_dig_in_item,
  };

  g_root_view.SetTitle("Main Menu");
  g_root_view.SetItems(g_items.data(), static_cast<int>(g_items.size()));
  g_root_view.SetCommitCallback(&OnMenuCommit, nullptr);

  g_views.Push(&g_root_view);

  OnMenuCommit(nullptr);
  g_sequencer.Init();
}

// ============================================================
//  Clock thread
// ============================================================
static void ClockThreadMain() {
  using clock_t = std::chrono::steady_clock;
  auto next = clock_t::now();
  bool clock_high = false;

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
        const seq::Voice& v = g_sequencer.voice(0);
        g_disp_step.store(v.last_played_step());
        g_disp_dac.store(v.last_dac());
        g_disp_trig.store(v.trig_active());
        std::memcpy(g_disp_cv,        v.cv_sequence(),      sizeof(g_disp_cv));
        std::memcpy(g_disp_trig_grid, v.trigger_sequence(), sizeof(g_disp_trig_grid));
      }
    }

    if (!g_clock_running.load()) {
      const auto now_ms = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              clock_t::now().time_since_epoch()).count());
      {
        std::lock_guard<std::mutex> lk(g_mutex);
        g_sequencer.Tick(now_ms);
        g_disp_trig.store(g_sequencer.voice(0).trig_active());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      next = clock_t::now();
      continue;
    }

    const int   bpm         = g_bpm.load();
    const int   period_ms   = (bpm > 0) ? (60000 / bpm) : 1000;
    const auto  half_period = std::chrono::milliseconds(period_ms / 2);

    next += half_period;
    std::this_thread::sleep_until(next);

    clock_high = !clock_high;
    const auto now_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            clock_t::now().time_since_epoch()).count());
    {
      std::lock_guard<std::mutex> lk(g_mutex);
      g_sequencer.OnClockEdge(clock_high, now_ms);
      g_sequencer.Tick(now_ms);

      const seq::Voice& v = g_sequencer.voice(0);
      g_disp_step.store(v.last_played_step());
      g_disp_dac.store(v.last_dac());
      g_disp_trig.store(v.trig_active());
      std::memcpy(g_disp_cv,        v.cv_sequence(),      sizeof(g_disp_cv));
      std::memcpy(g_disp_trig_grid, v.trigger_sequence(), sizeof(g_disp_trig_grid));
    }
  }
}

// ============================================================
//  OLED widget
// ============================================================
static void RenderOledWidget() {
  {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_views.Draw(g_oled);
  }
  ImGui::Begin("OLED 128x64");
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImVec2 p0 = ImGui::GetCursorScreenPos();
  const float scale = 3.0f;
  ImVec2 p1(p0.x + 128 * scale, p0.y + 64 * scale);
  dl->AddRectFilled(p0, p1, IM_COL32(8, 12, 20, 255));
  for (int y = 0; y < 64; ++y) {
    for (int x = 0; x < 128; ++x) {
      if (g_oled.buf[y * 128 + x]) {
        ImVec2 a(p0.x + x * scale, p0.y + y * scale);
        ImVec2 b(a.x + scale, a.y + scale);
        dl->AddRectFilled(a, b, IM_COL32(180, 220, 255, 255));
      }
    }
  }
  ImGui::Dummy(ImVec2(128 * scale, 64 * scale));
  ImGui::End();
}

// ============================================================
//  Sequencer Controls window
// ============================================================
static void RenderControlsWindow() {
  ImGui::Begin("Sequencer Controls");

  ImGui::SeparatorText("Internal clock");
  bool running = g_clock_running.load();
  if (ImGui::Checkbox("Clock running", &running)) g_clock_running.store(running);
  int bpm = g_bpm.load();
  if (ImGui::SliderInt("BPM", &bpm, 20, 240)) g_bpm.store(bpm);

  ImGui::SeparatorText("Direct param edit (syncs to menu)");
  bool changed = false;
  int  cv_prob, trig_prob, trig_len, steps, octs, start_note, clk_div;
  bool cv_erase, trig_erase, run;
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
    cv_erase      = g_cv_erase_item->value();
    trig_erase    = g_trig_erase_item->value();
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
  changed |= ImGui::SliderInt("Clock divider", &clk_div,    1, 16);
  changed |= ImGui::SliderInt("CV change %",   &cv_prob,    0, 100);
  changed |= ImGui::SliderInt("Trig change %", &trig_prob,  0, 100);
  changed |= ImGui::SliderInt("Trig length %", &trig_len,   0, 100);
  changed |= ImGui::Checkbox ("CV erase",      &cv_erase);
  changed |= ImGui::Checkbox ("Trig erase",    &trig_erase);
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
    g_cv_erase_item->set_value(cv_erase);
    g_trig_erase_item->set_value(trig_erase);
    g_cv_source_item->set_selected_index(cv_source_idx);
    g_dig_in_item->set_selected_index(dig_in_idx);
    OnMenuCommit(nullptr);
  }

  ImGui::SeparatorText("Engine inspector");
  ImGui::Text("Step %d   DAC %u   TRIG %s",
              g_disp_step.load(),
              static_cast<unsigned>(g_disp_dac.load()),
              g_disp_trig.load() ? "ON" : "off");
  ImGui::Text("Scale notes: %d", g_scale_count);
  ImGui::Text("Views in stack: %d", g_views.depth());

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

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(win);
  glfwTerminate();

  delete g_scale_item;       delete g_cv_prob_item;
  delete g_trig_prob_item;   delete g_trig_length_item;
  delete g_steps_item;       delete g_octaves_item;
  delete g_start_note_item;  delete g_clk_div_item;
  delete g_cv_erase_item;    delete g_trig_erase_item;
  delete g_run_item;         delete g_cv_source_item;
  delete g_dig_in_item;
  return 0;
}
