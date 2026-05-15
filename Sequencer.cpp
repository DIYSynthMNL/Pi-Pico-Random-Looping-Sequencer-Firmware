// Sequencer.cpp — see Sequencer.h.

#include "Sequencer.h"

namespace seq {

void Sequencer::Init() {
  for (int i = 0; i < kVoiceCount; ++i) {
    voices_[i].Init();
    divider_count_[i] = 0;
  }
  step_changed_on_clock_pulse_ = false;
  previous_clock_ticks_        = 0;
  clock_ms_                    = kDefaultClockMs;
}

void Sequencer::Reset() {
  for (int i = 0; i < kVoiceCount; ++i) {
    voices_[i].Reset();
    divider_count_[i] = 0;
  }
}

void Sequencer::Tick(uint32_t now_ms) {
  for (int i = 0; i < kVoiceCount; ++i) {
    voices_[i].Tick(now_ms);
  }
}

void Sequencer::OnClockEdge(bool rising, uint32_t now_ms) {
  if (!params_.enabled) return;

  if (rising && !step_changed_on_clock_pulse_) {
    previous_clock_ticks_        = now_ms;
    step_changed_on_clock_pulse_ = true;

    // Each voice advances when its accepted-edge counter is divisible by
    // its clock_divider. divider=1 → every edge. divider=4 → every 4th.
    // Counter wraps naturally on overflow (uint32_t).
    for (int i = 0; i < kVoiceCount; ++i) {
      int div = voices_[i].clock_divider();
      if (div < 1) div = 1;
      if (divider_count_[i] % static_cast<uint32_t>(div) == 0) {
        voices_[i].Advance(now_ms, clock_ms_,
                           params_.scale, params_.scale_length);
      }
      ++divider_count_[i];
    }
  }

  if (!rising && step_changed_on_clock_pulse_) {
    step_changed_on_clock_pulse_ = false;
    clock_ms_ = now_ms - previous_clock_ticks_;
  }
}

void Sequencer::OnDigitalEdge(bool rising, uint32_t now_ms) {
  (void)now_ms;
  if (!rising) return;
  switch (params_.digital_in_mode) {
    case DigitalInMode::Reset:
      Reset();
      break;
    case DigitalInMode::RunStop:
      params_.enabled = !params_.enabled;
      break;
    case DigitalInMode::Freeze:
      // Phase B will wire the held-while-high semantic.
      break;
    case DigitalInMode::None:
    default:
      break;
  }
}

}  // namespace seq
