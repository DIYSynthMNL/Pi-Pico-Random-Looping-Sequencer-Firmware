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

    // Phase A: every voice advances every edge.
    // Phase B (audit 2d) will gate this on the voice's clock_divider:
    //   if (divider_count_[i] % voices_[i].divider() == 0) Advance(...);
    for (int i = 0; i < kVoiceCount; ++i) {
      voices_[i].Advance(now_ms, clock_ms_,
                         params_.scale, params_.scale_length);
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
