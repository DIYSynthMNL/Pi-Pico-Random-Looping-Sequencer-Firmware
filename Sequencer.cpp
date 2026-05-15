// Sequencer.cpp — see Sequencer.h.

#include "Sequencer.h"

namespace seq {

void Sequencer::Init() {
  for (int i = 0; i < kVoiceCount; ++i) {
    voices_[i].Init(params_.scale, params_.scale_length);
    divider_count_[i]        = 0;
    sub_edges_remaining_[i]  = 0;
    sub_edge_interval_ms_[i] = 0;
    next_sub_edge_ms_[i]     = 0;
  }
  step_changed_on_clock_pulse_ = false;
  have_clock_period_           = false;
  has_measured_period_         = false;
  last_rising_ms_              = 0;
  clock_period_ms_             = kDefaultClockMs;
}

void Sequencer::Reset() {
  for (int i = 0; i < kVoiceCount; ++i) {
    voices_[i].Reset();
    divider_count_[i]       = 0;
    sub_edges_remaining_[i] = 0;   // cancel any in-flight sub-edges
  }
}

void Sequencer::Tick(uint32_t now_ms) {
  if (!params_.enabled) {
    // Transport disabled — drop any pending sub-edges so they don't
    // burst-fire when transport re-enables. Trigger-off timers still
    // run so an in-flight gate can close cleanly.
    for (int i = 0; i < kVoiceCount; ++i) sub_edges_remaining_[i] = 0;
    for (int i = 0; i < kVoiceCount; ++i) voices_[i].Tick(now_ms);
    return;
  }

  // Fire scheduled sub-edges that have come due. Each accepted sub-edge
  // increments the divider counter just like a real external edge.
  for (int i = 0; i < kVoiceCount; ++i) {
    while (sub_edges_remaining_[i] > 0 && now_ms >= next_sub_edge_ms_[i]) {
      int div  = voices_[i].clock_divider();
      int mult = voices_[i].clock_multiplier();
      if (div  < 1) div  = 1;
      if (mult < 1) mult = 1;
      const uint32_t voice_period =
          (clock_period_ms_ * static_cast<uint32_t>(div))
          / static_cast<uint32_t>(mult);
      if (divider_count_[i] % static_cast<uint32_t>(div) == 0) {
        voices_[i].Advance(next_sub_edge_ms_[i], voice_period,
                           params_.scale, params_.scale_length);
      }
      ++divider_count_[i];
      --sub_edges_remaining_[i];
      next_sub_edge_ms_[i] += sub_edge_interval_ms_[i];
    }
    voices_[i].Tick(now_ms);
  }
}


void Sequencer::OnClockEdge(bool rising, uint32_t now_ms) {
  if (!params_.enabled) return;

  if (rising && !step_changed_on_clock_pulse_) {
    step_changed_on_clock_pulse_ = true;

    // Measure period (rising-to-rising). On the very first rising edge
    // we don't have a previous to subtract from, so keep the default.
    // Sub-edge scheduling waits for has_measured_period_ — only true
    // after the *second* rising edge — so we don't fire bogus sub-edges
    // using the kDefaultClockMs guess on the first edge.
    if (have_clock_period_) {
      const uint32_t p = now_ms - last_rising_ms_;
      if (p > 0) {
        clock_period_ms_     = p;
        has_measured_period_ = true;
      }
    }
    have_clock_period_ = true;
    last_rising_ms_    = now_ms;

    // Each voice advances when its accepted-edge counter is divisible
    // by its clock_divider. Counter wraps naturally on overflow. When
    // multiplier > 1 the voice fires immediately on this edge and
    // schedules (mult-1) additional sub-edges spaced evenly across the
    // measured *period* (rising-to-rising) — NOT across the gate width.
    for (int i = 0; i < kVoiceCount; ++i) {
      int div  = voices_[i].clock_divider();
      int mult = voices_[i].clock_multiplier();
      if (div  < 1) div  = 1;
      if (mult < 1) mult = 1;

      const uint32_t sub_interval =
          clock_period_ms_ / static_cast<uint32_t>(mult);
      // Voice's effective step period: shared period × divider / multiplier.
      // This becomes the basis for trig length in Voice::Advance.
      const uint32_t voice_period =
          (clock_period_ms_ * static_cast<uint32_t>(div))
          / static_cast<uint32_t>(mult);

      if (divider_count_[i] % static_cast<uint32_t>(div) == 0) {
        voices_[i].Advance(now_ms, voice_period,
                           params_.scale, params_.scale_length);
      }
      ++divider_count_[i];

      // (Re-)set the sub-edge schedule for this period. Skip when we
      // don't yet have a measured period (first edge after Init/Reset).
      if (mult > 1 && has_measured_period_) {
        sub_edges_remaining_[i]  = mult - 1;
        sub_edge_interval_ms_[i] = sub_interval;
        next_sub_edge_ms_[i]     = now_ms + sub_interval;
      } else {
        sub_edges_remaining_[i]  = 0;
      }
    }
  }

  if (!rising && step_changed_on_clock_pulse_) {
    // Falling edge — just clear the debounce. Period (not gate width) is
    // enough for both trig length and sub-edge spacing.
    step_changed_on_clock_pulse_ = false;
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
