// Sequencer.h
// Top-level orchestrator: owns N voices, distributes clock edges to them
// according to each voice's clock divider, and holds the shared transport
// state + quantizer scale.
//
// Phase A ships with kVoiceCount = 1. Bumping to 4 (or any N) is a single
// constant change plus a per-voice menu page.
//
// Host responsibilities (unchanged from the v0.2 engine API):
//   - measure `now_ms` somehow
//   - call OnClockEdge() on each rising/falling edge of the external clock
//   - call OnDigitalEdge() on each edge of the digital-input jack
//   - call Tick() regularly (~every loop iteration)
//   - own the 12-bit DAC scale buffer
//   - read each voice's DAC value + trigger state, push to hardware

#pragma once
#include <cstdint>
#include "Voice.h"

namespace seq {

enum class DigitalInMode : uint8_t {
  None    = 0,
  Reset   = 1,   // rising edge → all voices Reset()
  RunStop = 2,   // rising edge → toggle transport enabled
  Freeze  = 3,   // while high → ignore clock edges (transient stop)
};

struct SequencerParams {
  bool          enabled         = true;
  DigitalInMode digital_in_mode = DigitalInMode::Reset;

  // Shared quantizer scale (host owns the buffer).
  const uint16_t* scale         = nullptr;
  int             scale_length  = 0;
};

class Sequencer {
 public:
  static constexpr int kVoiceCount = 1;

  void Init();
  void Reset();        // resets all voices to step 0
  void Tick(uint32_t now_ms);   // forwards to each voice's trigger-off timer

  void SetParams(const SequencerParams& p) { params_ = p; }

  void OnClockEdge(bool rising, uint32_t now_ms);
  void OnDigitalEdge(bool rising, uint32_t now_ms);

  // Voice access — host sets per-voice params and reads per-voice state.
  Voice& voice(int i) { return voices_[i]; }
  const Voice& voice(int i) const { return voices_[i]; }
  static constexpr int voice_count() { return kVoiceCount; }

  // Measured clock period (rising-to-rising). Used for both trigger-
  // length calculation and clock-multiplier sub-edge spacing. Floored
  // to kMinClockMsForTrig at use-time in Voice::Advance.
  uint32_t clock_period_ms() const { return clock_period_ms_; }

 private:
  Voice           voices_[kVoiceCount];
  SequencerParams params_{};

  // Clock edge debounce + period measurement (rising-to-rising).
  // Note: in v0.5.1 this field measured *gate width* (rising-to-falling),
  // which caused the multiplier to cluster sub-edges in the high half of
  // a 50%-duty square clock instead of spacing them across the period.
  bool      step_changed_on_clock_pulse_ = false;
  bool      have_clock_period_           = false;
  uint32_t  last_rising_ms_              = 0;
  uint32_t  clock_period_ms_             = kDefaultClockMs;

  // Per-voice edge counter — increments on each accepted (external + sub-)
  // edge. Voice advances when counter % divider == 0. The clock multiplier
  // schedules sub-edges that count just like external ones.
  uint32_t  divider_count_[kVoiceCount]  = {0};

  // Sub-edge scheduler state (one slot per voice). When clock_multiplier
  // is > 1, OnClockEdge sets up `sub_edges_remaining_` advances spaced by
  // `sub_edge_interval_ms_`; Tick() fires them as they come due.
  int       sub_edges_remaining_[kVoiceCount] = {0};
  uint32_t  sub_edge_interval_ms_[kVoiceCount] = {0};
  uint32_t  next_sub_edge_ms_[kVoiceCount]     = {0};
};

}  // namespace seq
