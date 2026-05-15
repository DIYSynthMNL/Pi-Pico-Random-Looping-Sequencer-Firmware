// Voice.cpp — see Voice.h.

#include "Voice.h"

namespace seq {

void Voice::Init() {
  const uint16_t root = ScaleRoot(nullptr, 0);
  for (int i = 0; i < kMaxSteps; ++i) {
    cv_sequence_[i]      = root;
    trigger_sequence_[i] = 1;
  }
  next_step_           = 0;
  last_played_step_    = 0;
  trigger_active_      = false;
  trigger_start_ticks_ = 0;
  trig_length_ms_      = 0;
  ticks_to_trigger_off_ = 0;
  last_dac_            = root;
  last_test_scale_ptr_ = nullptr;
  last_test_scale_len_ = 0;
  // test_cv_sequence_ filled lazily on first scale-aware Advance.
}

void Voice::Reset() {
  next_step_        = 0;
  last_played_step_ = 0;
}

void Voice::ClearCv(const uint16_t* scale, int scale_length) {
  const uint16_t root = ScaleRoot(scale, scale_length);
  for (int i = 0; i < kMaxSteps; ++i) cv_sequence_[i] = root;
}

void Voice::ClearTriggers() {
  for (int i = 0; i < kMaxSteps; ++i) trigger_sequence_[i] = 1;
}

void Voice::SetSequences(const uint16_t* cv, const uint8_t* trig) {
  if (cv) {
    for (int i = 0; i < kMaxSteps; ++i) cv_sequence_[i] = cv[i];
  }
  if (trig) {
    for (int i = 0; i < kMaxSteps; ++i) trigger_sequence_[i] = trig[i] ? 1 : 0;
  }
}

uint16_t Voice::ScaleRoot(const uint16_t* scale, int len) const {
  return (scale && len > 0) ? scale[0] : 0;
}

bool Voice::ProbabilityRoll(int pct) {
  if (pct <= 0)   return false;
  if (pct >= 100) return true;
  std::uniform_int_distribution<int> dist(1, 100);
  return dist(rng_) <= pct;
}

void Voice::RebuildTestSequenceIfNeeded(const uint16_t* scale, int len) {
  // Phase A: shape only. Phase B (audit 2c) will *prove* this saves work
  // by skipping the rebuild when the scale pointer/length haven't changed.
  if (scale == last_test_scale_ptr_ && len == last_test_scale_len_) return;
  last_test_scale_ptr_ = scale;
  last_test_scale_len_ = len;
  if (!scale || len <= 0) {
    for (int i = 0; i < kMaxSteps; ++i) test_cv_sequence_[i] = 0;
    return;
  }
  int idx = 0;
  for (int i = 0; i < kMaxSteps; ++i) {
    test_cv_sequence_[i] = scale[idx];
    idx = (idx + 1) % len;
  }
}

void Voice::RandomlyChangeCurrentStepCv(const uint16_t* scale, int len) {
  if (!scale || len <= 0) return;
  if (!ProbabilityRoll(params_.cv_change_pct)) return;
  std::uniform_int_distribution<int> dist(0, len - 1);
  cv_sequence_[next_step_] = scale[dist(rng_)];
}

void Voice::RandomlyChangeCurrentStepTrigger() {
  if (!ProbabilityRoll(params_.trig_change_pct)) return;
  std::uniform_int_distribution<int> coin(0, 1);
  trigger_sequence_[next_step_] = static_cast<uint8_t>(coin(rng_));
}

bool Voice::Advance(uint32_t now_ms, uint32_t gate_ms,
                    const uint16_t* scale, int scale_length) {
  int steps = params_.number_of_steps;
  if (steps < kMinSteps) steps = kMinSteps;
  if (steps > kMaxSteps) steps = kMaxSteps;
  if (next_step_ >= steps) next_step_ = 0;

  RebuildTestSequenceIfNeeded(scale, scale_length);
  RandomlyChangeCurrentStepCv(scale, scale_length);
  RandomlyChangeCurrentStepTrigger();

  switch (params_.cv_source) {
    case CvSource::Test:
      last_dac_ = test_cv_sequence_[next_step_];
      break;
    case CvSource::Tuning:
      last_dac_ = kTuningCvSequence[next_step_];
      break;
    case CvSource::Normal:
    default:
      last_dac_ = cv_sequence_[next_step_];
      break;
  }

  const uint32_t gate_basis = (gate_ms < kMinClockMsForTrig)
                              ? kDefaultClockMs
                              : gate_ms;
  trig_length_ms_ = (gate_basis * params_.trig_length_pct) / 100;
  if (trigger_sequence_[next_step_] == 1) {
    trigger_active_       = true;
    trigger_start_ticks_  = now_ms;
    ticks_to_trigger_off_ = now_ms + trig_length_ms_;
  }

  last_played_step_ = next_step_;
  next_step_        = (next_step_ + 1) % steps;
  return true;
}

bool Voice::Tick(uint32_t now_ms) {
  if (trigger_active_ && now_ms >= ticks_to_trigger_off_) {
    trigger_active_ = false;
    return true;
  }
  return false;
}

}  // namespace seq
