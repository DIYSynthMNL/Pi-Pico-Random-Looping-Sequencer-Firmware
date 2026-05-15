// SequencerEngine.cpp — see SequencerEngine.h for the design rationale.
//
// Audit notes (playground-v0.2.0):
//   - clock_ms_ seeds to kDefaultClockMs in Init() and is floored to
//     kMinClockMsForTrig in the trigger-length computation, so the very
//     first step and any manual / single-pulse clocking still emit a
//     visibly long trigger gate.
//   - The accessor returns last_played_step_ (the step that just fired),
//     not the next-to-play index — fixes the off-by-one inherited from
//     main.py.
//   - RNG is std::mt19937 + uniform_int_distribution, replacing rand()%n.

#include "SequencerEngine.h"

namespace rls {

void SequencerEngine::Init() {
  // Mirror populate_sequence_with_default() — every step starts at scale root,
  // every trigger starts on. If no scale has been supplied yet, default to 0.
  const uint16_t root = ScaleRoot();
  for (int i = 0; i < kMaxSteps; ++i) {
    cv_sequence_[i]      = root;
    trigger_sequence_[i] = 1;
  }
  next_step_                   = 0;
  last_played_step_            = 0;
  step_changed_on_clock_pulse_ = false;
  previous_clock_ticks_        = 0;
  // Seed clock_ms_ with a sensible default so the first step's trigger
  // length is non-zero even before any real period has been measured.
  clock_ms_                    = kDefaultClockMs;
  trigger_active_              = false;
  trigger_start_ticks_         = 0;
  trig_length_ms_              = 0;
  ticks_to_trigger_off_        = 0;
  last_dac_                    = root;
  RebuildTestSequence();
}

uint16_t SequencerEngine::ScaleRoot() const {
  return (params_.current_12bit_scale && params_.scale_length > 0)
         ? params_.current_12bit_scale[0]
         : 0;
}

bool SequencerEngine::ProbabilityRoll(int probability_0_100) {
  if (probability_0_100 <= 0)   return false;
  if (probability_0_100 >= 100) return true;
  // Inclusive 1..100 — matches the Python semantics where random.random()*100
  // ranges over [0, 100) and the check is "<= probability".
  std::uniform_int_distribution<int> dist(1, 100);
  return dist(rng_) <= probability_0_100;
}

void SequencerEngine::RebuildTestSequence() {
  // Fill kMaxSteps cells by cycling the current scale, mirroring
  // get_test_sequence() in main.py.
  if (!params_.current_12bit_scale || params_.scale_length <= 0) {
    for (int i = 0; i < kMaxSteps; ++i) test_cv_sequence_[i] = 0;
    return;
  }
  int idx = 0;
  for (int i = 0; i < kMaxSteps; ++i) {
    test_cv_sequence_[i] = params_.current_12bit_scale[idx];
    idx = (idx + 1) % params_.scale_length;
  }
}

void SequencerEngine::RandomlyChangeCurrentStepCv() {
  if (!params_.current_12bit_scale || params_.scale_length <= 0) return;
  if (!ProbabilityRoll(params_.cv_probability_of_change))        return;
  std::uniform_int_distribution<int> dist(0, params_.scale_length - 1);
  const int random_idx = dist(rng_);
  cv_sequence_[next_step_] = params_.current_12bit_scale[random_idx];
}

void SequencerEngine::RandomlyChangeCurrentStepTrigger() {
  if (!ProbabilityRoll(params_.trig_probability_of_change)) return;
  std::uniform_int_distribution<int> coin(0, 1);
  trigger_sequence_[next_step_] = static_cast<uint8_t>(coin(rng_));
}

bool SequencerEngine::OnClockEdge(bool rising, uint32_t now_ms) {
  // Clamp step bounds — host may have lowered the step count.
  int steps = params_.number_of_steps;
  if (steps < kMinSteps) steps = kMinSteps;
  if (steps > kMaxSteps) steps = kMaxSteps;

  // If we've run past the active sequence length, wrap.
  if (next_step_ >= steps) next_step_ = 0;

  if (rising && !step_changed_on_clock_pulse_) {
    // Rising edge: snapshot ticks, fire the step, advance.
    previous_clock_ticks_        = now_ms;
    step_changed_on_clock_pulse_ = true;

    RebuildTestSequence();              // mirror Python ordering
    RandomlyChangeCurrentStepCv();
    RandomlyChangeCurrentStepTrigger();

    if (params_.is_cv_erase) {
      cv_sequence_[next_step_] = ScaleRoot();
    }
    if (params_.is_trig_erase) {
      trigger_sequence_[next_step_] = 1;
    }

    // Pick the CV value to emit on this step based on the source mode.
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

    // Trigger output: open the gate for trigger_length_percent of the
    // most recently measured gate width. Floor the gate width so a manual
    // pulse or the very first edge still produces a visible gate.
    const uint32_t gate_basis = (clock_ms_ < kMinClockMsForTrig)
                                ? kDefaultClockMs
                                : clock_ms_;
    trig_length_ms_ = (gate_basis * params_.trigger_length_percent) / 100;
    if (trigger_sequence_[next_step_] == 1) {
      trigger_active_       = true;
      trigger_start_ticks_  = now_ms;
      ticks_to_trigger_off_ = now_ms + trig_length_ms_;
    }

    // Mark this step as played, advance the pointer.
    last_played_step_ = next_step_;
    next_step_        = (next_step_ + 1) % steps;
    return true;
  }

  if (!rising && step_changed_on_clock_pulse_) {
    // Falling edge: measure the high portion of the clock signal.
    // Matches main.py's rising→falling interval semantic.
    step_changed_on_clock_pulse_ = false;
    clock_ms_ = now_ms - previous_clock_ticks_;
  }
  return false;
}

bool SequencerEngine::Tick(uint32_t now_ms) {
  if (trigger_active_ && now_ms >= ticks_to_trigger_off_) {
    trigger_active_ = false;
    return true;
  }
  return false;
}

}  // namespace rls
