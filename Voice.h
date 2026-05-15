// Voice.h
// One voice of the generative ambient sequencer. The polyphonic version of
// the module (see project_generative_ambient_sequencer in Claude memory)
// will instantiate N of these; the v0.3 codebase ships with N=1 wired
// through Sequencer::kVoiceCount, but the engine is shaped so that growing
// to 4 is a parameter change, not a rewrite.
//
// A Voice owns:
//   - its own 16-step CV and trigger sequences
//   - its own probability / step-count / trigger-length parameters
//   - its own RNG state
//   - its own trigger-off timer
//
// The Voice does NOT own:
//   - the clock (Sequencer does — it distributes ticks)
//   - the quantizer scale buffer (Sequencer holds the pointer, passes it in)
//   - transport state (Sequencer)
//   - any GPIO / DAC / display (host)

#pragma once
#include <cstdint>
#include <random>

namespace seq {

constexpr int      kMaxSteps        = 16;
constexpr int      kMinSteps        = 2;
constexpr int      kMaxOctaves      = 5;
constexpr int      kMinOctaves      = 1;

constexpr uint32_t kDefaultClockMs    = 250;   // ≈ 240 BPM
constexpr uint32_t kMinClockMsForTrig = 50;

inline constexpr uint16_t kTuningCvSequence[kMaxSteps] = {
  816, 1632, 816, 1632, 816, 1632, 816, 1632,
  816, 1632, 816, 1632, 816, 1632, 816, 1632,
};

enum class CvSource : uint8_t {
  Normal = 0,
  Test   = 1,
  Tuning = 2,
};

struct VoiceParams {
  int  number_of_steps    = 16;
  int  cv_change_pct      = 0;
  int  trig_change_pct    = 0;
  int  trig_length_pct    = 50;

  // Skip (n-1) of every n clock edges arriving at the Sequencer. 1 = every
  // edge advances this voice. >1 lets each voice run at its own rate, which
  // is the basic ambient-poly trick (voice 1 every 1/4, voice 2 every 1/8…).
  // Wired through Sequencer in Phase B; field is here in Phase A to lock the
  // VoiceParams shape.
  int  clock_divider      = 1;

  CvSource cv_source      = CvSource::Normal;
};

class Voice {
 public:
  void Init();
  // Reset the playhead to step 0 without clearing the sequences themselves.
  void Reset();

  // One-shot clears, replacing the v0.2 destructive "erase" toggles.
  // ClearCv() forces every CV step to the supplied scale's root note.
  // ClearTriggers() forces every trigger to ON.
  // Called by the host (or a menu ActionItem) when the user asks.
  void ClearCv(const uint16_t* scale, int scale_length);
  void ClearTriggers();

  void SetParams(const VoiceParams& p) { params_ = p; }
  void Seed(uint64_t s) { rng_.seed(static_cast<uint32_t>(s)); }

  // Advance one step. Called by Sequencer when the voice's clock divider
  // says "this edge is yours." `now_ms` is the millisecond clock; `gate_ms`
  // is the current clock gate width (Sequencer's measurement, with the
  // kMinClockMsForTrig floor applied so the first/manual pulse still emits
  // a visible gate). `scale` + `scale_length` are the shared quantizer
  // scale, owned by Sequencer/host.
  // Returns true if a step actually fired.
  bool Advance(uint32_t now_ms, uint32_t gate_ms,
               const uint16_t* scale, int scale_length);

  // Trigger-off timer. Sequencer calls this regularly. Returns true if the
  // trigger transitioned off this call.
  bool Tick(uint32_t now_ms);

  // ---- Read-only state ----
  int       last_played_step() const { return last_played_step_; }
  uint16_t  last_dac()         const { return last_dac_; }
  bool      trig_active()      const { return trigger_active_; }
  int       clock_divider()    const { return params_.clock_divider; }
  const uint16_t* cv_sequence()      const { return cv_sequence_; }
  const uint8_t*  trigger_sequence() const { return trigger_sequence_; }

 private:
  VoiceParams params_{};

  uint16_t cv_sequence_[kMaxSteps]      = {0};
  uint8_t  trigger_sequence_[kMaxSteps] = {0};
  uint16_t test_cv_sequence_[kMaxSteps] = {0};

  // The scale pointer the test sequence was last built against. Lets us
  // skip RebuildTestSequence on every step (audit 2c, lands in Phase B,
  // shape is here in Phase A).
  const uint16_t* last_test_scale_ptr_ = nullptr;
  int             last_test_scale_len_ = 0;

  int   next_step_                = 0;
  int   last_played_step_         = 0;

  bool     trigger_active_        = false;
  uint32_t trigger_start_ticks_   = 0;
  uint32_t trig_length_ms_        = 0;
  uint32_t ticks_to_trigger_off_  = 0;

  uint16_t last_dac_              = 0;

  std::mt19937 rng_{std::random_device{}()};

  bool     ProbabilityRoll(int pct);
  void     RebuildTestSequenceIfNeeded(const uint16_t* scale, int len);
  void     RandomlyChangeCurrentStepCv(const uint16_t* scale, int len);
  void     RandomlyChangeCurrentStepTrigger();
  uint16_t ScaleRoot(const uint16_t* scale, int len) const;
};

}  // namespace seq
