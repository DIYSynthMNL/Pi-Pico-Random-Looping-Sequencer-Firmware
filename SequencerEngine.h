// SequencerEngine.h
// Host-agnostic random/looping quantized step sequencer engine.
// Port of the logic in DIYSynthMNL/Pi-Pico-Random-Looping-Sequencer/Software/main.py.
//
// Designed so a future polyphonic version (see project_generative_ambient_sequencer
// in Claude memory) can instantiate one of these per voice. No globals, no I/O,
// no time queries — host supplies now_ms.
//
// The two hosts (macOS playground, Pi Pico firmware) own:
//   - clock-edge detection on their respective GPIOs / keyboard
//   - I2C writes to the MCP4725 DAC and digital trigger output
//   - menu rendering on the SSD1306 OLED (or its desktop fake)
//   - the current 12-bit quantizer scale array (this engine reads only)
//   - re-seeding the engine's RNG from a real entropy source if desired
//     (`Seed()`); the default seed uses std::random_device which on Pico-SDK
//     may degrade to a fixed value.

#pragma once
#include <cstdint>
#include <random>

namespace rls {

constexpr int      kMaxSteps        = 16;
constexpr int      kMinSteps        = 2;
constexpr int      kMaxOctaves      = 5;
constexpr int      kMinOctaves      = 1;

// Default clock period used until the engine has actually measured one
// (also acts as the floor for the trigger-length calculation so that
// manual / first-pulse triggers stay visibly long).
constexpr uint32_t kDefaultClockMs  = 250;   // ≈ 240 BPM
constexpr uint32_t kMinClockMsForTrig = 50;  // gate-width floor for trig calc

// Hardcoded tuning sequence mirroring the Python `tuning_cv_sequence`:
// alternates 12 (~1V) and 24 (~2V) to verify octave spacing on the DAC.
inline constexpr uint16_t kTuningCvSequence[kMaxSteps] = {
  816, 1632, 816, 1632, 816, 1632, 816, 1632,
  816, 1632, 816, 1632, 816, 1632, 816, 1632,
};

// CV-output source. Was previously two independent bools (is_test, is_tuning)
// which made the menu show both as concurrent toggles while the engine
// silently prioritized one over the other — see the playground-v0.1.0 audit.
enum class CvSource : uint8_t {
  Normal = 0,   // play cv_sequence_[step] (random-mutating quantizer output)
  Test   = 1,   // play test_cv_sequence_[step] (ascending scale cycle)
  Tuning = 2,   // play kTuningCvSequence[step] (1V/2V alternation)
};

struct EngineParams {
  // Sequence length (clamped to [kMinSteps, kMaxSteps])
  int number_of_steps               = 16;

  // Probabilities are 0..100 (percent).
  int cv_probability_of_change      = 0;
  int trig_probability_of_change    = 0;

  // Gate width as a percentage of the measured clock gate-width (0..100).
  int trigger_length_percent        = 50;

  // Mode flags — direct port of the bool toggles in main.py.
  bool is_cv_erase                  = false;  // clamp every step's CV to scale root
  bool is_trig_erase                = false;  // force every step's trigger on

  // What the engine emits on the DAC each step.
  CvSource cv_source                = CvSource::Normal;

  // Current 12-bit DAC quantizer scale. Host owns this buffer (typically built
  // by rls::BuildScale into a host-side array on scale/octave/starting-note
  // change). The engine only reads it.
  const uint16_t* current_12bit_scale = nullptr;
  int             scale_length        = 0;
};

class SequencerEngine {
 public:
  void Init();
  void SetParams(const EngineParams& p) { params_ = p; }

  // Re-seed the engine's RNG. Default seed comes from std::random_device at
  // construction. Hosts can call this once with a high-entropy value
  // (e.g. ROSC + time_us_64 on Pico) to get genuinely unpredictable
  // randomness; on desktop this is rarely necessary.
  void Seed(uint64_t s) { rng_.seed(static_cast<uint32_t>(s)); }

  // Host calls on every detected edge of the clock input (rising = clock high).
  // The engine itself does not call any time function — `now_ms` is whatever
  // millisecond clock the host wants to use (e.g. time_us_64()/1000 on Pico,
  // glfwGetTime()*1000 on desktop). Returns true if the step advanced and the
  // host should latch the new DAC / trigger outputs from the accessors below.
  bool OnClockEdge(bool rising, uint32_t now_ms);

  // Host calls regularly (every loop iteration is fine). Turns the trigger
  // output off when its measured-length timer elapses. Returns true if the
  // trigger transitioned off this call.
  bool Tick(uint32_t now_ms);

  // ---- Read-only state for hosts ----
  // last_played_step() returns the step that just emitted its CV/trigger
  // (i.e. the step currently 'playing'), not the next-to-play step.
  int       last_played_step() const { return last_played_step_; }
  // Kept for compatibility with the v0.1 API; identical to last_played_step().
  int       current_step()     const { return last_played_step_; }
  uint16_t  last_dac()          const { return last_dac_; }
  bool      trig_active()       const { return trigger_active_; }
  uint32_t  last_clock_ms()     const { return clock_ms_; }
  const uint16_t* cv_sequence()      const { return cv_sequence_; }
  const uint8_t*  trigger_sequence() const { return trigger_sequence_; }

 private:
  EngineParams params_{};

  // Sequence state
  uint16_t cv_sequence_[kMaxSteps]      = {0};
  uint8_t  trigger_sequence_[kMaxSteps] = {0};
  uint16_t test_cv_sequence_[kMaxSteps] = {0};

  // `next_step_` is what OnClockEdge will play on the next rising edge.
  // `last_played_step_` is the step that just fired — what the OLED display
  // and the host's mini step grid want to highlight.
  int      next_step_                   = 0;
  int      last_played_step_            = 0;
  bool     step_changed_on_clock_pulse_ = false;

  // Clock timing
  uint32_t previous_clock_ticks_        = 0;
  uint32_t clock_ms_                    = kDefaultClockMs;

  // Trigger output timing
  bool     trigger_active_              = false;
  uint32_t trigger_start_ticks_         = 0;
  uint32_t trig_length_ms_              = 0;
  uint32_t ticks_to_trigger_off_        = 0;

  // Last DAC value the host should emit
  uint16_t last_dac_                    = 0;

  // RNG. std::random_device may return a fixed value on minimal embedded
  // toolchains; that's fine for the desktop playground. Hardware host should
  // call Seed() once at boot from a real entropy source.
  std::mt19937 rng_{std::random_device{}()};

  // Helpers
  bool     ProbabilityRoll(int probability_0_100);
  void     RebuildTestSequence();
  void     RandomlyChangeCurrentStepCv();
  void     RandomlyChangeCurrentStepTrigger();
  uint16_t ScaleRoot() const;
};

}  // namespace rls
