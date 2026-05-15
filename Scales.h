// Scales.h
// C++ port of Software/lib/mcp4725_musical_scales.py
//
// Scale intervals authored by Hector Miller-Bakewell (musical-scales library,
// MIT — see https://github.com/hmillerbakewell/musical-scales).
//
// "12-bit DAC value = note_number * 68" matches the MCP4725 calibration in
// the existing MicroPython firmware (1V/oct at the module's reference Vcc).

#pragma once
#include <cstdint>
#include <cstddef>

namespace seq {

constexpr int kMcp4725Multiplier = 68;
constexpr int kMaxScaleNotes     = 64;        // way more than any 5-oct scale needs

struct ScaleInterval {
  const char*    name;
  const uint8_t* steps;
  uint8_t        step_count;
};

namespace detail {
  static constexpr uint8_t k_acoustic[]                   = {2, 2, 2, 1, 2, 1, 2};
  static constexpr uint8_t k_aeolian[]                    = {2, 1, 2, 2, 1, 2, 2};
  static constexpr uint8_t k_algerian[]                   = {2, 1, 3, 1, 1, 3, 1, 2, 1, 2};
  static constexpr uint8_t k_augmented[]                  = {3, 1, 3, 1, 3, 1};
  static constexpr uint8_t k_bebop_dominant[]             = {2, 2, 1, 2, 2, 1, 1, 1};
  static constexpr uint8_t k_blues[]                      = {3, 2, 1, 1, 3, 2};
  static constexpr uint8_t k_chromatic[]                  = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  static constexpr uint8_t k_dorian[]                     = {2, 1, 2, 2, 2, 1, 2};
  static constexpr uint8_t k_double_harmonic[]            = {1, 3, 1, 2, 1, 3, 1};
  static constexpr uint8_t k_enigmatic[]                  = {1, 3, 2, 2, 2, 1, 1};
  static constexpr uint8_t k_flamenco[]                   = {1, 3, 1, 2, 1, 3, 1};
  static constexpr uint8_t k_half_diminished[]            = {2, 1, 2, 1, 2, 2, 2};
  static constexpr uint8_t k_harmonic_major[]             = {2, 2, 1, 2, 1, 3, 1};
  static constexpr uint8_t k_harmonic_minor[]             = {2, 1, 2, 2, 1, 3, 1};
  static constexpr uint8_t k_harmonics[]                  = {3, 1, 1, 2, 2, 3};
  static constexpr uint8_t k_hijaroshi[]                  = {4, 2, 1, 4, 1};
  static constexpr uint8_t k_hungarian_major[]            = {3, 1, 2, 1, 2, 1, 2};
  static constexpr uint8_t k_hungarian_minor[]            = {2, 1, 3, 1, 1, 3, 1};
  static constexpr uint8_t k_in[]                         = {1, 4, 2, 1, 4};
  static constexpr uint8_t k_insen[]                      = {1, 4, 2, 3, 2};
  static constexpr uint8_t k_ionian[]                     = {2, 2, 1, 2, 2, 2, 1};  // "major"
  static constexpr uint8_t k_iwato[]                      = {1, 4, 1, 4, 2};
  static constexpr uint8_t k_locrian[]                    = {1, 2, 2, 1, 2, 2, 2};
  static constexpr uint8_t k_locrian_major[]              = {2, 2, 1, 1, 2, 2, 2};
  static constexpr uint8_t k_lydian[]                     = {2, 2, 2, 1, 2, 2, 1};
  static constexpr uint8_t k_lydian_augmented[]           = {2, 2, 2, 2, 1, 2, 1};
  static constexpr uint8_t k_melodic_minor_asc[]          = {2, 1, 2, 2, 2, 2, 1};
  static constexpr uint8_t k_melodic_minor_desc[]         = {2, 1, 2, 2, 2, 2, 1};
  static constexpr uint8_t k_mixolydian[]                 = {2, 2, 1, 2, 2, 1, 2};
  static constexpr uint8_t k_neapolitan_major[]           = {1, 2, 2, 2, 2, 2, 1};
  static constexpr uint8_t k_neapolitan_minor[]           = {1, 2, 2, 2, 1, 3, 1};
  static constexpr uint8_t k_octatonic_c_d[]              = {2, 1, 2, 1, 2, 1, 2, 1};
  static constexpr uint8_t k_octatonic_c_cs[]             = {1, 2, 1, 2, 1, 2, 1};
  static constexpr uint8_t k_pentatonic_major[]           = {2, 2, 3, 2, 3};
  static constexpr uint8_t k_pentatonic_minor[]           = {3, 2, 2, 3, 2};
  static constexpr uint8_t k_persian[]                    = {1, 3, 1, 1, 2, 3, 1};
  static constexpr uint8_t k_phrygian[]                   = {1, 2, 2, 2, 1, 2, 2};
  static constexpr uint8_t k_phrygian_dominant[]          = {1, 3, 1, 2, 1, 2, 2};
  static constexpr uint8_t k_prometheus[]                 = {2, 2, 2, 3, 1, 2};
  static constexpr uint8_t k_romani[]                     = {2, 1, 3, 1, 1, 2, 2};
  static constexpr uint8_t k_super_locrian[]              = {1, 2, 1, 2, 2, 2, 2};
  static constexpr uint8_t k_tritone[]                    = {1, 3, 2, 1, 3, 2};
  static constexpr uint8_t k_two_semitone_tritone[]       = {1, 1, 4, 1, 1, 4};
  static constexpr uint8_t k_ukranian_dorian[]            = {2, 1, 3, 1, 2, 1, 2};
  static constexpr uint8_t k_whole_tone[]                 = {2, 2, 2, 2, 2, 2};
  static constexpr uint8_t k_yo[]                         = {3, 2, 2, 3, 2};
}

// Ordered by category so the SingleSelect picker shows related scales
// together. Section boundaries are tracked in kScaleSectionStarts /
// kScaleSectionNames below — used by the menu UI to draw section
// headers between groups.
inline constexpr ScaleInterval kScales[] = {
  // ── Common ── (everyday tonal scales)
  {"major",                    detail::k_ionian,              sizeof(detail::k_ionian)},
  {"minor",                    detail::k_aeolian,             sizeof(detail::k_aeolian)},
  {"dorian",                   detail::k_dorian,              sizeof(detail::k_dorian)},
  {"mixolydian",               detail::k_mixolydian,          sizeof(detail::k_mixolydian)},
  {"pentatonic major",         detail::k_pentatonic_major,    sizeof(detail::k_pentatonic_major)},
  {"pentatonic minor",         detail::k_pentatonic_minor,    sizeof(detail::k_pentatonic_minor)},
  {"blues",                    detail::k_blues,               sizeof(detail::k_blues)},
  {"harmonic minor",           detail::k_harmonic_minor,      sizeof(detail::k_harmonic_minor)},
  {"melodic minor ascending",  detail::k_melodic_minor_asc,   sizeof(detail::k_melodic_minor_asc)},
  // ── Modal ── (the seven church modes + close relatives)
  {"ionian",                   detail::k_ionian,              sizeof(detail::k_ionian)},
  {"aeolian",                  detail::k_aeolian,             sizeof(detail::k_aeolian)},
  {"phrygian",                 detail::k_phrygian,            sizeof(detail::k_phrygian)},
  {"lydian",                   detail::k_lydian,              sizeof(detail::k_lydian)},
  {"locrian",                  detail::k_locrian,             sizeof(detail::k_locrian)},
  {"acoustic",                 detail::k_acoustic,            sizeof(detail::k_acoustic)},
  {"melodic minor descending", detail::k_melodic_minor_desc,  sizeof(detail::k_melodic_minor_desc)},
  // ── Jazz / bebop ──
  {"bebop dominant",           detail::k_bebop_dominant,      sizeof(detail::k_bebop_dominant)},
  {"lydian augmented",         detail::k_lydian_augmented,    sizeof(detail::k_lydian_augmented)},
  {"locrian major",            detail::k_locrian_major,       sizeof(detail::k_locrian_major)},
  {"super locrian",            detail::k_super_locrian,       sizeof(detail::k_super_locrian)},
  {"half-diminished",          detail::k_half_diminished,     sizeof(detail::k_half_diminished)},
  {"prometheus",               detail::k_prometheus,          sizeof(detail::k_prometheus)},
  {"harmonic major",           detail::k_harmonic_major,      sizeof(detail::k_harmonic_major)},
  {"phrygian dominant",        detail::k_phrygian_dominant,   sizeof(detail::k_phrygian_dominant)},
  // ── Exotic / world ──
  {"algerian",                 detail::k_algerian,            sizeof(detail::k_algerian)},
  {"double harmonic",          detail::k_double_harmonic,     sizeof(detail::k_double_harmonic)},
  {"enigmatic",                detail::k_enigmatic,           sizeof(detail::k_enigmatic)},
  {"flamenco",                 detail::k_flamenco,            sizeof(detail::k_flamenco)},
  {"harmonics",                detail::k_harmonics,           sizeof(detail::k_harmonics)},
  {"hijaroshi",                detail::k_hijaroshi,           sizeof(detail::k_hijaroshi)},
  {"hungarian major",          detail::k_hungarian_major,     sizeof(detail::k_hungarian_major)},
  {"hungarian minor",          detail::k_hungarian_minor,     sizeof(detail::k_hungarian_minor)},
  {"in",                       detail::k_in,                  sizeof(detail::k_in)},
  {"insen",                    detail::k_insen,               sizeof(detail::k_insen)},
  {"iwato",                    detail::k_iwato,               sizeof(detail::k_iwato)},
  {"neapolitan major",         detail::k_neapolitan_major,    sizeof(detail::k_neapolitan_major)},
  {"neapolitan minor",         detail::k_neapolitan_minor,    sizeof(detail::k_neapolitan_minor)},
  {"persian",                  detail::k_persian,             sizeof(detail::k_persian)},
  {"romani",                   detail::k_romani,              sizeof(detail::k_romani)},
  {"ukranian dorian",          detail::k_ukranian_dorian,     sizeof(detail::k_ukranian_dorian)},
  {"yo",                       detail::k_yo,                  sizeof(detail::k_yo)},
  // ── Symmetric ── (equal-spaced)
  {"chromatic",                detail::k_chromatic,           sizeof(detail::k_chromatic)},
  {"whole-tone scale",         detail::k_whole_tone,          sizeof(detail::k_whole_tone)},
  {"octatonic c-c#",           detail::k_octatonic_c_cs,      sizeof(detail::k_octatonic_c_cs)},
  {"octatonic c-d",            detail::k_octatonic_c_d,       sizeof(detail::k_octatonic_c_d)},
  {"augmented",                detail::k_augmented,           sizeof(detail::k_augmented)},
  {"tritone",                  detail::k_tritone,             sizeof(detail::k_tritone)},
  {"two-semitone tritone",     detail::k_two_semitone_tritone,sizeof(detail::k_two_semitone_tritone)},
};
inline constexpr int kNumScales = sizeof(kScales) / sizeof(kScales[0]);

// Section boundaries — must match the order above. Each entry is the
// FIRST kScales index of that section. kScaleSectionNames is parallel.
inline constexpr int kScaleSectionCount = 5;
inline constexpr int         kScaleSectionStarts[kScaleSectionCount] = {0, 9, 16, 24, 40};
inline constexpr const char* kScaleSectionNames [kScaleSectionCount] = {
  "Common", "Modal", "Jazz", "Exotic", "Symmetric"
};

// Returns the section name for any scale index. Returns "" if out of range.
inline const char* ScaleSectionForIndex(int idx) {
  if (idx < 0 || idx >= kNumScales) return "";
  for (int s = kScaleSectionCount - 1; s >= 0; --s) {
    if (idx >= kScaleSectionStarts[s]) return kScaleSectionNames[s];
  }
  return "";
}

// Build a 12-bit DAC sequence walking the given interval pattern from
// `starting_note`, looping the pattern for `octaves` octaves. Mirrors
// get_scale_of_12_bit_values() in the Python source.
//
// `out` must hold at least kMaxScaleNotes entries; returns the number of
// notes written.
inline int BuildScale(const ScaleInterval& interval,
                      int starting_note,
                      int octaves,
                      uint16_t* out) {
  int count = 0;
  int note  = starting_note;
  out[count++] = static_cast<uint16_t>(note * kMcp4725Multiplier);
  for (int o = 0; o < octaves; ++o) {
    for (int s = 0; s < interval.step_count && count < kMaxScaleNotes; ++s) {
      note += interval.steps[s];
      out[count++] = static_cast<uint16_t>(note * kMcp4725Multiplier);
    }
  }
  return count;
}

inline int FindScaleIndex(const char* name) {
  for (int i = 0; i < kNumScales; ++i) {
    const char* a = kScales[i].name;
    const char* b = name;
    while (*a && *b && *a == *b) { ++a; ++b; }
    if (*a == 0 && *b == 0) return i;
  }
  return -1;
}

}  // namespace seq
