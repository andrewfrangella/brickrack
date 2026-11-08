// BrickRack - GPL-3.0-or-later.
//
// Quad Chiptune Oscillator, after Mutable Instruments Edges.
//
// Edges runs on an 8-bit AVR XMEGA (ATxmega32A4, 32 MHz). Its sources depend
// on avrlibx (hardware timers, AVR inline assembly in op.h and in
// digital_oscillator.cc, <avr/pgmspace.h>), so none of them compile on
// desktop. This wrapper:
//
//   * #includes the firmware's own data tables, edges/resources.cc (timer
//     count table, oscillator increments, bit-crusher increments, band-limited
//     triangle / NES triangle wavetables), unmodified;
//   * reproduces edges/timer_oscillator.{h,cc} (channels 1-3): the same pitch
//     -> timer period / prescaler computation with its 16-bit integer
//     quantisation, the LFO-range prescaler switch with hysteresis and the
//     same pulse widths (128, 85, 64, 32, 13 /256 or CV). The XMEGA timers run
//     in dual-slope PWM mode, f = 32 MHz / (prescaler * 2 * period); the
//     resulting (centre-aligned) pulse waves are rendered at 48 kHz as
//     band-limited (polyBLEP) pulses at exactly those frequencies and duty
//     cycles;
//   * reproduces edges/digital_oscillator.cc (channel 4) line by line with
//     the same 24-bit phase accumulator, table interpolation (the AVR asm
//     written out in C), NES LFSR noise, S&H noise and bit-crushed sine, at
//     the firmware's own DAC rate (32 MHz / 666 = 48048 Hz) through a
//     BlockAdapter, in 16-sample blocks;
//   * reproduces the parts of edges.cc / settings.h / ui.cc that are not
//     hardware-bound: ADC code -> pitch conversion (factory calibration
//     constants), semitone quantiser with hysteresis, MOD input as FM or as
//     PWM CV (OCTAL_ADC hardware revision), the button UI (short press cycles
//     the waveform, long press enters the channel menu: 1 = quantiser,
//     2 = sequencer, 3 = record a sequence), the per-channel note sequencer
//     ("arpeggiator", with the factory default sequences) and the LEDs.
//   * The XMOD switches are implemented as described in the manual: E1 hard-
//     syncs channel 2 to channel 1 (the firmware's sync switch: the channel 1
//     timer overflow restarts the channel 2 timer); E2 / E3 XOR channel 1 into
//     channel 2 / channel 3 (74AHC1G86 gates on the hardware).
//
// Not reproduced (hardware-bound): MIDI input and its polyphonic / chord /
// 3+1 modes, MIDI learn, the expander's sub-oscillator (channel 0), the
// calibration procedure (menu button 4; the module is always calibrated to
// 1 V/oct here), EEPROM storage (settings are hidden params / patch state).
//
// Voltage conventions: V/OCT 0 V = C4 with FREQ centred, FREQ +/-3 octaves,
// MOD +/-5 V = +/-5 octaves of FM (or 0..100 % pulse width in PWM-CV mode,
// 0 V = 50 %). Pulse outputs swing +/-5 V while the gate is high and rest at
// 0 V while it is low (the gate is a binary VCA). Channel 4 is the 12-bit DAC
// code scaled to +/-5 V (2048 = 0 V). V/OCT and GATE inputs are normalled
// 1 -> 2 -> 3 -> 4; MOD inputs are not.

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"

// --- Firmware data tables (edges/resources.cc, unmodified) -------------------
// Stand-ins for <avr/pgmspace.h>. EDGES_RESOURCES_H_ is pre-defined so the
// AVR-only resources.h is skipped. prog_uint16_t is 32 bits wide here because
// lut_res_bitcrusher_increments contains the literal 65536, which avr-gcc
// silently truncated to 0 in a uint16_t (C++17 rejects the narrowing); every
// read below casts back to uint16_t, which yields the same 0 as on the AVR.
typedef char prog_char;
typedef uint8_t prog_uint8_t;
typedef uint32_t prog_uint16_t;
typedef uint32_t prog_uint32_t;
#define PROGMEM
#define EDGES_RESOURCES_H_
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-const-variable"
#pragma clang diagnostic ignored "-Wzero-length-array"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include "edges/resources.cc"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#undef PROGMEM

namespace br {
namespace {

// From edges/resources.h.
const int WAV_RES_BANDLIMITED_TRIANGLE_0 = 0;
const int WAV_RES_BANDLIMITED_NES_TRIANGLE_0 = 8;
const int kLutSize = 97;  // LUT_RES_TIMER_COUNT_SIZE / LUT_RES_OSCILLATOR_INCREMENTS_SIZE

inline uint16_t lutTimerCount(int i) { return static_cast<uint16_t>(edges::lut_res_timer_count[i]); }
inline uint16_t lutOscIncrement(int i) {
  return static_cast<uint16_t>(edges::lut_res_oscillator_increments[i]);
}
inline uint16_t lutBitcrusher(int i) {
  return static_cast<uint16_t>(edges::lut_res_bitcrusher_increments[i]);
}

// avrlibx/utils/op.h helpers.
inline uint16_t U16U8MulShift8(uint16_t a, uint8_t b) {
  return static_cast<uint16_t>((static_cast<uint32_t>(a) * b) >> 8);
}
inline int16_t S16U12MulShift12(int16_t a, uint16_t b) {
  return static_cast<int16_t>((static_cast<int32_t>(a) * b) >> 12);
}

const int kNumChannels = 4;
const double kFCpu = 32000000.0;
const double kDacRate = kFCpu / 666.0;  // audio_rate_timer period 665

// --- edges/timer_oscillator.h ------------------------------------------------
enum PulseWidth {
  PULSE_WIDTH_50,
  PULSE_WIDTH_66,
  PULSE_WIDTH_75,
  PULSE_WIDTH_87,
  PULSE_WIDTH_95,
  PULSE_WIDTH_CV_CONTROLLED
};
const uint8_t kPulseWidths[] = {128, 85, 64, 32, 13, 10};
enum Prescaler { PRESCALER_CLK = 1, PRESCALER_CLK_8 = 8, PRESCALER_CLK_64 = 64 };

struct TimerOscillator {
  uint16_t value = 0;
  uint16_t period = 0;
  uint8_t cv_pw = 128;
  int prescaler = PRESCALER_CLK_8;

  void UpdatePitch(int16_t pitch, PulseWidth pulse_width) {
    if (pitch < (24 << 7) && prescaler != PRESCALER_CLK_64) prescaler = PRESCALER_CLK_64;
    if (pitch > (104 << 7) && prescaler != PRESCALER_CLK_8) prescaler = PRESCALER_CLK_8;
    UpdateTimerParameters(pitch, pulse_width);
  }

  void set_cv_pw(uint8_t pw) {
    if (pw > 250) pw = 250;
    if (pw < 6) pw = 6;
    cv_pw = pw;
  }

  // edges/timer_oscillator.cc
  void UpdateTimerParameters(int16_t pitch_in, PulseWidth pulse_width) {
    const int kOctave = 12 << 7;
    const int kFirstNoteNormalMode = 24 << 7;
    const int kFirstNoteLFOMode = -(12 << 7);
    int8_t shifts = 0;
    int pitch = pitch_in;
    pitch -= prescaler == PRESCALER_CLK_64 ? kFirstNoteLFOMode : kFirstNoteNormalMode;
    while (pitch < 0) pitch += kOctave;
    while (pitch >= kOctave) {
      pitch -= kOctave;
      ++shifts;
    }
    uint16_t index_integral = static_cast<uint16_t>(pitch >> 4);
    uint16_t index_fractional = static_cast<uint16_t>((pitch & 0xf) * 16);
    uint16_t count = lutTimerCount(index_integral);
    uint16_t next = lutTimerCount(index_integral + 1);
    count = static_cast<uint16_t>(
        count - U16U8MulShift8(static_cast<uint16_t>(count - next), static_cast<uint8_t>(index_fractional)));
    period = shifts >= 16 ? 0 : static_cast<uint16_t>(count >> shifts);
    uint8_t pw = pulse_width == PULSE_WIDTH_CV_CONTROLLED ? cv_pw : kPulseWidths[pulse_width];
    value = U16U8MulShift8(period, pw);
  }

  double frequency() const {
    return period ? kFCpu / (static_cast<double>(prescaler) * 2.0 * period) : 0.0;
  }
  float duty() const { return period ? static_cast<float>(value) / period : 0.f; }
};

// --- edges/digital_oscillator.{h,cc} -----------------------------------------
enum OscillatorShape {
  OSC_TRIANGLE,
  OSC_NES_TRIANGLE,
  OSC_PITCHED_NOISE,
  OSC_NES_NOISE_LONG,
  OSC_NES_NOISE_SHORT,
  OSC_SINE,
};

const int kAudioBlockSize = 16;

// InterpolateSample / InterpolateSample16 (AVR asm in the firmware): 9-bit
// table index from the top of the 16-bit phase, 7-bit fraction doubled.
inline uint16_t InterpolateSample16(const prog_uint8_t* table, uint16_t phase) {
  uint16_t index = phase >> 7;
  uint8_t frac = static_cast<uint8_t>((phase & 0x7f) << 1);
  return static_cast<uint16_t>(table[index + 1] * frac + table[index] * static_cast<uint8_t>(~frac));
}
inline uint8_t InterpolateSample(const prog_uint8_t* table, uint16_t phase) {
  return static_cast<uint8_t>(InterpolateSample16(table, phase) >> 8);
}

struct DigitalOscillator {
  OscillatorShape shape = OSC_TRIANGLE;
  int16_t pitch = 60 << 7;
  bool gate = true;
  uint8_t note = 60;
  uint32_t phase = 0;            // 24-bit (uint24_t: 16-bit integral + 8-bit fractional)
  uint32_t phase_increment = 0;  // 24-bit
  uint16_t sample = 0;
  uint16_t rng_state = 1;
  uint16_t aux_phase = 0;
  uint8_t cv_pw = 0;

  void UpdatePitch(int16_t p, OscillatorShape s) {
    pitch = p;
    // `sample` holds a 16-bit value for the sine (output >> 4) but a 12-bit
    // code for the noises (output as is): switching from SINE to a noise
    // replayed the stale 16-bit value as a code (up to +80 V) until the next
    // noise step. Restart it at mid-scale for the new shape.
    if (s != shape) sample = s == OSC_SINE ? 0x8000 : 0x0800;
    shape = s;
  }

  void ComputePhaseIncrement() {
    const int kOctave = 12 * 128;
    const int kPitchTableStart = 116 * 128;
    int ref_pitch = pitch - kPitchTableStart;
    uint8_t num_shifts = shape >= OSC_PITCHED_NOISE ? 0 : 1;
    while (ref_pitch < 0) {
      ref_pitch += kOctave;
      ++num_shifts;
    }
    // Guard: the firmware would read past the table for pitches above 128.
    if (ref_pitch > (kLutSize - 1) * 16 - 1) ref_pitch = (kLutSize - 1) * 16 - 1;
    uint16_t index_integral = static_cast<uint16_t>(ref_pitch >> 4);
    uint8_t index_fractional = static_cast<uint8_t>(ref_pitch << 4);
    uint16_t increment16 = lutOscIncrement(index_integral);
    uint16_t increment16_next = lutOscIncrement(index_integral + 1);
    uint32_t increment = static_cast<uint32_t>(static_cast<uint16_t>(
                             increment16 + U16U8MulShift8(static_cast<uint16_t>(increment16_next - increment16),
                                                          index_fractional)))
                         << 8;
    increment = num_shifts >= 24 ? 0 : increment >> num_shifts;
    note = static_cast<uint8_t>(static_cast<uint16_t>(pitch) >> 7);
    if (note < 12) note = 12;
    phase_increment = increment;
  }

  // Render(): one 16-sample block of 12-bit DAC codes.
  void Render(uint16_t* out) {
    if (gate) ComputePhaseIncrement();
    if (!gate) {
      for (int i = 0; i < kAudioBlockSize; ++i) out[i] = 2048;
      return;
    }
    switch (shape) {
      case OSC_TRIANGLE:
      case OSC_NES_TRIANGLE:
        RenderBandlimitedTriangle(out);
        break;
      case OSC_PITCHED_NOISE:
        RenderNoise(out);
        break;
      case OSC_NES_NOISE_LONG:
      case OSC_NES_NOISE_SHORT:
        RenderNoiseNES(out);
        break;
      case OSC_SINE:
        RenderSine(out);
        break;
    }
  }

  void RenderSine(uint16_t* out) {
    uint16_t aux_phase_increment = lutBitcrusher(cv_pw);
    for (int i = 0; i < kAudioBlockSize; ++i) {
      phase = (phase + phase_increment) & 0xffffff;
      aux_phase = static_cast<uint16_t>(aux_phase + aux_phase_increment);
      if (aux_phase < aux_phase_increment || !aux_phase_increment) {
        sample = InterpolateSample16(edges::wav_res_bandlimited_triangle_6,
                                     static_cast<uint16_t>(phase >> 8));
      }
      out[i] = sample >> 4;
    }
  }

  void RenderBandlimitedTriangle(uint16_t* out) {
    uint8_t n = static_cast<uint8_t>(note - 12);
    uint8_t balance_index = static_cast<uint8_t>((n << 4) | (n >> 4));  // U8Swap4
    uint8_t gain_2 = balance_index & 0xf0;
    uint8_t gain_1 = static_cast<uint8_t>(~gain_2);
    uint8_t wave_index = balance_index & 0xf;
    if (wave_index > 7) wave_index = 7;  // guard (only reachable with negative pitch)
    int base = shape == OSC_NES_TRIANGLE ? WAV_RES_BANDLIMITED_NES_TRIANGLE_0
                                         : WAV_RES_BANDLIMITED_TRIANGLE_0;
    const prog_uint8_t* wave_1 = edges::waveform_table[base + wave_index];
    wave_index = static_cast<uint8_t>(wave_index + 1 > 7 ? 7 : wave_index + 1);  // U8AddClip
    const prog_uint8_t* wave_2 = edges::waveform_table[base + wave_index];
    for (int i = 0; i < kAudioBlockSize; ++i) {
      phase = (phase + phase_increment) & 0xffffff;
      uint16_t p = static_cast<uint16_t>(phase >> 8);
      uint16_t s = static_cast<uint16_t>(InterpolateSample(wave_1, p) * gain_1 +
                                         InterpolateSample(wave_2, p) * gain_2);
      out[i] = s >> 4;
    }
  }

  void RenderNoiseNES(uint16_t* out) {
    uint16_t rng = rng_state;
    uint16_t s = sample;
    uint16_t inc_integral = static_cast<uint16_t>(phase_increment >> 8);
    for (int i = 0; i < kAudioBlockSize; ++i) {
      phase = (phase + phase_increment) & 0xffffff;
      if (static_cast<uint16_t>(phase >> 8) < inc_integral) {
        uint8_t tap = static_cast<uint8_t>(rng >> 1);
        if (shape == OSC_NES_NOISE_SHORT) tap >>= 5;
        uint8_t random_bit = (rng ^ tap) & 1;
        rng >>= 1;
        if (random_bit) {
          rng |= 0x4000;
          s = 0x0300;
        } else {
          s = 0x0cff;
        }
      }
      out[i] = s;
    }
    rng_state = rng;
    sample = s;
  }

  void RenderNoise(uint16_t* out) {
    uint16_t rng = rng_state;
    uint16_t s = sample;
    uint16_t inc_integral = static_cast<uint16_t>(phase_increment >> 8);
    for (int i = 0; i < kAudioBlockSize; ++i) {
      phase = (phase + phase_increment) & 0xffffff;
      if (static_cast<uint16_t>(phase >> 8) < inc_integral) {
        rng = static_cast<uint16_t>((rng >> 1) ^ (-(rng & 1) & 0xb400));
        s = rng & 0x0fff;
        s = static_cast<uint16_t>(512 + ((s * 3) >> 2));
      }
      out[i] = s;
    }
    rng_state = rng;
    sample = s;
  }
};

// --- edges/settings.h: ChannelData (factory defaults from settings.cc) ------
const int16_t kHysteresisThreshold = 32;

struct ChannelData {
  bool quantized = false;
  bool arpeggio = false;
  uint8_t num_arpeggio_steps = 3;
  int8_t arpeggio_step = 0;
  int16_t arpeggio_steps[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t offset = 1536;
  int16_t scale = 15360;
  int16_t fm_offset = -7680;

  int16_t dac_to_pitch(int16_t dac_code) const {
    int16_t pitch = S16U12MulShift12(scale, static_cast<uint16_t>(dac_code));
    pitch = static_cast<int16_t>(pitch + offset);
    if (pitch < 0) pitch = 0;
    if (quantized) {
      pitch = static_cast<int16_t>(pitch + 64);  // rounding
      pitch = static_cast<int16_t>(pitch & 0xff80);
    }
    if (pitch >= 16383) pitch = 16383;
    return pitch;
  }
  int16_t dac_to_fm(int16_t dac_code, bool cv_pw) const {
    return cv_pw ? 0
                 : static_cast<int16_t>(S16U12MulShift12(scale, static_cast<uint16_t>(dac_code)) +
                                        fm_offset);
  }
  int16_t arpeggio_offset() const {
    if (!arpeggio || arpeggio_step == 0 || arpeggio_step >= num_arpeggio_steps ||
        num_arpeggio_steps == 1) {
      return 0;
    }
    if (arpeggio_step == -1) return arpeggio_steps[num_arpeggio_steps - 2];
    return arpeggio_steps[arpeggio_step - 1];
  }
  void StartRecording() {
    num_arpeggio_steps = 1;
    arpeggio_step = -1;
    arpeggio = true;
  }
  void UpdateArpeggiatorStep(uint16_t root_dac_code, uint16_t dac_code) {
    if (num_arpeggio_steps > 1) {
      int16_t this_note = dac_to_pitch(static_cast<int16_t>(dac_code));
      int16_t root_note = dac_to_pitch(static_cast<int16_t>(root_dac_code));
      arpeggio_steps[num_arpeggio_steps - 2] = static_cast<int16_t>(this_note - root_note);
    }
  }
  void StopRecording() { arpeggio_step = 0; }
  bool NextArpeggiatorStep() {
    if (num_arpeggio_steps < 8) {
      ++num_arpeggio_steps;
      return true;
    }
    StopRecording();
    return false;
  }
};

// edges/ui.h modes.
enum UiMode { MODE_NORMAL, MODE_MENU, MODE_RECORDING };
const uint16_t kLongPressTime = 600;  // ms (Poll() runs at 1 kHz)

const char* kPulseNames[6] = {"50%", "66%", "75%", "87%", "95%", "PWM CV"};
const char* kShapeNames[6] = {"TRI", "NES TRI", "S&H NOISE", "LFSR L", "LFSR S", "SINE"};

// --- Band-limited pulse rendering for the timer channels -----------------
struct PulsePhase {
  float phase = 0.f;
  float inc = 0.f;
  float duty = 0.5f;
  bool level() const { return phase < duty * 0.5f || phase >= 1.f - duty * 0.5f; }
  // Time (in samples) to the next level change or wrap.
  float nextBoundary(float* b) const {
    float h = duty * 0.5f;
    float cand = 1.f;
    if (h > phase) cand = h;
    else if (1.f - h > phase) cand = 1.f - h;
    *b = cand;
    return inc > 0.f ? (cand - phase) / inc : 1e9f;
  }
};

class ChiptuneOsc : public Module {
 public:
  enum ParamIds {
    FREQ1_PARAM,
    FREQ2_PARAM,
    FREQ3_PARAM,
    FREQ4_PARAM,
    LEVEL1_PARAM,
    LEVEL2_PARAM,
    LEVEL3_PARAM,
    LEVEL4_PARAM,
    XMOD1_PARAM,
    XMOD2_PARAM,
    XMOD3_PARAM,
    WAVE1_PARAM,
    WAVE2_PARAM,
    WAVE3_PARAM,
    WAVE4_PARAM,
    SHAPE1_PARAM,  // hidden: waveform per channel
    SHAPE2_PARAM,
    SHAPE3_PARAM,
    SHAPE4_PARAM,
    QUANT1_PARAM,  // hidden: semitone quantiser
    QUANT2_PARAM,
    QUANT3_PARAM,
    QUANT4_PARAM,
    SEQ1_PARAM,  // hidden: sequencer ("arpeggio") on/off
    SEQ2_PARAM,
    SEQ3_PARAM,
    SEQ4_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    GATE1_INPUT,
    GATE2_INPUT,
    GATE3_INPUT,
    GATE4_INPUT,
    VOCT1_INPUT,
    VOCT2_INPUT,
    VOCT3_INPUT,
    VOCT4_INPUT,
    MOD1_INPUT,
    MOD2_INPUT,
    MOD3_INPUT,
    MOD4_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { OUT1_OUTPUT, OUT2_OUTPUT, OUT3_OUTPUT, OUT4_OUTPUT, MIX_OUTPUT, NUM_OUTPUTS };
  enum LightIds { LED1_LIGHT, LED2_LIGHT, LED3_LIGHT, LED4_LIGHT, NUM_LIGHTS };

  ChiptuneOsc() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < kNumChannels; ++i) {
      std::string n = std::to_string(i + 1);
      configParam(FREQ1_PARAM + i, -36.f, 36.f, 0.f, "Frequency " + n, "semitones");
      configParam(LEVEL1_PARAM + i, 0.f, 1.f, 0.5f, "Mix level " + n, "%", 100.f);
      configButton(WAVE1_PARAM + i, "Waveform " + n + " (hold: channel menu)");
      std::vector<std::string> labels;
      for (int k = 0; k < 6; ++k) labels.push_back(i < 3 ? kPulseNames[k] : kShapeNames[k]);
      configSwitch(SHAPE1_PARAM + i, 0.f, 5.f, 0.f, "Waveform " + n, labels).hidden = true;
      configSwitch(QUANT1_PARAM + i, 0.f, 1.f, 0.f, "Quantizer " + n, {"Off", "On"}).hidden = true;
      configSwitch(SEQ1_PARAM + i, 0.f, 1.f, 0.f, "Sequencer " + n, {"Off", "On"}).hidden = true;
      configInput(GATE1_INPUT + i, "Gate " + n);
      configInput(VOCT1_INPUT + i, "V/Oct " + n);
      configInput(MOD1_INPUT + i, i < 3 ? "FM / PWM " + n : "FM / bit-crush " + n);
      configOutput(OUT1_OUTPUT + i, "Channel " + n);
    }
    configSwitch(XMOD1_PARAM, 0.f, 1.f, 0.f, "XMOD: hard sync 2 to 1", {"Off", "On"});
    configSwitch(XMOD2_PARAM, 0.f, 1.f, 0.f, "XMOD: ring mod 1 x 2 (XOR)", {"Off", "On"});
    configSwitch(XMOD3_PARAM, 0.f, 1.f, 0.f, "XMOD: ring mod 1 x 3 (XOR)", {"Off", "On"});
    configOutput(MIX_OUTPUT, "Mix");

    adapter_.init(48000.f, static_cast<float>(kDacRate), kAudioBlockSize);
    initChannelData();
  }

  void onSampleRateChange(float sr) override {
    adapter_.init(sr, static_cast<float>(kDacRate), kAudioBlockSize);
  }

  void onReset() override {
    initChannelData();
    mode_ = MODE_NORMAL;
  }

  void saveState(StateMap& s) const override {
    for (int c = 0; c < kNumChannels; ++c) {
      std::string v = std::to_string(data_[c].num_arpeggio_steps);
      for (int k = 0; k < 7; ++k) v += "," + std::to_string(data_[c].arpeggio_steps[k]);
      s["seq" + std::to_string(c)] = v;
    }
  }
  void loadState(const StateMap& s) override {
    for (int c = 0; c < kNumChannels; ++c) {
      auto it = s.find("seq" + std::to_string(c));
      if (it == s.end()) continue;
      int v[8] = {0};
      int n = std::sscanf(it->second.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d", &v[0], &v[1], &v[2],
                          &v[3], &v[4], &v[5], &v[6], &v[7]);
      if (n != 8 || v[0] < 1 || v[0] > 8) continue;
      data_[c].num_arpeggio_steps = static_cast<uint8_t>(v[0]);
      data_[c].arpeggio_step = 0;
      for (int k = 0; k < 7; ++k) data_[c].arpeggio_steps[k] = static_cast<int16_t>(v[k + 1]);
    }
  }

  void process(const ProcessArgs& args) override {
    // Gate inputs (normalled 1 -> 2 -> 3 -> 4), 0.7 V threshold.
    uint8_t gate = 0;
    {
      float v = 0.f;
      for (int c = 0; c < kNumChannels; ++c) {
        const Port& in = inputs[static_cast<size_t>(GATE1_INPUT + c)];
        if (in.isConnected() || c == 0) v = in.getVoltage();
        gateTrigger_[c].process(v, 0.5f, 0.7f);
        if (gateTrigger_[c].isHigh()) gate |= static_cast<uint8_t>(1 << c);
      }
    }
    rawGate_ = gate;
    for (int c = 0; c < kNumChannels; ++c) {
      seqOn_[c] = params[static_cast<size_t>(SEQ1_PARAM + c)].value > 0.5f;
      data_[c].arpeggio = seqOn_[c];
      data_[c].quantized = params[static_cast<size_t>(QUANT1_PARAM + c)].value > 0.5f;
      effGate_[c] = (gate & (1 << c)) || seqOn_[c];
    }

    // ADC acquisition: each channel's pitch / MOD CV every 24 DAC samples.
    if (adcCounter_ <= 0) {
      adcCounter_ += 24.0 * args.sampleRate / kDacRate;
      updatePitches();
    }
    adcCounter_ -= 1.0;

    // UI poll at 1 kHz (every 48 DAC interrupts).
    pollCounter_ -= 1.0;
    if (pollCounter_ <= 0) {
      pollCounter_ += 48.0 * args.sampleRate / kDacRate;
      poll();
    }

    float out[4];
    renderPulses(args.sampleRate, out);

    // Channel 4: digital oscillator at the DAC rate.
    float d[1];
    adapter_.process(nullptr, d, [&](const float*, float* nout, int n) {
      digital_.gate = effGate_[3];
      uint16_t codes[kAudioBlockSize];
      digital_.Render(codes);
      for (int i = 0; i < n; ++i) nout[i] = (static_cast<float>(codes[i]) - 2048.f) / 2048.f;
    });
    out[3] = d[0] * 5.f;

    float mix = 0.f;
    for (int c = 0; c < kNumChannels; ++c) {
      outputs[static_cast<size_t>(OUT1_OUTPUT + c)].setVoltage(out[c]);
      mix += out[c] * params[static_cast<size_t>(LEVEL1_PARAM + c)].value;
    }
    // The mixer op-amp runs from +-12 V rails: four full-level channels sum to
    // 20 V. Soft-limit to about +-10.5 V (linear up to 8.5 V).
    const float a = std::fabs(mix);
    if (a > 8.5f) mix = std::copysign(8.5f + 2.f * std::tanh((a - 8.5f) * 0.5f), mix);
    outputs[MIX_OUTPUT].setVoltage(mix);

    if ((args.frame & 63) == 0) {
      for (int c = 0; c < kNumChannels; ++c) lights[static_cast<size_t>(c)] = (leds_ >> c) & 1 ? 1.f : 0.f;
    }
  }

  int shape(int c) const { return static_cast<int>(params[static_cast<size_t>(SHAPE1_PARAM + c)].value + 0.5f); }
  int uiMode() const { return mode_; }
  int editedChannel() const { return editedChannel_; }

 private:
  void initChannelData() {
    // edges/settings.cc: init_settings.
    const uint8_t steps[4] = {3, 3, 4, 5};
    const int16_t seq[4][7] = {{512, 1536, 0, 0, 0, 0, 0},
                               {512, 896, 0, 0, 0, 0, 0},
                               {512, 896, 1536, 0, 0, 0, 0},
                               {512, 896, 1536, 3072, 0, 0, 0}};
    for (int c = 0; c < kNumChannels; ++c) {
      data_[c] = ChannelData();
      data_[c].num_arpeggio_steps = steps[c];
      std::memcpy(data_[c].arpeggio_steps, seq[c], sizeof(seq[c]));
      previousCode_[c] = 0;
      previousPitch_[c] = 0;
    }
  }

  // Settings::dac_to_pitch (with quantiser hysteresis) + arpeggio offset.
  int16_t dacToPitch(int c, int16_t dac_code) {
    const ChannelData& data = data_[c];
    int16_t pitch = data.dac_to_pitch(dac_code);
    if (data.quantized) {
      int16_t delta = static_cast<int16_t>(dac_code - previousCode_[c]);
      if (delta < 0) delta = static_cast<int16_t>(-delta);
      if (delta < kHysteresisThreshold) {
        pitch = previousPitch_[c];
      } else {
        previousPitch_[c] = pitch;
        previousCode_[c] = dac_code;
      }
    }
    return static_cast<int16_t>(pitch + data.arpeggio_offset());
  }

  // Analog front end -> 12-bit ADC codes. The factory calibration maps 4096
  // codes to 120 semitones starting at note 12; V/OCT + FREQ are summed.
  void updatePitches() {
    float v = 0.f;
    for (int c = 0; c < kNumChannels; ++c) {
      const Port& in = inputs[static_cast<size_t>(VOCT1_INPUT + c)];
      if (in.isConnected() || c == 0) v = in.getVoltage();
      float note = 60.f + 12.f * v + params[static_cast<size_t>(FREQ1_PARAM + c)].value;
      cv_[c] = static_cast<uint16_t>(clampf(std::round((note - 12.f) * (4096.f / 120.f)), 0.f, 4095.f));
      float m = inputs[static_cast<size_t>(MOD1_INPUT + c)].getVoltage();
      cv_[c + 4] = static_cast<uint16_t>(clampf(std::round(2048.f + m * 409.6f), 0.f, 4095.f));
    }
    for (int c = 0; c < 3; ++c) {
      PulseWidth pw = static_cast<PulseWidth>(shape(c));
      int16_t pitch = static_cast<int16_t>(
          dacToPitch(c, static_cast<int16_t>(cv_[c])) +
          data_[c].dac_to_fm(static_cast<int16_t>(cv_[c + 4]), pw == PULSE_WIDTH_CV_CONTROLLED));
      timer_[c].set_cv_pw(static_cast<uint8_t>(cv_[c + 4] >> 4));
      timer_[c].UpdatePitch(pitch, pw);
    }
    OscillatorShape s = static_cast<OscillatorShape>(shape(3));
    // settings.dac_to_fm(3, ...) tests pw == PULSE_WIDTH_CV_CONTROLLED, which
    // is OSC_SINE for channel 4: MOD then drives the bit-crusher instead.
    int16_t pitch = static_cast<int16_t>(
        dacToPitch(3, static_cast<int16_t>(cv_[3])) +
        data_[3].dac_to_fm(static_cast<int16_t>(cv_[7]), s == OSC_SINE));
    digital_.UpdatePitch(pitch, s);
    digital_.cv_pw = static_cast<uint8_t>(cv_[7] >> 4);
  }

  // Ui::Poll() at 1 kHz.
  void poll() {
    ++ledsPwmCounter_;
    ++switchTimeCounter_;
    // Sequencer clock: new notes on the gate inputs (edges.cc main loop).
    for (int c = 0; c < kNumChannels; ++c) {
      uint8_t mask = static_cast<uint8_t>(1 << c);
      if ((rawGate_ & mask) && !(uiGate_ & mask)) {
        ChannelData& d = data_[c];
        if (d.arpeggio_step != -1) {
          ++d.arpeggio_step;
          if (d.arpeggio_step >= d.num_arpeggio_steps) d.arpeggio_step = 0;
        }
      }
    }
    uiGate_ = rawGate_;

    uint8_t leds = 0;
    switch (mode_) {
      case MODE_NORMAL:
        leds = uiGate_;
        break;
      case MODE_MENU:
        if (data_[editedChannel_].quantized) leds |= 0x1;
        if (data_[editedChannel_].arpeggio) leds |= 0x2;
        leds |= (ledsPwmCounter_ & 128) ? 0x4 : 0x8;
        break;
      case MODE_RECORDING:
        leds |= static_cast<uint8_t>(1 << ((data_[editedChannel_].num_arpeggio_steps - 1) & 0x03));
        break;
    }
    leds_ = leds;

    // Scan switches (pressed = 0, as with the pull-ups on the hardware).
    for (int i = 0; i < kNumChannels; ++i) {
      bool pressed = params[static_cast<size_t>(WAVE1_PARAM + i)].value > 0.5f;
      debounce_[i] = static_cast<uint8_t>((debounce_[i] << 1) | (pressed ? 0 : 1));
    }
    for (int i = 0; i < kNumChannels; ++i) {
      if (debounce_[i] == 0xfe) switchTimeCounter_ = 0;
      if (debounce_[i] == 0x00 && switchTimeCounter_ == kLongPressTime) onSwitchHeld(i);
      if (debounce_[i] == 0x01 && switchTimeCounter_ < kLongPressTime) onSwitchReleased(i);
    }

    if (mode_ == MODE_RECORDING) {
      ChannelData& d = data_[editedChannel_];
      if (d.num_arpeggio_steps == 1) rootCv_ = cv_[editedChannel_];
      d.UpdateArpeggiatorStep(rootCv_, cv_[editedChannel_]);
    }
  }

  void onSwitchHeld(int index) {
    if (mode_ == MODE_MENU) {
      mode_ = MODE_NORMAL;
    } else {
      mode_ = MODE_MENU;
      editedChannel_ = index;
    }
  }

  void setParam(int id, float v) { params[static_cast<size_t>(id)].set(v); }

  void onSwitchReleased(int index) {
    switch (mode_) {
      case MODE_MENU:
        switch (index) {
          case 0:
            setParam(QUANT1_PARAM + editedChannel_,
                     data_[editedChannel_].quantized ? 0.f : 1.f);
            break;
          case 1:
            setParam(SEQ1_PARAM + editedChannel_, data_[editedChannel_].arpeggio ? 0.f : 1.f);
            break;
          case 2:
            mode_ = MODE_RECORDING;
            rootCv_ = cv_[editedChannel_];
            data_[editedChannel_].StartRecording();
            setParam(SEQ1_PARAM + editedChannel_, 1.f);
            break;
          case 3:
            // Calibration: not needed (always calibrated), excluded.
            break;
        }
        break;
      case MODE_NORMAL: {
        int v = shape(index) + 1;
        if (v > PULSE_WIDTH_CV_CONTROLLED) v = PULSE_WIDTH_50;
        setParam(SHAPE1_PARAM + index, static_cast<float>(v));
        break;
      }
      case MODE_RECORDING: {
        ChannelData& d = data_[editedChannel_];
        int current_step = d.num_arpeggio_steps - 1;
        if (index == (current_step & 3)) {
          d.StopRecording();
          mode_ = MODE_NORMAL;
        } else if (index == ((current_step + 1) & 3)) {
          if (!d.NextArpeggiatorStep()) mode_ = MODE_NORMAL;
        }
        break;
      }
    }
  }

  // Channels 1-3: centre-aligned pulses from the dual-slope timers, with
  // hard sync (E1) and XOR cross-modulation (E2, E3), band-limited with
  // polyBLEP. Output is delayed by one sample.
  void renderPulses(float sampleRate, float* out) {
    const bool sync = params[XMOD1_PARAM].value > 0.5f;
    const bool xor2 = params[XMOD2_PARAM].value > 0.5f;
    const bool xor3 = params[XMOD3_PARAM].value > 0.5f;
    for (int c = 0; c < 3; ++c) {
      double f = timer_[c].frequency();
      osc_[c].inc = static_cast<float>(std::min(f / sampleRate, 0.45));
      osc_[c].duty = timer_[c].duty();
    }
    const bool g1 = effGate_[0], g2 = effGate_[1], g3 = effGate_[2];
    const bool active[3] = {g1, g2 || (xor2 && g1), g3 || (xor3 && g1)};
    auto voltages = [&](float* v) {
      // PWM compare value is 0 while the gate is low: the pin stays low.
      bool p1 = g1 && osc_[0].level();
      bool p2 = g2 && osc_[1].level();
      bool p3 = g3 && osc_[2].level();
      bool pins[3] = {p1, xor2 ? (p2 != p1) : p2, xor3 ? (p3 != p1) : p3};
      for (int k = 0; k < 3; ++k) v[k] = active[k] ? (pins[k] ? 5.f : -5.f) : 0.f;
    };

    float level[3];
    voltages(level);
    float next[3] = {0.f, 0.f, 0.f};
    for (int k = 0; k < 3; ++k) {
      // Gate changes happen on the sample boundary.
      float delta = level[k] - lastLevel_[k];
      if (delta != 0.f) pending_[k] += 0.5f * delta;
    }

    float t = 0.f;
    for (int guard = 0; guard < 64; ++guard) {
      float dtMin = 1e9f, bound = 0.f;
      int which = -1;
      for (int c = 0; c < 3; ++c) {
        float b;
        float dt = osc_[c].nextBoundary(&b);
        if (dt < dtMin) {
          dtMin = dt;
          which = c;
          bound = b;
        }
      }
      if (which < 0 || t + dtMin >= 1.f) {
        float rest = 1.f - t;
        for (int c = 0; c < 3; ++c) osc_[c].phase += osc_[c].inc * rest;
        break;
      }
      for (int c = 0; c < 3; ++c) {
        if (c == which) {
          osc_[c].phase = bound;
        } else {
          osc_[c].phase += osc_[c].inc * dtMin;
          if (osc_[c].phase >= 1.f) osc_[c].phase -= 1.f;
        }
      }
      t += dtMin;
      if (bound >= 1.f) {
        osc_[which].phase = 0.f;
        if (which == 0 && sync) osc_[1].phase = 0.f;  // TCD0 overflow restarts channel 2
      }
      float v[3];
      voltages(v);
      for (int k = 0; k < 3; ++k) {
        float delta = v[k] - level[k];
        if (delta != 0.f) {
          pending_[k] += 0.5f * delta * (1.f - t) * (1.f - t);
          next[k] -= 0.5f * delta * t * t;
          level[k] = v[k];
        }
      }
    }
    for (int k = 0; k < 3; ++k) {
      out[k] = pending_[k];
      pending_[k] = level[k] + next[k];
      lastLevel_[k] = level[k];
    }
  }

  TimerOscillator timer_[3];
  PulsePhase osc_[3];
  float pending_[3] = {0.f, 0.f, 0.f};
  float lastLevel_[3] = {0.f, 0.f, 0.f};
  DigitalOscillator digital_;
  BlockAdapter<0, 1> adapter_;
  ChannelData data_[kNumChannels];
  int16_t previousCode_[kNumChannels] = {0, 0, 0, 0};
  int16_t previousPitch_[kNumChannels] = {0, 0, 0, 0};
  uint16_t cv_[2 * kNumChannels] = {0, 0, 0, 0, 2048, 2048, 2048, 2048};
  SchmittTrigger gateTrigger_[kNumChannels];
  bool effGate_[kNumChannels] = {false, false, false, false};
  bool seqOn_[kNumChannels] = {false, false, false, false};
  uint8_t rawGate_ = 0;
  uint8_t uiGate_ = 0;
  uint8_t leds_ = 0;
  uint8_t debounce_[kNumChannels] = {0xff, 0xff, 0xff, 0xff};
  uint16_t ledsPwmCounter_ = 0;
  uint16_t switchTimeCounter_ = 0;
  uint16_t rootCv_ = 0;
  int mode_ = MODE_NORMAL;
  int editedChannel_ = 0;
  double adcCounter_ = 0.0;
  double pollCounter_ = 0.0;
};

void buildPanel(PanelSpec& p) {
  p.hp = 20;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  p.title = "CHIPTUNE OSC";
  using M = ChiptuneOsc;
  const uint32_t dark = 0xFF5E5E60;
  const uint32_t teal = 0xFF168A8C;

  // Positions measured on the original panel (0.298 mm/px).
  const float xGate = 11.3f, xVoct = 24.1f, xMod = 35.8f, xFreq = 48.6f, xXmod = 63.2f,
              xOut = 77.5f, xLevel = 90.6f;
  const float rows[4] = {25.0f, 45.6f, 65.9f, 86.4f};
  const float hy = 15.2f;
  p.label(xGate, hy, "GATE", 2.8f);
  p.label(xVoct, hy, "V/OCT", 2.8f);
  p.label(xMod, hy, "MOD", 2.8f);
  p.label(xFreq, hy, "FREQ", 2.8f);
  p.label(xXmod, hy, "XMOD", 2.8f);
  p.box(84.6f, hy - 2.0f, 12.0f, 4.0f, teal, 1.5f);
  p.label(xLevel, hy, "LEVEL", 2.6f, kAlignCenter, 0xFFFFFFFF);

  // Output column: dark boxes linked by a dotted line down to the mix.
  for (float y = rows[0]; y < rows[3]; y += 1.6f) p.line(xOut, y, xOut, y + 0.7f, 0.35f, p.ink);
  for (float y = rows[3]; y < 109.5f; y += 1.6f) p.line(xOut, y, xOut, y + 0.7f, 0.35f, p.ink);
  for (float x = xOut; x < 85.f; x += 1.6f) p.line(x, 110.9f, x + 0.7f, 110.9f, 0.35f, p.ink);
  p.box(72.0f, 11.9f, 11.0f, 18.6f, dark, 1.2f);
  p.label(xOut, hy, "OUT", 2.8f, kAlignCenter, 0xFFFFFFFF);
  for (int i = 1; i < 4; ++i) p.box(72.0f, rows[i] - 5.5f, 11.0f, 11.0f, dark, 1.2f);

  const char* nums[4] = {"1", "2", "3", "4"};
  for (int i = 0; i < 4; ++i) {
    float y = rows[i];
    p.label(4.2f, y, nums[i], 3.0f);
    p.input(M::GATE1_INPUT + i, xGate, y);
    p.input(M::VOCT1_INPUT + i, xVoct, y);
    p.input(M::MOD1_INPUT + i, xMod, y);
    p.knob(M::FREQ1_PARAM + i, xFreq, y, 8.5f);
    if (i < 3) p.toggleSwitch(M::XMOD1_PARAM + i, xXmod, y);
    p.output(M::OUT1_OUTPUT + i, xOut, y);
    p.knob(M::LEVEL1_PARAM + i, xLevel, y, 8.0f);
    if (i < 3) {
      // Normalling arrows (gate and V/OCT chain to the next channel).
      for (float x : {xGate, xVoct}) {
        for (float yy = y + 4.0f; yy < rows[i + 1] - 4.6f; yy += 1.4f)
          p.line(x, yy, x, yy + 0.6f, 0.3f, p.ink);
      }
    }
  }
  const char* xmodNames[3] = {"SYNC", "1x2", "1x3"};
  for (int i = 0; i < 3; ++i) p.label(xXmod, rows[i] + 6.2f, xmodNames[i], 2.4f);

  p.line(2.4f, 98.0f, 56.6f, 98.0f, 0.6f, 0xFFB8B8B8);
  const float bx[4] = {17.0f, 25.3f, 33.7f, 42.0f};
  for (int i = 0; i < 4; ++i) {
    p.light(M::LED1_LIGHT + i, bx[i], 102.6f, 0xFFFF3B30, 2.4f);
    p.button(M::WAVE1_PARAM + i, bx[i], 109.6f, 5.5f);
    p.label(bx[i] - 3.4f, 102.6f, nums[i], 2.4f);
  }
  p.label(47.4f, 109.6f, "WAVEFORM SELECT", 2.7f, kAlignLeft);
  // Current waveform per channel (shown with icons on the hardware panel).
  p.display(12.8f, 113.2f, 33.4f, 3.6f, [bx](Canvas& c, float x, float y, float w, float h,
                                             float s, const Module* m) {
    auto* e = static_cast<const ChiptuneOsc*>(m);
    float fs = std::max(13.f, 2.3f * s);
    for (int i = 0; i < 4; ++i) {
      int sh = e->shape(i);
      if (sh < 0) sh = 0;
      if (sh > 5) sh = 5;
      const char* name = i < 3 ? kPulseNames[sh] : kShapeNames[sh];
      float cx = x + (bx[i] - 12.8f) * s;
      c.text(cx, y + h * 0.5f, name, fs, 0xFF168A8C, kAlignCenter);
    }
  });
  p.display(47.4f, 113.0f, 22.0f, 3.6f, [](Canvas& c, float x, float y, float w, float h, float s,
                                           const Module* m) {
    auto* e = static_cast<const ChiptuneOsc*>(m);
    const char* txt = nullptr;
    char buf[24];
    if (e->uiMode() == MODE_MENU) {
      std::snprintf(buf, sizeof(buf), "MENU CH %d", e->editedChannel() + 1);
      txt = buf;
    } else if (e->uiMode() == MODE_RECORDING) {
      std::snprintf(buf, sizeof(buf), "REC CH %d", e->editedChannel() + 1);
      txt = buf;
    }
    if (txt) c.text(x, y + h * 0.5f, txt, std::max(13.f, 2.5f * s), 0xFFC8414B, kAlignLeft);
  });

  p.box(84.9f, 100.4f, 11.2f, 15.8f, dark, 1.2f);
  p.label(xLevel, 103.4f, "MIX", 2.8f, kAlignCenter, 0xFFFFFFFF);
  p.output(M::MIX_OUTPUT, xLevel, 110.9f);
  p.line(2.4f, 119.2f, 99.2f, 119.2f, 0.6f, 0xFFB8B8B8);
}

Model model = [] {
  Model m;
  m.slug = "chiptune-osc";
  m.name = "Quad Chiptune Oscillator";
  m.shortName = "CHIPTUNE OSC";
  m.basedOn = "Edges";
  m.category = Category::Oscillator;
  m.description = "Three timer-based pulse oscillators and a NES-style triangle/noise voice.";
  m.create = [] { return new ChiptuneOsc(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
