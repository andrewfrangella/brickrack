// BrickRack - GPL-3.0-or-later.
//
// Topographic Drum Sequencer, after Mutable Instruments Grids.
//
// Grids is an 8-bit AVR (ATmega88) firmware. Its sources depend on avrlib
// (AVR inline assembly in avrlib/op.h, <avr/pgmspace.h>, <avr/eeprom.h>, a
// static-only PatternGenerator bound to the hardware), so they cannot be
// compiled on desktop without modification. This wrapper therefore:
//
//   * #includes the firmware's own data tables, grids/resources.cc (drum map
//     nodes, Euclidean patterns, tempo phase increments), unmodified, with a
//     few PROGMEM typedefs standing in for <avr/pgmspace.h>;
//   * reproduces grids/pattern_generator.cc, grids/clock.cc and the timer
//     interrupt / pot scanning of grids/grids.cc line by line as a per-instance
//     class, with the same 8-bit integer arithmetic (the avrlib asm helpers
//     U8Mix, U8U8MulShift8, ... are written out in C with identical results)
//     and the same avrlib::Random LFSR.
//
// The firmware's control interrupt runs at F_CPU / 32 / 78 = 8012.8 Hz; it is
// run here at exactly that rate from the 48 kHz engine.
//
// Differences from the hardware (hardware-bound features):
//   * MIDI clock input is not emulated.
//   * The settings mode (long press on TAP + turning knobs) is replaced by
//     hidden parameters and a module-menu item (Y button).
//   * Patching the CLOCK input selects the external clock even when the TEMPO
//     knob is not fully counter-clockwise (on hardware the knob must be at its
//     minimum, which still works here too).
//   * The power-on "factory testing" retrigger-on-reset quirk is not emulated.

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"

// --- Firmware data tables (grids/resources.cc, unmodified) -------------------
// Stand-ins for <avr/pgmspace.h>; flash data is plain const data on desktop.
// GRIDS_RESOURCES_H_ is pre-defined so that the AVR-only resources.h (which
// includes <avr/pgmspace.h> and avrlib/resources_manager.h) is skipped.
typedef char prog_char;
typedef uint8_t prog_uint8_t;
typedef uint16_t prog_uint16_t;
typedef uint32_t prog_uint32_t;
#define PROGMEM
#define GRIDS_RESOURCES_H_
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-const-variable"
#pragma clang diagnostic ignored "-Wzero-length-array"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include "grids/resources.cc"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#undef PROGMEM

namespace br {
namespace {

// --- avrlib/op.h helpers (AVR asm in the original, same results) ------------
inline uint8_t U8Mix(uint8_t a, uint8_t b, uint8_t balance) {
  return static_cast<uint8_t>((b * balance + a * (255 - balance)) >> 8);
}
inline uint8_t U8U8MulShift8(uint8_t a, uint8_t b) {
  return static_cast<uint8_t>((a * b) >> 8);
}
inline uint16_t U8U8Mul(uint8_t a, uint8_t b) { return static_cast<uint16_t>(a * b); }

// --- grids/pattern_generator.h constants ------------------------------------
const uint8_t kNumParts = 3;
const uint8_t kPulsesPerStep = 3;  // 24 ppqn ; 8 steps per quarter note.
const uint8_t kStepsPerPattern = 32;
const uint8_t kPulseDuration = 8;  // 8 ticks of the main clock.

enum OutputMode { OUTPUT_MODE_EUCLIDEAN, OUTPUT_MODE_DRUMS };
enum ClockResolution { CLOCK_RESOLUTION_4_PPQN, CLOCK_RESOLUTION_8_PPQN, CLOCK_RESOLUTION_24_PPQN };
enum OutputBits { OUTPUT_BIT_COMMON = 0x08, OUTPUT_BIT_CLOCK = 0x10, OUTPUT_BIT_RESET = 0x20 };

// grids/hardware_config.h
enum LedBits { LED_CLOCK = 1, LED_BD = 8, LED_SD = 4, LED_HH = 2 };
enum InputBits { INPUT_CLOCK = 2, INPUT_RESET = 4, INPUT_SW_RESET = 8 };

const prog_uint8_t* const kDrumMap[5][5] = {
    {grids::node_10, grids::node_8, grids::node_0, grids::node_9, grids::node_11},
    {grids::node_15, grids::node_7, grids::node_13, grids::node_12, grids::node_6},
    {grids::node_18, grids::node_14, grids::node_4, grids::node_5, grids::node_3},
    {grids::node_23, grids::node_16, grids::node_21, grids::node_1, grids::node_2},
    {grids::node_24, grids::node_19, grids::node_17, grids::node_20, grids::node_22},
};

// avrlib/random.h: 16-bit Galois LFSR.
struct AvrRandom {
  uint16_t rng_state = 0x21;
  void Update() {
    rng_state = static_cast<uint16_t>((rng_state >> 1) ^ (-(rng_state & 1) & 0xb400));
  }
  uint16_t state() const { return rng_state; }
  uint8_t GetByte() {
    Update();
    return static_cast<uint8_t>(rng_state >> 8);
  }
};

// grids/pattern_generator.{h,cc}, as a per-instance class.
struct PatternGenerator {
  // Options
  ClockResolution clock_resolution = CLOCK_RESOLUTION_4_PPQN;
  OutputMode output_mode = OUTPUT_MODE_DRUMS;
  bool output_clock = false;
  bool tap_tempo = false;
  bool gate_mode = false;
  bool swing = false;

  // Settings (union in the firmware: euclidean_length aliases x/y/randomness).
  uint8_t x = 0, y = 0, randomness = 0;
  uint8_t density[kNumParts] = {0, 0, 0};

  uint8_t pulse = 0;
  uint8_t step = 0;
  uint8_t euclidean_step[kNumParts] = {0, 0, 0};
  bool first_beat = false;
  bool beat = false;
  uint8_t state = 0;
  uint8_t part_perturbation[kNumParts] = {0, 0, 0};
  uint8_t pulse_duration_counter = 0;
  AvrRandom random;

  uint8_t euclidean_length(int i) const { return i == 0 ? x : (i == 1 ? y : randomness); }

  void Reset() {
    step = 0;
    pulse = 0;
    std::memset(euclidean_step, 0, sizeof(euclidean_step));
  }
  void Retrigger() { Evaluate(); }

  void TickClock(uint8_t num_pulses) {
    Evaluate();
    beat = (step & 0x7) == 0;
    first_beat = step == 0;
    pulse = static_cast<uint8_t>(pulse + num_pulses);
    while (pulse >= kPulsesPerStep) {
      pulse = static_cast<uint8_t>(pulse - kPulsesPerStep);
      if (!(step & 1)) {
        for (uint8_t i = 0; i < kNumParts; ++i) ++euclidean_step[i];
      }
      ++step;
    }
    if (step >= kStepsPerPattern) step = static_cast<uint8_t>(step - kStepsPerPattern);
  }

  int8_t swing_amount() const {
    if (swing && output_mode == OUTPUT_MODE_DRUMS) {
      int8_t value = static_cast<int8_t>(U8U8MulShift8(randomness, 42 + 1));
      return (!(step & 2)) ? value : static_cast<int8_t>(-value);
    }
    return 0;
  }

  void IncrementPulseCounter() {
    ++pulse_duration_counter;
    // Zero all pulses after 1ms.
    if (pulse_duration_counter >= kPulseDuration && !gate_mode) state = 0;
  }
  void ClockFallingEdge() {
    if (gate_mode) state = 0;
  }

  uint8_t led_pattern() const {
    uint8_t result = 0;
    if (state & 1) result |= LED_BD;
    if (state & 2) result |= LED_SD;
    if (state & 4) result |= LED_HH;
    return result;
  }

  static uint8_t ReadDrumMap(uint8_t step, uint8_t instrument, uint8_t x, uint8_t y) {
    uint8_t i = x >> 6;
    uint8_t j = y >> 6;
    const prog_uint8_t* a_map = kDrumMap[i][j];
    const prog_uint8_t* b_map = kDrumMap[i + 1][j];
    const prog_uint8_t* c_map = kDrumMap[i][j + 1];
    const prog_uint8_t* d_map = kDrumMap[i + 1][j + 1];
    uint8_t offset = static_cast<uint8_t>((instrument * kStepsPerPattern) + step);
    uint8_t a = a_map[offset];
    uint8_t b = b_map[offset];
    uint8_t c = c_map[offset];
    uint8_t d = d_map[offset];
    uint8_t xb = static_cast<uint8_t>(x << 2);
    uint8_t yb = static_cast<uint8_t>(y << 2);
    return U8Mix(U8Mix(a, b, xb), U8Mix(c, d, xb), yb);
  }

  void EvaluateDrums() {
    // At the beginning of a pattern, decide on perturbation levels.
    if (step == 0) {
      for (uint8_t i = 0; i < kNumParts; ++i) {
        uint8_t r = swing ? 0 : static_cast<uint8_t>(randomness >> 2);
        part_perturbation[i] = U8U8MulShift8(random.GetByte(), r);
      }
    }
    uint8_t instrument_mask = 1;
    uint8_t accent_bits = 0;
    for (uint8_t i = 0; i < kNumParts; ++i) {
      uint8_t level = ReadDrumMap(step, i, x, y);
      if (level < 255 - part_perturbation[i]) {
        level = static_cast<uint8_t>(level + part_perturbation[i]);
      } else {
        level = 255;
      }
      uint8_t threshold = static_cast<uint8_t>(~density[i]);
      if (level > threshold) {
        if (level > 192) accent_bits |= instrument_mask;
        state |= instrument_mask;
      }
      instrument_mask = static_cast<uint8_t>(instrument_mask << 1);
    }
    if (output_clock) {
      state |= accent_bits ? OUTPUT_BIT_COMMON : 0;
      state |= step == 0 ? OUTPUT_BIT_RESET : 0;
    } else {
      state |= static_cast<uint8_t>(accent_bits << 3);
    }
  }

  void EvaluateEuclidean() {
    // Refresh only on sixteenth notes.
    if (step & 1) return;
    uint8_t instrument_mask = 1;
    uint8_t reset_bits = 0;
    for (uint8_t i = 0; i < kNumParts; ++i) {
      uint8_t length = static_cast<uint8_t>((euclidean_length(i) >> 3) + 1);
      uint8_t dens = density[i] >> 3;
      uint16_t address = static_cast<uint16_t>(U8U8Mul(static_cast<uint8_t>(length - 1), 32) + dens);
      while (euclidean_step[i] >= length) euclidean_step[i] = static_cast<uint8_t>(euclidean_step[i] - length);
      uint32_t step_mask = 1UL << static_cast<uint32_t>(euclidean_step[i]);
      uint32_t pattern_bits = grids::lut_res_euclidean[address];
      if (pattern_bits & step_mask) state |= instrument_mask;
      if (euclidean_step[i] == 0) reset_bits |= instrument_mask;
      instrument_mask = static_cast<uint8_t>(instrument_mask << 1);
    }
    if (output_clock) {
      state |= reset_bits ? OUTPUT_BIT_COMMON : 0;
      state |= (reset_bits == 0x07) ? OUTPUT_BIT_RESET : 0;
    } else {
      state |= static_cast<uint8_t>(reset_bits << 3);
    }
  }

  void Evaluate() {
    state = 0;
    pulse_duration_counter = 0;
    random.Update();
    // Highest bits: clock and random bit.
    state |= 0x40;
    state |= random.state() & 0x80;
    if (output_clock) state |= OUTPUT_BIT_CLOCK;
    // Refresh only at step changes.
    if (pulse != 0) return;
    if (output_mode == OUTPUT_MODE_EUCLIDEAN) {
      EvaluateEuclidean();
    } else {
      EvaluateDrums();
    }
  }
};

// grids/clock.{h,cc}: 31-bit phase accumulator with swing.
struct GridsClock {
  bool locked = false;
  uint16_t bpm = 0;
  uint32_t phase = 0;
  uint32_t phase_increment = 0;
  uint8_t falling_edge = 0;

  void Update(uint16_t new_bpm, ClockResolution resolution) {
    bpm = new_bpm;
    phase_increment = grids::lut_res_tempo_phase_increment[new_bpm];
    if (resolution == CLOCK_RESOLUTION_4_PPQN) {
      phase_increment >>= 1;
    } else if (resolution == CLOCK_RESOLUTION_24_PPQN) {
      phase_increment = (phase_increment << 1) + phase_increment;
    }
  }
  void Reset() { phase = 0; }
  void Tick() { phase += phase_increment; }
  void Wrap(int8_t amount) {
    uint8_t msb = static_cast<uint8_t>(phase >> 24);
    if (amount == 0) {
      phase &= 0x7fffffffUL;
      falling_edge = 0x40;
    } else {
      if (msb >= 128 + amount) phase &= 0x00ffffffUL;
      falling_edge = static_cast<uint8_t>((128 + amount) >> 1);
    }
  }
  bool raising_edge() const { return phase < phase_increment; }
  bool past_falling_edge() const { return static_cast<uint8_t>(phase >> 24) >= falling_edge; }
};

// Control interrupt rate of the hardware: F_CPU / 32 / kUpdatePeriod.
constexpr double kFCpu = 20000000.0;
constexpr int kUpdatePeriod = 20000000 / 32 / 8000;  // 78
constexpr double kControlRate = kFCpu / 32.0 / kUpdatePeriod;
const uint8_t kTicksGranularity[] = {6, 3, 1};

class TopographicSeq : public Module {
 public:
  enum ParamIds {
    TEMPO_PARAM,
    MAP_X_PARAM,
    MAP_Y_PARAM,
    CHAOS_PARAM,
    FILL1_PARAM,
    FILL2_PARAM,
    FILL3_PARAM,
    TAP_PARAM,
    RESOLUTION_PARAM,  // hidden settings
    TAP_TEMPO_PARAM,
    SWING_PARAM,
    GATE_MODE_PARAM,
    OUTPUT_CLOCK_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    CLOCK_INPUT,
    RESET_INPUT,
    MAP_X_INPUT,
    MAP_Y_INPUT,
    CHAOS_INPUT,
    FILL1_INPUT,
    FILL2_INPUT,
    FILL3_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { TRIG1_OUTPUT, TRIG2_OUTPUT, TRIG3_OUTPUT, ACC1_OUTPUT, ACC2_OUTPUT, ACC3_OUTPUT, NUM_OUTPUTS };
  enum LightIds { BD_LIGHT, SD_LIGHT, HH_LIGHT, CLOCK_LIGHT, NUM_LIGHTS };

  TopographicSeq() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(TEMPO_PARAM, 0.f, 1.f, 0.45f, "Tempo (fully CCW = external clock)", "%", 100.f);
    configParam(MAP_X_PARAM, 0.f, 1.f, 0.5f, "Map X / Length 1", "%", 100.f);
    configParam(MAP_Y_PARAM, 0.f, 1.f, 0.5f, "Map Y / Length 2", "%", 100.f);
    configParam(CHAOS_PARAM, 0.f, 1.f, 0.f, "Chaos / Length 3", "%", 100.f);
    configParam(FILL1_PARAM, 0.f, 1.f, 0.5f, "Fill 1 (BD)", "%", 100.f);
    configParam(FILL2_PARAM, 0.f, 1.f, 0.5f, "Fill 2 (SD)", "%", 100.f);
    configParam(FILL3_PARAM, 0.f, 1.f, 0.5f, "Fill 3 (HH)", "%", 100.f);
    configButton(TAP_PARAM, "Tap tempo / reset");
    configSwitch(RESOLUTION_PARAM, 0.f, 2.f, 0.f, "Clock input resolution",
                 {"4 ppqn", "8 ppqn", "24 ppqn"}).hidden = true;
    configSwitch(TAP_TEMPO_PARAM, 0.f, 1.f, 0.f, "TAP button", {"Reset", "Tap tempo"}).hidden = true;
    configSwitch(SWING_PARAM, 0.f, 1.f, 0.f, "Swing (chaos knob sets amount)", {"Off", "On"}).hidden =
        true;
    configSwitch(GATE_MODE_PARAM, 0.f, 1.f, 0.f, "Output mode", {"Triggers", "Gates"}).hidden = true;
    configSwitch(OUTPUT_CLOCK_PARAM, 0.f, 1.f, 0.f, "ACC outputs",
                 {"Accent 1/2/3", "Accent/Clock/Reset"}).hidden = true;

    configInput(CLOCK_INPUT, "Clock");
    configInput(RESET_INPUT, "Reset");
    configInput(MAP_X_INPUT, "Map X CV");
    configInput(MAP_Y_INPUT, "Map Y CV");
    configInput(CHAOS_INPUT, "Chaos CV");
    configInput(FILL1_INPUT, "Fill 1 CV");
    configInput(FILL2_INPUT, "Fill 2 CV");
    configInput(FILL3_INPUT, "Fill 3 CV");
    configOutput(TRIG1_OUTPUT, "Trigger 1 (BD)");
    configOutput(TRIG2_OUTPUT, "Trigger 2 (SD)");
    configOutput(TRIG3_OUTPUT, "Trigger 3 (HH)");
    configOutput(ACC1_OUTPUT, "Accent 1 / common accent");
    configOutput(ACC2_OUTPUT, "Accent 2 / clock");
    configOutput(ACC3_OUTPUT, "Accent 3 / reset");
    init();
  }

  void onReset() override {
    euclidean_ = false;
    init();
  }

  void saveState(StateMap& s) const override { s["euclidean"] = euclidean_ ? "1" : "0"; }
  void loadState(const StateMap& s) override {
    auto it = s.find("euclidean");
    if (it != s.end()) euclidean_ = it->second == "1";
  }
  void appendMenu(std::vector<MenuAction>& items) override {
    items.push_back({"Pattern mode",
                     [this] { return std::string(euclidean_ ? "EUCLIDEAN" : "DRUMS"); },
                     [this](int) { euclidean_ = !euclidean_; }});
  }

  bool euclidean() const { return euclidean_; }

  void process(const ProcessArgs& args) override {
    // Latch input edges at audio rate so that short pulses are never missed
    // by the 8 kHz control interrupt.
    if (clockTrigger_.process(inputs[CLOCK_INPUT].getVoltage())) clockLatch_ = true;
    if (resetTrigger_.process(inputs[RESET_INPUT].getVoltage())) resetLatch_ = true;

    isrPhase_ += kControlRate * args.sampleTime;
    while (isrPhase_ >= 1.0) {
      isrPhase_ -= 1.0;
      ScanPots();
      Isr();
    }

    uint8_t s = outputState_;
    for (int i = 0; i < NUM_OUTPUTS; ++i) {
      outputs[static_cast<size_t>(i)].setVoltage((s & (1 << i)) ? 5.f : 0.f);
    }
    if ((args.frame & 63) == 0) {
      lights[BD_LIGHT] = (ledPattern_ & LED_BD) ? 1.f : 0.f;
      lights[SD_LIGHT] = (ledPattern_ & LED_SD) ? 1.f : 0.f;
      lights[HH_LIGHT] = (ledPattern_ & LED_HH) ? 1.f : 0.f;
      lights[CLOCK_LIGHT] = (ledPattern_ & LED_CLOCK) ? 1.f : 0.f;
    }
  }

 private:
  void init() {
    pg_ = PatternGenerator();
    clock_ = GridsClock();
    readOptions();
    clock_.Update(120, CLOCK_RESOLUTION_24_PPQN);
    clock_.locked = false;
    pg_.Reset();
    clock_.Update(120, pg_.clock_resolution);
    tapDuration_ = 0;
    ledPattern_ = 0;
    ledOffTimer_ = 0;
    swingAmount_ = 0;
    previousInputs_ = 0;
    previousState_ = 0;
    outputState_ = 0;
    switchState_ = 0xff;
    switchHoldTime_ = 0;
    debouncePrescaler_ = 0;
  }

  // Hidden params / menu -> firmware options (the firmware's settings mode).
  void readOptions() {
    int res = static_cast<int>(params[RESOLUTION_PARAM].value + 0.5f);
    ClockResolution r = static_cast<ClockResolution>(res < 0 ? 0 : (res > 2 ? 2 : res));
    if (r != pg_.clock_resolution) {
      pg_.clock_resolution = r;
      clock_.Update(clock_.bpm, r);
      pg_.Reset();
    }
    bool tap = params[TAP_TEMPO_PARAM].value > 0.5f;
    if (tap != pg_.tap_tempo) {
      pg_.tap_tempo = tap;
      if (!tap) clock_.locked = false;
    }
    pg_.swing = params[SWING_PARAM].value > 0.5f;
    pg_.gate_mode = params[GATE_MODE_PARAM].value > 0.5f;
    pg_.output_clock = params[OUTPUT_CLOCK_PARAM].value > 0.5f;
    pg_.output_mode = euclidean_ ? OUTPUT_MODE_EUCLIDEAN : OUTPUT_MODE_DRUMS;
  }

  // Knob + CV, as the summed (inverted, then re-inverted by the firmware)
  // 8-bit ADC reading. Knob min = 0 V, max = 5 V.
  uint8_t readPot(int param, int input) const {
    float v = params[static_cast<size_t>(param)].value;
    if (input >= 0) v += inputs[static_cast<size_t>(input)].getVoltage() / 5.f;
    v = clampf(v, 0.f, 1.f);
    return static_cast<uint8_t>(v * 255.f + 0.5f);
  }

  // grids.cc: ScanPots() (main loop), normal mode.
  void ScanPots() {
    readOptions();
    uint8_t bpm = readPot(TEMPO_PARAM, -1);
    bpm = static_cast<uint8_t>(U8U8MulShift8(bpm, 220) + 20);
    if (bpm != clock_.bpm && !clock_.locked) clock_.Update(bpm, pg_.clock_resolution);
    pg_.x = readPot(MAP_X_PARAM, MAP_X_INPUT);
    pg_.y = readPot(MAP_Y_PARAM, MAP_Y_INPUT);
    pg_.randomness = readPot(CHAOS_PARAM, CHAOS_INPUT);
    pg_.density[0] = readPot(FILL1_PARAM, FILL1_INPUT);
    pg_.density[1] = readPot(FILL2_PARAM, FILL2_INPUT);
    pg_.density[2] = readPot(FILL3_PARAM, FILL3_INPUT);
  }

  // grids.cc: ISR(TIMER2_COMPA_vect).
  void Isr() {
    ++tapDuration_;
    if (++debouncePrescaler_ >= 10) {
      HandleTapButton();
      debouncePrescaler_ = 0;
    }
    HandleClockResetInputs();
    pg_.IncrementPulseCounter();
    UpdateShiftRegister();
    UpdateLeds();
  }

  void HandleClockResetInputs() {
    uint8_t inputs_value = 0;
    if (clockLatch_ || clockTrigger_.isHigh()) inputs_value |= INPUT_CLOCK;
    if (resetLatch_ || resetTrigger_.isHigh()) inputs_value |= INPUT_RESET;
    clockLatch_ = resetLatch_ = false;

    uint8_t num_ticks = 0;
    uint8_t increment = kTicksGranularity[pg_.clock_resolution];
    bool external = (clock_.bpm < 40 && !clock_.locked) || inputs[CLOCK_INPUT].isConnected();
    if (external) {
      if ((inputs_value & INPUT_CLOCK) && !(previousInputs_ & INPUT_CLOCK)) num_ticks = increment;
      if (!(inputs_value & INPUT_CLOCK) && (previousInputs_ & INPUT_CLOCK)) pg_.ClockFallingEdge();
    } else {
      clock_.Tick();
      clock_.Wrap(swingAmount_);
      if (clock_.raising_edge()) num_ticks = increment;
      if (clock_.past_falling_edge()) pg_.ClockFallingEdge();
    }

    if ((inputs_value & INPUT_RESET) && !(previousInputs_ & INPUT_RESET)) {
      pg_.Reset();
      if (!external) {
        pg_.Retrigger();
        clock_.Reset();
      }
    }
    previousInputs_ = inputs_value;

    if (num_ticks) {
      swingAmount_ = pg_.swing_amount();
      pg_.TickClock(num_ticks);
    }
  }

  void HandleTapButton() {
    // Switch reads 0 when pressed (pull-up), as on the hardware.
    switchState_ = static_cast<uint8_t>(switchState_ << 1);
    if (!(params[TAP_PARAM].value > 0.5f)) switchState_ |= 1;
    if (switchState_ == 0xfe) {  // just pressed
      if (!pg_.tap_tempo) {
        pg_.Reset();
        bool external = (clock_.bpm < 40 && !clock_.locked) || inputs[CLOCK_INPUT].isConnected();
        if (!external) clock_.Reset();
      } else {
        uint32_t new_bpm = tapDuration_
            ? static_cast<uint32_t>((kFCpu * 60.0) / (32.0 * kUpdatePeriod * tapDuration_))
            : 0;
        if (new_bpm >= 30 && new_bpm <= 480) {
          clock_.Update(static_cast<uint16_t>(new_bpm), pg_.clock_resolution);
          clock_.Reset();
          clock_.locked = true;
        } else {
          clock_.locked = false;
        }
        tapDuration_ = 0;
      }
      switchHoldTime_ = 0;
    } else if (switchState_ == 0x00) {
      // Long press enters the settings mode on the hardware: the settings are
      // hidden params here, so nothing happens.
      if (switchHoldTime_ < 0xffff) ++switchHoldTime_;
    }
  }

  void UpdateShiftRegister() {
    if (pg_.state != previousState_) {
      previousState_ = pg_.state;
      outputState_ = previousState_;
      if (!previousState_) {
        ledOffTimer_ = 200;  // Switch off the LEDs, but not now.
      } else {
        ledBits_ = pg_.led_pattern();
        ledOffTimer_ = 0;
      }
    }
  }

  void UpdateLeds() {
    if (ledOffTimer_) {
      --ledOffTimer_;
      if (!ledOffTimer_) ledBits_ = 0;
    }
    uint8_t pattern = ledBits_;
    if (pg_.tap_tempo ? pg_.beat : pg_.first_beat) pattern |= LED_CLOCK;
    ledPattern_ = pattern;
  }

  PatternGenerator pg_;
  GridsClock clock_;
  SchmittTrigger clockTrigger_, resetTrigger_;
  bool clockLatch_ = false, resetLatch_ = false;
  bool euclidean_ = false;
  double isrPhase_ = 0.0;
  uint32_t tapDuration_ = 0;
  uint8_t ledBits_ = 0;
  uint8_t ledPattern_ = 0;
  uint8_t ledOffTimer_ = 0;
  int8_t swingAmount_ = 0;
  uint8_t previousInputs_ = 0;
  uint8_t previousState_ = 0;
  uint8_t outputState_ = 0;
  uint8_t switchState_ = 0xff;
  uint16_t switchHoldTime_ = 0;
  uint8_t debouncePrescaler_ = 0;
};

void buildPanel(PanelSpec& p) {
  p.hp = 16;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  p.title = "TOPOGRAPHIC SEQ";
  const uint32_t red = 0xFFC8414B;
  const uint32_t teal = 0xFF168A8C;
  const uint32_t ledCol = 0xFFFF3B30;
  using M = TopographicSeq;

  // Positions measured on the original panel (0.2973 mm/px).
  const float c1 = 8.9f, c2 = 23.5f;
  const float rows[5] = {21.1f, 42.5f, 63.9f, 85.3f, 106.7f};

  // Left: CV jacks and knobs.
  p.input(M::CLOCK_INPUT, c1, rows[0]);
  p.box(c1 - 5.2f, rows[0] + 5.3f, 10.4f, 4.0f, red, 1.6f);
  p.label(c1, rows[0] + 7.3f, "CLOCK", 2.8f, kAlignCenter, 0xFFFFFFFF);
  p.knob(M::TEMPO_PARAM, c2, rows[0], 8.f);
  p.label(c2, rows[0] + 7.3f, "TEMPO", 2.8f);

  p.input(M::RESET_INPUT, c1, rows[1], "");
  p.label(c1, rows[1] + 7.3f, "RESET", 2.8f);
  p.button(M::TAP_PARAM, c2, rows[1], 7.f, "", 0, M::CLOCK_LIGHT);
  p.label(c2, rows[1] + 7.3f, "TAP (reset)", 2.8f);

  p.input(M::MAP_X_INPUT, c1, rows[2]);
  p.knob(M::MAP_X_PARAM, c2, rows[2], 8.f);
  p.label(c1, rows[2] + 7.3f, "MAP X", 2.8f);
  p.input(M::MAP_Y_INPUT, c1, rows[3]);
  p.knob(M::MAP_Y_PARAM, c2, rows[3], 8.f);
  p.label(c1, rows[3] + 7.3f, "MAP Y", 2.8f);
  p.input(M::CHAOS_INPUT, c1, rows[4]);
  p.knob(M::CHAOS_PARAM, c2, rows[4], 8.f);
  p.label(c1, rows[4] + 7.3f, "CHAOS", 2.8f);

  // Right: fill knobs on the diagonal, LEDs, fill CV, outputs.
  const float fx[3] = {47.6f, 59.5f, 72.2f};
  const float fy[3] = {21.1f, 42.5f, 63.9f};
  const float fd[3] = {9.f, 12.f, 9.f};
  const float ledY = 77.0f, cvY = 87.1f, trigY = 99.0f, accY = 110.9f;
  const char* nums[3] = {"1", "2", "3"};
  for (int i = 0; i < 3; ++i) {
    // Dotted guide from each knob down to its LED.
    for (float y = fy[i] + fd[i] * 0.5f + 1.2f; y < ledY - 2.2f; y += 1.6f)
      p.line(fx[i], y, fx[i], y + 0.7f, 0.35f, p.ink);
    p.knob(M::FILL1_PARAM + i, fx[i], fy[i], fd[i]);
    p.light(M::BD_LIGHT + i, fx[i], ledY, ledCol, 2.6f);
    p.label(fx[i] - 4.0f, ledY, nums[i], 3.0f);
    p.input(M::FILL1_INPUT + i, fx[i], cvY);
  }
  p.box(34.6f, cvY - 2.0f, 8.0f, 4.0f, teal, 1.6f);
  p.label(38.6f, cvY, "FILL", 2.8f, kAlignCenter, 0xFFFFFFFF);

  p.box(33.9f, 93.1f, 45.2f, 24.0f, 0xFF5E5E60, 1.4f);
  p.label(38.4f, trigY, "TRIG", 2.8f, kAlignCenter, 0xFFFFFFFF);
  p.label(38.4f, accY, "ACC", 2.8f, kAlignCenter, 0xFFFFFFFF);
  for (int i = 0; i < 3; ++i) {
    p.output(M::TRIG1_OUTPUT + i, fx[i], trigY);
    p.output(M::ACC1_OUTPUT + i, fx[i], accY);
  }

  p.line(3.0f, 120.2f, 78.3f, 120.2f, 0.5f, 0xFFB8B8B8);

  // Mode readout (drums / euclidean), set from the module menu.
  p.display(18.64f, 9.6f, 44.0f, 4.0f, [](Canvas& c, float x, float y, float w, float h, float s,
                                          const Module* m) {
    bool e = static_cast<const TopographicSeq*>(m)->euclidean();
    c.text(x + w * 0.5f, y + h * 0.5f, e ? "EUCLIDEAN MODE" : "DRUM MAP MODE",
           std::max(12.f, 2.7f * s), 0xFF168A8C, kAlignCenter);
  });
}

Model model = [] {
  Model m;
  m.slug = "topographic-seq";
  m.name = "Topographic Drum Sequencer";
  m.shortName = "TOPOGRAPHIC SEQ";
  m.basedOn = "Grids";
  m.category = Category::Sequencer;
  m.description = "Drum patterns from a 2D map of rhythms, with fill, chaos and Euclidean mode.";
  m.create = [] { return new TopographicSeq(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
