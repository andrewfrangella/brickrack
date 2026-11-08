// BrickRack - GPL-3.0-or-later.
//
// Tidal Modulator: wraps the Tides (2015) Generator (tides/generator.cc)
// exactly as tides/tides.cc drives it. The DAC runs at 48 kHz (a 96 kHz timer
// alternating between the two DAC channels); the generator renders 16-sample
// blocks into a double buffer that the sample interrupt plays back, one
// sample per DAC tick (every 4th tick in the LOW range). CvScaler's
// smoothing runs at the same rates: level at 48 kHz, everything else at
// 12 kHz. The two buttons and the three bicolour LEDs follow tides/ui.cc.
//
// This file is compiled with -Dtides=tides2015 (see tides.cmake) so that it
// does not clash with Tides 2, which uses the same namespace.

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "stmlib/utils/dsp.h"
#include "tides/generator.h"
#include "tides/resources.h"

namespace br {
namespace {

const float kNativeRate = 48000.f;
const float kLongPressMs = 1000.f;
const float kGateHigh = 10.f;

const char* kModeNames[3] = {"AD", "Looping (cycle)", "AR"};
const char* kRangeNames[3] = {"Low", "Medium", "High"};

class TidalMod : public Module {
 public:
  enum ParamIds {
    MODE_BUTTON_PARAM,
    RANGE_BUTTON_PARAM,
    FREQUENCY_PARAM,
    FM_PARAM,
    SHAPE_PARAM,
    SLOPE_PARAM,
    SMOOTHNESS_PARAM,
    // Hidden: the settings the firmware keeps in flash (ui.h).
    MODE_PARAM,
    RANGE_PARAM,
    SYNC_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    SHAPE_INPUT,
    SLOPE_INPUT,
    SMOOTHNESS_INPUT,
    TRIG_INPUT,
    FREEZE_INPUT,
    PITCH_INPUT,
    FM_INPUT,
    LEVEL_INPUT,
    CLOCK_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { HIGH_OUTPUT, LOW_OUTPUT, UNI_OUTPUT, BI_OUTPUT, NUM_OUTPUTS };
  // {red, green} pairs, drawn as RGB lights with ZERO_LIGHT as blue.
  enum LightIds { MODE_LIGHT = 0, RANGE_LIGHT = 2, PHASE_LIGHT = 4, ZERO_LIGHT = 6, NUM_LIGHTS };

  TidalMod() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configButton(MODE_BUTTON_PARAM, "Mode");
    configButton(RANGE_BUTTON_PARAM, "Range (hold 1 s: clock sync / PLL)");
    configParam(FREQUENCY_PARAM, -48.f, 48.f, 0.f, "Frequency", "semitones").displayPrecision = 1;
    configParam(FM_PARAM, -1.f, 1.f, 0.f, "FM amount", "%", 100.f);
    configParam(SHAPE_PARAM, -1.f, 1.f, 0.f, "Shape", "%", 100.f);
    configParam(SLOPE_PARAM, -1.f, 1.f, 0.f, "Slope", "%", 100.f);
    configParam(SMOOTHNESS_PARAM, -1.f, 1.f, 0.f, "Smoothness", "%", 100.f);
    // Defaults from Ui::Init(): mode_counter_ = 1 (looping), range_counter_ = 2 (high).
    configSwitch(MODE_PARAM, 0.f, 2.f, 1.f, "Mode", {kModeNames[0], kModeNames[1], kModeNames[2]})
        .hidden = true;
    configSwitch(RANGE_PARAM, 0.f, 2.f, 2.f, "Range",
                 {kRangeNames[0], kRangeNames[1], kRangeNames[2]})
        .hidden = true;
    configSwitch(SYNC_PARAM, 0.f, 1.f, 0.f, "Clock sync (PLL)", {"Off", "On"}).hidden = true;

    configInput(SHAPE_INPUT, "Shape");
    configInput(SLOPE_INPUT, "Slope");
    configInput(SMOOTHNESS_INPUT, "Smoothness");
    configInput(TRIG_INPUT, "Trigger");
    configInput(FREEZE_INPUT, "Freeze");
    configInput(PITCH_INPUT, "V/Oct");
    configInput(FM_INPUT, "FM");
    configInput(LEVEL_INPUT, "Level");
    configInput(CLOCK_INPUT, "Clock");
    configOutput(HIGH_OUTPUT, "High tide");
    configOutput(LOW_OUTPUT, "Low tide");
    configOutput(UNI_OUTPUT, "Unipolar");
    configOutput(BI_OUTPUT, "Bipolar");

    // The firmware's DSP objects are zero-initialised globals: do the same
    // (Init() doesn't set every member).
    std::memset(static_cast<void*>(&generator_), 0, sizeof(generator_));
    generator_.Init();
    syncSettings();
    // Native rate == engine rate: the adapter only buffers (block size 1);
    // the generator does its own 16-sample double buffering.
    adapter_.init(48000.f, kNativeRate, 1);
  }

  void onSampleRateChange(float sr) override { adapter_.init(sr, kNativeRate, 1); }

  void process(const ProcessArgs& args) override {
    nowMs_ += args.sampleTime * 1000.f;
    handleButtons();

    float in[3] = {inputs[TRIG_INPUT].getVoltage(), inputs[FREEZE_INPUT].getVoltage(),
                   inputs[CLOCK_INPUT].getVoltage()};
    float out[4];
    adapter_.process(in, out, [&](const float* nin, float* nout, int n) {
      for (int i = 0; i < n; ++i) tick(&nin[i * 3], &nout[i * 4]);
    });
    for (int i = 0; i < 4; ++i) outputs[static_cast<size_t>(i)].setVoltage(out[i]);

    if ((args.frame & 31) == 0) updateLights();
  }

 private:
  int intParam(int id) const { return static_cast<int>(params[static_cast<size_t>(id)].value + 0.5f); }

  // Applies the hidden settings to the generator (Ui::UpdateMode/UpdateRange).
  void syncSettings() {
    auto mode = static_cast<tides::GeneratorMode>(intParam(MODE_PARAM));
    auto range = static_cast<tides::GeneratorRange>(2 - intParam(RANGE_PARAM));
    bool sync = intParam(SYNC_PARAM) != 0;
    if (mode != generator_.mode()) generator_.set_mode(mode);
    if (range != generator_.range()) generator_.set_range(range);
    if (sync != generator_.sync()) generator_.set_sync(sync);
  }

  // The firmware's mode/range counters step 0, 1, 2, 1, 0, ... ("& 3, 3 -> 1").
  static int counterValue(int counter) {
    int i = counter & 3;
    return i == 3 ? 1 : i;
  }
  void stepCounter(int& counter, int paramId) {
    if (counterValue(counter) != intParam(paramId)) counter = intParam(paramId);  // menu edit
    ++counter;
    params[static_cast<size_t>(paramId)].set(static_cast<float>(counterValue(counter)));
  }

  // Ui::Poll / OnSwitchReleased. A press longer than 1 s fires while held.
  void handleButtons() {
    for (int b = 0; b < 2; ++b) {
      bool pressed = params[static_cast<size_t>(MODE_BUTTON_PARAM + b)].value > 0.5f;
      if (pressed && !down_[b]) {
        pressStart_[b] = nowMs_;
        armed_[b] = true;
      }
      double held = nowMs_ - pressStart_[b];
      bool fire = false, longPress = false;
      if (pressed && armed_[b] && held > kLongPressMs) {
        fire = longPress = true;
      } else if (!pressed && down_[b] && armed_[b]) {
        fire = true;
      }
      if (fire) {
        armed_[b] = false;
        if (b == 0) {
          // A long press enters calibration only with LEVEL held high; we
          // have no calibration, so it always steps the mode.
          stepCounter(modeCounter_, MODE_PARAM);
        } else if (longPress) {
          params[SYNC_PARAM].set(intParam(SYNC_PARAM) ? 0.f : 1.f);
        } else {
          stepCounter(rangeCounter_, RANGE_PARAM);
        }
      }
      down_[b] = pressed;
    }
  }

  // CvScaler::ProcessControlRate(), on voltages. Pot + CV are summed in the
  // analog domain on the hardware: +/-5 V CV spans the full knob range.
  void processControlRate() {
    auto sum = [&](int param, int input) {
      float v = params[static_cast<size_t>(param)].value + inputs[static_cast<size_t>(input)].getVoltage() / 5.f;
      return static_cast<int32_t>(clampf(v, -1.f, 1.f) * 32767.f);
    };
    shape_ = (shape_ * 7 + sum(SHAPE_PARAM, SHAPE_INPUT)) >> 3;
    slope_ = (slope_ * 7 + sum(SLOPE_PARAM, SLOPE_INPUT)) >> 3;
    smoothness_ = (smoothness_ * 7 + sum(SMOOTHNESS_PARAM, SMOOTHNESS_INPUT)) >> 3;
    vOct_ += (params[FREQUENCY_PARAM].value + inputs[PITCH_INPUT].getVoltage() * 12.f - vOct_) * 0.25f;
    fm_ += (inputs[FM_INPUT].getVoltage() * 12.f - fm_) * 0.25f;
    attenuverter_ += (params[FM_PARAM].value - attenuverter_) * (1.f / 16.f);
  }

  // CvScaler::pitch(): FM through the attenuverter curve, 1/128 semitones.
  int16_t pitch() const {
    int32_t a = static_cast<int32_t>(std::fabs(attenuverter_) * 32767.f);
    float curve = stmlib::Interpolate88(tides::lut_attenuverter_curve, static_cast<uint16_t>(a << 1)) / 65536.f;
    if (attenuverter_ < 0.f) curve = -curve;
    float semitones = 60.f + vOct_ + fm_ * curve;
    // Generator::set_pitch() then adds 12 << 7 (HIGH range) or down to
    // -(120 << 7) (LOW range) in int16 arithmetic, and
    // ComputeAntialiasAttenuation() adds another 12 << 7: keep the sums from
    // wrapping (the hardware's int32 -> int16 return wraps there too).
    return static_cast<int16_t>(
        clampf(semitones * 128.f, -32768.f + (120 << 7), 32767.f - (24 << 7)));
  }

  // One 48 kHz DAC tick of TIM1_UP_IRQHandler, plus the main loop's block render.
  void tick(const float* in, float* out) {
    // GateInput::Read().
    uint8_t control = 0;
    if (trig_.process(in[0])) control |= tides::CONTROL_GATE_RISING;
    if (trig_.isHigh()) control |= tides::CONTROL_GATE;
    else if (prevGate_) control |= tides::CONTROL_GATE_FALLING;
    prevGate_ = trig_.isHigh();
    freeze_.process(in[1]);
    if (freeze_.isHigh()) control |= tides::CONTROL_FREEZE;
    if (clock_.process(in[2])) control |= tides::CONTROL_CLOCK_RISING;
    if (clock_.isHigh()) control |= tides::CONTROL_CLOCK;
    pendingControl_ |= control & (tides::CONTROL_GATE_RISING | tides::CONTROL_GATE_FALLING |
                                  tides::CONTROL_CLOCK_RISING);

    if (++dacDivider_ >= generator_.clock_divider()) {
      dacDivider_ = 0;
      // Edges seen on skipped ticks (LOW range) are carried to the next one.
      uint8_t c = (control & ~(tides::CONTROL_GATE_RISING | tides::CONTROL_GATE_FALLING |
                               tides::CONTROL_CLOCK_RISING)) | pendingControl_;
      pendingControl_ = 0;
      const tides::GeneratorSample& sample = generator_.Process(c);
      uint32_t uni = sample.unipolar;
      int32_t bi = sample.bipolar;
      uni = uni * level_ >> 16;
      bi = -bi * static_cast<int32_t>(level_) >> 16;
      // DAC: UNI 0..+8 V, BI +/-5 V. Gate outputs high while the flag is set.
      held_[2] = uni / 65535.f * 8.f;
      held_[3] = bi / 32768.f * 5.f;
      held_[0] = (sample.flags & tides::FLAG_END_OF_ATTACK) ? kGateHigh : 0.f;
      held_[1] = (sample.flags & tides::FLAG_END_OF_RELEASE) ? kGateHigh : 0.f;
      ledValue_ = uni;
      ledEndOfAttack_ = sample.flags & tides::FLAG_END_OF_ATTACK;
    }
    for (int i = 0; i < 4; ++i) out[i] = held_[i];

    // CvScaler::ProcessSampleRate(): LEVEL (normalled to full scale).
    int32_t raw = inputs[LEVEL_INPUT].isConnected()
                      ? static_cast<int32_t>(clampf(inputs[LEVEL_INPUT].getVoltage() / 8.f, 0.f, 1.f) * 65535.f)
                      : 65535;
    levelRaw_ = (levelRaw_ * 15 + raw) >> 4;
    level_ = levelRaw_ < 32 ? 0 : static_cast<uint32_t>(levelRaw_);
    // ProcessControlRate runs on every 8th tick of the 96 kHz timer.
    if ((++controlDivider_ & 3) == 0) processControlRate();

    // Main loop: refill the double buffer as soon as a block has been played.
    if (generator_.writable_block()) {
      syncSettings();
      generator_.set_pitch(pitch());
      generator_.set_shape(static_cast<int16_t>(shape_));
      generator_.set_slope(static_cast<int16_t>(slope_));
      generator_.set_smoothness(static_cast<int16_t>(smoothness_));
      generator_.Process();
    }
  }

  void setLed(int light, float red, float green) {
    lights[static_cast<size_t>(light)] = red;
    lights[static_cast<size_t>(light + 1)] = green;
  }

  // Ui::Poll() LED painting and Ui::set_led_color().
  void updateLights() {
    int mode = intParam(MODE_PARAM);
    setLed(MODE_LIGHT, mode == tides::GENERATOR_MODE_AR, mode == tides::GENERATOR_MODE_AD);
    bool sync = intParam(SYNC_PARAM) != 0;
    switch (2 - intParam(RANGE_PARAM)) {
      case tides::GENERATOR_RANGE_LOW: setLed(RANGE_LIGHT, 0.f, 1.f); break;
      case tides::GENERATOR_RANGE_MEDIUM:
        setLed(RANGE_LIGHT, sync ? 0.125f : 0.f, sync ? 0.25f : 0.f);
        break;
      default: setLed(RANGE_LIGHT, 1.f, 0.f); break;
    }
    if (sync && (static_cast<uint32_t>(nowMs_) & 128)) setLed(RANGE_LIGHT, 0.f, 0.f);
    float b = ledValue_ / 65535.f;
    setLed(PHASE_LIGHT, ledEndOfAttack_ ? b : 0.f, ledEndOfAttack_ ? 0.f : b);
  }

  tides::Generator generator_;
  BlockAdapter<3, 4> adapter_;

  SchmittTrigger trig_, freeze_, clock_;
  bool prevGate_ = false;
  uint8_t pendingControl_ = 0;
  uint32_t dacDivider_ = 0;
  uint32_t controlDivider_ = 0;
  float held_[4] = {0.f, 0.f, 0.f, 0.f};

  int32_t shape_ = 0, slope_ = 0, smoothness_ = 0;
  float vOct_ = 0.f, fm_ = 0.f, attenuverter_ = 0.f;
  int32_t levelRaw_ = 65535;
  uint32_t level_ = 65535;

  uint16_t ledValue_ = 0;
  bool ledEndOfAttack_ = false;

  int modeCounter_ = 1, rangeCounter_ = 2;
  double nowMs_ = 0.0;
  double pressStart_[2] = {0.0, 0.0};
  bool down_[2] = {false, false}, armed_[2] = {false, false};
};

void buildPanel(PanelSpec& p) {
  // 14 HP. Positions measured on the original panel (mm = px * 0.21 here).
  p.hp = 14;
  p.bg = 0xFFF3F1EE;
  p.ink = 0xFF242426;
  p.title = "TIDAL MOD";
  const uint32_t teal = 0xFF14958C, red = 0xFFC0404B, dark = 0xFF242426;
  const uint32_t white = 0xFFFFFFFF, grey = 0xFFB4B4B4;

  auto tag = [&](float x, float y, float w, const char* text, uint32_t col, float size) {
    p.box(x - w * 0.5f, y - 2.2f, w, 4.4f, col, 1.0f);
    p.label(x, y, text, size, kAlignCenter, white);
  };
  auto dot = [&](float x, float y, const char* text, uint32_t col) {
    p.box(x - 1.7f, y - 1.7f, 3.4f, 3.4f, col, 1.7f);
    p.label(x, y, text, 2.2f, kAlignCenter, white);
  };

  // Mode and range buttons, with their option markers and LEDs.
  p.button(TidalMod::MODE_BUTTON_PARAM, 12.0f, 22.0f, 7.f, "", 0xFFD8D9DB);
  dot(3.8f, 18.6f, "AD", teal);
  dot(3.8f, 22.0f, "C", dark);
  dot(3.8f, 25.4f, "AR", red);
  p.rgbLight(TidalMod::MODE_LIGHT, TidalMod::MODE_LIGHT + 1, TidalMod::ZERO_LIGHT, 21.0f, 22.0f, 2.8f);

  p.button(TidalMod::RANGE_BUTTON_PARAM, 12.0f, 36.5f, 7.f, "", 0xFFD8D9DB);
  dot(3.8f, 33.1f, "L", teal);
  dot(3.8f, 36.5f, "M", dark);
  dot(3.8f, 39.9f, "H", red);
  p.rgbLight(TidalMod::RANGE_LIGHT, TidalMod::RANGE_LIGHT + 1, TidalMod::ZERO_LIGHT, 21.0f, 36.3f, 2.8f);

  // Output (phase) LED next to the frequency knob.
  p.rgbLight(TidalMod::PHASE_LIGHT, TidalMod::PHASE_LIGHT + 1, TidalMod::ZERO_LIGHT, 21.0f, 29.2f, 2.8f);
  p.line(22.8f, 29.2f, 27.2f, 29.2f, 0.3f, 0xFF8A8A8A);
  p.line(13.6f, 22.0f, 19.2f, 22.0f, 0.3f, 0xFF8A8A8A);
  p.line(13.6f, 36.4f, 19.2f, 36.4f, 0.3f, 0xFF8A8A8A);

  p.knob(TidalMod::FREQUENCY_PARAM, 35.5f, 29.2f, 13.f);
  tag(35.5f, 43.7f, 19.f, "FREQUENCY", teal, 3.0f);
  p.knob(TidalMod::FM_PARAM, 59.6f, 29.2f, 9.f);
  tag(59.6f, 43.7f, 8.f, "FM", teal, 3.0f);
  p.label(55.0f, 21.4f, "-", 3.0f);
  p.label(64.2f, 21.4f, "+", 3.0f);

  p.knob(TidalMod::SHAPE_PARAM, 11.3f, 59.0f, 10.f);
  p.label(11.3f, 70.1f, "SHAPE", 3.0f);
  p.knob(TidalMod::SLOPE_PARAM, 35.5f, 59.0f, 10.f);
  p.label(35.5f, 70.1f, "SLOPE", 3.0f);
  p.knob(TidalMod::SMOOTHNESS_PARAM, 59.6f, 59.0f, 10.f);
  p.label(59.6f, 70.1f, "SMOOTHNESS", 3.0f);

  p.input(TidalMod::SHAPE_INPUT, 11.3f, 78.1f);
  p.input(TidalMod::SLOPE_INPUT, 35.5f, 78.1f);
  p.input(TidalMod::SMOOTHNESS_INPUT, 59.6f, 78.1f);

  p.line(3.0f, 85.5f, 68.1f, 85.5f, 0.6f, grey);

  p.input(TidalMod::TRIG_INPUT, 11.3f, 96.4f, "TRIG");
  p.input(TidalMod::FREEZE_INPUT, 23.4f, 96.4f, "FREEZE");
  p.input(TidalMod::PITCH_INPUT, 35.5f, 96.4f);
  tag(35.5f, 90.1f, 9.f, "V/OCT", teal, 2.7f);
  p.input(TidalMod::FM_INPUT, 47.6f, 96.4f);
  tag(47.6f, 90.1f, 6.f, "FM", teal, 2.7f);
  p.input(TidalMod::LEVEL_INPUT, 59.6f, 96.4f, "LEVEL");

  p.input(TidalMod::CLOCK_INPUT, 11.3f, 111.4f, "CLOCK");
  p.box(16.8f, 102.3f, 48.7f, 14.5f, 0xFF5E5E60, 1.2f);
  p.output(TidalMod::HIGH_OUTPUT, 23.4f, 111.6f);
  p.output(TidalMod::LOW_OUTPUT, 35.5f, 111.6f);
  p.output(TidalMod::UNI_OUTPUT, 47.6f, 111.6f);
  p.output(TidalMod::BI_OUTPUT, 59.6f, 111.6f);
  p.label(23.4f, 105.2f, "HIGH", 2.7f, kAlignCenter, white);
  p.label(35.5f, 105.2f, "LOW", 2.7f, kAlignCenter, white);
  p.label(47.6f, 105.2f, "UNI", 2.7f, kAlignCenter, white);
  p.label(59.6f, 105.2f, "BI", 2.7f, kAlignCenter, white);
  p.label(29.45f, 107.8f, "TIDE", 2.2f, kAlignCenter, white);

  p.line(3.0f, 119.1f, 68.1f, 119.1f, 0.6f, grey);
}

Model model = [] {
  Model m;
  m.slug = "tidal-modulator";
  m.name = "Tidal Modulator";
  m.shortName = "TIDAL MOD";
  m.basedOn = "Tides";
  m.category = Category::Modulator;
  m.description = "Tidal modulator: AD/AR envelope, LFO or oscillator with shape and slope.";
  m.create = [] { return new TidalMod(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
