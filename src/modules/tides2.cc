// BrickRack - GPL-3.0-or-later.
//
// Tidal Modulator 2: wraps the Tides (2018) PolySlopeGenerator and
// RampExtractor (tides2/poly_slope_generator.cc, tides2/ramp/) exactly as
// tides2/tides.cc drives them: 62.5 kHz, 8-sample blocks (io_buffer.h), CVs
// read once per block through a copy of CvReader::Read(), outputs in volts
// (the DAC calibration maps 1.0 to 1 V). The three buttons and their bicolour
// LEDs follow tides2/ui.cc.

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/hysteresis_quantizer.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/gate_flags.h"
#include "tides2/io_buffer.h"
#include "tides2/poly_slope_generator.h"
#include "tides2/ramp/ramp_extractor.h"

namespace br {
namespace {

using stmlib::GateFlags;

const float kNativeRate = 62500.f;  // tides2/io_buffer.h
const int kBlockSize = 8;
const float kLongPressMs = 1200.f;  // ui.cc: a long press does nothing

// From tides2/tides.cc.
const float kRoot[3] = {0.125f / kNativeRate, 2.0f / kNativeRate, 130.81f / kNativeRate};
const tides::Ratio kRatios[20] = {
    {0.0625f, 16}, {0.125f, 8}, {0.1666666f, 6}, {0.25f, 4}, {0.3333333f, 3},
    {0.5f, 2},     {0.6666666f, 3}, {0.75f, 4}, {0.8f, 5},    {1.0f, 1},
    {1.0f, 1},     {1.25f, 4},  {1.3333333f, 3}, {1.5f, 2},  {2.0f, 1},
    {3.0f, 1},     {4.0f, 1},   {6.0f, 1},     {8.0f, 1},    {16.0f, 1},
};
// From tides2/cv_reader.cc.
const float kShapeBreakpoints[] = {0.0f, 0.26f, 0.34f, 0.42f, 0.5f,
                                   0.58f, 0.66f, 0.74f, 1.0f, 1.0f};

const char* kRangeNames[3] = {"Low (0.125 Hz)", "Medium (2 Hz)", "High (C3, audio)"};
const char* kModeNames[3] = {"AD", "Looping (cycle)", "AR"};
const char* kOutputNames[4] = {"Gates", "Amplitude", "Slope / phase", "Frequency"};

inline float centerDetent(float x) {  // CvReader::CenterDetent
  if (x < 0.49f) {
    x *= 1.02040816f;
  } else if (x > 0.51f) {
    x -= 0.02f;
    x *= 1.02040816f;
  } else {
    x = 0.5f;
  }
  return x;
}

// CvReaderChannel: one-pole smoothed pot, attenuverter (cubic law) and CV.
struct CvChannel {
  float potLp = 0.f, attLp = 0.f, cvLp = 0.f;
  template <bool kHasPot, bool kHasAtt>
  float process(float pot, float potScale, float potOffset, float potCoef, float cv,
                float cvCoef, float att, float lo, float hi) {
    cvLp += cvCoef * (cv - cvLp);
    float a = 1.f;
    if (kHasAtt) {
      attLp += potCoef * (att - attLp);
      a = attLp - 0.5f;
      a = a * a * a * 8.f;
    }
    float value = cvLp * a;
    if (kHasPot) {
      potLp += potCoef * (pot - potLp);
      value += potLp * potScale + potOffset;
    }
    return clampf(value, lo, hi);
  }
};

class TidalMod2 : public Module {
 public:
  enum ParamIds {
    FREQUENCY_PARAM,
    SHAPE_PARAM,
    SLOPE_PARAM,
    SMOOTHNESS_PARAM,
    SHIFT_PARAM,
    SLOPE_ATT_PARAM,
    FREQUENCY_ATT_PARAM,
    SMOOTHNESS_ATT_PARAM,
    SHAPE_ATT_PARAM,
    SHIFT_ATT_PARAM,
    RANGE_BUTTON_PARAM,
    MODE_BUTTON_PARAM,
    OUTPUT_BUTTON_PARAM,
    // Hidden: the firmware's persistent state (settings.h).
    RANGE_PARAM,
    MODE_PARAM,
    OUTPUT_MODE_PARAM,
    COLOR_BLIND_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    SLOPE_INPUT,
    FM_INPUT,
    VOCT_INPUT,
    SMOOTHNESS_INPUT,
    SHAPE_INPUT,
    SHIFT_INPUT,
    TRIG_INPUT,
    CLOCK_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { OUT1_OUTPUT, OUT2_OUTPUT, OUT3_OUTPUT, OUT4_OUTPUT, NUM_OUTPUTS };
  // Pairs of {green, red}: range, mode, output mode, then the 4 output LEDs.
  enum LightIds {
    RANGE_LIGHT = 0,
    MODE_LIGHT = 2,
    OUTPUT_MODE_LIGHT = 4,
    OUT_LIGHT = 6,
    ZERO_LIGHT = 14,  // always 0: blue channel of the RGB-rendered setting LEDs
    NUM_LIGHTS
  };

  TidalMod2() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(FREQUENCY_PARAM, 0.f, 1.f, 0.5f, "Frequency", "semitones", 96.f, -48.f)
        .displayPrecision = 1;
    configParam(SHAPE_PARAM, 0.f, 1.f, 0.5f, "Shape", "%", 100.f);
    configParam(SLOPE_PARAM, 0.f, 1.f, 0.5f, "Slope", "%", 100.f);
    configParam(SMOOTHNESS_PARAM, 0.f, 1.f, 0.5f, "Smoothness", "%", 100.f);
    configParam(SHIFT_PARAM, 0.f, 1.f, 0.5f, "Shift/level", "%", 100.f);
    configParam(SLOPE_ATT_PARAM, -1.f, 1.f, 0.f, "Slope CV amount", "%", 100.f);
    configParam(FREQUENCY_ATT_PARAM, -1.f, 1.f, 0.f, "FM amount (fine tune if unpatched)",
                "%", 100.f);
    configParam(SMOOTHNESS_ATT_PARAM, -1.f, 1.f, 0.f, "Smoothness CV amount", "%", 100.f);
    configParam(SHAPE_ATT_PARAM, -1.f, 1.f, 0.f, "Shape CV amount", "%", 100.f);
    configParam(SHIFT_ATT_PARAM, -1.f, 1.f, 0.f, "Shift CV amount", "%", 100.f);
    configButton(RANGE_BUTTON_PARAM, "Frequency range");
    configButton(MODE_BUTTON_PARAM, "Ramp mode");
    configButton(OUTPUT_BUTTON_PARAM, "Output mode");
    // Defaults from tides2/settings.cc.
    configSwitch(RANGE_PARAM, 0.f, 2.f, 2.f, "Range",
                 {kRangeNames[0], kRangeNames[1], kRangeNames[2]})
        .hidden = true;
    configSwitch(MODE_PARAM, 0.f, 2.f, 1.f, "Ramp mode",
                 {kModeNames[0], kModeNames[1], kModeNames[2]})
        .hidden = true;
    configSwitch(OUTPUT_MODE_PARAM, 0.f, 3.f, 0.f, "Output mode",
                 {kOutputNames[0], kOutputNames[1], kOutputNames[2], kOutputNames[3]})
        .hidden = true;
    configSwitch(COLOR_BLIND_PARAM, 0.f, 1.f, 0.f, "Colour-blind LED mode", {"Off", "On"})
        .hidden = true;

    configInput(SLOPE_INPUT, "Slope");
    configInput(FM_INPUT, "FM");
    configInput(VOCT_INPUT, "V/Oct");
    configInput(SMOOTHNESS_INPUT, "Smoothness");
    configInput(SHAPE_INPUT, "Shape");
    configInput(SHIFT_INPUT, "Shift/level");
    configInput(TRIG_INPUT, "Trigger");
    configInput(CLOCK_INPUT, "Clock");
    configOutput(OUT1_OUTPUT, "Out 1");
    configOutput(OUT2_OUTPUT, "Out 2");
    configOutput(OUT3_OUTPUT, "Out 3");
    configOutput(OUT4_OUTPUT, "Out 4");

    // The firmware's DSP objects are zero-initialised globals: do the same
    // (Init() doesn't set every member).
    std::memset(static_cast<void*>(&poly_), 0, sizeof(poly_));
    std::memset(static_cast<void*>(&rampExtractor_), 0, sizeof(rampExtractor_));
    std::memset(static_cast<void*>(&ratioQuantizer_), 0, sizeof(ratioQuantizer_));
    poly_.Init();
    ratioQuantizer_.Init(20, 0.05f, false);
    rampExtractor_.Init(kNativeRate, 40.0f / kNativeRate);
    std::fill(&noGate_[0], &noGate_[kBlockSize], stmlib::GATE_FLAG_LOW);
    previousOutputMode_ = static_cast<tides::OutputMode>(intParam(OUTPUT_MODE_PARAM));
    adapter_.init(48000.f, kNativeRate, kBlockSize);
  }

  void onSampleRateChange(float sr) override { adapter_.init(sr, kNativeRate, kBlockSize); }

  void process(const ProcessArgs& args) override {
    nowMs_ += args.sampleTime * 1000.f;
    for (int b = 0; b < 3; ++b) handleButton(b);

    // TRIG and CLOCK go through the adapter so their edges keep native-rate timing.
    float in[2] = {inputs[TRIG_INPUT].getVoltage(), inputs[CLOCK_INPUT].getVoltage()};
    float out[4];
    adapter_.process(in, out, [&](const float* nin, float* nout, int n) {
      renderBlock(nin, nout, n);
    });
    for (int i = 0; i < 4; ++i) outputs[static_cast<size_t>(OUT1_OUTPUT + i)].setVoltage(out[i]);

    if ((args.frame & 31) == 0) updateLights(out);
  }

 private:
  int intParam(int id) const { return static_cast<int>(params[static_cast<size_t>(id)].value + 0.5f); }

  // Ui::OnSwitchReleased: a short press cycles the setting on release.
  void handleButton(int b) {
    bool pressed = params[static_cast<size_t>(RANGE_BUTTON_PARAM + b)].value > 0.5f;
    if (pressed && !buttonDown_[b]) pressStart_[b] = nowMs_;
    if (!pressed && buttonDown_[b] && nowMs_ - pressStart_[b] <= kLongPressMs) {
      static const int kTarget[3] = {RANGE_PARAM, MODE_PARAM, OUTPUT_MODE_PARAM};
      static const int kCount[3] = {3, 3, 4};
      Param& p = params[static_cast<size_t>(kTarget[b])];
      p.set(static_cast<float>((intParam(kTarget[b]) + 1) % kCount[b]));
    }
    buttonDown_[b] = pressed;
  }

  // CvReader::Read(), with ADC values replaced by voltages: the V/OCT and
  // FM inputs are 12 semitones per volt (calibration scales -60 and -96 for
  // +/-5 V and +/-8 V ADC spans), the other CVs +/-5 V for the full range.
  void readCv(tides::Parameters* p) {
    auto pot = [&](int id) { return params[static_cast<size_t>(id)].value; };
    auto att = [&](int id) { return (params[static_cast<size_t>(id)].value + 1.f) * 0.5f; };
    auto volts = [&](int id) { return inputs[static_cast<size_t>(id)].getVoltage(); };

    float note = cv_[0].process<true, false>(centerDetent(pot(FREQUENCY_PARAM)), 96.f, -48.f,
                                             0.003f, volts(VOCT_INPUT) * 12.f, 0.2f, 1.f,
                                             -96.f, 96.f);
    noteLp_ += 0.2f * (note - noteLp_);
    p->frequency = noteLp_;

    p->fm = cv_[1].process<false, true>(0.f, 0.f, 0.f, 0.003f, volts(FM_INPUT) * 12.f, 0.3f,
                                        att(FREQUENCY_ATT_PARAM), -96.f, 96.f);
    if (!inputs[FM_INPUT].isConnected()) p->fm = cv_[1].attLp * 2.f - 1.f;

    struct Row { int potId, attId, inputId; };
    const Row rows[4] = {{SHAPE_PARAM, SHAPE_ATT_PARAM, SHAPE_INPUT},
                         {SLOPE_PARAM, SLOPE_ATT_PARAM, SLOPE_INPUT},
                         {SMOOTHNESS_PARAM, SMOOTHNESS_ATT_PARAM, SMOOTHNESS_INPUT},
                         {SHIFT_PARAM, SHIFT_ATT_PARAM, SHIFT_INPUT}};
    float* dest[4] = {&p->shape, &p->slope, &p->smoothness, &p->shift};
    for (int i = 0; i < 4; ++i) {
      float potValue = pot(rows[i].potId);
      if (i == 0) {
        potValue = stmlib::Interpolate(kShapeBreakpoints, potValue, 8.0f);
      } else if (i == 3) {
        potValue = centerDetent(potValue);
      }
      *dest[i] = cv_[i + 2].process<true, true>(potValue, 1.f, 0.f, 0.003f,
                                                volts(rows[i].inputId) / 5.f, 0.1f,
                                                att(rows[i].attId), 0.f, 1.f);
    }
  }

  // tides2/tides.cc Process().
  void renderBlock(const float* nin, float* nout, int n) {
    tides::Parameters parameters;
    readCv(&parameters);

    const tides::RampMode rampMode = static_cast<tides::RampMode>(intParam(MODE_PARAM));
    const tides::OutputMode outputMode = static_cast<tides::OutputMode>(intParam(OUTPUT_MODE_PARAM));
    const int rangeSetting = intParam(RANGE_PARAM);
    const tides::Range range = rangeSetting < 2 ? tides::RANGE_CONTROL : tides::RANGE_AUDIO;
    const bool halfSpeed = outputMode >= tides::OUTPUT_MODE_SLOPE_PHASE;
    float transposition = clampf(parameters.frequency + parameters.fm, -128.f, 127.f);

    const bool trigPatched = inputs[TRIG_INPUT].isConnected();
    const bool clockPatched = inputs[CLOCK_INPUT].isConnected();

    // Gate flags. In the half-speed output modes the firmware works at SR/2
    // and only looks at every other gate sample.
    size_t size = static_cast<size_t>(n);
    const int step = halfSpeed ? 2 : 1;
    if (halfSpeed) size >>= 1;
    for (size_t i = 0; i < size; ++i) {
      for (int g = 0; g < 2; ++g) {
        bool high = false;
        for (int k = 0; k < step; ++k) {  // keep both samples' Schmitt state current
          gateIn_[g].process(nin[(i * step + k) * 2 + g]);
          if (k == 0) high = gateIn_[g].isHigh();
        }
        gateFlags_[g] = stmlib::ExtractGateFlags(gateFlags_[g], high);
        gates_[g][i] = gateFlags_[g];
      }
    }

    float frequency;
    if (clockPatched) {
      if (mustResetRampExtractor_) rampExtractor_.Reset();
      tides::Ratio r = ratioQuantizer_.Lookup(kRatios, 0.5f + transposition * 0.0105f);
      frequency = rampExtractor_.Process(range == tides::RANGE_AUDIO,
                                         range == tides::RANGE_AUDIO && rampMode == tides::RAMP_MODE_AR,
                                         r, gates_[1], ramp_, size);
      mustResetRampExtractor_ = false;
    } else {
      frequency = kRoot[rangeSetting] * stmlib::SemitonesToRatio(transposition);
      if (halfSpeed) frequency *= 2.0f;
      mustResetRampExtractor_ = true;
    }

    if (outputMode != previousOutputMode_) {
      poly_.Reset();
      previousOutputMode_ = outputMode;
    }

    poly_.Render(rampMode, outputMode, range, frequency, parameters.slope, parameters.shape,
                 parameters.smoothness, parameters.shift, trigPatched ? gates_[0] : noGate_,
                 !trigPatched && clockPatched ? ramp_ : nullptr, out_, size);

    for (size_t i = 0; i < size; ++i) {
      for (int j = 0; j < 4; ++j) {
        float v = clampf(out_[i].channel[j], -8.125f, 8.125f);  // DAC span
        if (halfSpeed) {
          nout[(2 * i) * 4 + j] = nout[(2 * i + 1) * 4 + j] = v;
        } else {
          nout[i * 4 + j] = v;
        }
      }
    }
  }

  // Ui::UpdateLEDs (palette green, yellow, red, off; MakeColor for colour-blind
  // mode) plus the four output LEDs (green positive, red negative).
  void setColor(int light, int value) {
    bool g = false, r = false;
    uint32_t ms = static_cast<uint32_t>(nowMs_);
    if (intParam(COLOR_BLIND_PARAM)) {
      uint32_t pwm = ms & 15;
      uint32_t tri = (ms >> 5) & 31;
      tri = tri < 16 ? tri : 31 - tri;
      if (value == 0) g = pwm < 4 + (tri >> 2);
      else if (value == 1) g = r = true;
      else if (value == 2) r = pwm == 0;
    } else {
      g = value == 0 || value == 1;
      r = value == 1 || value == 2;
    }
    lights[static_cast<size_t>(light)] = g ? 1.f : 0.f;
    lights[static_cast<size_t>(light + 1)] = r ? 1.f : 0.f;
  }

  void updateLights(const float* out) {
    setColor(MODE_LIGHT, intParam(MODE_PARAM));
    setColor(RANGE_LIGHT, intParam(RANGE_PARAM));
    setColor(OUTPUT_MODE_LIGHT, (intParam(OUTPUT_MODE_PARAM) + 3) % 4);
    for (int i = 0; i < 4; ++i) {
      lights[static_cast<size_t>(OUT_LIGHT + 2 * i)] = clampf(out[i] / 8.f, 0.f, 1.f);
      lights[static_cast<size_t>(OUT_LIGHT + 2 * i + 1)] = clampf(-out[i] / 8.f, 0.f, 1.f);
    }
  }

  tides::PolySlopeGenerator poly_;
  tides::RampExtractor rampExtractor_;
  stmlib::HysteresisQuantizer2 ratioQuantizer_;
  BlockAdapter<2, 4> adapter_;

  CvChannel cv_[6];
  float noteLp_ = 0.f;
  SchmittTrigger gateIn_[2];
  GateFlags gateFlags_[2] = {stmlib::GATE_FLAG_LOW, stmlib::GATE_FLAG_LOW};
  GateFlags gates_[2][kBlockSize];
  GateFlags noGate_[kBlockSize];
  float ramp_[kBlockSize];
  tides::PolySlopeGenerator::OutputSample out_[kBlockSize];
  tides::OutputMode previousOutputMode_;
  bool mustResetRampExtractor_ = true;

  double nowMs_ = 0.0;
  double pressStart_[3] = {0.0, 0.0, 0.0};
  bool buttonDown_[3] = {false, false, false};
};

void buildPanel(PanelSpec& p) {
  // 14 HP. Positions measured on the original panel (mm = px * 0.1285).
  p.hp = 14;
  p.bg = 0xFFF3F1EE;
  p.ink = 0xFF242426;
  p.title = "TIDAL MOD 2";
  const uint32_t teal = 0xFF14958C, ochre = 0xFFD9A030, red = 0xFFC0404B;
  const uint32_t green = 0xFF35E06A, ledRed = 0xFFFF3B30;
  const uint32_t white = 0xFFFFFFFF;

  // Small coloured "pills" standing for the hardware's setting icons.
  auto pill = [&](float x, float y, const char* text, uint32_t col, float w = 4.4f) {
    p.box(x - w * 0.5f, y - 1.9f, w, 3.8f, col, 1.9f);
    p.label(x, y, text, 2.2f, kAlignCenter, white);
  };

  // Range button + LED (top left), output mode (top right), ramp mode (centre).
  p.button(TidalMod2::RANGE_BUTTON_PARAM, 7.4f, 16.3f, 5.6f, "", 0xFFD8D9DB);
  // Setting LEDs mix red + green into yellow, so they are drawn as RGB lights.
  p.rgbLight(TidalMod2::RANGE_LIGHT + 1, TidalMod2::RANGE_LIGHT, TidalMod2::ZERO_LIGHT, 13.4f,
             16.3f, 2.8f);
  pill(18.6f, 16.3f, "1/8", teal);
  pill(23.4f, 16.3f, "2", ochre, 3.8f);
  pill(28.2f, 16.3f, "C3", red);
  p.label(7.4f, 11.6f, "RANGE", 2.4f);

  p.button(TidalMod2::OUTPUT_BUTTON_PARAM, 63.7f, 16.3f, 5.6f, "", 0xFFD8D9DB);
  p.rgbLight(TidalMod2::OUTPUT_MODE_LIGHT + 1, TidalMod2::OUTPUT_MODE_LIGHT,
             TidalMod2::ZERO_LIGHT, 57.7f, 16.3f, 2.8f);
  pill(42.6f, 16.3f, "AMP", teal, 5.4f);
  pill(48.0f, 16.3f, "PHS", ochre, 5.4f);
  pill(53.4f, 16.3f, "FRQ", red, 5.4f);
  p.label(63.7f, 11.6f, "OUTPUT", 2.4f);

  pill(30.6f, 26.8f, "AD", teal);
  pill(35.4f, 26.8f, "LOOP", ochre, 5.0f);
  pill(40.2f, 26.8f, "AR", red);
  p.rgbLight(TidalMod2::MODE_LIGHT + 1, TidalMod2::MODE_LIGHT, TidalMod2::ZERO_LIGHT, 35.4f,
             31.6f, 2.8f);
  p.button(TidalMod2::MODE_BUTTON_PARAM, 35.4f, 37.6f, 5.6f, "", 0xFFD8D9DB);

  // Main knobs.
  p.knob(TidalMod2::FREQUENCY_PARAM, 16.3f, 33.4f, 13.f);
  p.label(16.3f, 45.9f, "FREQUENCY", 3.2f);
  p.knob(TidalMod2::SHAPE_PARAM, 54.4f, 33.4f, 13.f);
  p.label(54.4f, 45.9f, "SHAPE", 3.2f);
  p.knob(TidalMod2::SMOOTHNESS_PARAM, 35.4f, 55.0f, 10.f);
  p.label(35.4f, 66.0f, "SMOOTHNESS", 3.0f);
  p.knob(TidalMod2::SLOPE_PARAM, 11.6f, 60.7f, 10.f);
  p.label(11.6f, 71.1f, "SLOPE", 3.0f);
  p.knob(TidalMod2::SHIFT_PARAM, 59.1f, 60.7f, 10.f);
  p.label(59.1f, 71.1f, "SHIFT/LEVEL", 3.0f);

  // Attenuverters and CV jacks.
  const float attX[5] = {9.3f, 22.5f, 35.4f, 48.4f, 61.4f};
  const int attIds[5] = {TidalMod2::SLOPE_ATT_PARAM, TidalMod2::FREQUENCY_ATT_PARAM,
                         TidalMod2::SMOOTHNESS_ATT_PARAM, TidalMod2::SHAPE_ATT_PARAM,
                         TidalMod2::SHIFT_ATT_PARAM};
  for (int i = 0; i < 5; ++i) {
    p.knob(attIds[i], attX[i], 80.9f, 6.5f);
    p.label(attX[i] - 3.6f, 76.0f, "-", 2.8f);
    p.label(attX[i] + 3.6f, 76.0f, "+", 2.8f);
  }
  const float jackX[6] = {6.7f, 18.6f, 29.6f, 41.1f, 52.7f, 64.0f};
  p.input(TidalMod2::SLOPE_INPUT, jackX[0], 96.4f);
  p.input(TidalMod2::FM_INPUT, jackX[1], 96.4f);
  p.input(TidalMod2::VOCT_INPUT, jackX[2], 96.4f, "V/OCT");
  p.input(TidalMod2::SMOOTHNESS_INPUT, jackX[3], 96.4f);
  p.input(TidalMod2::SHAPE_INPUT, jackX[4], 96.4f);
  p.input(TidalMod2::SHIFT_INPUT, jackX[5], 96.4f);
  // Dotted lines on the hardware: attenuverter -> its CV jack.
  const int attToJack[5] = {0, 1, 3, 4, 5};
  for (int i = 0; i < 5; ++i)
    p.line(attX[i], 84.6f, jackX[attToJack[i]], 92.8f, 0.3f, 0xFF8A8A8A);
  p.line(16.3f, 48.5f, attX[1], 76.8f, 0.3f, 0xFF8A8A8A);   // FREQUENCY
  p.line(35.4f, 68.4f, attX[2], 76.8f, 0.3f, 0xFF8A8A8A);   // SMOOTHNESS
  p.line(54.4f, 48.5f, attX[3], 76.8f, 0.3f, 0xFF8A8A8A);   // SHAPE

  p.input(TidalMod2::TRIG_INPUT, 6.7f, 110.8f, "TRIG");
  p.input(TidalMod2::CLOCK_INPUT, 18.2f, 110.8f, "CLOCK");
  p.box(23.8f, 102.4f, 45.3f, 14.5f, 0xFF5E5E60, 1.2f);
  const float outX[4] = {29.6f, 41.1f, 52.7f, 64.0f};
  const char* outNames[4] = {"1", "2", "3", "4"};
  for (int i = 0; i < 4; ++i) {
    p.output(TidalMod2::OUT1_OUTPUT + i, outX[i], 111.4f);
    p.bicolorLight(TidalMod2::OUT_LIGHT + 2 * i, TidalMod2::OUT_LIGHT + 2 * i + 1,
                   outX[i] - 3.2f, 105.0f, green, ledRed, 2.4f);
    p.label(outX[i] + 1.6f, 105.0f, outNames[i], 2.9f, kAlignCenter, white);
  }
}

Model model = [] {
  Model m;
  m.slug = "tidal-modulator-2";
  m.name = "Tidal Modulator 2";
  m.shortName = "TIDAL MOD 2";
  m.basedOn = "Tides (2018)";
  m.category = Category::Modulator;
  m.description = "Polyphonic slope generator: 4 related LFOs, envelopes or oscillators.";
  m.create = [] { return new TidalMod2(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
