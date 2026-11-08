// BrickRack - GPL-3.0-or-later.
//
// Meta Modulator: wraps the Warps Modulator (warps/dsp/modulator.cc) at its
// native 96 kHz in 60-sample blocks, as warps.cc's FillBuffer() runs it.
// Parameters are filled the way cv_scaler.cc does (pot/CV low-pass filters,
// LEVEL CV as attenuated VCA control and internal-oscillator V/Oct, wavefolder
// bleed fix, easter-egg frequency-shifter mappings); the button, LEDs and the
// secret handshake follow ui.cc.

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"
#include "warps/dsp/modulator.h"

namespace br {
namespace {

const char* kCarrierShapes[4] = {"Off (external carrier)", "Sine / saw", "Triangle / pulse",
                                 "Saw / noise"};

// ui.cc palettes for the algorithm LED.
const uint8_t kPalette[10][3] = {
    {0, 192, 64}, {64, 255, 0},  {255, 255, 0}, {255, 64, 0},  {255, 0, 0},
    {255, 0, 64}, {255, 0, 255}, {0, 0, 255},   {0, 255, 192}, {0, 255, 192},
};
const uint8_t kEasterEggPalette[10][3] = {
    {0, 0, 64},     {0, 0, 255},  {0, 255, 192}, {0, 192, 64}, {64, 255, 0},
    {255, 255, 0},  {255, 192, 0}, {255, 64, 0}, {255, 0, 0},  {255, 0, 0},
};

// Warps' CV ADC spans about 9.17 V (from the C1 = 0.8 / C3 = 0.5818 readings
// documented in cv_scaler.h), so a CV of x volts reads as x / 9.17.
constexpr float kCvScale = 1.f / 9.17f;

class MetaModulator : public Module {
 public:
  enum ParamIds {
    ALGORITHM_PARAM,
    TIMBRE_PARAM,
    STATE_PARAM,  // INT. OSC button
    LEVEL1_PARAM,
    LEVEL2_PARAM,
    CARRIER_SHAPE_PARAM,  // hidden: 0 = external, 1..3 = internal oscillator shape
    NUM_PARAMS
  };
  enum InputIds {
    LEVEL1_INPUT,
    LEVEL2_INPUT,
    ALGORITHM_INPUT,
    TIMBRE_INPUT,
    CARRIER_INPUT,
    MODULATOR_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { MODULATOR_OUTPUT, AUX_OUTPUT, NUM_OUTPUTS };
  enum LightIds {
    CARRIER_RED_LIGHT,
    CARRIER_GREEN_LIGHT,
    ALGO_RED_LIGHT,
    ALGO_GREEN_LIGHT,
    ALGO_BLUE_LIGHT,
    OFF_LIGHT,
    NUM_LIGHTS
  };

  // lp_[] indices (warps' ADC channel order).
  enum { LEVEL1_POT, LEVEL2_POT, ALGO_POT, PARAM_POT, LEVEL1_CV, LEVEL2_CV, ALGO_CV, PARAM_CV,
         NUM_CHANNELS };

  MetaModulator() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(ALGORITHM_PARAM, 0.f, 8.f, 0.f, "Algorithm", "", 1.f, 1.f);  // shown 1..9
    configParam(TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Timbre", "%", 100.f);
    configButton(STATE_PARAM, "Internal oscillator");
    configParam(LEVEL1_PARAM, 0.f, 1.f, 1.f, "External oscillator amplitude / internal oscillator frequency", "%", 100.f);
    configParam(LEVEL2_PARAM, 0.f, 1.f, 1.f, "Modulator amplitude", "%", 100.f);
    configSwitch(CARRIER_SHAPE_PARAM, 0.f, 3.f, 0.f, "Internal oscillator",
                 std::vector<std::string>(kCarrierShapes, kCarrierShapes + 4))
        .hidden = true;

    configInput(LEVEL1_INPUT, "Level 1 / V/Oct");
    configInput(LEVEL2_INPUT, "Level 2");
    configInput(ALGORITHM_INPUT, "Algorithm");
    configInput(TIMBRE_INPUT, "Timbre");
    configInput(CARRIER_INPUT, "Carrier");
    configInput(MODULATOR_INPUT, "Modulator");
    configOutput(MODULATOR_OUTPUT, "Out");
    configOutput(AUX_OUTPUT, "Aux");

    std::memset(static_cast<void*>(&modulator_), 0, sizeof(modulator_));
    modulator_.Init(kNativeRate);
    std::fill(lp_, lp_ + NUM_CHANNELS, 0.f);
    adapter_.init(48000.f, kNativeRate, kBlockSize);
  }

  void onSampleRateChange(float sampleRate) override {
    adapter_.init(sampleRate, kNativeRate, kBlockSize);
  }

  void process(const ProcessArgs& args) override {
    if (button_.process(params[STATE_PARAM].value)) onButton(args.frame);

    modulator_.set_easter_egg(easterEgg_);
    // The codec takes 16 Vpp at full scale.
    float in[2] = {inputs[CARRIER_INPUT].getVoltage() / 8.f,
                   inputs[MODULATOR_INPUT].getVoltage() / 8.f};
    float out[2];
    adapter_.process(in, out, [&](const float* nin, float* nout, int n) {
      renderBlock(nin, nout, n);
    });
    outputs[MODULATOR_OUTPUT].setVoltage(out[0] * 5.f);
    outputs[AUX_OUTPUT].setVoltage(out[1] * 5.f);

    if ((args.frame & 63) == 0) updateLights(args.frame * 1000.0 / args.sampleRate);
  }

  void saveState(StateMap& s) const override { s["easterEgg"] = easterEgg_ ? "1" : "0"; }
  void loadState(const StateMap& s) override {
    auto it = s.find("easterEgg");
    if (it != s.end()) easterEgg_ = it->second == "1";
  }
  void onReset() override { easterEgg_ = false; }
  void appendMenu(std::vector<MenuAction>& items) override {
    items.push_back({"Frequency shifter (easter egg)",
                     [this] { return std::string(easterEgg_ ? "ON" : "OFF"); },
                     [this](int) { toggleEasterEgg(); }});
  }

 private:
  static constexpr float kNativeRate = 96000.f;
  static constexpr int kBlockSize = 60;

  // Ui::OnSwitchPressed(): cycle the carrier, unless the secret handshake
  // (LEVEL 1, LEVEL 2 and TIMBRE fully CCW, ALGORITHM at 2, 4, 3, 6, 1, 5 on
  // six consecutive presses) toggles the frequency shifter.
  void onButton(int64_t frame) {
    for (int i = 0; i < 5; ++i) handshake_[i] = handshake_[i + 1];
    handshake_[5] = easterEggDigit();
    const uint8_t expected[6] = {2, 4, 3, 6, 1, 5};
    if (std::equal(handshake_, handshake_ + 6, expected)) {
      toggleEasterEgg();
    } else {
      int shape = static_cast<int>(params[CARRIER_SHAPE_PARAM].value + 0.5f);
      params[CARRIER_SHAPE_PARAM].set(static_cast<float>((shape + 1) & 3));
    }
  }

  uint8_t easterEggDigit() const {
    if (lp_[LEVEL1_POT] < 0.05f && lp_[LEVEL2_POT] < 0.05f && lp_[PARAM_POT] < 0.05f) {
      return static_cast<uint8_t>(lp_[ALGO_POT] * 8.f + 0.5f);
    }
    return 0;
  }

  void toggleEasterEgg() {
    easterEgg_ = !easterEgg_;
    params[CARRIER_SHAPE_PARAM].set(1.f);
    danceStart_ = -1.0;  // UI_MODE_EASTER_EGG_DANCE, started by updateLights()
  }

  // CvScaler::Read().
  void readCv(warps::Parameters* p) {
    const float pots[4] = {params[LEVEL1_PARAM].value, params[LEVEL2_PARAM].value,
                           params[ALGORITHM_PARAM].value / 8.f, params[TIMBRE_PARAM].value};
    const int cvIn[4] = {LEVEL1_INPUT, LEVEL2_INPUT, ALGORITHM_INPUT, TIMBRE_INPUT};
    for (int i = 0; i < 4; ++i) {
      lp_[i] += 0.33f * 0.08f * (pots[i] - lp_[i]);
      lp_[4 + i] += 0.08f * (inputs[cvIn[i]].getVoltage() * kCvScale - lp_[4 + i]);
    }
    // BIND(): LEVEL pots attenuate their CV, ALGORITHM / TIMBRE CVs add.
    for (int i = 0; i < 2; ++i) {
      float pot = lp_[LEVEL1_POT + i];
      p->channel_drive[i] = clampf(pot * pot * lp_[LEVEL1_CV + i] * 1.6f, 0.f, 1.f);
    }
    // (The firmware also runs the ALGORITHM pot through lut_pot_curve, which
    // only compensates the dead zones of the physical pot: not needed here.)
    p->modulation_algorithm = clampf(lp_[ALGO_POT] + lp_[ALGO_CV] * 2.f, 0.f, 1.f);
    // Just below 1: Interpolate(lut_xfade_in/out, x, 256) reads entry 257 of
    // 257 at x = 1.
    p->modulation_parameter = clampf(lp_[PARAM_POT] + lp_[PARAM_CV] * 2.f, 0.f, 0.9999f);

    // Prevent wavefolder bleed caused by a slight offset in the pot or ADC.
    if (p->modulation_algorithm <= 0.125f) {
      p->modulation_algorithm = clampf(p->modulation_algorithm * 1.08f - 0.01f, 0.f, 1.f);
    }

    // Easter egg parameter mappings.
    p->frequency_shift_pot = lp_[ALGO_POT];
    p->frequency_shift_cv = clampf(lp_[ALGO_CV] * 2.f, -1.f, 1.f);
    // Just below 1, like modulation_parameter (lut_sin + 256 read at 1024).
    p->phase_shift = clampf(lp_[ALGO_POT] + lp_[ALGO_CV] * 2.f, 0.f, 0.9999f);

    // Internal oscillator: LEVEL 1 CV is its V/Oct input (12 semitones / V).
    // The ADC covers pitch_offset + [0, 1] * pitch_scale = -10..100 semitones
    // with the default calibration (settings.cc), i.e. -0.83..+8.33 V.
    float note = clampf(inputs[LEVEL1_INPUT].getVoltage(), -10.f / 12.f, 100.f / 12.f) * 12.f;
    float interval = note - noteCv_;
    if (interval < -0.4f || interval > 0.4f) {
      noteCv_ = note;
    } else {
      noteCv_ += 0.1f * interval;
    }
    note = 60.f * params[LEVEL1_PARAM].value + 12.f;
    notePot_ += 0.1f * (note - notePot_);
    p->note = notePot_ + noteCv_;

    // Normalisation probe: unpatched LEVEL jacks leave the pot in control.
    for (int i = 0; i < 2; ++i) {
      if (!inputs[LEVEL1_INPUT + i].isConnected()) {
        float pot = lp_[LEVEL1_POT + i];
        p->channel_drive[i] = pot * pot;
      }
    }
    if (!inputs[LEVEL1_INPUT].isConnected()) p->note = notePot_ + 24.f;
    // Keep the internal oscillator below 48 kHz (the native Nyquist). Above
    // it, with phase modulation from the CARRIER input, the polyBLEP phase
    // increment exceeds 1 and the phase (and the oscillator's output, which
    // feeds the wavefolder table) runs away. The hardware can reach this
    // corner (LEVEL 1 at max + 8 V) but it is only aliasing there.
    p->note = std::min(p->note, 150.f);

    p->carrier_shape = static_cast<int32_t>(params[CARRIER_SHAPE_PARAM].value + 0.5f);
  }

  // warps.cc: FillBuffer().
  void renderBlock(const float* nin, float* nout, int n) {
    readCv(modulator_.mutable_parameters());
    // CvScaler::DetectAudioNormalization() mutes unpatched audio inputs.
    const bool patched[2] = {inputs[CARRIER_INPUT].isConnected(),
                             inputs[MODULATOR_INPUT].isConnected()};
    for (int i = 0; i < n; ++i) {
      in_[i].l = patched[0] ? toShort(nin[2 * i]) : 0;
      in_[i].r = patched[1] ? toShort(nin[2 * i + 1]) : 0;
    }
    modulator_.Process(in_, out_, static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      nout[2 * i] = out_[i].l / 32768.f;
      nout[2 * i + 1] = out_[i].r / 32768.f;
    }
  }

  // Symmetric clip (+-32767, like a codec): the firmware's float conversions
  // assume |x| <= 1, e.g. the phase modulation 1 + l / 32768 stays > 0.
  static short toShort(float x) {
    float v = clampf(x * 32768.f, -32767.f, 32767.f);
    return static_cast<short>(v);
  }

  // Ui::Poll(), normal mode and easter-egg dance.
  void updateLights(double ms) {
    const warps::Parameters& p = modulator_.parameters();
    int shape = static_cast<int>(params[CARRIER_SHAPE_PARAM].value + 0.5f);
    float rgb[3];
    if (danceStart_ < 0.0) danceStart_ = ms;
    if (danceStart_ > 0.0 && ms - danceStart_ < 6000.0) {
      int64_t t = static_cast<int64_t>(ms);
      bool blink = (t & 127) > 64;
      const uint8_t* c = kEasterEggPalette[(t >> 9) % 9];
      for (int i = 0; i < 3; ++i) rgb[i] = c[i] / 255.f;
      lights[CARRIER_RED_LIGHT] = 0.f;
      lights[CARRIER_GREEN_LIGHT] = blink ? 1.f : 0.f;
    } else {
      danceStart_ = 0.0;
      float zone = (easterEgg_ ? p.phase_shift : p.modulation_algorithm) * 8.f;
      const uint8_t(*palette)[3] = easterEgg_ ? kEasterEggPalette : kPalette;
      int zi = std::min(std::max(static_cast<int>(zone), 0), 8);
      float zf = zone - zi;
      for (int i = 0; i < 3; ++i) {
        rgb[i] = (palette[zi][i] + (palette[zi + 1][i] - palette[zi][i]) * zf) / 255.f;
      }
      lights[CARRIER_RED_LIGHT] = shape >= 2 ? 1.f : 0.f;
      lights[CARRIER_GREEN_LIGHT] = (shape > 0 && shape <= 2) ? 1.f : 0.f;
    }
    lights[ALGO_RED_LIGHT] = rgb[0];
    lights[ALGO_GREEN_LIGHT] = rgb[1];
    lights[ALGO_BLUE_LIGHT] = rgb[2];
    lights[OFF_LIGHT] = 0.f;
  }

  warps::Modulator modulator_;
  BlockAdapter<2, 2> adapter_;
  warps::ShortFrame in_[kBlockSize];
  warps::ShortFrame out_[kBlockSize];
  float lp_[NUM_CHANNELS];
  float noteCv_ = 0.f, notePot_ = 0.f;
  bool easterEgg_ = false;
  uint8_t handshake_[6] = {0, 0, 0, 0, 0, 0};
  double danceStart_ = 0.0;
  ButtonEdge button_;
};

void buildPanel(PanelSpec& p) {
  p.hp = 10;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  p.title = "META MOD";
  const uint32_t red = 0xFFC8414B;
  const uint32_t teal = 0xFF168A8C;
  const float cx = 25.2f, cy = 33.0f;

  // Algorithm ring with the nine algorithm markers (zone k sits where the
  // knob points at k / 8 of its travel).
  p.display(cx - 21.5f, cy - 21.5f, 43.f, 43.f,
            [](Canvas& c, float x, float y, float w, float h, float s, const Module*) {
              const float mx = x + w * 0.5f, my = y + h * 0.5f, r = 19.3f * s;
              c.strokeCircle(mx, my, r, std::max(1.f, 0.35f * s), 0xFFC4C4C4);
              for (int k = 0; k < 9; ++k) {
                float a = (0.75f + 1.5f * k / 8.f) * 3.14159265f;
                float px = mx + std::cos(a) * r, py = my + std::sin(a) * r;
                c.fillCircle(px, py, 1.9f * s, 0xFF5E5E60);
                const char digit[2] = {static_cast<char>('1' + k), 0};
                c.text(px, py, digit, std::max(10.f, 2.4f * s), 0xFFFFFFFF,
                       kAlignCenter);
              }
            });
  p.rgbLight(MetaModulator::ALGO_RED_LIGHT, MetaModulator::ALGO_GREEN_LIGHT,
             MetaModulator::ALGO_BLUE_LIGHT, cx, 23.2f, 4.f);
  p.knob(MetaModulator::ALGORITHM_PARAM, cx, 35.0f, 13.f);
  p.label(cx, 54.4f, "ALGORITHM", 3.3f);

  // Internal oscillator button + LED.
  p.rgbLight(MetaModulator::CARRIER_RED_LIGHT, MetaModulator::CARRIER_GREEN_LIGHT,
             MetaModulator::OFF_LIGHT, 8.0f, 58.4f, 2.6f);
  p.button(MetaModulator::STATE_PARAM, 8.0f, 64.3f, 5.f, "", 0xFFD8D9DB);
  p.box(12.6f, 62.1f, 11.6f, 4.4f, red, 2.2f);
  p.label(18.4f, 64.3f, "INT. OSC", 2.8f, kAlignCenter, 0xFFFFFFFF);

  p.knob(MetaModulator::TIMBRE_PARAM, 38.6f, 65.5f, 9.f);
  p.label(38.6f, 75.8f, "TIMBRE", 3.2f);

  // Level knobs with their coloured arcs.
  p.display(1.f, 68.f, 28.f, 15.f,
            [](Canvas& c, float x, float y, float w, float h, float s, const Module*) {
              const uint32_t red = 0xFFC8414B, teal = 0xFF168A8C;
              const float a0 = 0.75f * 3.14159265f, a1 = 2.25f * 3.14159265f;
              c.arc(x + 7.0f * s, y + 7.2f * s, 5.6f * s, a0, a1, std::max(1.5f, 0.7f * s), red);
              c.arc(x + 20.1f * s, y + 7.2f * s, 5.6f * s, a0, a1, std::max(1.5f, 0.7f * s), teal);
            });
  p.knob(MetaModulator::LEVEL1_PARAM, 8.0f, 75.2f, 9.f);
  p.knob(MetaModulator::LEVEL2_PARAM, 21.1f, 75.2f, 9.f);

  p.line(3.f, 83.3f, 47.8f, 83.3f, 0.4f, 0xFFC4C4C4);

  // CV row.
  p.box(2.4f, 87.0f, 9.4f, 4.2f, red, 2.1f);
  p.label(7.1f, 89.1f, "LEVEL", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.box(14.3f, 87.0f, 9.4f, 4.2f, teal, 2.1f);
  p.label(19.0f, 89.1f, "LEVEL", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.input(MetaModulator::LEVEL1_INPUT, 7.1f, 96.4f);
  p.input(MetaModulator::LEVEL2_INPUT, 19.0f, 96.4f);
  p.input(MetaModulator::ALGORITHM_INPUT, 31.4f, 96.4f, "ALGO", -7.3f);
  p.input(MetaModulator::TIMBRE_INPUT, 43.4f, 96.4f, "TIMBRE", -7.3f);

  // Audio row.
  p.box(4.9f, 102.6f, 4.4f, 4.4f, red, 2.2f);
  p.label(7.1f, 104.8f, "1", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.box(16.8f, 102.6f, 4.4f, 4.4f, teal, 2.2f);
  p.label(19.0f, 104.8f, "2", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.input(MetaModulator::CARRIER_INPUT, 7.1f, 111.2f);
  p.input(MetaModulator::MODULATOR_INPUT, 19.0f, 111.2f);
  p.box(26.3f, 102.4f, 22.0f, 14.4f, 0xFF5E5E60, 1.2f);
  p.output(MetaModulator::MODULATOR_OUTPUT, 31.4f, 111.2f);
  p.output(MetaModulator::AUX_OUTPUT, 43.4f, 111.2f);
  p.label(31.4f, 105.0f, "OUT", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.label(43.4f, 105.0f, "AUX", 2.7f, kAlignCenter, 0xFFFFFFFF);
}

Model model = [] {
  Model m;
  m.slug = "meta-modulator";
  m.name = "Meta Modulator";
  m.shortName = "META MOD";
  m.basedOn = "Warps";
  m.category = Category::Effect;
  m.description = "Cross-modulation: crossfade, folding, ring mod, XOR, comparator, vocoder.";
  m.create = [] { return new MetaModulator(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
