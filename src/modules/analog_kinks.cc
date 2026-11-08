// BrickRack - GPL-3.0-or-later.
//
// Rectifier / Logic / S&H, after Kinks. This is an original digital model of
// an analog circuit (precision rectifiers, diode min/max, JFET sample & hold
// and a transistor noise source), not firmware.
//
//   SIGN   inverted, half-wave rectified (positive part) and full-wave
//          rectified copies of the input
//   LOGIC  analog OR (max) and AND (min) of two inputs
//   S&H    the input is sampled on each rising edge of TRIG; the input is
//          normalled to the module's white-noise output
//
// Outputs saturate softly near the +/-12 V rails. LEDs: green positive,
// red negative (sign section shows the input, logic shows MAX, S&H the output).

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"

namespace br {
namespace {

inline float railClip(float x, float knee = 9.f, float limit = 10.5f) {
  float a = std::fabs(x);
  if (a <= knee) return x;
  float r = limit - knee;
  float y = knee + r * std::tanh((a - knee) / r);
  return x < 0.f ? -y : y;
}

class RectLogic : public Module {
 public:
  enum ParamIds { NUM_PARAMS };
  enum InputIds { SIGN_INPUT, A_INPUT, B_INPUT, SH_INPUT, TRIG_INPUT, NUM_INPUTS };
  enum OutputIds {
    INV_OUTPUT,
    HALF_OUTPUT,
    FULL_OUTPUT,
    MAX_OUTPUT,
    MIN_OUTPUT,
    NOISE_OUTPUT,
    SH_OUTPUT,
    NUM_OUTPUTS
  };
  enum LightIds {
    SIGN_LIGHT,
    LOGIC_LIGHT = SIGN_LIGHT + 2,
    SH_LIGHT = LOGIC_LIGHT + 2,
    NUM_LIGHTS = SH_LIGHT + 2
  };

  RectLogic() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configInput(SIGN_INPUT, "Sign in");
    configInput(A_INPUT, "Logic in A");
    configInput(B_INPUT, "Logic in B");
    configInput(SH_INPUT, "S&H in (normalled to noise)");
    configInput(TRIG_INPUT, "S&H trigger");
    configOutput(INV_OUTPUT, "Inverted");
    configOutput(HALF_OUTPUT, "Half-wave rectified");
    configOutput(FULL_OUTPUT, "Full-wave rectified");
    configOutput(MAX_OUTPUT, "Max (OR)");
    configOutput(MIN_OUTPUT, "Min (AND)");
    configOutput(NOISE_OUTPUT, "Noise");
    configOutput(SH_OUTPUT, "S&H out");
  }

  void process(const ProcessArgs& args) override {
    float in = railClip(inputs[SIGN_INPUT].getVoltage());
    outputs[INV_OUTPUT].setVoltage(-in);
    outputs[HALF_OUTPUT].setVoltage(std::max(in, 0.f));
    outputs[FULL_OUTPUT].setVoltage(std::fabs(in));

    float a = railClip(inputs[A_INPUT].getVoltage());
    float b = railClip(inputs[B_INPUT].getVoltage());
    float mx = std::max(a, b);
    outputs[MAX_OUTPUT].setVoltage(mx);
    outputs[MIN_OUTPUT].setVoltage(std::min(a, b));

    // Transistor junction noise: roughly Gaussian (sum of three uniforms),
    // ~2 V RMS, with a gentle low-pass from the amplifier's bandwidth.
    float n = (uniform() + uniform() + uniform() - 1.5f) * 4.f;
    noise_ += (n - noise_) * 0.85f;
    outputs[NOISE_OUTPUT].setVoltage(noise_);
    float shIn = inputs[SH_INPUT].isConnected() ? railClip(inputs[SH_INPUT].getVoltage()) : noise_;
    if (trig_.process(inputs[TRIG_INPUT].getVoltage())) held_ = shIn;
    outputs[SH_OUTPUT].setVoltage(held_);

    const float v[3] = {in, mx, held_};
    for (int i = 0; i < 3; ++i) {
      pos_[i] = std::max(pos_[i] * 0.9995f, v[i]);
      neg_[i] = std::max(neg_[i] * 0.9995f, -v[i]);
    }
    if ((args.frame & 63) == 0) {
      for (int i = 0; i < 3; ++i) {
        lights[static_cast<size_t>(2 * i)] = std::min(1.f, pos_[i] * 0.2f);
        lights[static_cast<size_t>(2 * i + 1)] = std::min(1.f, neg_[i] * 0.2f);
      }
    }
  }

 private:
  float uniform() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return static_cast<float>(rng_ >> 8) * (1.f / 16777216.f);
  }

  uint32_t rng_ = 0x9E3779B9u;
  float noise_ = 0.f;
  float held_ = 0.f;
  SchmittTrigger trig_;
  float pos_[3] = {0.f, 0.f, 0.f};
  float neg_[3] = {0.f, 0.f, 0.f};
};

void buildPanel(PanelSpec& p) {
  p.hp = 4;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  p.title = "";  // the full title does not fit 4 HP: two lines below
  p.label(10.16f, 6.6f, "RECT / LOGIC", 2.9f);
  p.label(10.16f, 10.2f, "S&H", 2.9f);
  const float xl = 5.14f, xr = 14.65f, ly = 6.3f;
  const uint32_t rule = 0xFFB8B8B8;
  const uint32_t g = 0xFF35E06A, r = 0xFFFF3B30;

  p.bicolorLight(RectLogic::SIGN_LIGHT, RectLogic::SIGN_LIGHT + 1, xl, 20.9f, g, r, 2.6f);
  p.label(xr, 20.9f, "SIGN", 3.0f);
  p.input(RectLogic::SIGN_INPUT, xl, 28.6f, "IN", ly);
  p.output(RectLogic::INV_OUTPUT, xr, 28.6f, "INV", ly);
  p.output(RectLogic::HALF_OUTPUT, xl, 41.4f, "HALF", ly);
  p.output(RectLogic::FULL_OUTPUT, xr, 41.4f, "FULL", ly);
  p.line(1.2f, 52.0f, 19.1f, 52.0f, 0.6f, rule);

  p.bicolorLight(RectLogic::LOGIC_LIGHT, RectLogic::LOGIC_LIGHT + 1, xl, 56.4f, g, r, 2.6f);
  p.label(xr, 56.4f, "LOGIC", 3.0f);
  p.input(RectLogic::A_INPUT, xl, 64.0f, "IN A", ly);
  p.input(RectLogic::B_INPUT, xr, 64.0f, "IN B", ly);
  p.output(RectLogic::MAX_OUTPUT, xl, 76.8f, "MAX", ly);
  p.output(RectLogic::MIN_OUTPUT, xr, 76.8f, "MIN", ly);
  p.line(1.2f, 87.4f, 19.1f, 87.4f, 0.6f, rule);

  p.bicolorLight(RectLogic::SH_LIGHT, RectLogic::SH_LIGHT + 1, xl, 91.8f, g, r, 2.6f);
  p.label(xr, 91.8f, "S&H", 3.0f);
  p.input(RectLogic::SH_INPUT, xl, 99.4f, "IN", ly);
  p.input(RectLogic::TRIG_INPUT, xr, 99.4f, "TRIG", ly);
  p.output(RectLogic::NOISE_OUTPUT, xl, 112.2f, "NOISE", ly);
  p.output(RectLogic::SH_OUTPUT, xr, 112.2f, "OUT", ly);
}

Model model = [] {
  Model m;
  m.slug = "rectifier-logic";
  m.name = "Rectifier / Logic / S&H";
  m.shortName = "RECT / LOGIC / S&H";
  m.basedOn = "Kinks";
  m.category = Category::Utility;
  m.description = "Inverter and half/full-wave rectifier, analog max/min, noise and sample & hold.";
  m.create = [] { return new RectLogic(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
