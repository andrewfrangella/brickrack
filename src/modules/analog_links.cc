// BrickRack - GPL-3.0-or-later.
//
// Buffered Mult / Mixer, after Links. This is an original digital model of an
// analog circuit (DC-coupled op-amp buffers and summers), not firmware.
//
//   1:3  one input buffered to three outputs
//   2:2  two inputs summed, the sum on both outputs
//   3:1  three inputs summed to one output
//
// Everything is DC coupled with unity gain. Sums saturate softly near the
// +/-12 V rails of the rail-to-rail op-amps (about +/-11 V). Each section has a
// bicolour LED (green positive, red negative).

#include <algorithm>
#include <cmath>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"

namespace br {
namespace {

inline float railClip(float x, float knee = 9.f, float limit = 11.f) {
  float a = std::fabs(x);
  if (a <= knee) return x;
  float r = limit - knee;
  float y = knee + r * std::tanh((a - knee) / r);
  return x < 0.f ? -y : y;
}

class BufferedMult : public Module {
 public:
  enum ParamIds { NUM_PARAMS };
  enum InputIds { A_INPUT, B1_INPUT, B2_INPUT, C1_INPUT, C2_INPUT, C3_INPUT, NUM_INPUTS };
  enum OutputIds { A1_OUTPUT, A2_OUTPUT, A3_OUTPUT, B1_OUTPUT, B2_OUTPUT, C_OUTPUT, NUM_OUTPUTS };
  enum LightIds { A_LIGHT, B_LIGHT = A_LIGHT + 2, C_LIGHT = B_LIGHT + 2, NUM_LIGHTS = C_LIGHT + 2 };

  BufferedMult() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configInput(A_INPUT, "1:3 in");
    configInput(B1_INPUT, "2:2 in 1");
    configInput(B2_INPUT, "2:2 in 2");
    configInput(C1_INPUT, "3:1 in 1");
    configInput(C2_INPUT, "3:1 in 2");
    configInput(C3_INPUT, "3:1 in 3");
    configOutput(A1_OUTPUT, "1:3 out 1");
    configOutput(A2_OUTPUT, "1:3 out 2");
    configOutput(A3_OUTPUT, "1:3 out 3");
    configOutput(B1_OUTPUT, "2:2 out 1");
    configOutput(B2_OUTPUT, "2:2 out 2");
    configOutput(C_OUTPUT, "3:1 out");
  }

  void process(const ProcessArgs& args) override {
    float a = railClip(inputs[A_INPUT].getVoltage());
    float b = railClip(inputs[B1_INPUT].getVoltage() + inputs[B2_INPUT].getVoltage());
    float c = railClip(inputs[C1_INPUT].getVoltage() + inputs[C2_INPUT].getVoltage() +
                       inputs[C3_INPUT].getVoltage());
    outputs[A1_OUTPUT].setVoltage(a);
    outputs[A2_OUTPUT].setVoltage(a);
    outputs[A3_OUTPUT].setVoltage(a);
    outputs[B1_OUTPUT].setVoltage(b);
    outputs[B2_OUTPUT].setVoltage(b);
    outputs[C_OUTPUT].setVoltage(c);
    const float v[3] = {a, b, c};
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
  float pos_[3] = {0.f, 0.f, 0.f};
  float neg_[3] = {0.f, 0.f, 0.f};
};

void buildPanel(PanelSpec& p) {
  p.hp = 4;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  const float xl = 5.67f, xr = 14.5f;
  const uint32_t rule = 0xFFB8B8B8;
  const uint32_t g = 0xFF35E06A, r = 0xFFFF3B30;
  const float ly = 6.4f;  // labels sit under the jacks, as on the hardware

  p.bicolorLight(BufferedMult::A_LIGHT, BufferedMult::A_LIGHT + 1, 9.9f, 20.9f, g, r, 2.6f);
  p.label(15.6f, 20.9f, "1:3", 3.0f);
  p.input(BufferedMult::A_INPUT, xl, 28.4f, "IN", ly);
  p.output(BufferedMult::A1_OUTPUT, xr, 28.4f, "OUT", ly);
  p.output(BufferedMult::A2_OUTPUT, xl, 41.4f, "OUT", ly);
  p.output(BufferedMult::A3_OUTPUT, xr, 41.4f, "OUT", ly);
  p.line(1.2f, 52.2f, 19.1f, 52.2f, 0.6f, rule);

  p.bicolorLight(BufferedMult::B_LIGHT, BufferedMult::B_LIGHT + 1, 9.9f, 56.6f, g, r, 2.6f);
  p.label(15.6f, 56.6f, "2:2", 3.0f);
  p.input(BufferedMult::B1_INPUT, xl, 64.0f, "IN", ly);
  p.input(BufferedMult::B2_INPUT, xr, 64.0f, "IN", ly);
  p.output(BufferedMult::B1_OUTPUT, xl, 77.0f, "OUT", ly);
  p.output(BufferedMult::B2_OUTPUT, xr, 77.0f, "OUT", ly);
  p.line(1.2f, 87.6f, 19.1f, 87.6f, 0.6f, rule);

  p.bicolorLight(BufferedMult::C_LIGHT, BufferedMult::C_LIGHT + 1, 9.9f, 92.0f, g, r, 2.6f);
  p.label(15.6f, 92.0f, "3:1", 3.0f);
  p.input(BufferedMult::C1_INPUT, xl, 99.4f, "IN", ly);
  p.input(BufferedMult::C2_INPUT, xr, 99.4f, "IN", ly);
  p.input(BufferedMult::C3_INPUT, xl, 112.4f, "IN", ly);
  p.output(BufferedMult::C_OUTPUT, xr, 112.4f, "OUT", ly);
}

Model model = [] {
  Model m;
  m.slug = "buffered-mult";
  m.name = "Buffered Mult / Mixer";
  m.shortName = "MULT / MIX";
  m.basedOn = "Links";
  m.category = Category::Utility;
  m.description = "DC-coupled 1:3 buffered mult, 2:2 summing mult and 3:1 mixer.";
  m.create = [] { return new BufferedMult(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
