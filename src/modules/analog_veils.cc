// BrickRack - GPL-3.0-or-later.
//
// Quad VCA, after Veils. This is an original digital model of an analog
// circuit (V2164 VCAs, op-amp summing chain), not firmware: Veils has no code.
//
// Per channel: the GAIN knob scales the CV input, which is normalled to +5 V,
// so with nothing patched the knob sets the gain directly. The RESPONSE
// trimmer blends the control law from exponential (cubic, audio taper) to
// linear. CV above +5 V amplifies up to x2 (+6 dB). Unpatched outputs are
// summed into the next channel (the hardware's normalisation chain); the sum
// saturates softly at the op-amp rails (about +/-10.5 V). Bicolour LEDs show
// each output (green positive, red negative).

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"

namespace br {
namespace {

// Linear below `knee`, then a tanh shoulder that never exceeds `limit`.
inline float railClip(float x, float knee = 8.f, float limit = 10.5f) {
  float a = std::fabs(x);
  if (a <= knee) return x;
  float r = limit - knee;
  float y = knee + r * std::tanh((a - knee) / r);
  return x < 0.f ? -y : y;
}

class QuadVca : public Module {
 public:
  enum ParamIds { GAIN_PARAM, RESPONSE_PARAM = GAIN_PARAM + 4, NUM_PARAMS = RESPONSE_PARAM + 4 };
  enum InputIds { IN_INPUT, CV_INPUT = IN_INPUT + 4, NUM_INPUTS = CV_INPUT + 4 };
  enum OutputIds { OUT_OUTPUT, NUM_OUTPUTS = OUT_OUTPUT + 4 };
  enum LightIds { OUT_LIGHT, NUM_LIGHTS = OUT_LIGHT + 8 };  // [ch*2 + {green, red}]

  QuadVca() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < 4; ++i) {
      char n[48];
      std::snprintf(n, sizeof(n), "Channel %d gain", i + 1);
      configParam(GAIN_PARAM + i, 0.f, 1.f, 0.f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Channel %d response (exp - lin)", i + 1);
      configParam(RESPONSE_PARAM + i, 0.f, 1.f, 1.f, n, "% lin", 100.f);
      std::snprintf(n, sizeof(n), "Channel %d in", i + 1);
      configInput(IN_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Channel %d CV (normalled to +5 V)", i + 1);
      configInput(CV_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Channel %d out", i + 1);
      configOutput(OUT_OUTPUT + i, n);
    }
  }

  void process(const ProcessArgs& args) override {
    float acc = 0.f;
    for (int i = 0; i < 4; ++i) {
      // Knob smoothing (~3 ms) avoids zipper noise from UI steps.
      knob_[i] += (params[GAIN_PARAM + i].value - knob_[i]) * 0.008f;
      float cv = inputs[CV_INPUT + i].isConnected() ? inputs[CV_INPUT + i].getVoltage() * 0.2f : 1.f;
      float g = clampf(knob_[i] * cv, 0.f, 2.f);
      float r = params[RESPONSE_PARAM + i].value;
      float gain = std::min(2.f, r * g + (1.f - r) * g * g * g);
      acc = railClip(acc + inputs[IN_INPUT + i].getVoltage() * gain);
      outputs[OUT_OUTPUT + i].setVoltage(acc);
      pos_[i] = std::max(pos_[i] * 0.9995f, acc);
      neg_[i] = std::max(neg_[i] * 0.9995f, -acc);
      if (outputs[OUT_OUTPUT + i].isConnected()) acc = 0.f;
    }
    if ((args.frame & 63) == 0) {
      for (int i = 0; i < 4; ++i) {
        lights[static_cast<size_t>(OUT_LIGHT + 2 * i)] = std::min(1.f, pos_[i] * 0.2f);
        lights[static_cast<size_t>(OUT_LIGHT + 2 * i + 1)] = std::min(1.f, neg_[i] * 0.2f);
      }
    }
  }

 private:
  float knob_[4] = {0.f, 0.f, 0.f, 0.f};
  float pos_[4] = {0.f, 0.f, 0.f, 0.f};
  float neg_[4] = {0.f, 0.f, 0.f, 0.f};
};

void dotted(PanelSpec& p, float x0, float y0, float x1, float y1, uint32_t col,
            float step = 1.1f) {
  float dx = x1 - x0, dy = y1 - y0;
  float len = std::sqrt(dx * dx + dy * dy);
  int n = std::max(1, static_cast<int>(len / step));
  for (int k = 0; k < n; k += 2) {
    float t0 = static_cast<float>(k) / n;
    float t1 = static_cast<float>(k) / n + 0.5f / n;
    p.line(x0 + dx * t0, y0 + dy * t0, x0 + dx * t1, y0 + dy * t1, 0.5f, col);
  }
}

void buildPanel(PanelSpec& p) {
  p.hp = 12;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  const uint32_t dark = 0xFF5E5E60;
  const uint32_t white = 0xFFFFFFFF;
  const uint32_t ink = 0xFF2B2B2E;
  p.box(47.3f, 12.6f, 11.2f, 103.6f, dark, 1.2f);
  const char* nums[4] = {"1", "2", "3", "4"};
  for (int i = 0; i < 4; ++i) {
    float dy = i * 26.6f;
    // Gain knob, LED and channel number.
    p.knob(QuadVca::GAIN_PARAM + i, 9.6f, 23.6f + dy, 9.5f);
    p.bicolorLight(QuadVca::OUT_LIGHT + 2 * i, QuadVca::OUT_LIGHT + 2 * i + 1, 9.6f, 31.0f + dy,
                   0xFF35E06A, 0xFFFF3B30, 2.6f);
    p.label(9.6f, 35.2f + dy, nums[i], 3.2f);
    // Response trimmer with its lin/exp legend.
    p.knob(QuadVca::RESPONSE_PARAM + i, 27.4f, 20.8f + dy, 6.5f, "RESPONSE", -5.6f);
    p.label(23.6f, 26.4f + dy, "EXP", 2.2f);
    p.label(31.2f, 26.4f + dy, "LIN", 2.2f);
    // Gain knob -> CV jack.
    dotted(p, 13.0f, 26.0f + dy, 22.5f, 30.8f + dy, ink);
    dotted(p, 22.5f, 30.8f + dy, 37.6f, 30.8f + dy, ink);
    p.input(QuadVca::IN_INPUT + i, 41.4f, 18.0f + dy, "IN", 6.0f);
    p.input(QuadVca::CV_INPUT + i, 41.4f, 30.8f + dy);
    p.output(QuadVca::OUT_OUTPUT + i, 52.9f, 18.0f + dy);
    p.label(52.9f, 24.0f + dy, "OUT", 2.7f, kAlignCenter, white);
    if (i < 3) {
      dotted(p, 52.9f, 27.0f + dy, 52.9f, 39.6f + dy, white);
      p.line(52.9f, 39.9f + dy, 51.9f, 38.7f + dy, 0.45f, white);
      p.line(52.9f, 39.9f + dy, 53.9f, 38.7f + dy, 0.45f, white);
    }
  }
  p.line(2.2f, 119.6f, 58.8f, 119.6f, 0.6f, 0xFFB8B8B8);
}

Model model = [] {
  Model m;
  m.slug = "quad-vca";
  m.name = "Quad VCA";
  m.shortName = "QUAD VCA";
  m.basedOn = "Veils";
  m.category = Category::Utility;
  m.description = "Four VCAs with lin/exp response, CV normalled to +5 V, outputs chained into a mix.";
  m.create = [] { return new QuadVca(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
