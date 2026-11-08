// BrickRack - GPL-3.0-or-later.
//
// Quad VC Polarizer, after Blinds. This is an original digital model of an
// analog circuit (V2164 four-quadrant multipliers, op-amp summing chain), not
// firmware: Blinds has no code.
//
// Per channel: gain = GAIN knob (-1..+1, centre = 0) + CV attenuverter x
// CV / 5 V, limited to +/-2. Inputs are normalled to +5 V, so an unpatched
// channel outputs a CV-controllable offset. Unpatched outputs are summed into
// the next channel, and the sum saturates softly near the +/-12 V rails.
// Big LED: output polarity/level. Small LED: the modulated gain.

#include <algorithm>
#include <cmath>
#include <cstdio>

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

class QuadPolarizer : public Module {
 public:
  enum ParamIds { GAIN_PARAM, MOD_PARAM = GAIN_PARAM + 4, NUM_PARAMS = MOD_PARAM + 4 };
  enum InputIds { IN_INPUT, CV_INPUT = IN_INPUT + 4, NUM_INPUTS = CV_INPUT + 4 };
  enum OutputIds { OUT_OUTPUT, NUM_OUTPUTS = OUT_OUTPUT + 4 };
  enum LightIds {
    OUT_LIGHT,                           // [ch*2 + {green, red}]
    GAIN_LIGHT = OUT_LIGHT + 8,          // [ch*2 + {green, red}]
    NUM_LIGHTS = GAIN_LIGHT + 8
  };

  QuadPolarizer() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < 4; ++i) {
      char n[48];
      std::snprintf(n, sizeof(n), "Channel %d gain", i + 1);
      configParam(GAIN_PARAM + i, -1.f, 1.f, 0.f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Channel %d CV amount", i + 1);
      configParam(MOD_PARAM + i, -1.f, 1.f, 0.f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Channel %d in (normalled to +5 V)", i + 1);
      configInput(IN_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Channel %d gain CV", i + 1);
      configInput(CV_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Channel %d out", i + 1);
      configOutput(OUT_OUTPUT + i, n);
    }
  }

  void process(const ProcessArgs& args) override {
    float acc = 0.f;
    for (int i = 0; i < 4; ++i) {
      knob_[i] += (params[GAIN_PARAM + i].value - knob_[i]) * 0.008f;
      mod_[i] += (params[MOD_PARAM + i].value - mod_[i]) * 0.008f;
      float gain = clampf(knob_[i] + mod_[i] * inputs[CV_INPUT + i].getVoltage() * 0.2f, -2.f, 2.f);
      acc = railClip(acc + inputs[IN_INPUT + i].getNormalVoltage(5.f) * gain);
      outputs[OUT_OUTPUT + i].setVoltage(acc);
      pos_[i] = std::max(pos_[i] * 0.9995f, acc);
      neg_[i] = std::max(neg_[i] * 0.9995f, -acc);
      gain_[i] = gain;
      if (outputs[OUT_OUTPUT + i].isConnected()) acc = 0.f;
    }
    if ((args.frame & 63) == 0) {
      for (int i = 0; i < 4; ++i) {
        size_t o = static_cast<size_t>(2 * i);
        lights[OUT_LIGHT + o] = std::min(1.f, pos_[i] * 0.2f);
        lights[OUT_LIGHT + o + 1] = std::min(1.f, neg_[i] * 0.2f);
        lights[GAIN_LIGHT + o] = std::min(1.f, std::max(0.f, gain_[i]));
        lights[GAIN_LIGHT + o + 1] = std::min(1.f, std::max(0.f, -gain_[i]));
      }
    }
  }

 private:
  float knob_[4] = {0.f, 0.f, 0.f, 0.f};
  float mod_[4] = {0.f, 0.f, 0.f, 0.f};
  float gain_[4] = {0.f, 0.f, 0.f, 0.f};
  float pos_[4] = {0.f, 0.f, 0.f, 0.f};
  float neg_[4] = {0.f, 0.f, 0.f, 0.f};
};

void dotted(PanelSpec& p, float x0, float y0, float x1, float y1, uint32_t col,
            float step = 1.1f) {
  float dx = x1 - x0, dy = y1 - y0;
  float len = std::sqrt(dx * dx + dy * dy);
  int n = std::max(1, static_cast<int>(len / step));
  for (int k = 0; k < n; k += 2) {
    float t0 = static_cast<float>(k) / n, t1 = t0 + 0.5f / n;
    p.line(x0 + dx * t0, y0 + dy * t0, x0 + dx * t1, y0 + dy * t1, 0.5f, col);
  }
}

void buildPanel(PanelSpec& p) {
  p.hp = 12;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  const uint32_t white = 0xFFFFFFFF;
  const uint32_t ink = 0xFF2B2B2E;
  const uint32_t g = 0xFF35E06A, r = 0xFFFF3B30;
  p.box(47.3f, 12.6f, 11.2f, 103.6f, 0xFF5E5E60, 1.2f);
  const char* nums[4] = {"1", "2", "3", "4"};
  for (int i = 0; i < 4; ++i) {
    float dy = i * 26.6f;
    // Polarizing gain knob with its -/0/+ legend, LED and channel number.
    p.label(4.4f, 16.2f + dy, "-", 3.4f);
    p.line(9.6f, 15.0f + dy, 9.6f, 17.4f + dy, 0.5f, ink);
    p.label(14.8f, 16.2f + dy, "+", 3.4f);
    p.knob(QuadPolarizer::GAIN_PARAM + i, 9.6f, 24.0f + dy, 9.5f);
    p.bicolorLight(QuadPolarizer::OUT_LIGHT + 2 * i, QuadPolarizer::OUT_LIGHT + 2 * i + 1, 9.6f,
                   31.3f + dy, g, r, 2.6f);
    p.label(9.6f, 35.4f + dy, nums[i], 3.2f);
    // CV attenuverter and gain LED.
    p.knob(QuadPolarizer::MOD_PARAM + i, 27.4f, 24.0f + dy, 6.5f);
    p.label(23.6f, 19.6f + dy, "-", 3.0f);
    p.label(31.2f, 19.6f + dy, "+", 3.0f);
    p.bicolorLight(QuadPolarizer::GAIN_LIGHT + 2 * i, QuadPolarizer::GAIN_LIGHT + 2 * i + 1, 27.4f,
                   33.4f + dy, g, r, 2.0f);
    dotted(p, 14.6f, 24.0f + dy, 23.9f, 24.0f + dy, ink);
    dotted(p, 30.9f, 24.0f + dy, 35.6f, 25.2f + dy, ink);
    dotted(p, 35.6f, 25.2f + dy, 38.9f, 28.6f + dy, ink);
    p.input(QuadPolarizer::IN_INPUT + i, 41.4f, 18.0f + dy, "IN", 6.0f);
    p.input(QuadPolarizer::CV_INPUT + i, 41.4f, 30.8f + dy);
    p.output(QuadPolarizer::OUT_OUTPUT + i, 52.9f, 18.0f + dy);
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
  m.slug = "quad-polarizer";
  m.name = "Quad VC Polarizer";
  m.shortName = "QUAD POLARIZER";
  m.basedOn = "Blinds";
  m.category = Category::Utility;
  m.description = "Four bipolar VCAs/polarizers with CV attenuverters, +5 V offsets, chained into a mix.";
  m.create = [] { return new QuadPolarizer(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
