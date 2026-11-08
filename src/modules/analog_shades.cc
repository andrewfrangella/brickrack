// BrickRack - GPL-3.0-or-later.
//
// Triple Attenuverter, after Shades. This is an original digital model of an
// analog circuit (op-amp current-splitting attenuverters), not firmware.
//
// Each channel's switch selects attenuverter (-1..+1, centre = 0) or
// attenuator (0..1). Unpatched inputs are normalled to a +5 V reference (a
// jumper on the hardware selects +10 V instead: module menu), so a channel
// with nothing patched is an offset generator. Unpatched outputs are summed
// into the next channel. Outputs saturate softly near the +/-12 V rails.
// Each channel has a mint (positive) and a pink (negative) LED.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"

namespace br {
namespace {

inline float railClip(float x, float knee = 9.5f, float limit = 11.5f) {
  float a = std::fabs(x);
  if (a <= knee) return x;
  float r = limit - knee;
  float y = knee + r * std::tanh((a - knee) / r);
  return x < 0.f ? -y : y;
}

class Attenuverters : public Module {
 public:
  enum ParamIds { GAIN_PARAM, MODE_PARAM = GAIN_PARAM + 3, NUM_PARAMS = MODE_PARAM + 3 };
  enum InputIds { IN_INPUT, NUM_INPUTS = IN_INPUT + 3 };
  enum OutputIds { OUT_OUTPUT, NUM_OUTPUTS = OUT_OUTPUT + 3 };
  enum LightIds { OUT_LIGHT, NUM_LIGHTS = OUT_LIGHT + 6 };  // [ch*2 + {pos, neg}]

  Attenuverters() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < 3; ++i) {
      char n[48];
      std::snprintf(n, sizeof(n), "Channel %d level", i + 1);
      configParam(GAIN_PARAM + i, 0.f, 1.f, 0.5f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Channel %d mode", i + 1);
      configSwitch(MODE_PARAM + i, 0.f, 1.f, 1.f, n, {"Attenuator", "Attenuverter"});
      std::snprintf(n, sizeof(n), "Channel %d in (normalled to +5 V)", i + 1);
      configInput(IN_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Channel %d out", i + 1);
      configOutput(OUT_OUTPUT + i, n);
    }
  }

  void process(const ProcessArgs& args) override {
    const float normal = tenVolts_ ? 10.f : 5.f;
    float acc = 0.f;
    for (int i = 0; i < 3; ++i) {
      knob_[i] += (params[GAIN_PARAM + i].value - knob_[i]) * 0.008f;
      bool atv = params[MODE_PARAM + i].value > 0.5f;
      float gain = atv ? 2.f * knob_[i] - 1.f : knob_[i];
      float in = inputs[IN_INPUT + i].getNormalVoltage(normal);
      acc = railClip(acc + in * gain);
      outputs[OUT_OUTPUT + i].setVoltage(acc);
      pos_[i] = std::max(pos_[i] * 0.9995f, acc);
      neg_[i] = std::max(neg_[i] * 0.9995f, -acc);
      if (outputs[OUT_OUTPUT + i].isConnected()) acc = 0.f;
    }
    if ((args.frame & 63) == 0) {
      for (int i = 0; i < 3; ++i) {
        lights[static_cast<size_t>(2 * i)] = std::min(1.f, pos_[i] * 0.2f);
        lights[static_cast<size_t>(2 * i + 1)] = std::min(1.f, neg_[i] * 0.2f);
      }
    }
  }

  void saveState(StateMap& s) const override { s["normal10V"] = tenVolts_ ? "1" : "0"; }
  void loadState(const StateMap& s) override {
    auto it = s.find("normal10V");
    if (it != s.end()) tenVolts_ = it->second == "1";
  }
  void appendMenu(std::vector<MenuAction>& items) override {
    items.push_back({"Unpatched inputs (jumper)",
                     [this] { return std::string(tenVolts_ ? "+10 V" : "+5 V"); },
                     [this](int) { tenVolts_ = !tenVolts_; }});
  }
  void onReset() override { tenVolts_ = false; }

 private:
  bool tenVolts_ = false;
  float knob_[3] = {0.5f, 0.5f, 0.5f};
  float pos_[3] = {0.f, 0.f, 0.f};
  float neg_[3] = {0.f, 0.f, 0.f};
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

// Small dark badge with a ramp glyph: one ramp (attenuator) or a ramp through
// zero (attenuverter), as printed next to the hardware's switches.
void modeBadge(PanelSpec& p, float x, float y, bool bipolar) {
  const uint32_t w = 0xFFFFFFFF;
  p.box(x - 2.6f, y - 1.3f, 5.2f, 2.6f, 0xFF5E5E60, 0.6f);
  if (bipolar) {
    p.line(x - 1.7f, y - 0.0f, x + 1.7f, y - 0.0f, 0.25f, w);
    p.line(x - 1.7f, y + 0.8f, x + 1.7f, y - 0.8f, 0.4f, w);
  } else {
    p.line(x - 1.7f, y + 0.8f, x + 1.7f, y + 0.8f, 0.3f, w);
    p.line(x - 1.7f, y + 0.8f, x + 1.7f, y - 0.8f, 0.4f, w);
    p.line(x + 1.7f, y + 0.8f, x + 1.7f, y - 0.8f, 0.3f, w);
  }
}

void buildPanel(PanelSpec& p) {
  p.hp = 6;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  const char* nums[3] = {"1", "2", "3"};
  const float rowY[3] = {21.0f, 41.4f, 61.8f};
  for (int i = 0; i < 3; ++i) {
    float y = rowY[i];
    modeBadge(p, 5.6f, y - 7.0f, true);
    p.toggleSwitch(Attenuverters::MODE_PARAM + i, 5.6f, y);
    modeBadge(p, 5.6f, y + 7.0f, false);
    p.label(11.0f, y, nums[i], 3.4f);
    p.knob(Attenuverters::GAIN_PARAM + i, 21.4f, y, 10.f);
  }
  p.box(17.6f, 74.6f, 10.6f, 43.0f, 0xFF5E5E60, 1.2f);
  const float jackY[3] = {82.0f, 96.4f, 110.8f};
  for (int i = 0; i < 3; ++i) {
    float y = jackY[i];
    p.label(6.6f, y - 6.3f, nums[i], 3.2f);
    p.input(Attenuverters::IN_INPUT + i, 6.6f, y);
    p.light(Attenuverters::OUT_LIGHT + 2 * i, 14.0f, y - 2.6f, 0xFF7FE3C4, 2.4f);
    p.light(Attenuverters::OUT_LIGHT + 2 * i + 1, 14.0f, y + 2.6f, 0xFFFF8FB4, 2.4f);
    p.output(Attenuverters::OUT_OUTPUT + i, 22.9f, y);
    if (i < 2) {
      dotted(p, 22.9f, y + 3.9f, 22.9f, y + 10.0f, 0xFFFFFFFF);
      p.line(22.9f, y + 10.3f, 22.0f, y + 9.2f, 0.45f, 0xFFFFFFFF);
      p.line(22.9f, y + 10.3f, 23.8f, y + 9.2f, 0.45f, 0xFFFFFFFF);
    }
  }
  p.label(22.9f, 77.0f, "OUT", 2.6f, kAlignCenter, 0xFFFFFFFF);
  p.line(1.6f, 120.0f, 28.9f, 120.0f, 0.6f, 0xFFB8B8B8);
}

Model model = [] {
  Model m;
  m.slug = "attenuverters";
  m.name = "Triple Attenuverter";
  m.shortName = "ATTENUVERTERS";
  m.basedOn = "Shades";
  m.category = Category::Utility;
  m.description = "Three attenuverters/attenuators, +5 V offsets when unpatched, chained into a mix.";
  m.create = [] { return new Attenuverters(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
