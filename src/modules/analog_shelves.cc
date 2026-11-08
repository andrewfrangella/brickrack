// BrickRack - GPL-3.0-or-later.
//
// EQ Filter, after Shelves. This is an original digital model of an analog
// circuit (V2164 state-variable equaliser), not firmware: Shelves has no code.
//
// Four bands in series: high shelf, two parametric peaks with Q, low shelf.
// Each band is a zero-delay-feedback state-variable filter (Simper/Cytomic
// SVF) with the standard bell/shelf output mixes, which behaves like the
// hardware's VCA-controlled SVFs under fast modulation.
//
//   FREQ   20 Hz - 20 kHz per band; band and global FREQ CVs are 1 V/oct
//   GAIN   +/-15 dB on the knob; band and global GAIN CVs add 3 dB/V
//          (total limited to +/-24 dB)
//   Q      0.5 - 25 on the knob; Q CV is exponential (x2 per 2 V)
//   OUT    DC coupled; saturates softly near the rails, the clip LED lights
//          when the output exceeds ~9.5 V.
// Coefficients are updated every 8 samples from smoothed controls.

#include <algorithm>
#include <cmath>
#include <string>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"

namespace br {
namespace {

inline float railClip(float x, float knee = 8.5f, float limit = 10.5f) {
  float a = std::fabs(x);
  if (a <= knee) return x;
  float r = limit - knee;
  float y = knee + r * std::tanh((a - knee) / r);
  return x < 0.f ? -y : y;
}

struct Svf {
  float a1 = 0.f, a2 = 0.f, a3 = 0.f, m0 = 1.f, m1 = 0.f, m2 = 0.f;
  float ic1 = 0.f, ic2 = 0.f;

  enum Type { kLowShelf, kBell, kHighShelf };
  // f normalised to the sample rate (f / fs), dB gain, Q.
  void set(Type type, float fn, float db, float q) {
    float A = std::pow(10.f, db * (1.f / 40.f));
    float g = std::tan(static_cast<float>(M_PI) * fn);
    float k;
    switch (type) {
      case kLowShelf:
        g /= std::sqrt(A);
        k = 1.f / q;
        m0 = 1.f;
        m1 = k * (A - 1.f);
        m2 = A * A - 1.f;
        break;
      case kHighShelf:
        g *= std::sqrt(A);
        k = 1.f / q;
        m0 = A * A;
        m1 = k * (1.f - A) * A;
        m2 = 1.f - A * A;
        break;
      default:
        k = 1.f / (q * A);
        m0 = 1.f;
        m1 = k * (A * A - 1.f);
        m2 = 0.f;
        break;
    }
    a1 = 1.f / (1.f + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
  }
  float process(float v0) {
    float v3 = v0 - ic2;
    float v1 = a1 * ic1 + a2 * v3;
    float v2 = ic2 + a2 * ic1 + a3 * v3;
    ic1 = 2.f * v1 - ic1;
    ic2 = 2.f * v2 - ic2;
    return m0 * v0 + m1 * v1 + m2 * v2;
  }
  void reset() { ic1 = ic2 = 0.f; }
};

class EqFilter : public Module {
 public:
  // Bands: 0 = high shelf, 1 = peak 1, 2 = peak 2, 3 = low shelf (panel order).
  enum ParamIds {
    FREQ_PARAM,                  // 4
    GAIN_PARAM = FREQ_PARAM + 4,  // 4
    Q1_PARAM = GAIN_PARAM + 4,
    Q2_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    FREQ_INPUT,                  // 4 (per band)
    GAIN_INPUT = FREQ_INPUT + 4,  // 4
    Q1_INPUT = GAIN_INPUT + 4,
    Q2_INPUT,
    GLOBAL_FREQ_INPUT,
    GLOBAL_GAIN_INPUT,
    IN_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { OUT_OUTPUT, NUM_OUTPUTS };
  enum LightIds { CLIP_LIGHT, NUM_LIGHTS };

  EqFilter() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    const char* band[4] = {"High shelf", "Peak 1", "Peak 2", "Low shelf"};
    const float defFreq[4] = {0.8f, 0.65f, 0.45f, 0.25f};
    for (int b = 0; b < 4; ++b) {
      std::string n = band[b];
      // 20 Hz * 1000^x
      configParam(FREQ_PARAM + b, 0.f, 1.f, defFreq[b], n + " frequency", "Hz", 20.f)
          .displayBase = 1000.f;
      configParam(GAIN_PARAM + b, -1.f, 1.f, 0.f, n + " gain", "dB", 15.f);
      configInput(FREQ_INPUT + b, n + " frequency (V/oct)");
      configInput(GAIN_INPUT + b, n + " gain CV (3 dB/V)");
    }
    configParam(Q1_PARAM, 0.f, 1.f, 0.3f, "Peak 1 Q", "%", 100.f);
    configParam(Q2_PARAM, 0.f, 1.f, 0.3f, "Peak 2 Q", "%", 100.f);
    configInput(Q1_INPUT, "Peak 1 Q CV");
    configInput(Q2_INPUT, "Peak 2 Q CV");
    configInput(GLOBAL_FREQ_INPUT, "All bands frequency (V/oct)");
    configInput(GLOBAL_GAIN_INPUT, "All bands gain CV (3 dB/V)");
    configInput(IN_INPUT, "Audio");
    configOutput(OUT_OUTPUT, "Audio");
    for (int i = 0; i < 4; ++i) {
      freq_[i] = params[FREQ_PARAM + i].value;
      gain_[i] = 0.f;
    }
  }

  void process(const ProcessArgs& args) override {
    if (counter_-- <= 0) {
      counter_ = 7;
      updateCoefficients(args.sampleRate);
    }
    float x = inputs[IN_INPUT].getVoltage();
    for (int b = 0; b < 4; ++b) x = band_[b].process(x);
    if (!std::isfinite(x)) {
      for (Svf& s : band_) s.reset();
      x = 0.f;
    }
    float out = railClip(x);
    outputs[OUT_OUTPUT].setVoltage(out);
    clip_ = std::fabs(x) > 9.5f ? 1.f : clip_ * 0.9997f;
    if ((args.frame & 63) == 0) lights[CLIP_LIGHT] = clip_;
  }

  void onReset() override {
    for (Svf& s : band_) s.reset();
  }

 private:
  void updateCoefficients(float sr) {
    const float k = 0.06f;  // control smoothing per update (~3 ms at 6 kHz)
    float gFreq = inputs[GLOBAL_FREQ_INPUT].getVoltage();
    float gGain = inputs[GLOBAL_GAIN_INPUT].getVoltage() * 3.f;
    for (int b = 0; b < 4; ++b) {
      freq_[b] += (params[FREQ_PARAM + b].value - freq_[b]) * k;
      gain_[b] += (params[GAIN_PARAM + b].value - gain_[b]) * k;
      float oct = freq_[b] * 9.9658f + inputs[FREQ_INPUT + b].getVoltage() + gFreq;
      float f = 20.f * std::exp2(clampf(oct, -1.f, 10.5f));
      float fn = std::min(f / sr, 0.45f);
      float db = clampf(gain_[b] * 15.f + inputs[GAIN_INPUT + b].getVoltage() * 3.f + gGain,
                        -24.f, 24.f);
      Svf::Type type = b == 0 ? Svf::kHighShelf : (b == 3 ? Svf::kLowShelf : Svf::kBell);
      float q = 0.7071f;
      if (type == Svf::kBell) {
        int qi = b - 1;
        q_[qi] += (params[Q1_PARAM + qi].value - q_[qi]) * k;
        float qo = q_[qi] * 5.64f - 1.f + inputs[Q1_INPUT + qi].getVoltage() * 0.5f;  // 0.5..25
        q = std::exp2(clampf(qo, -1.5f, 5.3f));
      }
      band_[b].set(type, fn, db, q);
    }
  }

  Svf band_[4];
  float freq_[4];
  float gain_[4];
  float q_[2] = {0.3f, 0.3f};
  float clip_ = 0.f;
  int counter_ = 0;
};

// Small round badge with a response-curve glyph.
void bandIcon(PanelSpec& p, float x, float y, int type, uint32_t col) {
  const uint32_t w = 0xFFFFFFFF;
  p.box(x - 1.9f, y - 1.9f, 3.8f, 3.8f, col, 1.9f);
  if (type == 0) {  // high shelf
    p.line(x - 1.2f, y + 0.5f, x - 0.1f, y + 0.5f, 0.3f, w);
    p.line(x - 0.1f, y + 0.5f, x + 0.4f, y - 0.5f, 0.3f, w);
    p.line(x + 0.4f, y - 0.5f, x + 1.2f, y - 0.5f, 0.3f, w);
  } else if (type == 3) {  // low shelf
    p.line(x - 1.2f, y - 0.5f, x - 0.4f, y - 0.5f, 0.3f, w);
    p.line(x - 0.4f, y - 0.5f, x + 0.1f, y + 0.5f, 0.3f, w);
    p.line(x + 0.1f, y + 0.5f, x + 1.2f, y + 0.5f, 0.3f, w);
  } else {  // peak
    p.line(x - 1.3f, y + 0.5f, x - 0.4f, y + 0.5f, 0.3f, w);
    p.line(x - 0.4f, y + 0.5f, x, y - 0.7f, 0.3f, w);
    p.line(x, y - 0.7f, x + 0.4f, y + 0.5f, 0.3f, w);
    p.line(x + 0.4f, y + 0.5f, x + 1.3f, y + 0.5f, 0.3f, w);
  }
}

void badge(PanelSpec& p, float x, float y, float w, const char* text, uint32_t col) {
  p.box(x - w * 0.5f, y - 1.8f, w, 3.6f, col, 0.9f);
  p.label(x, y, text, 2.8f, kAlignCenter, 0xFFFFFFFF);
}

// Coloured arc around a Q trimmer, as printed on the hardware.
void qArc(PanelSpec& p, float x, float y, uint32_t col) {
  p.display(x - 5.f, y - 5.f, 10.f, 10.f, [col](Canvas& c, float px, float py, float w, float h,
                                                 float s, const Module*) {
    c.arc(px + w * 0.5f, py + h * 0.5f, 4.6f * s, 0.75f * static_cast<float>(M_PI),
          2.25f * static_cast<float>(M_PI), std::max(1.f, 0.45f * s), col);
  });
}

void dotted(PanelSpec& p, float x0, float y0, float x1, float y1, uint32_t col,
            float step = 1.0f) {
  float dx = x1 - x0, dy = y1 - y0;
  float len = std::sqrt(dx * dx + dy * dy);
  int n = std::max(1, static_cast<int>(len / step));
  for (int k = 0; k < n; k += 2) {
    float t0 = static_cast<float>(k) / n, t1 = t0 + 0.5f / n;
    p.line(x0 + dx * t0, y0 + dy * t0, x0 + dx * t1, y0 + dy * t1, 0.45f, col);
  }
}

void buildPanel(PanelSpec& p) {
  p.hp = 16;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  const uint32_t red = 0xFFC8414B, teal = 0xFF168A8C, grey = 0xFF6E6E70;
  const uint32_t rule = 0xFFC4C4C4, ink = 0xFF2B2B2E;
  const float jx0 = 8.7f, jx1 = 22.4f, fx = 45.2f, gx = 69.4f, ix = 57.3f;
  const float kd = 11.5f;

  // Gain knob +/- legend.
  auto pm = [&](float y) {
    p.label(61.5f, y, "-", 3.6f);
    p.line(gx, y - 1.6f, gx, y + 1.0f, 0.5f, ink);
    p.label(77.2f, y, "+", 3.6f);
  };

  // High shelf.
  p.input(EqFilter::FREQ_INPUT + 0, jx0, 17.2f, "FREQ", 6.4f);
  p.input(EqFilter::GAIN_INPUT + 0, jx1, 17.2f, "GAIN", 6.4f);
  p.knob(EqFilter::FREQ_PARAM + 0, fx, 19.7f, kd);
  p.label(fx, 29.4f, "FREQ", 3.0f);
  bandIcon(p, ix, 24.8f, 0, grey);
  pm(14.2f);
  p.knob(EqFilter::GAIN_PARAM + 0, gx, 19.7f, kd);
  p.label(gx, 29.4f, "GAIN", 3.0f);
  p.line(3.6f, 27.6f, 28.9f, 27.6f, 0.5f, rule);
  p.line(28.9f, 27.6f, 34.4f, 33.4f, 0.5f, rule);
  p.line(34.4f, 33.4f, 77.6f, 33.4f, 0.5f, rule);

  // Peak 1 (red).
  p.input(EqFilter::FREQ_INPUT + 1, jx0, 33.8f, "FREQ", 6.4f);
  p.input(EqFilter::GAIN_INPUT + 1, jx1, 33.8f, "GAIN", 6.4f);
  p.input(EqFilter::Q1_INPUT, jx0, 48.2f);
  badge(p, jx0, 54.6f, 5.0f, "Q", red);
  qArc(p, jx1, 48.2f, red);
  p.knob(EqFilter::Q1_PARAM, jx1, 48.2f, 6.5f);
  badge(p, jx1, 54.6f, 5.0f, "Q", red);
  p.knob(EqFilter::FREQ_PARAM + 1, fx, 43.3f, kd);
  badge(p, fx, 53.2f, 10.2f, "FREQ", red);
  bandIcon(p, ix, 48.8f, 1, red);
  pm(37.9f);
  p.knob(EqFilter::GAIN_PARAM + 1, gx, 43.3f, kd);
  badge(p, gx, 53.2f, 9.6f, "GAIN", red);
  p.line(3.6f, 57.6f, 77.6f, 57.6f, 0.5f, rule);

  // Peak 2 (teal).
  p.input(EqFilter::Q2_INPUT, jx0, 63.5f);
  badge(p, jx0, 69.9f, 5.0f, "Q", teal);
  qArc(p, jx1, 63.5f, teal);
  p.knob(EqFilter::Q2_PARAM, jx1, 63.5f, 6.5f);
  badge(p, jx1, 69.9f, 5.0f, "Q", teal);
  p.input(EqFilter::FREQ_INPUT + 2, jx0, 77.7f, "FREQ", 6.4f);
  p.input(EqFilter::GAIN_INPUT + 2, jx1, 77.7f, "GAIN", 6.4f);
  p.knob(EqFilter::FREQ_PARAM + 2, fx, 67.1f, kd);
  badge(p, fx, 77.2f, 10.2f, "FREQ", teal);
  bandIcon(p, ix, 72.6f, 2, teal);
  pm(61.7f);
  p.knob(EqFilter::GAIN_PARAM + 2, gx, 67.1f, kd);
  badge(p, gx, 77.2f, 9.6f, "GAIN", teal);
  p.line(77.6f, 81.6f, 34.4f, 81.6f, 0.5f, rule);
  p.line(34.4f, 81.6f, 28.9f, 88.0f, 0.5f, rule);
  p.line(28.9f, 88.0f, 3.6f, 88.0f, 0.5f, rule);

  // Low shelf.
  p.input(EqFilter::FREQ_INPUT + 3, jx0, 94.2f, "FREQ", 6.4f);
  p.input(EqFilter::GAIN_INPUT + 3, jx1, 94.2f, "GAIN", 6.4f);
  p.knob(EqFilter::FREQ_PARAM + 3, fx, 90.9f, kd);
  p.label(fx, 100.6f, "FREQ", 3.0f);
  bandIcon(p, ix, 96.1f, 3, grey);
  pm(85.7f);
  p.knob(EqFilter::GAIN_PARAM + 3, gx, 90.9f, kd);
  p.label(gx, 100.6f, "GAIN", 3.0f);
  p.line(3.6f, 103.4f, 77.6f, 103.4f, 0.5f, rule);

  // Global CVs, input, clip LED and output.
  p.input(EqFilter::GLOBAL_FREQ_INPUT, jx0, 109.6f, "FREQ", 6.4f);
  p.input(EqFilter::GLOBAL_GAIN_INPUT, jx1, 109.6f, "GAIN", 6.4f);
  p.input(EqFilter::IN_INPUT, fx, 109.6f, "IN", 6.4f);
  p.light(EqFilter::CLIP_LIGHT, ix, 109.6f, 0xFFFF3B30, 2.6f);
  dotted(p, ix + 1.8f, 109.6f, gx - 4.2f, 109.6f, ink);
  p.label(ix, 114.3f, "CLIP", 2.4f);
  p.box(63.9f, 104.4f, 10.9f, 13.8f, 0xFF5E5E60, 1.2f);
  p.output(EqFilter::OUT_OUTPUT, gx, 109.6f);
  p.label(gx, 115.9f, "OUT", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.line(3.2f, 119.6f, 77.8f, 119.6f, 0.6f, 0xFFB8B8B8);
}

Model model = [] {
  Model m;
  m.slug = "eq-filter";
  m.name = "EQ Filter";
  m.shortName = "EQ FILTER";
  m.basedOn = "Shelves";
  m.category = Category::Filter;
  m.description = "Voltage-controlled EQ: low/high shelves and two resonant peaks, 1 V/oct tracking.";
  m.create = [] { return new EqFilter(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
