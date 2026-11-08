// BrickRack - GPL-3.0-or-later.
//
// Liquid Filter, after Ripples. This is an original digital model of an analog
// circuit, not firmware: Ripples has no code.
//
// The hardware is a four-stage cascade of V2164 VCA-driven integrators (four
// one-pole low-pass stages) with an LM13700 OTA closing the resonance loop
// around the fourth stage, an AC-coupled input and an OTA VCA after LP4.
//
// Model, run at 2x oversampling (halfband FIR up/down):
//   * four zero-delay-feedback (TPT) one-pole stages sharing one cutoff;
//   * the resonance OTA is a tanh in the feedback path, solved without a
//     unit delay using the "linearise around the previous sample" trick, so
//     it self-oscillates with a stable, level-limited sine;
//   * a gentle tanh on the input stage (hot signals compress like the
//     hardware's input) and partial passband-loss compensation;
//   * BP2 = 2 (LP1 - LP2) (unity-peak two-pole band-pass), LP2, LP4;
//   * LP4>VCA = LP4 x GAIN CV (linear OTA VCA, unity at +8 V, open when
//     unpatched).
// Cutoff: 20 Hz - 20 kHz on the knob, 1 V/oct input, FM attenuverter.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"

namespace br {
namespace {

inline float tanhFast(float x) {
  // Pade-style approximation, accurate to ~2e-3 and exactly saturating.
  if (x > 3.f) return 1.f;
  if (x < -3.f) return -1.f;
  float x2 = x * x;
  return x * (27.f + x2) / (27.f + 9.f * x2);
}

inline float railClip(float x, float knee = 9.f, float limit = 11.f) {
  float a = std::fabs(x);
  if (a <= knee) return x;
  float r = limit - knee;
  float y = knee + r * std::tanh((a - knee) / r);
  return x < 0.f ? -y : y;
}

// tan(x) for 0 <= x <= ~0.75 (Pade [3/2]); used for the bilinear pre-warp.
inline float tanPrewarp(float x) {
  float x2 = x * x;
  return x * (15.f - x2) / (15.f - 6.f * x2);
}

// 39-tap Kaiser (beta 7) halfband: 10 non-zero taps per side, flat to 0.19 fs
// and -70 dB beyond 0.31 fs at the doubled rate.
struct Halfband {
  static constexpr int kSide = 10;
  float c[kSide];  // coefficient for offsets +-(2j+1); the centre tap is 0.5
  Halfband() {
    const int n = 4 * kSide - 1, m = (n - 1) / 2;
    auto i0 = [](double x) {
      double s = 1.0, t = 1.0;
      for (int k = 1; k < 30; ++k) {
        t *= (x / (2.0 * k)) * (x / (2.0 * k));
        s += t;
      }
      return s;
    };
    const double beta = 7.0;
    double h[64], sumOdd = 0.0;
    for (int i = 0; i < n; ++i) {
      int k = i - m;
      double sinc = k == 0 ? 1.0 : std::sin(M_PI * k / 2.0) / (M_PI * k / 2.0);
      double r = 2.0 * i / (n - 1) - 1.0;
      h[i] = 0.5 * sinc * i0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0(beta);
      if (k & 1) sumOdd += h[i];
    }
    // Normalise so the odd taps sum to exactly 0.5 (the centre tap is 0.5).
    for (int j = 0; j < kSide; ++j) c[j] = static_cast<float>(h[m + 2 * j + 1] * 0.5 / sumOdd);
  }
};
const Halfband& halfband() {
  static const Halfband hb;
  return hb;
}

// 2x interpolator: one input sample in, two output samples out.
struct Upsampler2x {
  static constexpr int L = 2 * Halfband::kSide;
  float buf[2 * L] = {};
  int pos = 0;
  void process(float x, float out[2]) {
    const float* c = halfband().c;
    pos = pos == 0 ? L - 1 : pos - 1;
    buf[pos] = buf[pos + L] = x;
    const float* b = buf + pos;  // b[i] = x[n - i]
    float acc = 0.f;
    for (int j = 0; j < Halfband::kSide; ++j)
      acc += c[j] * (b[Halfband::kSide - 1 - j] + b[Halfband::kSide + j]);
    out[0] = 2.f * acc;
    out[1] = b[Halfband::kSide - 1];
  }
};

// 2x decimator: two input samples in (even, odd), one output sample out.
struct Downsampler2x {
  static constexpr int L = 2 * Halfband::kSide;
  float even[2 * L] = {};
  float odd[2 * L] = {};
  int pos = 0;
  float process(float e, float o) {
    const float* c = halfband().c;
    pos = pos == 0 ? L - 1 : pos - 1;
    even[pos] = even[pos + L] = e;
    odd[pos] = odd[pos + L] = o;
    const float* b = even + pos;
    float acc = 0.f;
    for (int j = 0; j < Halfband::kSide; ++j)
      acc += c[j] * (b[Halfband::kSide - 1 - j] + b[Halfband::kSide + j]);
    return acc + 0.5f * odd[pos + Halfband::kSide];
  }
};

class LiquidFilter : public Module {
 public:
  enum ParamIds { RES_PARAM, FREQ_PARAM, FM_PARAM, NUM_PARAMS };
  enum InputIds { RES_INPUT, FREQ_INPUT, FM_INPUT, IN_INPUT, GAIN_INPUT, NUM_INPUTS };
  enum OutputIds { BP2_OUTPUT, LP2_OUTPUT, LP4_OUTPUT, LP4VCA_OUTPUT, NUM_OUTPUTS };
  enum LightIds { NUM_LIGHTS };

  LiquidFilter() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(RES_PARAM, 0.f, 1.f, 0.f, "Resonance", "%", 100.f);
    configParam(FREQ_PARAM, 0.f, 1.f, 0.6f, "Frequency", "Hz", 20.f).displayBase = 1000.f;
    configParam(FM_PARAM, -1.f, 1.f, 0.f, "FM amount", "%", 100.f);
    configInput(RES_INPUT, "Resonance CV");
    configInput(FREQ_INPUT, "Frequency (V/oct)");
    configInput(FM_INPUT, "FM");
    configInput(IN_INPUT, "Audio");
    configInput(GAIN_INPUT, "VCA gain (+8 V = unity, open when unpatched)");
    configOutput(BP2_OUTPUT, "2-pole band-pass");
    configOutput(LP2_OUTPUT, "2-pole low-pass");
    configOutput(LP4_OUTPUT, "4-pole low-pass");
    configOutput(LP4VCA_OUTPUT, "4-pole low-pass through VCA");
    onSampleRateChange(48000.f);
  }

  void onSampleRateChange(float sr) override {
    osRate_ = 2.f * sr;
    hpCoef_ = 1.f - std::exp(-2.f * static_cast<float>(M_PI) * 7.2f / sr);  // 100k / 220n
  }

  void process(const ProcessArgs& args) override {
    // Controls (smoothed knobs; CVs are used at audio rate).
    freqKnob_ += (params[FREQ_PARAM].value - freqKnob_) * 0.01f;
    resKnob_ += (params[RES_PARAM].value - resKnob_) * 0.01f;
    float pitch = freqKnob_ * 9.9658f /* log2(1000) */ + inputs[FREQ_INPUT].getVoltage() +
                  params[FM_PARAM].value * inputs[FM_INPUT].getVoltage();
    float f = 20.f * std::exp2(clampf(pitch, -2.f, 10.5f));
    f = std::min(f, 0.21f * osRate_);
    float g = tanPrewarp(static_cast<float>(M_PI) * f / osRate_);
    float G = g / (1.f + g);
    float res = clampf(resKnob_ + inputs[RES_INPUT].getVoltage() * 0.2f, 0.f, 1.1f);
    float k = 4.5f * res;
    float comp = 1.f + 0.25f * k;

    // Input: AC coupling (7 Hz), then the input stage's soft compression.
    float in = inputs[IN_INPUT].getVoltage();
    dc_ += (in - dc_) * hpCoef_;
    float x = (in - dc_) * 0.2f;

    float up[2];
    upsampler_.process(x, up);
    float lp1[2], lp2[2], lp4[2];
    const float G2 = G * G, G4 = G2 * G2, oneMinusG = 1.f - G;
    for (int i = 0; i < 2; ++i) {
      // A -120 dB noise floor lets self-oscillation start as in the circuit.
      noise_ = noise_ * 1664525u + 1013904223u;
      float hiss = static_cast<float>(static_cast<int32_t>(noise_)) * 4.6e-16f;
      float u0 = 1.6f * tanhFast(up[i] * 0.625f) * comp + hiss;
      // Linear prediction of y4 given the current states, with the OTA's tanh
      // linearised around the previous output (t = tanh(c y) / (c y)).
      float S = oneMinusG * (G2 * G * s_[0] + G2 * s_[1] + G * s_[2] + s_[3]);
      float cy = 0.65f * y4_;
      float t = std::fabs(cy) < 1e-4f ? 1.f : tanhFast(cy) / cy;
      float y4 = (G4 * u0 + S) / (1.f + k * t * G4);
      float u = u0 - k * t * y4;
      // Run the four TPT stages.
      float v, y;
      v = (u - s_[0]) * G; y = v + s_[0]; s_[0] = y + v; lp1[i] = y;
      v = (y - s_[1]) * G; y = v + s_[1]; s_[1] = y + v; lp2[i] = y;
      v = (y - s_[2]) * G; y = v + s_[2]; s_[2] = y + v;
      v = (y - s_[3]) * G; y = v + s_[3]; s_[3] = y + v; lp4[i] = y;
      y4_ = y;
    }
    float bp = 2.f * (dnBp_.process(lp1[0] - lp2[0], lp1[1] - lp2[1]));
    float l2 = dnLp2_.process(lp2[0], lp2[1]);
    float l4 = dnLp4_.process(lp4[0], lp4[1]);
    if (!std::isfinite(l4) || !std::isfinite(bp)) reset();

    float vca = inputs[GAIN_INPUT].isConnected()
                    ? clampf(inputs[GAIN_INPUT].getVoltage() * 0.125f, 0.f, 1.25f)
                    : 1.f;
    outputs[BP2_OUTPUT].setVoltage(railClip(bp * 5.f));
    outputs[LP2_OUTPUT].setVoltage(railClip(l2 * 5.f));
    outputs[LP4_OUTPUT].setVoltage(railClip(l4 * 5.f));
    outputs[LP4VCA_OUTPUT].setVoltage(railClip(l4 * 5.f * vca));
  }

  void onReset() override { reset(); }

 private:
  void reset() {
    for (float& s : s_) s = 0.f;
    y4_ = 0.f;
    dc_ = 0.f;
    upsampler_ = Upsampler2x();
    dnBp_ = Downsampler2x();
    dnLp2_ = Downsampler2x();
    dnLp4_ = Downsampler2x();
  }

  float osRate_ = 96000.f;
  float hpCoef_ = 0.f;
  float dc_ = 0.f;
  float freqKnob_ = 0.6f, resKnob_ = 0.f;
  float s_[4] = {0.f, 0.f, 0.f, 0.f};
  float y4_ = 0.f;
  uint32_t noise_ = 1u;
  Upsampler2x upsampler_;
  Downsampler2x dnBp_, dnLp2_, dnLp4_;
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
  p.hp = 8;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  const uint32_t ink = 0xFF2B2B2E;
  const uint32_t red = 0xFFC8414B, teal = 0xFF168A8C, white = 0xFFFFFFFF;
  const float xa = 8.6f, xb = 20.2f, xc = 32.1f;

  // A light ring around the frequency knob stands in for the paisley.
  p.display(9.0f, 30.0f, 24.0f, 26.0f, [](Canvas& c, float x, float y, float w, float h,
                                                  float s, const Module*) {
    float cx = x + w * 0.5f, cy = y + h * 0.46f;
    c.strokeCircle(cx, cy, 9.6f * s, std::max(1.f, 0.35f * s), 0xFFD9D6CE);
    c.strokeCircle(cx, cy, 10.8f * s, std::max(1.f, 0.2f * s), 0xFFD9D6CE);
  });

  p.knob(LiquidFilter::RES_PARAM, xa, 21.1f, 10.f);
  p.knob(LiquidFilter::FREQ_PARAM, xb, 42.0f, 14.f);
  p.knob(LiquidFilter::FM_PARAM, xc, 64.1f, 6.5f);
  p.label(27.6f, 58.6f, "-", 3.2f);
  p.line(xc, 57.3f, xc, 59.7f, 0.45f, ink);
  p.label(36.6f, 58.6f, "+", 3.2f);
  dotted(p, xa, 26.6f, xa, 77.6f, ink);
  dotted(p, xb, 49.6f, xb, 77.6f, ink);
  dotted(p, xc, 67.8f, xc, 77.6f, ink);

  p.box(xa - 3.6f, 78.3f, 7.2f, 3.4f, red, 1.0f);
  p.label(xa, 80.0f, "RES", 2.7f, kAlignCenter, white);
  p.label(xb, 80.0f, "FREQ", 2.7f);
  p.box(xc - 3.0f, 78.3f, 6.0f, 3.4f, teal, 1.0f);
  p.label(xc, 80.0f, "FM", 2.7f, kAlignCenter, white);
  p.input(LiquidFilter::RES_INPUT, xa, 86.6f);
  p.input(LiquidFilter::FREQ_INPUT, xb, 86.6f);
  p.input(LiquidFilter::FM_INPUT, xc, 86.6f);

  p.box(14.2f, 91.0f, 23.8f, 26.4f, 0xFF5E5E60, 1.2f);
  p.input(LiquidFilter::IN_INPUT, xa, 99.1f, "IN", -5.9f);
  p.output(LiquidFilter::BP2_OUTPUT, xb, 99.1f);
  p.output(LiquidFilter::LP2_OUTPUT, xc, 99.1f);
  p.label(xb, 93.4f, "BP2", 2.7f, kAlignCenter, white);
  p.label(xc, 93.4f, "LP2", 2.7f, kAlignCenter, white);
  p.input(LiquidFilter::GAIN_INPUT, xa, 111.3f, "GAIN", -5.9f);
  p.output(LiquidFilter::LP4_OUTPUT, xb, 111.3f);
  p.output(LiquidFilter::LP4VCA_OUTPUT, xc, 111.3f);
  p.label(xb, 105.5f, "LP4", 2.7f, kAlignCenter, white);
  p.label(xc, 105.5f, "LP4>VCA", 2.5f, kAlignCenter, white);
  p.line(1.2f, 120.0f, 39.4f, 120.0f, 0.6f, 0xFFB8B8B8);
}

Model model = [] {
  Model m;
  m.slug = "liquid-filter";
  m.name = "Liquid Filter";
  m.shortName = "LIQUID FILTER";
  m.basedOn = "Ripples";
  m.category = Category::Filter;
  m.description = "Resonant 4-pole OTA-style filter: BP2, LP2, LP4 and LP4 into a VCA; self-oscillates.";
  m.create = [] { return new LiquidFilter(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
