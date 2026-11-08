// BrickRack - GPL-3.0-or-later.
//
// Dual Multimode Filter, after Blades. This is an original digital model of
// an analog circuit, not firmware: Blades has no code.
//
// Topology read from the hardware design files (blades_v40), per filter:
//   * input stage: DRIVE (knob + CV) sets a V2164 VCA feeding a zener-limited
//     amplifier (+/-5.7 V); the small CLEAN/DIRTY knob (audio taper) cross-
//     fades the clean input with that overdriven signal;
//   * a two-integrator state-variable filter (V2164 integrators) with LP, BP
//     and HP outputs; the resonance VCA reduces the damping until the filter
//     self-oscillates, and zener diodes on the band-pass integrator limit the
//     oscillation amplitude, so loud inputs squash the resonance;
//   * MODE crossfades LP -> BP -> HP (knob + CV, 5 V = full sweep).
// Global: IN 2 and V/OCT 2 are normalled to IN 1 and V/OCT 1. ROUTING (knob +
// attenuated CV) morphs MAIN from filter 1 alone, through the parallel mix of
// both filters, to filter 1 feeding filter 2 in series. SHIFT (latching, lit
// when on) links filter 2's cutoff to filter 1's: FREQUENCY 2 then sets an
// interval of +/-4 octaves.
//
// Model: zero-delay-feedback (TPT/Simper) SVF with the band-pass state soft-
// limited inside the loop, tanh zener stage, 2x oversampling with halfband
// FIR filters.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"

namespace br {
namespace {

inline float tanhFast(float x) {
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

inline float tanPrewarp(float x) {
  float x2 = x * x;
  return x * (15.f - x2) / (15.f - 6.f * x2);
}

// 39-tap Kaiser (beta 7) halfband: flat to 0.19 fs, -70 dB beyond 0.31 fs.
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
    for (int j = 0; j < kSide; ++j) c[j] = static_cast<float>(h[m + 2 * j + 1] * 0.5 / sumOdd);
  }
};
const Halfband& halfband() {
  static const Halfband hb;
  return hb;
}

struct Upsampler2x {
  static constexpr int L = 2 * Halfband::kSide;
  float buf[2 * L] = {};
  int pos = 0;
  void process(float x, float out[2]) {
    const float* c = halfband().c;
    pos = pos == 0 ? L - 1 : pos - 1;
    buf[pos] = buf[pos + L] = x;
    const float* b = buf + pos;
    float acc = 0.f;
    for (int j = 0; j < Halfband::kSide; ++j)
      acc += c[j] * (b[Halfband::kSide - 1 - j] + b[Halfband::kSide + j]);
    out[0] = 2.f * acc;
    out[1] = b[Halfband::kSide - 1];
  }
};

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

// One Blades channel at the oversampled rate (signals in units of 5 V).
struct BladeFilter {
  float ic1 = 0.f, ic2 = 0.f;
  // Per-base-sample coefficients.
  float g = 0.1f, k = 2.f, a1 = 1.f, a2 = 0.f, a3 = 0.f;
  float drive = 1.f, dirty = 0.f, mode = 0.f;

  void setup(float gIn, float kIn) {
    g = gIn;
    k = kIn;
    a1 = 1.f / (1.f + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
  }

  // Returns the mode-morphed output; `level` receives the driven input.
  float process(float x, float& level) {
    // Input stage: zener-limited VCA path crossfaded with the clean signal.
    float hot = 1.14f * tanhFast(x * drive * (1.f / 1.14f));
    float in = x + (hot - x) * dirty;
    level = in;
    // TPT SVF; the band-pass integrator is soft-limited (zener clamp).
    float v3 = in - ic2;
    float v1 = a1 * ic1 + a2 * v3;
    // Zener clamp: transparent below the knee, then a firm shoulder, so the
    // self-oscillation level is set by the diodes rather than the curvature.
    float a = std::fabs(v1);
    if (a > 0.9f) {
      float lim = 0.9f + 0.3f * tanhFast((a - 0.9f) * (1.f / 0.3f));
      v1 = v1 < 0.f ? -lim : lim;
    }
    float v2 = ic2 + g * v1;
    ic1 = clampf(2.f * v1 - ic1, -4.f, 4.f);
    ic2 = clampf(2.f * v2 - ic2, -8.f, 8.f);
    float hp = in - k * v1 - v2;
    // LP -> BP -> HP crossfade.
    float m = mode * 2.f;
    return m < 1.f ? v2 + (v1 - v2) * m : v1 + (hp - v1) * (m - 1.f);
  }
};

class DualFilter : public Module {
 public:
  enum ParamIds {
    FREQ1_PARAM,
    FREQ2_PARAM,
    RESO1_PARAM,
    RESO2_PARAM,
    MODE1_PARAM,
    MODE2_PARAM,
    DRIVE1_PARAM,
    DRIVE2_PARAM,
    DIRT1_PARAM,
    DIRT2_PARAM,
    FM1_PARAM,
    FM2_PARAM,
    RESOCV1_PARAM,
    RESOCV2_PARAM,
    ROUTING_PARAM,
    ROUTINGCV_PARAM,
    SHIFT_PARAM,
    NUM_PARAMS
  };
  enum InputIds {
    IN1_INPUT,
    IN2_INPUT,
    VOCT1_INPUT,
    VOCT2_INPUT,
    FM1_INPUT,
    FM2_INPUT,
    RESO1_INPUT,
    RESO2_INPUT,
    MODE1_INPUT,
    MODE2_INPUT,
    DRIVE1_INPUT,
    DRIVE2_INPUT,
    ROUTING_INPUT,
    NUM_INPUTS
  };
  enum OutputIds { OUT1_OUTPUT, OUT2_OUTPUT, MAIN_OUTPUT, NUM_OUTPUTS };
  enum LightIds { LEVEL1_LIGHT, LEVEL2_LIGHT, SHIFT_LIGHT, NUM_LIGHTS };

  DualFilter() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < 2; ++i) {
      char n[64];
      std::snprintf(n, sizeof(n), "Filter %d frequency", i + 1);
      configParam(FREQ1_PARAM + i, 0.f, 1.f, i == 0 ? 0.55f : 0.65f, n, "Hz", 15.f).displayBase =
          1448.15f;  // 15 Hz * 2^(10.5 * knob)
      std::snprintf(n, sizeof(n), "Filter %d resonance", i + 1);
      configParam(RESO1_PARAM + i, 0.f, 1.f, 0.f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Filter %d mode (LP - BP - HP)", i + 1);
      configParam(MODE1_PARAM + i, 0.f, 1.f, 0.f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Filter %d drive", i + 1);
      configParam(DRIVE1_PARAM + i, 0.f, 1.f, 0.3f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Filter %d clean / dirty", i + 1);
      configParam(DIRT1_PARAM + i, 0.f, 1.f, 0.f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Filter %d FM amount", i + 1);
      configParam(FM1_PARAM + i, -1.f, 1.f, 0.f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), "Filter %d resonance CV amount", i + 1);
      configParam(RESOCV1_PARAM + i, -1.f, 1.f, 0.f, n, "%", 100.f);
      std::snprintf(n, sizeof(n), i == 0 ? "Filter 1 in" : "Filter 2 in (normalled to in 1)");
      configInput(IN1_INPUT + i, n);
      std::snprintf(n, sizeof(n), i == 0 ? "Filter 1 V/oct" : "Filter 2 V/oct (normalled to 1)");
      configInput(VOCT1_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Filter %d FM", i + 1);
      configInput(FM1_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Filter %d resonance CV", i + 1);
      configInput(RESO1_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Filter %d mode CV", i + 1);
      configInput(MODE1_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Filter %d drive CV", i + 1);
      configInput(DRIVE1_INPUT + i, n);
      std::snprintf(n, sizeof(n), "Filter %d", i + 1);
      configOutput(OUT1_OUTPUT + i, n);
    }
    configParam(ROUTING_PARAM, 0.f, 1.f, 0.5f, "Routing (1 only - parallel - series)", "%", 100.f);
    configParam(ROUTINGCV_PARAM, -1.f, 1.f, 0.f, "Routing CV amount", "%", 100.f);
    configToggle(SHIFT_PARAM, "Shift: filter 2 follows filter 1", false);
    configInput(ROUTING_INPUT, "Routing CV");
    configOutput(MAIN_OUTPUT, "Main");
    onSampleRateChange(48000.f);
  }

  void onSampleRateChange(float sr) override { osRate_ = 2.f * sr; }

  void process(const ProcessArgs& args) override {
    // --- Control rate work, once per engine sample. ---
    for (int i = 0; i < 6; ++i) {
      int id = kSmoothed[i];
      smooth_[i] += (params[id].value - smooth_[i]) * 0.01f;
    }
    float voct1 = inputs[VOCT1_INPUT].getVoltage();
    float voct2 = inputs[VOCT2_INPUT].getNormalVoltage(voct1);
    float pitch1 = smooth_[0] * 10.5f + voct1 + params[FM1_PARAM].value * inputs[FM1_INPUT].getVoltage();
    float pitch2;
    bool shift = params[SHIFT_PARAM].value > 0.5f;
    if (shift) {
      pitch2 = pitch1 + (smooth_[1] - 0.5f) * 8.f +
               (inputs[VOCT2_INPUT].isConnected() ? voct2 - voct1 : 0.f) +
               params[FM2_PARAM].value * inputs[FM2_INPUT].getVoltage();
    } else {
      pitch2 = smooth_[1] * 10.5f + voct2 + params[FM2_PARAM].value * inputs[FM2_INPUT].getVoltage();
    }
    const float pitch[2] = {pitch1, pitch2};
    for (int i = 0; i < 2; ++i) {
      float f = 15.f * std::exp2(clampf(pitch[i], -1.f, 10.6f));
      f = std::min(f, 0.21f * osRate_);
      float g = tanPrewarp(static_cast<float>(M_PI) * f / osRate_);
      float r = clampf(smooth_[2 + i] +
                           params[RESOCV1_PARAM + i].value * inputs[RESO1_INPUT + i].getVoltage() * 0.2f,
                       0.f, 1.2f);
      // Damping from 2 (Q 0.5) down through zero: self-oscillation from ~88 %,
      // with enough negative damping at the top for a prompt start-up.
      float r3 = r * r * r;
      float k = std::max(-0.2f, 2.f * std::exp2(-6.f * r) - 0.11f * r3 * r3);
      filter_[i].setup(g, k);
      float m = clampf(params[MODE1_PARAM + i].value + inputs[MODE1_INPUT + i].getVoltage() * 0.2f, 0.f, 1.f);
      filter_[i].mode = m;
      float d = clampf(smooth_[4 + i] + inputs[DRIVE1_INPUT + i].getVoltage() * 0.2f, 0.f, 1.2f);
      filter_[i].drive = std::exp2(d * 5.f - 1.f);  // -6 dB .. +24 dB (+30 dB with CV)
      filter_[i].dirty = params[DIRT1_PARAM + i].value * params[DIRT1_PARAM + i].value;
    }
    float route = clampf(params[ROUTING_PARAM].value +
                             params[ROUTINGCV_PARAM].value * inputs[ROUTING_INPUT].getVoltage() * 0.2f,
                         0.f, 1.f);
    route_ += (route - route_) * 0.01f;
    // 0 .. 0.5: filter 2 fades into MAIN (1 only -> parallel).
    // 0.5 .. 1: filter 2's input crossfades to filter 1's output (series),
    //           while filter 1 fades out of MAIN.
    float mix2 = std::min(1.f, route_ * 2.f);
    float series = std::max(0.f, route_ * 2.f - 1.f);

    float in1 = inputs[IN1_INPUT].getVoltage() * 0.2f;
    float in2 = inputs[IN2_INPUT].isConnected() ? inputs[IN2_INPUT].getVoltage() * 0.2f : in1;
    float u1[2], u2[2];
    up1_.process(in1, u1);
    up2_.process(in2, u2);
    float y1[2], y2[2];
    for (int n = 0; n < 2; ++n) {
      noise_ = noise_ * 1664525u + 1013904223u;
      float hiss = static_cast<float>(static_cast<int32_t>(noise_)) * 4.6e-15f;
      float lvl1, lvl2;
      y1[n] = filter_[0].process(u1[n] + hiss, lvl1);
      float x2 = u2[n] + (y1[n] - u2[n]) * series;
      y2[n] = filter_[1].process(x2 - hiss, lvl2);
      level_[0] = std::max(level_[0] * 0.9998f, std::fabs(lvl1));
      level_[1] = std::max(level_[1] * 0.9998f, std::fabs(lvl2));
    }
    float o1 = dn1_.process(y1[0], y1[1]);
    float o2 = dn2_.process(y2[0], y2[1]);
    if (!std::isfinite(o1) || !std::isfinite(o2)) {
      filter_[0] = BladeFilter();
      filter_[1] = BladeFilter();
      dn1_ = Downsampler2x();
      dn2_ = Downsampler2x();
      o1 = o2 = 0.f;
    }
    float main = o1 * (1.f - series) + o2 * mix2;
    outputs[OUT1_OUTPUT].setVoltage(railClip(o1 * 5.f));
    outputs[OUT2_OUTPUT].setVoltage(railClip(o2 * 5.f));
    outputs[MAIN_OUTPUT].setVoltage(railClip(main * 5.f));

    if ((args.frame & 63) == 0) {
      lights[LEVEL1_LIGHT] = std::min(1.f, level_[0]);
      lights[LEVEL2_LIGHT] = std::min(1.f, level_[1]);
      lights[SHIFT_LIGHT] = shift ? 1.f : 0.f;
    }
  }

  void onReset() override {
    filter_[0] = BladeFilter();
    filter_[1] = BladeFilter();
  }

 private:
  static constexpr int kSmoothed[6] = {FREQ1_PARAM,  FREQ2_PARAM,  RESO1_PARAM,
                                       RESO2_PARAM,  DRIVE1_PARAM, DRIVE2_PARAM};
  float smooth_[6] = {0.55f, 0.65f, 0.f, 0.f, 0.3f, 0.3f};
  float osRate_ = 96000.f;
  float route_ = 0.5f;
  float level_[2] = {0.f, 0.f};
  uint32_t noise_ = 1u;
  BladeFilter filter_[2];
  Upsampler2x up1_, up2_;
  Downsampler2x dn1_, dn2_;
};

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

// Response glyphs for the MODE knob: 0 = LP, 1 = BP, 2 = HP.
void modeGlyph(PanelSpec& p, float x, float y, int type, uint32_t col) {
  const float w = 0.4f;
  if (type == 0) {
    p.line(x - 1.6f, y - 0.6f, x + 0.4f, y - 0.6f, w, col);
    p.line(x + 0.4f, y - 0.6f, x + 1.4f, y + 1.0f, w, col);
  } else if (type == 2) {
    p.line(x - 1.4f, y + 1.0f, x - 0.4f, y - 0.6f, w, col);
    p.line(x - 0.4f, y - 0.6f, x + 1.6f, y - 0.6f, w, col);
  } else {
    p.line(x - 1.8f, y + 0.8f, x - 0.8f, y + 0.8f, w, col);
    p.line(x - 0.8f, y + 0.8f, x, y - 0.8f, w, col);
    p.line(x, y - 0.8f, x + 0.8f, y + 0.8f, w, col);
    p.line(x + 0.8f, y + 0.8f, x + 1.8f, y + 0.8f, w, col);
  }
}

void arrow(PanelSpec& p, float x0, float x1, float y, uint32_t col) {
  p.line(x0, y, x1, y, 0.4f, col);
  p.line(x1, y, x1 - 0.8f, y - 0.6f, 0.4f, col);
  p.line(x1, y, x1 - 0.8f, y + 0.6f, 0.4f, col);
}

void buildPanel(PanelSpec& p) {
  p.hp = 18;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  const uint32_t ink = 0xFF2B2B2E, grey = 0xFF77777A, white = 0xFFFFFFFF;
  const uint32_t pink = 0xFFC8414B, teal = 0xFF168A8C;
  const float W = 91.44f;

  for (int i = 0; i < 2; ++i) {
    // Mirror filter 2 around the centre line.
    auto X = [&](float x) { return i == 0 ? x : W - x; };
    const uint32_t col = i == 0 ? pink : teal;
    int o = i;  // param/port offset

    // Paisley outline stand-in around the frequency knob.
    p.display(X(28.0f) - 12.f, 30.0f, 24.f, 26.f, [](Canvas& c, float x, float y, float w,
                                                       float h, float s, const Module*) {
      float cx = x + w * 0.5f, cy = y + h * 0.5f;
      c.strokeCircle(cx, cy, 10.2f * s, std::max(1.f, 0.35f * s), 0xFFDAD7CF);
      c.strokeCircle(cx, cy, 11.3f * s, std::max(1.f, 0.2f * s), 0xFFDAD7CF);
    });

    p.knob(DualFilter::DRIVE1_PARAM + o, X(9.5f), 20.6f, 10.f);
    p.label(X(9.5f), 30.6f, "DRIVE", 3.0f);
    dotted(p, X(9.5f), 32.4f, X(9.5f), 35.2f, ink);
    p.knob(DualFilter::DIRT1_PARAM + o, X(9.5f), 39.8f, 6.5f);
    // Sine "clean" glyph.
    {
      float x = X(9.5f) - 5.2f, y = 43.4f;
      float px = x - 1.5f, py = y;
      for (int s = 1; s <= 8; ++s) {
        float t = static_cast<float>(s) / 8.f;
        float nx = x - 1.5f + 3.f * t, ny = y - 0.8f * std::sin(t * 2.f * static_cast<float>(M_PI));
        p.line(px, py, nx, ny, 0.35f, grey);
        px = nx;
        py = ny;
      }
    }
    // Jagged "dirty" glyph.
    {
      float x = X(9.5f) + 5.2f, y = 43.4f;
      p.line(x - 1.4f, y + 0.6f, x - 0.7f, y - 0.8f, 0.35f, grey);
      p.line(x - 0.7f, y - 0.8f, x - 0.2f, y + 0.7f, 0.35f, grey);
      p.line(x - 0.2f, y + 0.7f, x + 0.4f, y - 0.6f, 0.35f, grey);
      p.line(x + 0.4f, y - 0.6f, x + 0.9f, y + 0.6f, 0.35f, grey);
      p.line(x + 0.9f, y + 0.6f, x + 1.4f, y - 0.4f, 0.35f, grey);
    }

    modeGlyph(p, X(9.5f), 52.0f, 1, grey);
    p.knob(DualFilter::MODE1_PARAM + o, X(9.5f), 60.0f, 9.5f);
    modeGlyph(p, X(3.4f), 66.4f, i == 0 ? 0 : 2, grey);
    modeGlyph(p, X(15.8f), 66.4f, i == 0 ? 2 : 0, grey);
    p.label(X(9.5f), 71.1f, "MODE", 3.0f);
    dotted(p, X(9.5f), 72.9f, X(9.5f), 77.6f, ink);
    p.input(DualFilter::MODE1_INPUT + o, X(9.5f), 81.0f);
    p.input(DualFilter::DRIVE1_INPUT + o, X(9.5f), 96.4f, "DRIVE", -6.2f);
    p.input(DualFilter::IN1_INPUT + o, X(9.5f), 110.5f, "IN", -5.8f);

    p.light(DualFilter::LEVEL1_LIGHT + o, X(21.2f), 14.5f, i == 0 ? 0xFFFF8FB4 : 0xFF7FE3C4, 2.6f);
    p.box(X(21.3f) - 1.9f, 27.4f, 3.8f, 3.8f, col, 1.9f);
    p.label(X(21.3f), 29.3f, i == 0 ? "1" : "2", 2.8f, kAlignCenter, white);
    p.knob(DualFilter::RESO1_PARAM + o, X(33.4f), 18.4f, 10.f);
    p.label(X(33.4f), 27.6f, "RESO", 3.0f);
    p.knob(DualFilter::FREQ1_PARAM + o, X(28.0f), 43.0f, 14.f);
    p.label(X(28.0f), 55.6f, "FREQUENCY", 3.0f);

    // Attenuverters and their CV jacks.
    p.knob(DualFilter::FM1_PARAM + o, X(21.3f), 81.0f, 6.5f);
    p.knob(DualFilter::RESOCV1_PARAM + o, X(33.4f), 81.0f, 6.5f);
    for (float ax : {21.3f, 33.4f}) {
      p.label(X(ax) - 3.2f, 75.6f, "-", 3.0f);
      p.line(X(ax), 74.6f, X(ax), 76.8f, 0.45f, ink);
      p.label(X(ax) + 3.2f, 75.6f, "+", 3.0f);
      dotted(p, X(ax), 84.6f, X(ax), 92.4f, ink);
    }
    dotted(p, X(28.0f), 57.6f, X(23.5f), 66.0f, ink);
    dotted(p, X(23.5f), 66.0f, X(21.3f), 73.6f, ink);
    p.input(DualFilter::FM1_INPUT + o, X(21.3f), 96.4f);
    p.label(X(33.4f), 89.8f, "RESO", 3.0f);
    p.input(DualFilter::RESO1_INPUT + o, X(33.4f), 96.4f);
    p.input(DualFilter::VOCT1_INPUT + o, X(21.3f), 110.5f, "V/OCT", -5.8f);
  }

  // Centre column: SHIFT, ROUTING.
  const float cx = W * 0.5f;
  p.button(DualFilter::SHIFT_PARAM, cx, 34.4f, 6.0f, "", 0xFFD8D9DB, DualFilter::SHIFT_LIGHT);
  p.label(cx, 40.6f, "SHIFT", 3.0f);
  dotted(p, cx + 3.4f, 34.0f, 54.5f, 37.0f, ink);
  dotted(p, 54.5f, 37.0f, 59.6f, 41.6f, ink);
  arrow(p, cx - 1.6f, cx + 1.6f, 50.6f, pink);
  arrow(p, cx - 1.6f, cx + 1.6f, 51.9f, teal);
  p.knob(DualFilter::ROUTING_PARAM, cx, 60.0f, 9.5f);
  arrow(p, 36.0f, 38.8f, 67.0f, pink);
  arrow(p, 51.4f, 53.6f, 67.0f, pink);
  arrow(p, 53.8f, 56.0f, 67.0f, teal);
  p.label(cx, 71.1f, "ROUTING", 3.0f);
  dotted(p, cx, 72.9f, cx, 74.6f, ink);
  p.knob(DualFilter::ROUTINGCV_PARAM, cx, 81.0f, 6.5f);
  p.label(cx - 3.2f, 75.6f, "-", 3.0f);
  p.label(cx + 3.2f, 75.6f, "+", 3.0f);
  dotted(p, cx, 84.6f, cx, 92.4f, ink);
  p.input(DualFilter::ROUTING_INPUT, cx, 96.4f);

  p.box(28.0f, 102.4f, 35.4f, 14.6f, 0xFF5E5E60, 1.2f);
  p.output(DualFilter::OUT1_OUTPUT, 33.4f, 110.5f);
  p.output(DualFilter::MAIN_OUTPUT, cx, 110.5f);
  p.output(DualFilter::OUT2_OUTPUT, W - 33.4f, 110.5f);
  p.label(33.4f, 104.9f, "1", 3.0f, kAlignCenter, white);
  p.label(cx, 104.9f, "MAIN", 3.0f, kAlignCenter, white);
  p.label(W - 33.4f, 104.9f, "2", 3.0f, kAlignCenter, white);
  p.line(3.4f, 119.6f, W - 3.4f, 119.6f, 0.6f, 0xFFB8B8B8);
}

Model model = [] {
  Model m;
  m.slug = "dual-multimode-filter";
  m.name = "Dual Multimode Filter";
  m.shortName = "DUAL FILTER";
  m.basedOn = "Blades";
  m.category = Category::Filter;
  m.description = "Two drivable LP/BP/HP morphing filters, self-oscillating, parallel to series routing.";
  m.create = [] { return new DualFilter(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
