// BrickRack - GPL-3.0-or-later.
//
// Streaming sample-rate conversion and block adaptation.
//
// The original firmware runs every module at its own codec rate (Clouds and
// Elements at 32 kHz, Warps at 96 kHz, Tides 2 at 62.5 kHz, ...) and in fixed
// block sizes. The engine runs at one rate, one sample at a time, so each
// wrapper uses a BlockAdapter: engine-rate input frames are converted to the
// native rate, rendered by the untouched DSP code one block at a time, and
// converted back. This keeps pitch tables, filter coefficients and timing
// identical to the hardware.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace br {

// Windowed-sinc streaming resampler. Push one input frame at a time; zero or
// more output frames are emitted through a callback.
template <int CH>
class Resampler {
 public:
  static constexpr int kHalf = 8;            // taps on each side
  static constexpr int kTaps = 2 * kHalf;    // total taps
  static constexpr int kPhases = 128;

  void init(double inRate, double outRate) {
    bypass_ = std::fabs(inRate - outRate) < 1e-6;
    step_ = inRate / outRate;
    t_ = -static_cast<double>(kHalf);
    w_ = 0;
    std::memset(buf_, 0, sizeof(buf_));
    if (bypass_) return;
    // Cut-off relative to the input Nyquist frequency.
    const double fc = std::min(1.0, outRate / inRate) * 0.92;
    table_.assign(static_cast<size_t>((kPhases + 1) * kTaps), 0.f);
    for (int p = 0; p <= kPhases; ++p) {
      double frac = static_cast<double>(p) / kPhases;
      double sum = 0.0;
      float* row = &table_[static_cast<size_t>(p * kTaps)];
      for (int j = 0; j < kTaps; ++j) {
        double x = frac + kHalf - 1 - j;  // distance from output time to tap
        double s = x == 0.0 ? 1.0 : std::sin(M_PI * fc * x) / (M_PI * fc * x);
        double wn = (x + kHalf) / (2.0 * kHalf);  // 0..1 across the window
        double win = 0.42 - 0.5 * std::cos(2 * M_PI * wn) + 0.08 * std::cos(4 * M_PI * wn);
        if (wn <= 0.0 || wn >= 1.0) win = 0.0;
        double h = fc * s * win;
        row[j] = static_cast<float>(h);
        sum += h;
      }
      if (sum != 0.0) {
        for (int j = 0; j < kTaps; ++j) row[j] = static_cast<float>(row[j] / sum);
      }
    }
  }

  template <typename Emit>
  void push(const float* frame, Emit&& emit) {
    if (bypass_) {
      emit(frame);
      return;
    }
    w_ = (w_ + 1) % kTaps;
    for (int c = 0; c < CH; ++c) {
      buf_[c][w_] = frame[c];
      buf_[c][w_ + kTaps] = frame[c];
    }
    t_ -= 1.0;
    // t_ is the time of the next output relative to the newest input sample.
    while (std::floor(t_) <= -kHalf) {
      double fl = std::floor(t_);
      double frac = t_ - fl;
      double pf = frac * kPhases;
      int p = static_cast<int>(pf);
      if (p >= kPhases) p = kPhases - 1;
      float a = static_cast<float>(pf - p);
      const float* r0 = &table_[static_cast<size_t>(p * kTaps)];
      const float* r1 = r0 + kTaps;
      float out[CH];
      for (int c = 0; c < CH; ++c) {
        const float* x = &buf_[c][w_ + 1];
        float s0 = 0.f, s1 = 0.f;
        for (int j = 0; j < kTaps; ++j) {
          s0 += r0[j] * x[j];
          s1 += r1[j] * x[j];
        }
        out[c] = s0 + (s1 - s0) * a;
      }
      emit(out);
      t_ += step_;
    }
  }

 private:
  bool bypass_ = true;
  double step_ = 1.0;
  double t_ = 0.0;
  int w_ = 0;
  float buf_[CH][2 * kTaps];
  std::vector<float> table_;
};

// Runs fixed-size blocks of native-rate DSP from a per-sample engine.
// IN_CH / OUT_CH may be 0 / >0. `render(in, out, n)` receives interleaved
// native-rate frames (in may be null when IN_CH == 0).
template <int IN_CH, int OUT_CH>
class BlockAdapter {
 public:
  static constexpr int kInCh = IN_CH > 0 ? IN_CH : 1;
  static constexpr int kFifoSize = 8192;

  void init(float engineRate, float nativeRate, int blockSize) {
    blockSize_ = blockSize;
    inRs_.init(engineRate, nativeRate);
    outRs_.init(nativeRate, engineRate);
    nativeIn_.assign(static_cast<size_t>(blockSize * kInCh), 0.f);
    nativeOut_.assign(static_cast<size_t>(blockSize * OUT_CH), 0.f);
    nativeCount_ = 0;
    std::memset(fifo_, 0, sizeof(fifo_));
    head_ = tail_ = 0;
    // Prefill so the output never runs dry between bursts.
    int prefill = static_cast<int>(std::ceil(blockSize * engineRate / nativeRate)) + 2;
    if (engineRate != nativeRate) prefill += 2;
    for (int i = 0; i < prefill; ++i) pushOut(zero_);
    std::memset(last_, 0, sizeof(last_));
  }

  int blockSize() const { return blockSize_; }

  // Call once per engine sample.
  template <typename Render>
  void process(const float* in, float* out, Render&& render) {
    float dummy[1] = {0.f};
    const float* src = IN_CH > 0 ? in : dummy;
    inRs_.push(src, [&](const float* f) {
      std::memcpy(&nativeIn_[static_cast<size_t>(nativeCount_ * kInCh)], f,
                  sizeof(float) * kInCh);
      if (++nativeCount_ == blockSize_) {
        nativeCount_ = 0;
        render(IN_CH > 0 ? nativeIn_.data() : nullptr, nativeOut_.data(), blockSize_);
        for (int i = 0; i < blockSize_; ++i) {
          outRs_.push(&nativeOut_[static_cast<size_t>(i * OUT_CH)],
                      [&](const float* o) { pushOut(o); });
        }
      }
    });
    if (head_ != tail_) {
      std::memcpy(last_, fifo_[tail_], sizeof(float) * OUT_CH);
      tail_ = (tail_ + 1) % kFifoSize;
    }
    std::memcpy(out, last_, sizeof(float) * OUT_CH);
  }

 private:
  void pushOut(const float* o) {
    int next = (head_ + 1) % kFifoSize;
    if (next == tail_) return;  // overflow: drop (should not happen)
    std::memcpy(fifo_[head_], o, sizeof(float) * OUT_CH);
    head_ = next;
  }

  int blockSize_ = 1;
  Resampler<kInCh> inRs_;
  Resampler<OUT_CH> outRs_;
  std::vector<float> nativeIn_, nativeOut_;
  int nativeCount_ = 0;
  float fifo_[kFifoSize][OUT_CH];
  int head_ = 0, tail_ = 0;
  float last_[OUT_CH];
  float zero_[OUT_CH] = {};
};

// Helpers shared by module wrappers.
inline float clampf(float x, float lo, float hi) { return std::min(std::max(x, lo), hi); }

// Schmitt trigger with Eurorack-ish thresholds (rising above 1.7 V, falling below 0.7 V).
struct SchmittTrigger {
  bool state = false;
  // Returns true on a rising edge.
  bool process(float v, float lo = 0.7f, float hi = 1.7f) {
    if (state) {
      if (v <= lo) state = false;
    } else if (v >= hi) {
      state = true;
      return true;
    }
    return false;
  }
  bool isHigh() const { return state; }
};

// Edge detector for UI push buttons (param > 0.5).
struct ButtonEdge {
  bool last = false;
  bool process(float v) {
    bool now = v > 0.5f;
    bool rising = now && !last;
    last = now;
    return rising;
  }
};

// Slow light decay helper.
inline float lightDecay(float current, float target, float sampleTime, float tau = 0.08f) {
  float k = std::min(1.f, sampleTime / tau);
  return current + (target - current) * k;
}

}  // namespace br
