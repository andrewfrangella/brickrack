// Checks BlockAdapter round trips: a sine through engine->native->engine must
// come back with the same frequency and amplitude (only delayed).
#include <cmath>
#include <cstdio>
#include "core/resampler.h"
using namespace br;
static int check(float native, int block, float freq) {
  BlockAdapter<1, 1> ad;
  ad.init(48000.f, native, block);
  const int N = 48000;
  std::vector<float> out(N);
  for (int i = 0; i < N; ++i) {
    float in = std::sin(2 * M_PI * freq * i / 48000.0);
    ad.process(&in, &out[i], [](const float* a, float* b, int n) { for (int k = 0; k < n; ++k) b[k] = a[k]; });
  }
  // Least-squares fit of a*sin + b*cos over the 2nd half (any delay), then residual.
  double ss = 0, sc = 0, cc = 0, ys = 0, yc = 0;
  for (int i = N / 2; i < N; ++i) {
    double s_ = std::sin(2 * M_PI * freq * i / 48000.0), c_ = std::cos(2 * M_PI * freq * i / 48000.0);
    ss += s_ * s_; sc += s_ * c_; cc += c_ * c_; ys += out[i] * s_; yc += out[i] * c_;
  }
  double det = ss * cc - sc * sc;
  double a = (ys * cc - yc * sc) / det, b = (yc * ss - ys * sc) / det;
  double e = 0;
  for (int i = N / 2; i < N; ++i) {
    double d = out[i] - a * std::sin(2 * M_PI * freq * i / 48000.0) - b * std::cos(2 * M_PI * freq * i / 48000.0);
    e += d * d;
  }
  double rmsErr = std::sqrt(e / (N / 2));
  double amp = std::sqrt(a * a + b * b);
  std::printf("native %6.0f block %3d f %6.0f Hz: gain %.4f, residual %.5f %s\n", native, block, freq, amp, rmsErr,
              (rmsErr < 0.01 && std::fabs(amp - 1) < 0.02) ? "OK" : "FAIL");
  return (rmsErr < 0.01 && std::fabs(amp - 1) < 0.02) ? 0 : 1;
}
int main() {
  int f = 0;
  for (float r : {48000.f, 32000.f, 96000.f, 62500.f, 31250.f})
    for (float fr : {100.f, 1000.f, 5000.f}) f += check(r, r == 96000.f ? 60 : 32, fr);
  return f;
}
