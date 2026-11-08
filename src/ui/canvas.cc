// BrickRack - GPL-3.0-or-later.

#include "ui/canvas.h"

#include <algorithm>
#include <cmath>

#include "ui/font.h"

namespace br {

namespace {
Font* g_font = nullptr;

inline float clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }

// Decodes one UTF-8 code point.
int nextCodepoint(const std::string& s, size_t& i) {
  unsigned char c = static_cast<unsigned char>(s[i++]);
  if (c < 0x80) return c;
  int n = (c >= 0xF0) ? 3 : (c >= 0xE0) ? 2 : 1;
  int cp = c & (0x3F >> n);
  for (int k = 0; k < n && i < s.size(); ++k) {
    cp = (cp << 6) | (static_cast<unsigned char>(s[i++]) & 0x3F);
  }
  return cp;
}
}  // namespace

uint32_t mixColor(uint32_t a, uint32_t b, float t) {
  t = clamp01(t);
  auto ch = [&](int sh) {
    float x = ((a >> sh) & 0xFF) * (1.f - t) + ((b >> sh) & 0xFF) * t;
    return static_cast<uint32_t>(x + 0.5f) << sh;
  };
  return ch(24) | ch(16) | ch(8) | ch(0);
}

uint32_t scaleColor(uint32_t c, float k) {
  auto ch = [&](int sh) {
    float x = ((c >> sh) & 0xFF) * k;
    x = x > 255.f ? 255.f : x;
    return static_cast<uint32_t>(x + 0.5f) << sh;
  };
  return (c & 0xFF000000) | ch(16) | ch(8) | ch(0);
}

void Canvas::setFont(Font* font) { g_font = font; }
Font* Canvas::font() { return g_font; }

Canvas::Canvas(int w, int h) { resize(w, h); }

void Canvas::resize(int w, int h) {
  w_ = w;
  h_ = h;
  px_.assign(static_cast<size_t>(w * h), 0);
  clip_.clear();
}

void Canvas::clear(uint32_t color) { std::fill(px_.begin(), px_.end(), color); }

void Canvas::pushClip(Rect r) {
  {
    // Always stay inside the canvas, so blendPixel never sees out-of-range
    // coordinates even if a caller passes an oversized root clip.
    int x0 = std::max(r.x, 0), y0 = std::max(r.y, 0);
    int x1 = std::min(r.x + r.w, w_), y1 = std::min(r.y + r.h, h_);
    r = {x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0)};
  }
  if (!clip_.empty()) {
    const Rect& c = clip_.back();
    int x0 = std::max(r.x, c.x), y0 = std::max(r.y, c.y);
    int x1 = std::min(r.x + r.w, c.x + c.w), y1 = std::min(r.y + r.h, c.y + c.h);
    r = {x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0)};
  }
  clip_.push_back(r);
}

void Canvas::popClip() {
  if (!clip_.empty()) clip_.pop_back();
}

Rect Canvas::clipBox(float x0, float y0, float x1, float y1) const {
  int ix0 = static_cast<int>(std::floor(x0)), iy0 = static_cast<int>(std::floor(y0));
  int ix1 = static_cast<int>(std::ceil(x1)), iy1 = static_cast<int>(std::ceil(y1));
  int cx0 = 0, cy0 = 0, cx1 = w_, cy1 = h_;
  if (!clip_.empty()) {
    const Rect& c = clip_.back();
    cx0 = c.x;
    cy0 = c.y;
    cx1 = c.x + c.w;
    cy1 = c.y + c.h;
  }
  ix0 = std::max(ix0, cx0);
  iy0 = std::max(iy0, cy0);
  ix1 = std::min(ix1, cx1);
  iy1 = std::min(iy1, cy1);
  return {ix0, iy0, std::max(0, ix1 - ix0), std::max(0, iy1 - iy0)};
}

void Canvas::blendPixel(int x, int y, uint32_t color, float coverage) {
  float a = ((color >> 24) & 0xFF) / 255.f * coverage;
  if (a <= 0.002f) return;
  uint32_t& d = px_[static_cast<size_t>(y * w_ + x)];
  if (a >= 0.998f) {
    d = color | 0xFF000000;
    return;
  }
  int ia = static_cast<int>(a * 256.f);
  int na = 256 - ia;
  uint32_t dr = (d >> 16) & 0xFF, dg = (d >> 8) & 0xFF, db = d & 0xFF;
  uint32_t sr = (color >> 16) & 0xFF, sg = (color >> 8) & 0xFF, sb = color & 0xFF;
  uint32_t r = (sr * ia + dr * na) >> 8;
  uint32_t g = (sg * ia + dg * na) >> 8;
  uint32_t b = (sb * ia + db * na) >> 8;
  d = 0xFF000000 | (r << 16) | (g << 8) | b;
}

void Canvas::fillRect(float x, float y, float w, float h, uint32_t color) {
  Rect r = clipBox(x, y, x + w, y + h);
  for (int j = r.y; j < r.y + r.h; ++j) {
    float cy = clamp01(std::min(j + 1.f, y + h) - std::max(static_cast<float>(j), y));
    for (int i = r.x; i < r.x + r.w; ++i) {
      float cx = clamp01(std::min(i + 1.f, x + w) - std::max(static_cast<float>(i), x));
      blendPixel(i, j, color, cx * cy);
    }
  }
}

void Canvas::fillRoundRect(float x, float y, float w, float h, float radius, uint32_t color) {
  radius = std::min(radius, std::min(w, h) * 0.5f);
  if (radius < 0.5f) {
    fillRect(x, y, w, h, color);
    return;
  }
  Rect r = clipBox(x, y, x + w, y + h);
  float hx = w * 0.5f, hy = h * 0.5f, cx = x + hx, cy = y + hy;
  for (int j = r.y; j < r.y + r.h; ++j) {
    float py = std::fabs(j + 0.5f - cy) - (hy - radius);
    for (int i = r.x; i < r.x + r.w; ++i) {
      float px = std::fabs(i + 0.5f - cx) - (hx - radius);
      float qx = std::max(px, 0.f), qy = std::max(py, 0.f);
      float d = std::sqrt(qx * qx + qy * qy) + std::min(std::max(px, py), 0.f) - radius;
      blendPixel(i, j, color, clamp01(0.5f - d));
    }
  }
}

void Canvas::strokeRoundRect(float x, float y, float w, float h, float radius, float width,
                             uint32_t color) {
  radius = std::min(radius, std::min(w, h) * 0.5f);
  Rect r = clipBox(x - width, y - width, x + w + width, y + h + width);
  float hx = w * 0.5f, hy = h * 0.5f, cx = x + hx, cy = y + hy;
  for (int j = r.y; j < r.y + r.h; ++j) {
    float py = std::fabs(j + 0.5f - cy) - (hy - radius);
    for (int i = r.x; i < r.x + r.w; ++i) {
      float px = std::fabs(i + 0.5f - cx) - (hx - radius);
      float qx = std::max(px, 0.f), qy = std::max(py, 0.f);
      float d = std::sqrt(qx * qx + qy * qy) + std::min(std::max(px, py), 0.f) - radius;
      float cov = clamp01(width * 0.5f + 0.5f - std::fabs(d));
      blendPixel(i, j, color, cov);
    }
  }
}

void Canvas::fillCircle(float cx, float cy, float r, uint32_t color) {
  Rect b = clipBox(cx - r - 1, cy - r - 1, cx + r + 1, cy + r + 1);
  for (int j = b.y; j < b.y + b.h; ++j) {
    float dy = j + 0.5f - cy;
    for (int i = b.x; i < b.x + b.w; ++i) {
      float dx = i + 0.5f - cx;
      float d = std::sqrt(dx * dx + dy * dy);
      blendPixel(i, j, color, clamp01(r - d + 0.5f));
    }
  }
}

void Canvas::strokeCircle(float cx, float cy, float r, float width, uint32_t color) {
  float ro = r + width * 0.5f;
  Rect b = clipBox(cx - ro - 1, cy - ro - 1, cx + ro + 1, cy + ro + 1);
  for (int j = b.y; j < b.y + b.h; ++j) {
    float dy = j + 0.5f - cy;
    for (int i = b.x; i < b.x + b.w; ++i) {
      float dx = i + 0.5f - cx;
      float d = std::sqrt(dx * dx + dy * dy);
      blendPixel(i, j, color, clamp01(width * 0.5f + 0.5f - std::fabs(d - r)));
    }
  }
}

void Canvas::fillCircleGradient(float cx, float cy, float r, uint32_t inner, uint32_t outer,
                                float ox, float oy) {
  Rect b = clipBox(cx - r - 1, cy - r - 1, cx + r + 1, cy + r + 1);
  float gx = cx + ox * r, gy = cy + oy * r;
  for (int j = b.y; j < b.y + b.h; ++j) {
    float dy = j + 0.5f - cy;
    for (int i = b.x; i < b.x + b.w; ++i) {
      float dx = i + 0.5f - cx;
      float d = std::sqrt(dx * dx + dy * dy);
      float cov = clamp01(r - d + 0.5f);
      if (cov <= 0.f) continue;
      float gdx = i + 0.5f - gx, gdy = j + 0.5f - gy;
      float t = std::sqrt(gdx * gdx + gdy * gdy) / (r * 1.6f);
      blendPixel(i, j, mixColor(inner, outer, t), cov);
    }
  }
}

void Canvas::line(float x0, float y0, float x1, float y1, float width, uint32_t color) {
  float hw = width * 0.5f;
  Rect b = clipBox(std::min(x0, x1) - hw - 1, std::min(y0, y1) - hw - 1,
                   std::max(x0, x1) + hw + 1, std::max(y0, y1) + hw + 1);
  float dx = x1 - x0, dy = y1 - y0;
  float len2 = dx * dx + dy * dy;
  for (int j = b.y; j < b.y + b.h; ++j) {
    float py = j + 0.5f - y0;
    for (int i = b.x; i < b.x + b.w; ++i) {
      float px = i + 0.5f - x0;
      float t = len2 > 0.f ? clamp01((px * dx + py * dy) / len2) : 0.f;
      float ex = px - t * dx, ey = py - t * dy;
      float d = std::sqrt(ex * ex + ey * ey);
      blendPixel(i, j, color, clamp01(hw - d + 0.5f));
    }
  }
}

void Canvas::polyline(const float* xy, int n, float width, uint32_t color) {
  if (n < 2) return;
  // Bounding box of the whole polyline, then per-pixel min distance: avoids
  // double-blending at joints so translucent strokes stay even.
  float minx = xy[0], maxx = xy[0], miny = xy[1], maxy = xy[1];
  for (int k = 1; k < n; ++k) {
    minx = std::min(minx, xy[2 * k]);
    maxx = std::max(maxx, xy[2 * k]);
    miny = std::min(miny, xy[2 * k + 1]);
    maxy = std::max(maxy, xy[2 * k + 1]);
  }
  float hw = width * 0.5f;
  // Process each segment's bbox but evaluate distance to neighbouring segments
  // too, so each pixel is only blended by the segment closest to it.
  for (int s = 0; s < n - 1; ++s) {
    float ax = xy[2 * s], ay = xy[2 * s + 1], bx = xy[2 * s + 2], by = xy[2 * s + 3];
    Rect b = clipBox(std::min(ax, bx) - hw - 1, std::min(ay, by) - hw - 1,
                     std::max(ax, bx) + hw + 1, std::max(ay, by) + hw + 1);
    for (int j = b.y; j < b.y + b.h; ++j) {
      for (int i = b.x; i < b.x + b.w; ++i) {
        float px = i + 0.5f, py = j + 0.5f;
        float best = 1e9f;
        int bestSeg = -1;
        int lo = std::max(0, s - 2), hi = std::min(n - 2, s + 2);
        for (int k = lo; k <= hi; ++k) {
          float x0 = xy[2 * k], y0 = xy[2 * k + 1];
          float ddx = xy[2 * k + 2] - x0, ddy = xy[2 * k + 3] - y0;
          float l2 = ddx * ddx + ddy * ddy;
          float t = l2 > 0.f ? clamp01(((px - x0) * ddx + (py - y0) * ddy) / l2) : 0.f;
          float ex = px - x0 - t * ddx, ey = py - y0 - t * ddy;
          float d = ex * ex + ey * ey;
          if (d < best) {
            best = d;
            bestSeg = k;
          }
        }
        if (bestSeg != s) continue;
        blendPixel(i, j, color, clamp01(hw - std::sqrt(best) + 0.5f));
      }
    }
  }
}

void Canvas::arc(float cx, float cy, float r, float a0, float a1, float width, uint32_t color) {
  int n = std::max(4, static_cast<int>(std::fabs(a1 - a0) * r / 3.f));
  std::vector<float> pts;
  pts.reserve(static_cast<size_t>(2 * (n + 1)));
  for (int k = 0; k <= n; ++k) {
    float a = a0 + (a1 - a0) * k / n;
    pts.push_back(cx + std::cos(a) * r);
    pts.push_back(cy + std::sin(a) * r);
  }
  polyline(pts.data(), n + 1, width, color);
}

float Canvas::textWidth(const std::string& s, float size) {
  if (!g_font) return 0.f;
  int ps = std::max(4, static_cast<int>(size + 0.5f));
  float w = 0.f;
  int prev = 0;
  for (size_t i = 0; i < s.size();) {
    int cp = nextCodepoint(s, i);
    if (prev) w += g_font->kern(prev, cp, ps);
    w += g_font->glyph(cp, ps).advance;
    prev = cp;
  }
  return w;
}

float Canvas::text(float x, float y, const std::string& s, float size, uint32_t color, int align,
                   bool middle) {
  if (!g_font || s.empty()) return 0.f;
  int ps = std::max(4, static_cast<int>(size + 0.5f));
  float w = textWidth(s, size);
  if (align == 1) x -= w * 0.5f;
  else if (align == 2) x -= w;
  // Baseline: centre cap-height around y when `middle`.
  float baseline = middle ? y + ps * 0.36f : y + g_font->ascent(ps);
  float pen = std::round(x);
  int by = static_cast<int>(std::round(baseline));
  int prev = 0;
  for (size_t i = 0; i < s.size();) {
    int cp = nextCodepoint(s, i);
    if (prev) pen += g_font->kern(prev, cp, ps);
    const Font::Glyph& g = g_font->glyph(cp, ps);
    int gx = static_cast<int>(std::round(pen)) + g.xoff;
    int gy = by + g.yoff;
    if (!g.bitmap.empty()) {
      Rect b = clipBox(static_cast<float>(gx), static_cast<float>(gy),
                       static_cast<float>(gx + g.w), static_cast<float>(gy + g.h));
      for (int j = b.y; j < b.y + b.h; ++j) {
        const uint8_t* row = &g.bitmap[static_cast<size_t>((j - gy) * g.w)];
        for (int i2 = b.x; i2 < b.x + b.w; ++i2) {
          uint8_t a = row[i2 - gx];
          if (a) blendPixel(i2, j, color, a / 255.f);
        }
      }
    }
    pen += g.advance;
    prev = cp;
  }
  return w;
}

void Canvas::blit(const Canvas& src, int dx, int dy) {
  Rect b = clipBox(static_cast<float>(dx), static_cast<float>(dy),
                   static_cast<float>(dx + src.w_), static_cast<float>(dy + src.h_));
  for (int j = b.y; j < b.y + b.h; ++j) {
    const uint32_t* s = &src.px_[static_cast<size_t>((j - dy) * src.w_ + (b.x - dx))];
    uint32_t* d = &px_[static_cast<size_t>(j * w_ + b.x)];
    for (int i = 0; i < b.w; ++i) {
      uint32_t c = s[i];
      uint32_t a = c >> 24;
      if (a == 0xFF) d[i] = c;
      else if (a) blendPixel(b.x + i, j, c, 1.f);
    }
  }
}

void Canvas::blitScaled(const Canvas& src, float dx, float dy, float scale) {
  if (scale <= 0.f) return;
  float w = src.w_ * scale, h = src.h_ * scale;
  Rect b = clipBox(dx, dy, dx + w, dy + h);
  float inv = 1.f / scale;
  for (int j = b.y; j < b.y + b.h; ++j) {
    float sy = (j + 0.5f - dy) * inv - 0.5f;
    int y0 = std::max(0, std::min(src.h_ - 1, static_cast<int>(std::floor(sy))));
    int y1 = std::min(src.h_ - 1, y0 + 1);
    float fy = clamp01(sy - y0);
    for (int i = b.x; i < b.x + b.w; ++i) {
      float sx = (i + 0.5f - dx) * inv - 0.5f;
      int x0 = std::max(0, std::min(src.w_ - 1, static_cast<int>(std::floor(sx))));
      int x1 = std::min(src.w_ - 1, x0 + 1);
      float fx = clamp01(sx - x0);
      uint32_t c00 = src.px_[static_cast<size_t>(y0 * src.w_ + x0)];
      uint32_t c10 = src.px_[static_cast<size_t>(y0 * src.w_ + x1)];
      uint32_t c01 = src.px_[static_cast<size_t>(y1 * src.w_ + x0)];
      uint32_t c11 = src.px_[static_cast<size_t>(y1 * src.w_ + x1)];
      uint32_t c = mixColor(mixColor(c00, c10, fx), mixColor(c01, c11, fx), fy);
      blendPixel(i, j, c, 1.f);
    }
  }
}

}  // namespace br
