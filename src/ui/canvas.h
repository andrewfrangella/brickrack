// BrickRack - GPL-3.0-or-later.
//
// A small anti-aliased software rasteriser drawing into an ARGB8888 buffer.
// Everything on screen is drawn with it; the platform layer uploads the final
// buffer as one texture per frame. This keeps rendering identical on the
// desktop and on the device, and lets us produce screenshots headlessly.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace br {

class Font;

inline uint32_t argb(int a, int r, int g, int b) {
  return (static_cast<uint32_t>(a) << 24) | (static_cast<uint32_t>(r) << 16) |
         (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
}
inline uint32_t withAlpha(uint32_t c, float a) {
  int al = static_cast<int>(((c >> 24) & 0xFF) * (a < 0 ? 0 : (a > 1 ? 1 : a)) + 0.5f);
  return (c & 0x00FFFFFF) | (static_cast<uint32_t>(al) << 24);
}
uint32_t mixColor(uint32_t a, uint32_t b, float t);
uint32_t scaleColor(uint32_t c, float k);  // brightness multiply (keeps alpha)

struct Rect {
  int x = 0, y = 0, w = 0, h = 0;
  bool contains(int px, int py) const { return px >= x && py >= y && px < x + w && py < y + h; }
};

class Canvas {
 public:
  Canvas() = default;
  Canvas(int w, int h);
  void resize(int w, int h);

  int width() const { return w_; }
  int height() const { return h_; }
  uint32_t* pixels() { return px_.data(); }
  const uint32_t* pixels() const { return px_.data(); }

  void clear(uint32_t color);
  void pushClip(Rect r);
  void popClip();

  void fillRect(float x, float y, float w, float h, uint32_t color);
  void fillRoundRect(float x, float y, float w, float h, float radius, uint32_t color);
  void strokeRoundRect(float x, float y, float w, float h, float radius, float width,
                       uint32_t color);
  void fillCircle(float cx, float cy, float r, uint32_t color);
  void strokeCircle(float cx, float cy, float r, float width, uint32_t color);
  // Radial gradient disc: centre colour -> edge colour.
  void fillCircleGradient(float cx, float cy, float r, uint32_t inner, uint32_t outer,
                          float ox = 0.f, float oy = 0.f);
  void line(float x0, float y0, float x1, float y1, float width, uint32_t color);
  void polyline(const float* xy, int n, float width, uint32_t color);
  void arc(float cx, float cy, float r, float a0, float a1, float width, uint32_t color);

  // Text. `size` is the pixel height of the font's em box.
  float text(float x, float y, const std::string& s, float size, uint32_t color,
             int align = 0, bool middle = true);
  float textWidth(const std::string& s, float size);

  // Copies another canvas (alpha-blended) at integer position.
  void blit(const Canvas& src, int dx, int dy);
  // Copies a canvas scaled by an arbitrary factor (nearest/bilinear lite).
  void blitScaled(const Canvas& src, float dx, float dy, float scale);

  void blendPixel(int x, int y, uint32_t color, float coverage);

  static void setFont(Font* font);
  static Font* font();

 private:
  Rect clipBox(float x0, float y0, float x1, float y1) const;
  int w_ = 0, h_ = 0;
  std::vector<uint32_t> px_;
  std::vector<Rect> clip_;
};

}  // namespace br
