// BrickRack - GPL-3.0-or-later.

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace br {

class Font {
 public:
  struct Glyph {
    int w = 0, h = 0;
    int xoff = 0, yoff = 0;
    float advance = 0.f;
    std::vector<uint8_t> bitmap;
  };

  // Loads the embedded font. Returns false on failure.
  bool loadEmbedded();
  bool load(const uint8_t* data, size_t size);

  const Glyph& glyph(int codepoint, int pixelSize);
  float ascent(int pixelSize) const;
  float kern(int a, int b, int pixelSize) const;

 private:
  std::vector<uint8_t> data_;
  std::shared_ptr<void> info_;  // stbtt_fontinfo
  std::map<uint64_t, Glyph> cache_;
};

}  // namespace br
