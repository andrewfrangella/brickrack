// BrickRack - GPL-3.0-or-later.

#include "ui/font.h"

#define STB_TRUETYPE_IMPLEMENTATION

#include "stb_truetype.h"

#include "font_data.h"  // generated: kEmbeddedFont, kEmbeddedFontSize

namespace br {

bool Font::loadEmbedded() { return load(kEmbeddedFont, kEmbeddedFontSize); }

bool Font::load(const uint8_t* data, size_t size) {
  data_.assign(data, data + size);
  auto info = std::make_shared<stbtt_fontinfo>();
  if (!stbtt_InitFont(info.get(), data_.data(), stbtt_GetFontOffsetForIndex(data_.data(), 0))) {
    return false;
  }
  info_ = info;
  cache_.clear();
  return true;
}

const Font::Glyph& Font::glyph(int codepoint, int pixelSize) {
  uint64_t key = (static_cast<uint64_t>(pixelSize) << 32) | static_cast<uint32_t>(codepoint);
  auto it = cache_.find(key);
  if (it != cache_.end()) return it->second;
  Glyph g;
  auto* info = static_cast<stbtt_fontinfo*>(info_.get());
  if (info) {
    float scale = stbtt_ScaleForPixelHeight(info, static_cast<float>(pixelSize));
    int adv, lsb;
    stbtt_GetCodepointHMetrics(info, codepoint, &adv, &lsb);
    g.advance = adv * scale;
    int x0, y0, x1, y1;
    stbtt_GetCodepointBitmapBox(info, codepoint, scale, scale, &x0, &y0, &x1, &y1);
    g.w = x1 - x0;
    g.h = y1 - y0;
    g.xoff = x0;
    g.yoff = y0;
    if (g.w > 0 && g.h > 0) {
      g.bitmap.assign(static_cast<size_t>(g.w * g.h), 0);
      stbtt_MakeCodepointBitmap(info, g.bitmap.data(), g.w, g.h, g.w, scale, scale, codepoint);
    }
  }
  return cache_.emplace(key, std::move(g)).first->second;
}

float Font::ascent(int pixelSize) const {
  auto* info = static_cast<stbtt_fontinfo*>(info_.get());
  if (!info) return static_cast<float>(pixelSize);
  int asc, desc, gap;
  stbtt_GetFontVMetrics(info, &asc, &desc, &gap);
  return asc * stbtt_ScaleForPixelHeight(info, static_cast<float>(pixelSize));
}

float Font::kern(int a, int b, int pixelSize) const {
  auto* info = static_cast<stbtt_fontinfo*>(info_.get());
  if (!info) return 0.f;
  return stbtt_GetCodepointKernAdvance(info, a, b) *
         stbtt_ScaleForPixelHeight(info, static_cast<float>(pixelSize));
}

}  // namespace br
