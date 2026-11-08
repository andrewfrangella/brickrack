// BrickRack - GPL-3.0-or-later.

#include "core/model.h"

#include <algorithm>

namespace br {

namespace {

std::vector<Model*>& registry() {
  static std::vector<Model*> models;
  return models;
}

}  // namespace

const char* categoryName(Category c) {
  switch (c) {
    case Category::Oscillator: return "Oscillators";
    case Category::Resonator: return "Resonators";
    case Category::Effect: return "Effects";
    case Category::Filter: return "Filters & EQ";
    case Category::Modulator: return "Modulation";
    case Category::Envelope: return "Envelopes";
    case Category::Sequencer: return "Sequencing";
    case Category::Utility: return "Utilities";
    case Category::IO: return "Brick I/O";
  }
  return "?";
}

void registerModel(Model* model) {
  auto& r = registry();
  r.push_back(model);
  std::stable_sort(r.begin(), r.end(), [](const Model* a, const Model* b) {
    if (a->category != b->category) return a->category < b->category;
    return a->name < b->name;
  });
}

const std::vector<Model*>& allModels() { return registry(); }

const Model* findModel(const std::string& slug) {
  for (const Model* m : registry()) {
    if (m->slug == slug) return m;
  }
  return nullptr;
}

const PanelSpec& Model::panel() const {
  if (!panelBuilt_) {
    panel_.title = shortName;
    if (buildPanel) buildPanel(panel_);
    panelBuilt_ = true;
  }
  return panel_;
}

// ---------------------------------------------------------------------------
// PanelSpec builders

PanelSpec& PanelSpec::knob(int param, float x, float y, float diameter, const std::string& lbl,
                           float labelDy, uint32_t color) {
  Widget w;
  w.type = WidgetType::Knob;
  w.x = x;
  w.y = y;
  w.w = w.h = diameter;
  w.index = param;
  w.color = color;
  w.focusable = true;
  widgets.push_back(w);
  if (!lbl.empty()) {
    float dy = labelDy != 0.f ? labelDy : -(diameter * 0.5f + 2.6f);
    label(x, y + dy, lbl, diameter >= 10.f ? 3.4f : 2.9f);
  }
  return *this;
}

PanelSpec& PanelSpec::button(int param, float x, float y, float diameter, const std::string& lbl,
                             uint32_t color, int lightIndex, uint32_t lightColor) {
  Widget w;
  w.type = WidgetType::Button;
  w.x = x;
  w.y = y;
  w.w = w.h = diameter;
  w.index = param;
  w.index2 = lightIndex;
  w.color = color;
  w.color2 = lightColor;
  w.focusable = true;
  widgets.push_back(w);
  if (!lbl.empty()) label(x, y - diameter * 0.5f - 2.4f, lbl, 2.7f);
  return *this;
}

PanelSpec& PanelSpec::toggleSwitch(int param, float x, float y, const std::string& lbl) {
  Widget w;
  w.type = WidgetType::Switch;
  w.x = x;
  w.y = y;
  w.w = 4.0f;
  w.h = 8.0f;
  w.index = param;
  w.focusable = true;
  widgets.push_back(w);
  if (!lbl.empty()) label(x, y - 6.6f, lbl, 2.7f);
  return *this;
}

PanelSpec& PanelSpec::input(int port, float x, float y, const std::string& lbl, float labelDy) {
  Widget w;
  w.type = WidgetType::Input;
  w.x = x;
  w.y = y;
  w.w = w.h = 6.2f;
  w.index = port;
  w.focusable = true;
  widgets.push_back(w);
  if (!lbl.empty()) label(x, y + labelDy, lbl, 2.7f);
  return *this;
}

PanelSpec& PanelSpec::output(int port, float x, float y, const std::string& lbl, float labelDy) {
  Widget w;
  w.type = WidgetType::Output;
  w.x = x;
  w.y = y;
  w.w = w.h = 6.2f;
  w.index = port;
  w.focusable = true;
  widgets.push_back(w);
  if (!lbl.empty()) label(x, y + labelDy, lbl, 2.7f);
  return *this;
}

PanelSpec& PanelSpec::light(int l, float x, float y, uint32_t color, float diameter) {
  Widget w;
  w.type = WidgetType::Light;
  w.x = x;
  w.y = y;
  w.w = w.h = diameter;
  w.index = l;
  w.color = color;
  widgets.push_back(w);
  return *this;
}

PanelSpec& PanelSpec::rgbLight(int r, int g, int b, float x, float y, float diameter) {
  Widget w;
  w.type = WidgetType::Light;
  w.x = x;
  w.y = y;
  w.w = w.h = diameter;
  w.index = r;
  w.index2 = g;
  w.index3 = b;
  w.color = 0xFFFF0000;
  widgets.push_back(w);
  return *this;
}

PanelSpec& PanelSpec::bicolorLight(int a, int b, float x, float y, uint32_t colorA,
                                   uint32_t colorB, float diameter) {
  Widget w;
  w.type = WidgetType::Light;
  w.x = x;
  w.y = y;
  w.w = w.h = diameter;
  w.index = a;
  w.index2 = b;
  w.index3 = -2;  // marks bicolour mode
  w.color = colorA;
  w.color2 = colorB;
  widgets.push_back(w);
  return *this;
}

PanelSpec& PanelSpec::label(float x, float y, const std::string& text, float size, int align,
                            uint32_t color) {
  Widget w;
  w.type = WidgetType::Label;
  w.x = x;
  w.y = y;
  w.text = text;
  w.fontSize = size;
  w.align = align;
  w.color = color;
  widgets.push_back(w);
  return *this;
}

PanelSpec& PanelSpec::line(float x, float y, float x2, float y2, float width, uint32_t color) {
  Widget w;
  w.type = WidgetType::Line;
  w.x = x;
  w.y = y;
  w.x2 = x2;
  w.y2 = y2;
  w.w = width;
  w.color = color;
  widgets.push_back(w);
  return *this;
}

PanelSpec& PanelSpec::box(float x, float y, float w_, float h_, uint32_t color, float radius) {
  Widget w;
  w.type = WidgetType::Box;
  w.x = x;
  w.y = y;
  w.w = w_;
  w.h = h_;
  w.x2 = radius;
  w.color = color;
  widgets.push_back(w);
  return *this;
}

PanelSpec& PanelSpec::display(float x, float y, float w_, float h_, CustomDraw draw) {
  Widget w;
  w.type = WidgetType::Display;
  w.x = x;
  w.y = y;
  w.w = w_;
  w.h = h_;
  w.draw = std::move(draw);
  widgets.push_back(w);
  return *this;
}

}  // namespace br
