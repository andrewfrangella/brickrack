// BrickRack - GPL-3.0-or-later.
//
// Model = a module type (factory + panel description). Panels are described in
// millimetres on a Eurorack-sized faceplate (128.5 mm tall, 1 HP = 5.08 mm), so
// layouts can follow the original hardware closely; the UI scales them to the
// screen.

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace br {

class Module;
class Canvas;

constexpr float kPanelHeightMm = 128.5f;
constexpr float kHpMm = 5.08f;

enum class WidgetType {
  Knob,      // param; size = diameter mm
  Button,    // param (Button/Toggle); round push button
  Switch,    // param (Switch); vertical 2/3-position toggle
  Input,     // input port
  Output,    // output port
  Light,     // light index; index2/index3 for green/blue channels (RGB)
  Label,     // static text
  Line,      // decoration: x,y -> x2,y2
  Box,       // decoration: filled rounded rect x,y,w,h (top-left)
  Display,   // custom drawn area x,y,w,h (top-left); uses `draw`
};

enum Align { kAlignLeft = 0, kAlignCenter = 1, kAlignRight = 2 };

// Signature for custom drawing: canvas, pixel rect of the widget, px-per-mm, module.
using CustomDraw = std::function<void(Canvas&, float x, float y, float w, float h,
                                      float scale, const Module*)>;

struct Widget {
  WidgetType type = WidgetType::Label;
  float x = 0.f, y = 0.f;  // centre in mm (top-left for Box/Display)
  float w = 0.f, h = 0.f;  // size in mm
  float x2 = 0.f, y2 = 0.f;
  int index = -1;
  int index2 = -1;
  int index3 = -1;
  std::string text;
  uint32_t color = 0;   // 0 = use panel default
  uint32_t color2 = 0;
  float fontSize = 3.2f;  // mm
  int align = kAlignCenter;
  bool focusable = false;
  CustomDraw draw;
};

struct PanelSpec {
  int hp = 8;
  std::string title;  // may contain '\n' for a two-line title on narrow panels
  uint32_t bg = 0xFFD9DCDE;      // brushed-aluminium grey
  uint32_t ink = 0xFF2B2B2E;     // print colour
  uint32_t accent = 0xFF2B2B2E;  // knob caps / highlights
  uint32_t jackBg = 0;           // optional dark area colour for outputs
  std::vector<Widget> widgets;

  float widthMm() const { return static_cast<float>(hp) * kHpMm; }

  // Builder helpers. Positions are in millimetres from the top-left corner.
  PanelSpec& knob(int param, float x, float y, float diameter, const std::string& label = "",
                  float labelDy = 0.f, uint32_t color = 0);
  // `lightColor` tints the light inside the button (default warm white).
  PanelSpec& button(int param, float x, float y, float diameter = 6.f,
                    const std::string& label = "", uint32_t color = 0, int light = -1,
                    uint32_t lightColor = 0);
  PanelSpec& toggleSwitch(int param, float x, float y, const std::string& label = "");
  PanelSpec& input(int port, float x, float y, const std::string& label = "",
                   float labelDy = -6.0f);
  PanelSpec& output(int port, float x, float y, const std::string& label = "",
                    float labelDy = -6.0f);
  PanelSpec& light(int light, float x, float y, uint32_t color, float diameter = 2.5f);
  PanelSpec& rgbLight(int r, int g, int b, float x, float y, float diameter = 2.5f);
  PanelSpec& bicolorLight(int a, int b, float x, float y, uint32_t colorA, uint32_t colorB,
                          float diameter = 2.5f);
  PanelSpec& label(float x, float y, const std::string& text, float size = 3.2f,
                   int align = kAlignCenter, uint32_t color = 0);
  PanelSpec& line(float x, float y, float x2, float y2, float width = 0.3f, uint32_t color = 0);
  PanelSpec& box(float x, float y, float w, float h, uint32_t color, float radius = 1.0f);
  PanelSpec& display(float x, float y, float w, float h, CustomDraw draw);
};

enum class Category {
  Oscillator,
  Resonator,
  Effect,
  Filter,
  Modulator,
  Envelope,
  Sequencer,
  Utility,
  IO,
};

const char* categoryName(Category c);

struct Model {
  std::string slug;         // stable id used in patch files
  std::string name;         // display name, e.g. "Macro Oscillator 2"
  std::string shortName;    // panel title, e.g. "MACRO OSC 2"
  std::string basedOn;      // original firmware this is built from, "" for originals
  Category category = Category::Utility;
  std::string description;
  std::function<Module*()> create;
  std::function<void(PanelSpec&)> buildPanel;

  const PanelSpec& panel() const;

 private:
  mutable bool panelBuilt_ = false;
  mutable PanelSpec panel_;
};

void registerModel(Model* model);
const std::vector<Model*>& allModels();
const Model* findModel(const std::string& slug);

struct ModelRegistrar {
  explicit ModelRegistrar(Model* m) { registerModel(m); }
};

}  // namespace br

#define BR_CONCAT_(a, b) a##b
#define BR_CONCAT(a, b) BR_CONCAT_(a, b)
// Use at namespace scope in a module .cc: BR_REGISTER_MODEL(myModel);
#define BR_REGISTER_MODEL(model) \
  static ::br::ModelRegistrar BR_CONCAT(br_registrar_, __LINE__)(&(model))
