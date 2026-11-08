// BrickRack - GPL-3.0-or-later.

#include "ui/app.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <dirent.h>
#include <sys/stat.h>

#include "ui/font.h"

namespace br {

namespace {

constexpr float kTopBar = 34.f;
constexpr float kStatusBar = 62.f;
constexpr uint32_t kBgColor = 0xFF17181B;
constexpr uint32_t kRailColor = 0xFF9EA2A8;
constexpr uint32_t kBarColor = 0xFF0E0F11;
constexpr uint32_t kTextColor = 0xFFE9E9EC;
constexpr uint32_t kDimText = 0xFF8D9096;
constexpr uint32_t kFocusColor = 0xFFFFC83D;
constexpr uint32_t kPatchColor = 0xFF46D2FF;

const uint32_t kCableColors[] = {
    0xFFF2B630,  // yellow
    0xFFE5484D,  // red
    0xFF3E8EF7,  // blue
    0xFF46C46E,  // green
    0xFFB06CF0,  // purple
    0xFFF28A30,  // orange
    0xFFEDEDED,  // white
    0xFF2EC5C5,  // teal
};

constexpr float kZoomFactors[3] = {0.5f, 1.0f, 1.55f};

float deadzone(float v) {
  float a = std::fabs(v);
  if (a < 0.18f) return 0.f;
  float n = (a - 0.18f) / 0.82f;
  return std::copysign(n * n, v);
}

std::string upper(std::string s) {
  for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return s;
}

}  // namespace

const char* buttonName(Button b) {
  static const char* names[] = {"UP", "DOWN", "LEFT", "RIGHT", "A",     "B",      "X",    "Y",  "L1",
                                "R1", "L2",   "R2",   "START", "SELECT", "MENU", "L3", "R3"};
  return b < BTN_COUNT ? names[b] : "?";
}

App::App(Engine& engine, std::string dataDir) : engine_(engine), dataDir_(std::move(dataDir)) {
  ::mkdir(dataDir_.c_str(), 0755);
  ::mkdir((dataDir_ + "/patches").c_str(), 0755);
}

App::~App() = default;

// ---------------------------------------------------------------------------
// Geometry

float App::scale() const {
  float base = (kScreenH - kTopBar - kStatusBar - 8.f) / kPanelHeightMm;
  return base * kZoomFactors[zoom_];
}

float App::rackTop() const { return kTopBar + 4.f; }

float App::rackHeightPx() const { return kPanelHeightMm * scale(); }

float App::moduleX(int index) const {
  float x = 0.f;
  const auto& mods = engine_.modules();
  for (int i = 0; i < index && i < static_cast<int>(mods.size()); ++i) {
    x += mods[static_cast<size_t>(i)]->model->panel().widthMm() * scale();
  }
  return x;
}

// In overview zoom the rack wraps into rows, like a multi-row case. Otherwise
// it is a single horizontal row.
void App::modulePos(int index, float* x, float* y) const {
  if (zoom_ != 0) {
    *x = moduleX(index);
    *y = 0.f;
    return;
  }
  const float maxW = kScreenW - 24.f;
  const float rowH = rackHeightPx() + 10.f;
  float cx = 0.f, cy = 0.f;
  const auto& mods = engine_.modules();
  for (int i = 0; i < static_cast<int>(mods.size()); ++i) {
    float w = mods[static_cast<size_t>(i)]->model->panel().widthMm() * scale();
    if (cx > 0.f && cx + w > maxW) {
      cx = 0.f;
      cy += rowH;
    }
    if (i == index) break;
    cx += w;
  }
  *x = cx;
  *y = cy;
}

float App::overviewRowsHeight() const {
  const auto& mods = engine_.modules();
  if (mods.empty()) return rackHeightPx();
  float x, y;
  modulePos(static_cast<int>(mods.size()) - 1, &x, &y);
  return y + rackHeightPx();
}

float App::rackWidthPx() const { return moduleX(static_cast<int>(engine_.modules().size())); }

bool App::widgetCenter(int moduleIndex, int widgetIndex, float* x, float* y) const {
  const auto& mods = engine_.modules();
  if (moduleIndex < 0 || moduleIndex >= static_cast<int>(mods.size())) return false;
  const PanelSpec& p = mods[static_cast<size_t>(moduleIndex)]->model->panel();
  if (widgetIndex < 0 || widgetIndex >= static_cast<int>(p.widgets.size())) return false;
  const Widget& w = p.widgets[static_cast<size_t>(widgetIndex)];
  float s = scale();
  float cx = w.x, cy = w.y;
  if (w.type == WidgetType::Box || w.type == WidgetType::Display) {
    cx += w.w * 0.5f;
    cy += w.h * 0.5f;
  }
  float mx, my;
  modulePos(moduleIndex, &mx, &my);
  *x = mx - scrollX_ + cx * s;
  *y = rackTop() + my - scrollY_ + cy * s;
  return true;
}

int App::widgetForPort(const Model* m, bool output, int port) const {
  const PanelSpec& p = m->panel();
  WidgetType t = output ? WidgetType::Output : WidgetType::Input;
  for (size_t i = 0; i < p.widgets.size(); ++i) {
    if (p.widgets[i].type == t && p.widgets[i].index == port) return static_cast<int>(i);
  }
  return -1;
}

bool App::portCenter(int moduleId, bool output, int port, float* x, float* y) const {
  int idx = engine_.indexOf(moduleId);
  if (idx < 0) return false;
  const Module* m = engine_.modules()[static_cast<size_t>(idx)];
  int w = widgetForPort(m->model, output, port);
  if (w < 0) return false;
  return widgetCenter(idx, w, x, y);
}

// ---------------------------------------------------------------------------
// Focus

Module* App::focusedModule() const {
  const auto& mods = engine_.modules();
  if (focusIndex_ < 0 || focusIndex_ >= static_cast<int>(mods.size())) return nullptr;
  return mods[static_cast<size_t>(focusIndex_)];
}

const Widget* App::focusedWidget() const {
  Module* m = focusedModule();
  if (!m) return nullptr;
  const PanelSpec& p = m->model->panel();
  if (focusWidget_ < 0 || focusWidget_ >= static_cast<int>(p.widgets.size())) return nullptr;
  return &p.widgets[static_cast<size_t>(focusWidget_)];
}

void App::clampFocus() {
  const auto& mods = engine_.modules();
  if (mods.empty()) {
    focusIndex_ = 0;
    focusWidget_ = -1;
    return;
  }
  focusIndex_ = std::max(0, std::min(focusIndex_, static_cast<int>(mods.size()) - 1));
  const PanelSpec& p = mods[static_cast<size_t>(focusIndex_)]->model->panel();
  bool ok = focusWidget_ >= 0 && focusWidget_ < static_cast<int>(p.widgets.size()) &&
            p.widgets[static_cast<size_t>(focusWidget_)].focusable;
  if (!ok) {
    focusWidget_ = -1;
    // Top-most, left-most focusable widget.
    float best = 1e9f;
    for (size_t i = 0; i < p.widgets.size(); ++i) {
      const Widget& w = p.widgets[i];
      if (!w.focusable) continue;
      float score = w.y * 4.f + w.x;
      if (score < best) {
        best = score;
        focusWidget_ = static_cast<int>(i);
      }
    }
  }
}

void App::focusModule(int index, int widget) {
  focusIndex_ = index;
  focusWidget_ = widget;
  clampFocus();
  cameraSnap_ = true;
}

void App::setZoom(int z) {
  zoom_ = std::max(0, std::min(2, z));
  panelCache_.clear();
  cameraSnap_ = true;
}

void App::navigate(int dx, int dy) {
  const auto& mods = engine_.modules();
  if (mods.empty()) return;
  clampFocus();
  float s = scale();
  // Positions in mm. In overview (multi-row) zoom use the drawn layout so
  // up/down can move between rows; otherwise the single-row offset.
  const bool rows = zoom_ == 0;
  auto globalPos = [&](int mi, const Widget& w, float* gx, float* gy) {
    float mx, my;
    if (rows) {
      modulePos(mi, &mx, &my);
    } else {
      mx = moduleX(mi);
      my = 0.f;
    }
    *gx = mx / s + w.x;
    *gy = my / s + w.y;
  };
  float fx = 0.f, fy = 0.f;
  if (const Widget* fw = focusedWidget()) {
    globalPos(focusIndex_, *fw, &fx, &fy);
  } else {
    Widget dummy;
    globalPos(focusIndex_, dummy, &fx, &fy);
  }
  int bestM = -1, bestW = -1;
  float bestScore = 1e9f;
  int lo = rows ? 0 : std::max(0, focusIndex_ - 1);
  int hi = rows ? static_cast<int>(mods.size()) - 1
                : std::min(static_cast<int>(mods.size()) - 1, focusIndex_ + 1);
  for (int mi = lo; mi <= hi; ++mi) {
    const PanelSpec& p = mods[static_cast<size_t>(mi)]->model->panel();
    for (size_t wi = 0; wi < p.widgets.size(); ++wi) {
      const Widget& w = p.widgets[wi];
      if (!w.focusable) continue;
      if (mi == focusIndex_ && static_cast<int>(wi) == focusWidget_) continue;
      float gx, gy;
      globalPos(mi, w, &gx, &gy);
      float ddx = gx - fx, ddy = gy - fy;
      float along = dx != 0 ? ddx * dx : ddy * dy;
      float across = dx != 0 ? std::fabs(ddy) : std::fabs(ddx);
      if (along < 0.8f) continue;
      // Up/down stay within the module unless nothing else is there.
      float score = along + across * 2.2f;
      if (dy != 0 && mi != focusIndex_ && !rows) score += 1000.f;
      if (score < bestScore) {
        bestScore = score;
        bestM = mi;
        bestW = static_cast<int>(wi);
      }
    }
  }
  if (bestM >= 0) {
    focusIndex_ = bestM;
    focusWidget_ = bestW;
  }
}

void App::jumpModule(int dir) {
  const auto& mods = engine_.modules();
  if (mods.empty()) return;
  int target = std::max(0, std::min(focusIndex_ + dir, static_cast<int>(mods.size()) - 1));
  if (target == focusIndex_) return;
  // Keep a similar height when jumping.
  float fy = 40.f;
  if (const Widget* w = focusedWidget()) fy = w->y;
  focusIndex_ = target;
  const PanelSpec& p = mods[static_cast<size_t>(target)]->model->panel();
  float best = 1e9f;
  focusWidget_ = -1;
  for (size_t i = 0; i < p.widgets.size(); ++i) {
    const Widget& w = p.widgets[i];
    if (!w.focusable) continue;
    float score = std::fabs(w.y - fy) * 2.f + (dir > 0 ? w.x : p.widthMm() - w.x);
    if (score < best) {
      best = score;
      focusWidget_ = static_cast<int>(i);
    }
  }
  clampFocus();
}

void App::updateCamera(float dt, bool snap) {
  const auto& mods = engine_.modules();
  float viewW = kScreenW;
  float targetX = scrollX_;
  float total = rackWidthPx();
  if (!mods.empty()) {
    float mx = moduleX(focusIndex_);
    float mw = mods[static_cast<size_t>(focusIndex_)]->model->panel().widthMm() * scale();
    float wx = mx + mw * 0.5f;
    if (const Widget* w = focusedWidget()) wx = mx + w->x * scale();
    const float margin = 40.f;
    if (mw + 2 * margin <= viewW) {
      // Keep the whole module on screen.
      if (mx - margin < targetX) targetX = mx - margin;
      if (mx + mw + margin > targetX + viewW) targetX = mx + mw + margin - viewW;
    } else {
      if (wx - 120.f < targetX) targetX = wx - 120.f;
      if (wx + 120.f > targetX + viewW) targetX = wx + 120.f - viewW;
    }
    // While patching, keep the cable origin in view if possible.
  }
  if (zoom_ == 0) {
    targetX = -12.f;
  } else if (total <= viewW) {
    targetX = -(viewW - total) * 0.5f;
  } else {
    targetX = std::max(-40.f, std::min(targetX, total - viewW + 40.f));
  }
  float targetY = 0.f;
  float viewH = kScreenH - kTopBar - kStatusBar - 8.f;
  float contentH = zoom_ == 0 ? overviewRowsHeight() : rackHeightPx();
  if (contentH > viewH) {
    float mx = 0.f, my = 0.f;
    if (!mods.empty()) modulePos(focusIndex_, &mx, &my);
    float wy = my + rackHeightPx() * 0.5f;
    if (const Widget* w = focusedWidget()) wy = my + w->y * scale();
    targetY = std::max(0.f, std::min(wy - viewH * 0.5f, contentH - viewH));
  }
  if (snap) {
    scrollX_ = targetX;
    scrollY_ = targetY;
  } else {
    float k = std::min(1.f, dt * 14.f);
    scrollX_ += (targetX - scrollX_) * k;
    scrollY_ += (targetY - scrollY_) * k;
  }
}

// ---------------------------------------------------------------------------
// Input

void App::buttonEvent(Button b, bool down) {
  if (b >= BTN_COUNT) return;
  bool wasHeld = held_[b];
  held_[b] = down;

  // Controller gates are live regardless of UI state.
  if (b == BTN_L2) engine_.controller().gate[0].store(down);
  if (b == BTN_R2) engine_.controller().gate[1].store(down);
  if (b == BTN_L3) engine_.controller().gate[2].store(down);
  if (b == BTN_R3) engine_.controller().gate[3].store(down);

  if (!down) {
    if (b == BTN_A) pressA(false);
    return;
  }
  if (wasHeld && b != BTN_UP && b != BTN_DOWN && b != BTN_LEFT && b != BTN_RIGHT) return;
  repeatTimer_[b] = 0.f;

  if ((b == BTN_START && held_[BTN_SELECT]) || (b == BTN_SELECT && held_[BTN_START])) {
    openQuitConfirm();
    return;
  }
  if (b == BTN_MENU) {
    if (!menus_.empty()) menus_.clear();
    else openQuitConfirm();
    return;
  }

  if (!menus_.empty()) {
    menuButton(b);
    return;
  }

  if (editing_) {
    const Widget* w = focusedWidget();
    switch (b) {
      case BTN_UP: adjustFocused(+1.f, false); return;
      case BTN_DOWN: adjustFocused(-1.f, false); return;
      case BTN_RIGHT: adjustFocused(+1.f, true); return;
      case BTN_LEFT: adjustFocused(-1.f, true); return;
      case BTN_Y:
        if (Module* m = focusedModule()) {
          if (w && w->index >= 0) {
            Param& p = m->params[static_cast<size_t>(w->index)];
            p.set(p.defaultValue);
          }
        }
        return;
      case BTN_A:
      case BTN_B: editing_ = false; return;
      default: editing_ = false; break;  // fall through to normal handling
    }
  }

  switch (b) {
    case BTN_UP: navigate(0, -1); break;
    case BTN_DOWN: navigate(0, 1); break;
    case BTN_LEFT: navigate(-1, 0); break;
    case BTN_RIGHT: navigate(1, 0); break;
    case BTN_L1: jumpModule(-1); break;
    case BTN_R1: jumpModule(1); break;
    case BTN_A: pressA(true); break;
    case BTN_B: pressB(); break;
    case BTN_X: openBrowser(); break;
    case BTN_Y: openModuleMenu(); break;
    case BTN_START: openMainMenu(); break;
    case BTN_SELECT: setZoom((zoom_ + 1) % 3); break;
    default: break;
  }
}

void App::axisEvent(Axis a, float value) {
  if (a >= AXIS_COUNT) return;
  axis_[a] = std::max(-1.f, std::min(1.f, value));
  engine_.controller().axis[a].store(axis_[a]);
}

void App::pressA(bool down) {
  Module* m = focusedModule();
  const Widget* w = focusedWidget();
  if (!down) {
    if (aDownParam_ >= 0) {
      Module* hm = engine_.findModule(aDownModule_);
      if (hm && aDownParam_ < static_cast<int>(hm->params.size())) {
        hm->params[static_cast<size_t>(aDownParam_)].value = 0.f;
      }
      aDownParam_ = -1;
    }
    return;
  }
  if (!m || !w) return;
  switch (w->type) {
    case WidgetType::Knob:
      editing_ = true;
      break;
    case WidgetType::Button: {
      Param& p = m->params[static_cast<size_t>(w->index)];
      if (p.type == ParamType::Toggle) {
        p.value = p.value > 0.5f ? 0.f : 1.f;
      } else {
        p.value = 1.f;
        aDownParam_ = w->index;
        aDownModule_ = m->id;
      }
      break;
    }
    case WidgetType::Switch: {
      Param& p = m->params[static_cast<size_t>(w->index)];
      float v = p.value + 1.f;
      if (v > p.maxValue + 0.01f) v = p.minValue;
      p.set(v);
      break;
    }
    case WidgetType::Input:
    case WidgetType::Output:
      startOrFinishCable();
      break;
    default:
      break;
  }
}

void App::pressB() {
  if (pending_.active) {
    pending_.active = false;
    toast("Patching cancelled");
    return;
  }
  Module* m = focusedModule();
  const Widget* w = focusedWidget();
  if (m && w && (w->type == WidgetType::Input || w->type == WidgetType::Output)) {
    bool out = w->type == WidgetType::Output;
    bool any = out ? !engine_.cablesOnOutput(m->id, w->index).empty()
                   : engine_.cableOnInput(m->id, w->index) != nullptr;
    if (any) {
      engine_.removeCablesOnPort(m->id, out, w->index);
      toast("Cable removed");
    }
  }
}

void App::startOrFinishCable() {
  Module* m = focusedModule();
  const Widget* w = focusedWidget();
  if (!m || !w) return;
  bool out = w->type == WidgetType::Output;
  if (!pending_.active) {
    pending_.active = true;
    pending_.moduleId = m->id;
    pending_.output = out;
    pending_.port = w->index;
    pending_.color = kCableColors[engine_.nextCableColorIndex() %
                                  (sizeof(kCableColors) / sizeof(kCableColors[0]))];
    return;
  }
  if (pending_.moduleId == m->id && pending_.output == out && pending_.port == w->index) {
    pending_.active = false;
    return;
  }
  if (pending_.output == out) {
    toast(out ? "Pick an INPUT to finish the cable" : "Pick an OUTPUT to finish the cable");
    return;
  }
  int outM = out ? m->id : pending_.moduleId;
  int outP = out ? w->index : pending_.port;
  int inM = out ? pending_.moduleId : m->id;
  int inP = out ? pending_.port : w->index;
  engine_.addCable(outM, outP, inM, inP, pending_.color);
  pending_.active = false;
}

void App::adjustFocused(float delta, bool fine) {
  Module* m = focusedModule();
  const Widget* w = focusedWidget();
  if (!m || !w || w->index < 0 || w->type != WidgetType::Knob) return;
  Param& p = m->params[static_cast<size_t>(w->index)];
  if (p.snap) {
    p.set(p.value + (delta > 0 ? 1.f : -1.f));
    return;
  }
  float range = p.maxValue - p.minValue;
  p.set(p.value + delta * range * (fine ? 1.f / 400.f : 1.f / 40.f));
}

void App::update(float dt) {
  time_ += dt;
  if (toastTime_ > 0.f) toastTime_ -= dt;

  // D-pad auto-repeat.
  const Button rep[] = {BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT};
  for (Button b : rep) {
    if (!held_[b]) continue;
    float before = repeatTimer_[b];
    repeatTimer_[b] += dt;
    const float delay = 0.32f, rate = editing_ ? 0.035f : 0.075f;
    if (repeatTimer_[b] >= delay) {
      int n0 = before < delay ? -1 : static_cast<int>((before - delay) / rate);
      int n1 = static_cast<int>((repeatTimer_[b] - delay) / rate);
      for (int k = n0; k < n1; ++k) {
        // Re-enter as a fresh press, keeping the repeat timer running
        // (buttonEvent resets it for genuine presses).
        float keep = repeatTimer_[b];
        held_[b] = false;
        buttonEvent(b, true);
        held_[b] = true;
        repeatTimer_[b] = keep;
      }
    }
  }

  // Right stick adjusts the focused knob continuously.
  if (menus_.empty()) {
    float ry = deadzone(-axis_[AXIS_RY]);
    float rx = deadzone(axis_[AXIS_RX]);
    Module* m = focusedModule();
    const Widget* w = focusedWidget();
    if (m && w && w->type == WidgetType::Knob && w->index >= 0 && (ry != 0.f || rx != 0.f)) {
      Param& p = m->params[static_cast<size_t>(w->index)];
      float range = p.maxValue - p.minValue;
      if (p.snap) {
        if (stickAccParam_ != &p) {
          stickAccParam_ = &p;
          stickAcc_ = 0.f;
        }
        stickAcc_ += (ry + rx) * dt * 6.f;
        while (stickAcc_ >= 1.f) { p.set(p.value + 1.f); stickAcc_ -= 1.f; }
        while (stickAcc_ <= -1.f) { p.set(p.value - 1.f); stickAcc_ += 1.f; }
      } else {
        p.set(p.value + (ry * 0.8f + rx * 0.08f) * dt * range);
      }
    }
  }

  autosaveTimer_ += dt;
  if (autosaveTimer_ > 60.f) {
    autosaveTimer_ = 0.f;
    saveAutosave();
  }

  updateCamera(dt, cameraSnap_);
  cameraSnap_ = false;
}

// ---------------------------------------------------------------------------
// Menus

void App::toast(const std::string& msg) {
  toast_ = msg;
  toastTime_ = 2.2f;
}

void App::menuButton(Button b) {
  Menu& m = *menus_.back();
  auto selectable = [&](int i) {
    return i >= 0 && i < static_cast<int>(m.items.size()) && !m.items[static_cast<size_t>(i)].header;
  };
  switch (b) {
    case BTN_UP:
    case BTN_DOWN: {
      int dir = b == BTN_UP ? -1 : 1;
      int n = static_cast<int>(m.items.size());
      int i = m.selected;
      for (int k = 0; k < n; ++k) {
        i = (i + dir + n) % n;
        if (selectable(i)) break;
      }
      m.selected = i;
      break;
    }
    case BTN_L1:
    case BTN_R1: {
      int dir = b == BTN_L1 ? -1 : 1;
      int i = m.selected;
      for (int k = 0; k < 10; ++k) {
        int j = i + dir;
        if (j < 0 || j >= static_cast<int>(m.items.size())) break;
        i = j;
        if (k >= 7 && selectable(i)) break;
      }
      while (!selectable(i) && i > 0) --i;
      while (!selectable(i) && i < static_cast<int>(m.items.size()) - 1) ++i;
      m.selected = i;
      break;
    }
    case BTN_LEFT:
    case BTN_RIGHT:
      if (selectable(m.selected) && m.items[static_cast<size_t>(m.selected)].adjust) {
        auto adjust = m.items[static_cast<size_t>(m.selected)].adjust;  // copy: may close menu
        adjust(b == BTN_LEFT ? -1 : 1);
      }
      break;
    case BTN_A:
      if (selectable(m.selected)) {
        MenuItem item = m.items[static_cast<size_t>(m.selected)];  // copy: may close menu
        if (item.activate) item.activate();
        else if (item.adjust) item.adjust(0);
      }
      break;
    case BTN_B:
    case BTN_START:
    case BTN_Y:
    case BTN_X:
      menus_.pop_back();
      break;
    default:
      break;
  }
}

void App::openMainMenu() {
  auto menu = std::make_unique<Menu>();
  menu->title = "BRICKRACK";
  menu->items.push_back({"Resume", nullptr, nullptr, [this] { menus_.clear(); }});
  menu->items.push_back({"Add module...", nullptr, nullptr, [this] {
                           menus_.clear();
                           openBrowser();
                         }});
  menu->items.push_back(
      {"Master volume", [this] {
         char buf[16];
         std::snprintf(buf, sizeof(buf), "%d%%",
                       static_cast<int>(std::lround(engine_.masterVolume.load() * 100)));
         return std::string(buf);
       },
       [this](int dir) {
         float v = engine_.masterVolume.load() + dir * 0.05f;
         engine_.masterVolume.store(std::max(0.f, std::min(1.5f, v)));
       },
       nullptr});
  menu->items.push_back({"Save patch", [this] { return patchName_; }, nullptr, [this] {
                           if (savePatchAs(patchName_)) toast("Saved " + patchName_);
                           menus_.clear();
                         }});
  menu->items.push_back({"Save as new patch", nullptr, nullptr, [this] {
                           auto existing = listPatches();
                           char name[32];
                           for (int i = 1; i < 1000; ++i) {
                             std::snprintf(name, sizeof(name), "patch-%03d", i);
                             if (std::find(existing.begin(), existing.end(), name) ==
                                 existing.end())
                               break;
                           }
                           if (savePatchAs(name)) {
                             patchName_ = name;
                             toast(std::string("Saved ") + name);
                           }
                           menus_.clear();
                         }});
  menu->items.push_back({"Load patch...", nullptr, nullptr, [this] { openLoadMenu(); }});
  menu->items.push_back({"New patch (clear rack)", nullptr, nullptr, [this] {
                           resetInteraction();
                           engine_.clear();
                           const Model* out = findModel("audio-out");
                           if (out) addModule(out);
                           patchName_ = "untitled";
                           pending_.active = false;
                           menus_.clear();
                           focusModule(0);
                         }});
  menu->items.push_back({"Load demo patch", nullptr, nullptr, [this] {
                           buildDefaultPatch();
                           patchName_ = "demo";
                           menus_.clear();
                         }});
  menu->items.push_back({"Zoom", [this] {
                           const char* z[] = {"Overview", "Normal", "Close-up"};
                           return std::string(z[zoom_]);
                         },
                         [this](int dir) { setZoom((zoom_ + (dir < 0 ? 2 : 1)) % 3); }, nullptr});
  menu->items.push_back({"Controls / help", nullptr, nullptr, [this] { openHelp(); }});
  menu->items.push_back({"Quit", nullptr, nullptr, [this] { openQuitConfirm(); }});
  menus_.push_back(std::move(menu));
}

void App::openQuitConfirm() {
  auto menu = std::make_unique<Menu>();
  menu->title = "QUIT BRICKRACK?";
  menu->items.push_back({"Save and quit", nullptr, nullptr, [this] {
                           saveAutosave();
                           quit_ = true;
                         }});
  menu->items.push_back({"Cancel", nullptr, nullptr, [this] { menus_.pop_back(); }});
  menus_.push_back(std::move(menu));
}

void App::openHelp() {
  auto menu = std::make_unique<Menu>();
  menu->title = "CONTROLS";
  menu->wide = true;
  auto line = [&](const char* k, const char* v) {
    MenuItem it;
    it.label = k;
    std::string val = v;
    it.value = [val] { return val; };
    menu->items.push_back(it);
  };
  line("D-pad", "Move between knobs, buttons and jacks");
  line("A on knob", "Edit: D-pad up/down coarse, left/right fine");
  line("Right stick", "Turn the selected knob directly");
  line("A on jack", "Start a cable, then A on another jack");
  line("B on jack", "Remove the cables on that jack");
  line("A on button", "Press (hold for momentary buttons)");
  line("L1 / R1", "Previous / next module");
  line("X", "Add a module to the right");
  line("Y", "Module menu: hidden settings, move, delete");
  line("SELECT", "Zoom: overview / normal / close-up");
  line("START", "Main menu: save, load, volume");
  line("L2 / R2", "Gates on the BRICK CONTROL module");
  line("Left stick", "X/Y voltages on BRICK CONTROL");
  line("A on switch", "Step to the next position");
  line("L3 / R3", "More gates on BRICK CONTROL");
  line("L1 / R1 in menus", "Page up / down");
  line("START+SELECT / MENU", "Quit (asks to save)");
  menus_.push_back(std::move(menu));
}

std::vector<std::string> App::listPatches() const {
  std::vector<std::string> names;
  std::string dir = dataDir_ + "/patches";
  if (DIR* d = ::opendir(dir.c_str())) {
    while (dirent* e = ::readdir(d)) {
      std::string n = e->d_name;
      if (n.size() > 4 && n.substr(n.size() - 4) == ".brp") names.push_back(n.substr(0, n.size() - 4));
    }
    ::closedir(d);
  }
  std::sort(names.begin(), names.end());
  return names;
}

bool App::savePatchAs(const std::string& name) {
  std::string path = dataDir_ + "/patches/" + name + ".brp";
  bool ok = engine_.savePatch(path);
  if (!ok) toast("Could not save " + path);
  return ok;
}

void App::openLoadMenu() {
  auto menu = std::make_unique<Menu>();
  menu->title = "LOAD PATCH";
  auto names = listPatches();
  if (names.empty()) {
    MenuItem it;
    it.label = "(no saved patches)";
    it.header = true;
    menu->items.push_back(it);
  }
  for (const std::string& n : names) {
    menu->items.push_back({n, nullptr, nullptr, [this, n] {
                             std::string err;
                             resetInteraction();
                             if (engine_.loadPatch(dataDir_ + "/patches/" + n + ".brp", &err)) {
                               patchName_ = n;
                               toast("Loaded " + n);
                             } else {
                               toast("Load failed: " + err);
                             }
                             menus_.clear();
                             focusModule(0);
                           }});
  }
  menus_.push_back(std::move(menu));
}

void App::openModuleMenu() {
  Module* m = focusedModule();
  if (!m) {
    openBrowser();
    return;
  }
  auto menu = std::make_unique<Menu>();
  menu->title = upper(m->model->name);
  menu->wide = true;
  int moduleId = m->id;

  std::vector<MenuAction> actions;
  m->appendMenu(actions);
  if (!actions.empty()) {
    MenuItem h;
    h.label = "Settings";
    h.header = true;
    menu->items.push_back(h);
    for (const MenuAction& a : actions) {
      MenuItem it;
      it.label = a.label;
      it.value = a.value;
      auto act = a.activate;
      it.adjust = [act](int dir) {
        if (act) act(dir);
      };
      menu->items.push_back(it);
    }
  }

  MenuItem ph;
  ph.label = "Parameters";
  ph.header = true;
  menu->items.push_back(ph);
  for (size_t i = 0; i < m->params.size(); ++i) {
    Param* p = &m->params[i];
    if (p->type == ParamType::Button) continue;
    MenuItem it;
    it.label = p->name + (p->hidden ? "  *" : "");
    it.value = [p] { return p->displayString(); };
    it.adjust = [p](int dir) {
      if (dir == 0) {
        if (p->type == ParamType::Toggle) p->set(p->value > 0.5f ? 0.f : 1.f);
        else p->set(p->defaultValue);
        return;
      }
      if (p->snap) p->set(p->value + dir);
      else p->set(p->value + dir * (p->maxValue - p->minValue) / 50.f);
    };
    it.help = "LEFT/RIGHT change, A resets to default";
    menu->items.push_back(it);
  }

  MenuItem mh;
  mh.label = "Module";
  mh.header = true;
  menu->items.push_back(mh);
  menu->items.push_back({"Move left", nullptr, nullptr, [this, moduleId] {
                           int idx = engine_.indexOf(moduleId);
                           if (idx > 0) {
                             engine_.moveModule(moduleId, idx - 1);
                             focusIndex_ = idx - 1;
                           }
                         }});
  menu->items.push_back({"Move right", nullptr, nullptr, [this, moduleId] {
                           int idx = engine_.indexOf(moduleId);
                           if (idx >= 0 && idx + 1 < static_cast<int>(engine_.modules().size())) {
                             engine_.moveModule(moduleId, idx + 1);
                             focusIndex_ = idx + 1;
                           }
                         }});
  menu->items.push_back({"Duplicate", nullptr, nullptr, [this, moduleId] {
                           Module* src = engine_.findModule(moduleId);
                           if (!src) return;
                           Module* copy = src->model->create();
                           copy->model = src->model;
                           for (size_t i = 0; i < src->params.size() && i < copy->params.size(); ++i)
                             copy->params[i].value = src->params[i].value;
                           StateMap st;
                           src->saveState(st);
                           copy->loadState(st);
                           int idx = engine_.indexOf(moduleId);
                           engine_.addModule(copy, idx + 1);
                           menus_.clear();
                           focusModule(idx + 1);
                         }});
  menu->items.push_back({"Reset to defaults", nullptr, nullptr, [this, moduleId] {
                           engine_.resetModule(moduleId);
                           toast("Module reset");
                         }});
  menu->items.push_back({"Delete module", nullptr, nullptr, [this, moduleId] {
                           if (pending_.active && pending_.moduleId == moduleId)
                             pending_.active = false;
                           engine_.removeModule(moduleId);
                           menus_.clear();
                           clampFocus();
                           toast("Module deleted");
                         }});
  // Select the first selectable item.
  for (size_t i = 0; i < menu->items.size(); ++i) {
    if (!menu->items[i].header) {
      menu->selected = static_cast<int>(i);
      break;
    }
  }
  menus_.push_back(std::move(menu));
}

void App::openBrowser() {
  auto menu = std::make_unique<Menu>();
  menu->title = "ADD MODULE";
  menu->wide = true;
  int lastCat = -1;
  for (const Model* model : allModels()) {
    if (static_cast<int>(model->category) != lastCat) {
      lastCat = static_cast<int>(model->category);
      MenuItem h;
      h.label = categoryName(model->category);
      h.header = true;
      menu->items.push_back(h);
    }
    MenuItem it;
    it.label = model->name;
    std::string based = model->basedOn.empty() ? std::string() : "after " + model->basedOn;
    char hp[16];
    std::snprintf(hp, sizeof(hp), "%dHP", model->panel().hp);
    std::string val = based.empty() ? std::string(hp) : based + "  " + hp;
    it.value = [val] { return val; };
    it.activate = [this, model] {
      menus_.clear();
      addModule(model);
    };
    it.help = model->description;
    menu->items.push_back(it);
  }
  for (size_t i = 0; i < menu->items.size(); ++i) {
    if (!menu->items[i].header) {
      menu->selected = static_cast<int>(i);
      break;
    }
  }
  menus_.push_back(std::move(menu));
}

void App::addModule(const Model* model) {
  Module* m = model->create();
  m->model = model;
  int pos = engine_.modules().empty() ? 0 : focusIndex_ + 1;
  engine_.addModule(m, pos);
  focusIndex_ = pos;
  focusWidget_ = -1;
  clampFocus();
  toast("Added " + model->name);
}

// ---------------------------------------------------------------------------
// Patches

void App::saveAutosave() {
  engine_.savePatch(dataDir_ + "/patches/autosave.brp");
}

void App::loadStartupPatch() {
  std::string err;
  const std::string path = dataDir_ + "/patches/autosave.brp";
  resetInteraction();
  if (engine_.loadPatch(path, &err) && !engine_.modules().empty()) {
    patchName_ = "autosave";
  } else {
    // Keep an unreadable/empty autosave aside instead of overwriting it later.
    struct stat st;
    if (::stat(path.c_str(), &st) == 0) {
      std::rename(path.c_str(), (dataDir_ + "/patches/autosave-unreadable.brp").c_str());
    }
    buildDefaultPatch();
    patchName_ = "demo";
  }
  focusModule(0);
}

void App::resetInteraction() {
  pending_.active = false;
  editing_ = false;
  aDownParam_ = -1;
  stickAccParam_ = nullptr;
}

void App::buildDefaultPatch() {
  resetInteraction();
  engine_.clear();
  auto add = [&](const char* slug) -> int {
    const Model* model = findModel(slug);
    if (!model) return -1;
    Module* m = model->create();
    m->model = model;
    return engine_.addModule(m);
  };
  int ctl = add("brick-control");
  int seq = add("step-sequencer");
  int osc = add("macro-osc-2");
  int res = add("resonator");
  int verb = add("texture-synth");
  int out = add("audio-out");
  auto cable = [&](int om, int op, int im, int ip, int color) {
    if (om > 0 && im > 0) engine_.addCable(om, op, im, ip, kCableColors[color]);
  };
  // Port indices are defined by each module; resolve by name to stay robust.
  auto port = [&](int moduleId, bool output, const char* name) -> int {
    Module* m = engine_.findModule(moduleId);
    if (!m) return -1;
    auto& ports = output ? m->outputs : m->inputs;
    for (size_t i = 0; i < ports.size(); ++i)
      if (ports[i].name == name) return static_cast<int>(i);
    return -1;
  };
  auto link = [&](int om, const char* on, int im, const char* in, int color) {
    int op = port(om, true, on), ip = port(im, false, in);
    if (op >= 0 && ip >= 0) cable(om, op, im, ip, color);
  };
  auto setParam = [&](int moduleId, const char* name, float v) {
    Module* m = engine_.findModule(moduleId);
    if (!m) return;
    for (Param& p : m->params)
      if (p.name == name) p.set(v);
  };
  // Clouds ships 100% wet with density at noon (= no grains): give the demo
  // audible, musical settings instead.
  setParam(verb, "Density", 0.72f);
  setParam(verb, "Dry/wet", 0.5f);
  setParam(verb, "Blend (selected parameter)", 0.5f);
  setParam(verb, "Reverb", 0.4f);
  setParam(verb, "Size", 0.6f);
  setParam(osc, "Morph", 0.35f);
  setParam(seq, "Tempo", 96.f);
  link(seq, "Gate", osc, "Trigger", 0);
  link(seq, "Pitch", osc, "V/Oct", 2);
  link(ctl, "Left stick X", osc, "Timbre", 4);
  link(osc, "Out", res, "Audio in", 1);
  link(seq, "Gate", res, "Strum", 0);
  link(seq, "Pitch", res, "V/Oct", 2);
  link(res, "Odd", verb, "In L", 3);
  link(res, "Even", verb, "In R", 3);
  link(verb, "Out L", out, "Left", 6);
  link(verb, "Out R", out, "Right", 6);
  if (verb < 0) {
    link(res, "Odd", out, "Left", 6);
    link(res, "Even", out, "Right", 6);
  }
  if (res < 0) link(osc, "Out", out, "Left", 6);
  patchName_ = "demo";
  focusModule(0);
}

// ---------------------------------------------------------------------------
// Drawing

void App::render(Canvas& c) {
  c.clear(kBgColor);
  c.pushClip({0, static_cast<int>(kTopBar), kScreenW,
              static_cast<int>(kScreenH - kTopBar - kStatusBar)});
  drawRails(c);
  const auto& mods = engine_.modules();
  for (size_t i = 0; i < mods.size(); ++i) drawModule(c, static_cast<int>(i));
  drawCables(c);
  drawFocus(c);
  c.popClip();
  drawTopBar(c);
  drawStatusBar(c);
  for (auto& m : menus_) drawMenu(c, *m);
  drawToast(c);
}

void App::drawRails(Canvas& c) {
  float h = rackHeightPx();
  float s = scale();
  float content = zoom_ == 0 ? overviewRowsHeight() : h;
  for (float row = 0.f; row < content - 1.f; row += h + 10.f) {
    float top = rackTop() + row - scrollY_;
    c.fillRect(0, top - 1.f, kScreenW, 2.25f * s + 1.f, kRailColor);
    c.fillRect(0, top + h - 4.5f * s, kScreenW, 2.25f * s + 1.f, kRailColor);
    if (zoom_ != 0) break;
  }
  if (engine_.modules().empty()) {
    c.text(kScreenW * 0.5f, kScreenH * 0.5f, "Empty rack - press X to add a module", 26,
           kDimText, kAlignCenter);
  }
}

const Canvas& App::panelImage(const Model* m) {
  int key = zoom_;
  auto k = std::make_pair(m, key);
  auto it = panelCache_.find(k);
  if (it != panelCache_.end()) return *it->second;
  float s = scale();
  const PanelSpec& p = m->panel();
  auto img = std::make_unique<Canvas>(static_cast<int>(std::ceil(p.widthMm() * s)),
                                      static_cast<int>(std::ceil(kPanelHeightMm * s)));
  renderPanelStatic(*img, p, s);
  const Canvas& ref = *img;
  panelCache_[k] = std::move(img);
  return ref;
}

void App::renderPanelStatic(Canvas& img, const PanelSpec& p, float s) {
  img.clear(0x00000000);
  float W = p.widthMm() * s, H = kPanelHeightMm * s;
  img.fillRect(0, 0, W, H, p.bg);
  // Subtle bevel.
  img.fillRect(0, 0, W, 1.f, scaleColor(p.bg, 1.08f));
  img.fillRect(0, 0, 1.f, H, scaleColor(p.bg, 1.06f));
  img.fillRect(W - 1.f, 0, 1.f, H, scaleColor(p.bg, 0.72f));
  // Screws.
  auto screw = [&](float x, float y) {
    img.fillCircleGradient(x * s, y * s, 1.6f * s, 0xFFE4E6E8, 0xFF8C9096, -0.3f, -0.3f);
    img.line((x - 1.0f) * s, y * s, (x + 1.0f) * s, y * s, 0.35f * s, 0xFF55595F);
  };
  float sx = p.hp >= 10 ? 7.5f : p.widthMm() * 0.5f;
  screw(sx, 3.0f);
  screw(p.widthMm() - sx, kPanelHeightMm - 3.0f);
  if (p.hp >= 10) {
    screw(p.widthMm() - sx, 3.0f);
    screw(sx, kPanelHeightMm - 3.0f);
  }

  for (const Widget& w : p.widgets) {
    uint32_t ink = w.color ? w.color : p.ink;
    switch (w.type) {
      case WidgetType::Box:
        img.fillRoundRect(w.x * s, w.y * s, w.w * s, w.h * s, w.x2 * s, w.color);
        break;
      case WidgetType::Line:
        img.line(w.x * s, w.y * s, w.x2 * s, w.y2 * s, std::max(1.f, w.w * s), ink);
        break;
      default:
        break;
    }
  }
  for (const Widget& w : p.widgets) {
    uint32_t ink = w.color ? w.color : p.ink;
    switch (w.type) {
      case WidgetType::Label: {
        float size = std::max(13.f, w.fontSize * s * (zoom_ == 0 ? 1.f : 1.0f));
        if (zoom_ == 0) size = std::max(9.f, w.fontSize * s);
        img.text(w.x * s, w.y * s, w.text, size, ink, w.align);
        break;
      }
      case WidgetType::Knob: {
        // Knob bodies are static; only the indicator is drawn per frame.
        float cx = w.x * s, cy = w.y * s, r = w.w * 0.5f * s;
        bool small = w.w < 9.f;
        uint32_t body = w.color ? w.color : (small ? 0xFFB9BDC3 : 0xFF2E3034);
        uint32_t hi = scaleColor(body, small ? 1.18f : 2.1f);
        uint32_t lo = scaleColor(body, small ? 0.62f : 0.55f);
        img.fillCircle(cx + 0.06f * r, cy + 0.12f * r, r * 1.04f, 0x50000000);
        img.fillCircleGradient(cx, cy, r, hi, lo, -0.35f, -0.45f);
        img.strokeCircle(cx, cy, r * 0.97f, std::max(1.f, r * 0.05f), scaleColor(body, 0.5f));
        if (!small) {
          img.fillCircleGradient(cx, cy, r * 0.72f, scaleColor(body, 1.5f),
                                 scaleColor(body, 0.8f), -0.3f, -0.4f);
        }
        break;
      }
      case WidgetType::Switch: {
        float cx = w.x * s, cy = w.y * s;
        float hw = w.w * 0.5f * s, hh = w.h * 0.5f * s;
        img.fillRoundRect(cx - hw, cy - hh, 2 * hw, 2 * hh, hw * 0.5f, 0xFF2A2C30);
        break;
      }
      case WidgetType::Input:
      case WidgetType::Output: {
        float cx = w.x * s, cy = w.y * s, r = w.w * 0.5f * s;
        img.fillCircle(cx + 0.4f, cy + 0.8f, r * 1.02f, 0x40000000);
        img.fillCircleGradient(cx, cy, r, 0xFFF1F2F4, 0xFF8E939A, -0.4f, -0.4f);
        img.strokeCircle(cx, cy, r * 0.98f, 1.f, 0xFF6A6E74);
        img.fillCircle(cx, cy, r * 0.56f, 0xFF2A2B2E);
        img.fillCircle(cx, cy, r * 0.40f, 0xFF050505);
        break;
      }
      default:
        break;
    }
  }
  // Title.
  if (!p.title.empty()) {
    float size = std::max(zoom_ == 0 ? 9.f : 15.f, 3.5f * s);
    size_t nl = p.title.find('\n');
    if (nl == std::string::npos) {
      img.text(W * 0.5f, 7.6f * s, p.title, size, p.ink, kAlignCenter);
    } else {
      float small = size * 0.82f;
      img.text(W * 0.5f, 6.0f * s, p.title.substr(0, nl), small, p.ink, kAlignCenter);
      img.text(W * 0.5f, 6.0f * s + small * 1.05f, p.title.substr(nl + 1), small, p.ink,
               kAlignCenter);
    }
  }
}

void App::drawWidgetDynamic(Canvas& c, const Widget& w, const Module* m, float ox, float oy,
                            float s) {
  float cx = ox + w.x * s, cy = oy + w.y * s;
  switch (w.type) {
    case WidgetType::Knob: {
      const Param& p = m->params[static_cast<size_t>(w.index)];
      float r = w.w * 0.5f * s;
      bool small = w.w < 9.f;
      float v = p.normalized();
      if (p.snap && p.maxValue > p.minValue) {
        v = (std::round(p.value) - p.minValue) / (p.maxValue - p.minValue);
      }
      float a = (0.75f + 1.5f * v) * 3.14159265f;
      uint32_t ind = small ? 0xFF1E1F22 : 0xFFF4F4F4;
      float x0 = cx + std::cos(a) * r * 0.18f, y0 = cy + std::sin(a) * r * 0.18f;
      float x1 = cx + std::cos(a) * r * 0.86f, y1 = cy + std::sin(a) * r * 0.86f;
      c.line(x0, y0, x1, y1, std::max(1.5f, r * 0.13f), ind);
      break;
    }
    case WidgetType::Button: {
      const Param& p = m->params[static_cast<size_t>(w.index)];
      float r = w.w * 0.5f * s;
      bool down = p.value > 0.5f;
      uint32_t col = w.color ? w.color : 0xFFCDD0D4;
      c.fillCircle(cx, cy + 0.6f, r * 1.12f, 0x50000000);
      c.fillCircleGradient(cx, cy, r, scaleColor(col, down ? 0.85f : 1.15f),
                           scaleColor(col, down ? 0.55f : 0.7f), -0.3f, -0.4f);
      if (w.index2 >= 0 && w.index2 < static_cast<int>(m->lights.size())) {
        float b = m->lights[static_cast<size_t>(w.index2)];
        if (b > 0.01f) {
          c.fillCircle(cx, cy, r * 0.7f, withAlpha(w.color2 ? w.color2 : 0xFFFFE9A8, std::min(1.f, b)));
        }
      }
      break;
    }
    case WidgetType::Switch: {
      const Param& p = m->params[static_cast<size_t>(w.index)];
      float hw = w.w * 0.5f * s, hh = w.h * 0.5f * s;
      int n = static_cast<int>(std::lround(p.maxValue - p.minValue)) + 1;
      int idx = static_cast<int>(std::lround(p.value - p.minValue));
      float t = n > 1 ? static_cast<float>(idx) / (n - 1) : 0.f;
      float ly = cy + hh * 0.55f - t * hh * 1.1f;
      c.fillRoundRect(cx - hw * 0.8f, ly - hh * 0.38f, hw * 1.6f, hh * 0.76f, hw * 0.4f,
                      0xFFE3E5E8);
      break;
    }
    case WidgetType::Light: {
      float r = w.w * 0.5f * s;
      uint32_t col;
      float bright;
      auto L = [&](int i) {
        return (i >= 0 && i < static_cast<int>(m->lights.size()))
                   ? std::max(0.f, std::min(1.f, m->lights[static_cast<size_t>(i)]))
                   : 0.f;
      };
      if (w.index3 == -2) {  // bicolour
        // Additive, like two LED dies in one package: red + green = yellow.
        float a = L(w.index), b = L(w.index2);
        bright = std::max(a, b);
        if (bright > 0.f) {
          auto ch = [&](int sh) {
            float v = (((w.color >> sh) & 0xFF) * a + ((w.color2 >> sh) & 0xFF) * b) / bright;
            return std::min(255, static_cast<int>(v));
          };
          col = argb(255, ch(16), ch(8), ch(0));
        } else {
          col = w.color;
        }
      } else if (w.index2 >= 0) {  // RGB
        float r_ = L(w.index), g_ = L(w.index2), b_ = L(w.index3);
        bright = std::max(r_, std::max(g_, b_));
        if (bright > 0.f) {
          col = argb(255, static_cast<int>(255 * r_ / bright), static_cast<int>(255 * g_ / bright),
                     static_cast<int>(255 * b_ / bright));
        } else {
          col = 0xFF808080;
        }
      } else {
        bright = L(w.index);
        col = w.color ? w.color : 0xFFFF3030;
      }
      c.fillCircle(cx, cy, r * 1.1f, 0xFF1A1A1C);
      c.fillCircle(cx, cy, r * 0.9f, mixColor(0xFF3A3A3D, col, 0.18f + 0.82f * bright));
      if (bright > 0.05f) {
        c.fillCircle(cx, cy, r * 2.0f, withAlpha(col, 0.22f * bright));
        c.fillCircle(cx - r * 0.25f, cy - r * 0.25f, r * 0.35f,
                     withAlpha(0xFFFFFFFF, 0.5f * bright));
      }
      break;
    }
    case WidgetType::Display: {
      if (w.draw) w.draw(c, ox + w.x * s, oy + w.y * s, w.w * s, w.h * s, s, m);
      break;
    }
    default:
      break;
  }
}

void App::drawModule(Canvas& c, int index) {
  const Module* m = engine_.modules()[static_cast<size_t>(index)];
  const PanelSpec& p = m->model->panel();
  float s = scale();
  float mx, my;
  modulePos(index, &mx, &my);
  float ox = mx - scrollX_;
  float oy = rackTop() + my - scrollY_;
  float w = p.widthMm() * s;
  if (ox > kScreenW || ox + w < 0) return;
  if (oy > kScreenH || oy + rackHeightPx() < 0) return;
  const Canvas& img = panelImage(m->model);
  c.blit(img, static_cast<int>(std::round(ox)), static_cast<int>(std::round(oy)));
  float ix = std::round(ox), iy = std::round(oy);
  for (const Widget& wd : p.widgets) drawWidgetDynamic(c, wd, m, ix, iy, s);

  // While patching, highlight compatible jacks.
  if (pending_.active) {
    for (const Widget& wd : p.widgets) {
      bool isJack = wd.type == WidgetType::Input || wd.type == WidgetType::Output;
      if (!isJack) continue;
      bool out = wd.type == WidgetType::Output;
      if (out == pending_.output) continue;
      float pulse = 0.5f + 0.5f * std::sin(time_ * 6.f);
      c.strokeCircle(ix + wd.x * s, iy + wd.y * s, wd.w * 0.5f * s + 3.f, 2.f,
                     withAlpha(kPatchColor, 0.35f + 0.4f * pulse));
    }
  }
}

void App::drawCable(Canvas& c, float x0, float y0, float x1, float y1, uint32_t color,
                    float alpha) {
  float dx = x1 - x0, dy = y1 - y0;
  float dist = std::sqrt(dx * dx + dy * dy);
  float sag = std::min(160.f, dist * 0.28f + 12.f);
  float mx = (x0 + x1) * 0.5f, my = (y0 + y1) * 0.5f + sag;
  const int n = 24;
  float pts[2 * (n + 1)];
  for (int k = 0; k <= n; ++k) {
    float t = static_cast<float>(k) / n;
    float a = (1 - t) * (1 - t), b = 2 * (1 - t) * t, cc = t * t;
    pts[2 * k] = a * x0 + b * mx + cc * x1;
    pts[2 * k + 1] = a * y0 + b * my + cc * y1;
  }
  float s = scale();
  float width = std::max(3.f, 1.15f * s);
  c.polyline(pts, n + 1, width + 2.f, withAlpha(scaleColor(color, 0.45f), alpha));
  c.polyline(pts, n + 1, width, withAlpha(color, alpha));
  float pr = std::max(4.f, 2.3f * s);
  for (int e = 0; e < 2; ++e) {
    float px = e == 0 ? x0 : x1, py = e == 0 ? y0 : y1;
    c.fillCircle(px, py, pr + 1.f, withAlpha(scaleColor(color, 0.4f), 0.95f));
    c.fillCircle(px, py, pr, withAlpha(color, 0.97f));
    c.fillCircle(px, py, pr * 0.45f, withAlpha(scaleColor(color, 0.55f), 0.97f));
  }
}

void App::drawCables(Canvas& c) {
  for (const Cable& cb : engine_.cables()) {
    float x0, y0, x1, y1;
    if (!portCenter(cb.outModule, true, cb.outPort, &x0, &y0)) continue;
    if (!portCenter(cb.inModule, false, cb.inPort, &x1, &y1)) continue;
    if (std::max(x0, x1) < -200 || std::min(x0, x1) > kScreenW + 200) continue;
    drawCable(c, x0, y0, x1, y1, cb.color, 0.86f);
  }
  if (pending_.active) {
    float x0, y0;
    if (portCenter(pending_.moduleId, pending_.output, pending_.port, &x0, &y0)) {
      float x1 = x0, y1 = y0 + 40.f;
      if (focusedWidget()) widgetCenter(focusIndex_, focusWidget_, &x1, &y1);
      drawCable(c, x0, y0, x1, y1, pending_.color, 0.6f);
    }
  }
}

void App::drawFocus(Canvas& c) {
  const Widget* w = focusedWidget();
  if (!w || !menus_.empty()) return;
  float x, y;
  if (!widgetCenter(focusIndex_, focusWidget_, &x, &y)) return;
  float s = scale();
  float r = std::max(w->w, w->h) * 0.5f * s + 4.f;
  float pulse = 0.6f + 0.4f * std::sin(time_ * 5.f);
  uint32_t col = editing_ ? 0xFF5CFF8A : kFocusColor;
  c.strokeCircle(x, y, r + 1.f, 4.f, withAlpha(0xFF000000, 0.5f));
  c.strokeCircle(x, y, r, 2.6f, withAlpha(col, 0.7f + 0.3f * pulse));
  if (editing_) {
    // Arc showing the value.
    Module* m = focusedModule();
    if (m && w->index >= 0) {
      float v = m->params[static_cast<size_t>(w->index)].normalized();
      float a0 = 0.75f * 3.14159265f, a1 = a0 + 1.5f * 3.14159265f * v;
      c.arc(x, y, r + 5.f, a0, a0 + 1.5f * 3.14159265f, 3.f, withAlpha(0xFF000000, 0.5f));
      if (v > 0.001f) c.arc(x, y, r + 5.f, a0, a1, 3.f, col);
    }
  }
}

void App::drawTopBar(Canvas& c) {
  c.fillRect(0, 0, kScreenW, kTopBar, kBarColor);
  c.text(14, kTopBar * 0.5f, "BRICKRACK", 22, kFocusColor, kAlignLeft);
  float x = 14 + c.textWidth("BRICKRACK", 22) + 16;
  c.text(x, kTopBar * 0.5f, patchName_, 19, kDimText, kAlignLeft);

  char buf[64];
  float load = engine_.cpuLoad();
  std::snprintf(buf, sizeof(buf), "CPU %d%%", static_cast<int>(load * 100.f + 0.5f));
  uint32_t loadCol = load > 0.85f ? 0xFFFF5050 : (load > 0.6f ? 0xFFFFC040 : kDimText);
  float rx = kScreenW - 14.f;
  // Peak meters.
  for (int ch = 1; ch >= 0; --ch) {
    float pk = engine_.peakLevel(ch);
    float mw = 70.f;
    float y = kTopBar * 0.5f - 7.f + ch * 8.f;
    c.fillRect(rx - mw, y, mw, 5.f, 0xFF2A2C30);
    float lvl = std::min(1.f, pk);
    uint32_t col = lvl > 0.95f ? 0xFFFF4040 : (lvl > 0.7f ? 0xFFFFC040 : 0xFF46C46E);
    c.fillRect(rx - mw, y, mw * lvl, 5.f, col);
  }
  rx -= 84.f;
  std::snprintf(buf, sizeof(buf), "VOL %d%%",
                static_cast<int>(std::lround(engine_.masterVolume.load() * 100)));
  c.text(rx, kTopBar * 0.5f, buf, 19, kDimText, kAlignRight);
  rx -= c.textWidth(buf, 19) + 18.f;
  std::snprintf(buf, sizeof(buf), "CPU %d%%", static_cast<int>(load * 100.f + 0.5f));
  c.text(rx, kTopBar * 0.5f, buf, 19, loadCol, kAlignRight);
  rx -= c.textWidth(buf, 19) + 18.f;
  std::snprintf(buf, sizeof(buf), "%d modules  %d cables",
                static_cast<int>(engine_.modules().size()),
                static_cast<int>(engine_.cables().size()));
  c.text(rx, kTopBar * 0.5f, buf, 19, kDimText, kAlignRight);
}

void App::drawStatusBar(Canvas& c) {
  float y0 = kScreenH - kStatusBar;
  c.fillRect(0, y0, kScreenW, kStatusBar, kBarColor);
  c.fillRect(0, y0, kScreenW, 1, 0xFF2A2C30);
  Module* m = focusedModule();
  const Widget* w = focusedWidget();
  std::string left, value, hint;
  if (!m) {
    left = "Empty rack";
    hint = "X add module   START menu";
  } else {
    left = m->model->shortName;
    if (w) {
      switch (w->type) {
        case WidgetType::Knob:
        case WidgetType::Button:
        case WidgetType::Switch: {
          const Param& p = m->params[static_cast<size_t>(w->index)];
          left += "  ›  " + p.name;
          value = p.displayString();
          if (w->type == WidgetType::Knob) {
            hint = editing_ ? "D-pad adjust   Y default   A/B done" : "A edit   R-stick turn";
          } else if (w->type == WidgetType::Switch) {
            hint = "A switch";
          } else {
            hint = p.type == ParamType::Toggle ? "A toggle" : "A press";
          }
          break;
        }
        case WidgetType::Input:
        case WidgetType::Output: {
          bool out = w->type == WidgetType::Output;
          const Port& port = out ? m->outputs[static_cast<size_t>(w->index)]
                                 : m->inputs[static_cast<size_t>(w->index)];
          left += std::string("  ›  ") + (out ? "OUT " : "IN ") + port.name;
          char buf[32];
          std::snprintf(buf, sizeof(buf), "%+.2f V", port.voltage);
          value = buf;
          if (pending_.active) {
            bool origin = pending_.moduleId == m->id && pending_.port == w->index &&
                          pending_.output == out;
            hint = (pending_.output != out) ? "A connect   B cancel"
                                            : (origin ? "A cancel   B cancel" : "B cancel");
          } else {
            hint = port.connected ? "A patch   B unplug" : "A patch";
          }
          break;
        }
        default:
          break;
      }
    }
  }
  if (pending_.active) {
    Module* pm = engine_.findModule(pending_.moduleId);
    size_t nPorts = pm ? (pending_.output ? pm->outputs.size() : pm->inputs.size()) : 0;
    if (pm && pending_.port >= 0 && static_cast<size_t>(pending_.port) < nPorts) {
      const Port& pp = pending_.output ? pm->outputs[static_cast<size_t>(pending_.port)]
                                       : pm->inputs[static_cast<size_t>(pending_.port)];
      std::string from = "Cable from " + pm->model->shortName + " " + pp.name +
                         (pending_.output ? "  ›  pick an input" : "  ›  pick an output");
      c.fillRect(0, y0 - 30, kScreenW, 30, withAlpha(0xFF0B3C4F, 0.92f));
      c.text(14, y0 - 15, from, 19, kPatchColor, kAlignLeft);
    }
  }
  c.text(14, y0 + 21, left, 25, kTextColor, kAlignLeft);
  if (!value.empty()) c.text(kScreenW - 14.f, y0 + 21, value, 27, kFocusColor, kAlignRight);
  std::string common = "L1/R1 module   X add   Y module menu   SELECT zoom   START menu";
  c.text(14, y0 + 47, hint, 17, kDimText, kAlignLeft);
  c.text(kScreenW - 14.f, y0 + 47, common, 17, 0xFF5D6066, kAlignRight);
}

void App::drawMenu(Canvas& c, Menu& m) {
  c.fillRect(0, 0, kScreenW, kScreenH, 0x99000000);
  float w = m.wide ? 820.f : 520.f;
  const float rowH = 40.f;
  const int maxRows = 13;
  int rows = std::min(maxRows, static_cast<int>(m.items.size()));
  bool hasHelp = false;
  for (auto& it : m.items) if (!it.help.empty()) hasHelp = true;
  float h = 66.f + rows * rowH + (hasHelp ? 46.f : 14.f);
  float x = (kScreenW - w) * 0.5f, y = (kScreenH - h) * 0.5f;
  c.fillRoundRect(x + 4, y + 6, w, h, 12, 0x80000000);
  c.fillRoundRect(x, y, w, h, 12, 0xFF1E2024);
  c.strokeRoundRect(x, y, w, h, 12, 1.5f, 0xFF3A3D43);
  c.text(x + 22, y + 32, m.title, 26, kFocusColor, kAlignLeft);

  if (m.selected < m.scroll) m.scroll = m.selected;
  if (m.selected >= m.scroll + maxRows) m.scroll = m.selected - maxRows + 1;
  m.scroll = std::max(0, std::min(m.scroll, static_cast<int>(m.items.size()) - rows));

  float ry = y + 60.f;
  for (int i = m.scroll; i < m.scroll + rows; ++i) {
    const MenuItem& it = m.items[static_cast<size_t>(i)];
    float cy = ry + rowH * 0.5f;
    if (it.header) {
      c.text(x + 22, cy + 4, upper(it.label), 17, 0xFF7A7E86, kAlignLeft);
      c.fillRect(x + 22 + c.textWidth(upper(it.label), 17) + 10, cy + 4, w - 60 -
                 c.textWidth(upper(it.label), 17), 1, 0xFF34373D);
    } else {
      bool sel = i == m.selected;
      if (sel) c.fillRoundRect(x + 10, ry + 2, w - 20, rowH - 4, 8, 0xFF33373E);
      c.text(x + 26, cy, it.label, 23, sel ? 0xFFFFFFFF : 0xFFC9CBD0, kAlignLeft);
      if (it.value) {
        std::string v = it.value();
        bool adj = static_cast<bool>(it.adjust) && !it.activate;
        std::string shown = (sel && adj) ? "‹ " + v + " ›" : v;
        c.text(x + w - 26, cy, shown, 21, sel ? kFocusColor : 0xFF9A9DA4, kAlignRight);
      }
    }
    ry += rowH;
  }
  if (static_cast<int>(m.items.size()) > rows) {
    float trackH = rows * rowH;
    float th = trackH * rows / m.items.size();
    float ty = y + 60.f + trackH * m.scroll / m.items.size();
    c.fillRoundRect(x + w - 8, ty, 4, th, 2, 0xFF4A4E55);
  }
  if (hasHelp && m.selected >= 0 && m.selected < static_cast<int>(m.items.size())) {
    const std::string& help = m.items[static_cast<size_t>(m.selected)].help;
    c.pushClip({static_cast<int>(x + 10), static_cast<int>(y + h - 44),
                static_cast<int>(w - 20), 40});
    c.text(x + 22, y + h - 24, help, 18, 0xFF9A9DA4, kAlignLeft);
    c.popClip();
  }
}

void App::drawToast(Canvas& c) {
  if (toastTime_ <= 0.f || toast_.empty()) return;
  float a = std::min(1.f, toastTime_ * 3.f);
  float w = c.textWidth(toast_, 21) + 40.f;
  float x = (kScreenW - w) * 0.5f, y = kTopBar + 14.f;
  c.fillRoundRect(x, y, w, 40, 10, withAlpha(0xFF2B2E34, 0.95f * a));
  c.text(kScreenW * 0.5f, y + 20, toast_, 21, withAlpha(kTextColor, a), kAlignCenter);
}

}  // namespace br
