// BrickRack - GPL-3.0-or-later.
//
// The application: rack view, focus navigation, patch cables, menus.

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "core/engine.h"
#include "core/model.h"
#include "ui/canvas.h"
#include "ui/input.h"

namespace br {

struct MenuItem {
  std::string label;
  std::function<std::string()> value;    // optional right-aligned value
  std::function<void(int dir)> adjust;   // left/right (-1/+1); also A (0) if no `activate`
  std::function<void()> activate;        // A
  bool header = false;
  std::string help;                      // shown at the bottom of the menu
};

struct Menu {
  std::string title;
  std::vector<MenuItem> items;
  int selected = 0;
  int scroll = 0;
  bool wide = false;
};

class App {
 public:
  App(Engine& engine, std::string dataDir);
  ~App();

  void buttonEvent(Button b, bool down);
  void axisEvent(Axis a, float value);
  void update(float dt);
  void render(Canvas& canvas);

  bool wantsQuit() const { return quit_; }

  void loadStartupPatch();
  void saveAutosave();
  void buildDefaultPatch();

  // For headless screenshots / tests.
  void focusModule(int index, int widget = -1);
  void setZoom(int z);
  void openModuleMenuForTest() { openModuleMenu(); }
  void openBrowserForTest() { openBrowser(); }
  void openMainMenuForTest() { openMainMenu(); }
  void closeMenus() { menus_.clear(); }

  static constexpr int kScreenW = 1024;
  static constexpr int kScreenH = 768;

 private:
  // --- geometry
  float scale() const;
  float rackTop() const;
  float rackHeightPx() const;
  float moduleX(int index) const;  // linear px offset (navigation, single-row layout)
  void modulePos(int index, float* x, float* y) const;  // drawn position, rack space
  float overviewRowsHeight() const;
  float rackWidthPx() const;
  bool widgetCenter(int moduleIndex, int widgetIndex, float* x, float* y) const;  // screen px
  bool portCenter(int moduleId, bool output, int port, float* x, float* y) const;
  int widgetForPort(const Model* m, bool output, int port) const;

  // --- focus / navigation
  void clampFocus();
  void resetInteraction();  // drop pending cable / edit / held button state
  void navigate(int dx, int dy);
  void jumpModule(int dir);
  const Widget* focusedWidget() const;
  Module* focusedModule() const;
  void updateCamera(float dt, bool snap);

  // --- interaction
  void pressA(bool down);
  void pressB();
  void adjustFocused(float delta, bool fine);
  void startOrFinishCable();

  // --- menus
  void openMainMenu();
  void openModuleMenu();
  void openBrowser();
  void openLoadMenu();
  void openHelp();
  void openQuitConfirm();
  void menuButton(Button b);
  void addModule(const Model* model);
  void toast(const std::string& msg);
  bool savePatchAs(const std::string& name);
  std::vector<std::string> listPatches() const;

  // --- drawing
  void drawRails(Canvas& c);
  void drawModule(Canvas& c, int index);
  const Canvas& panelImage(const Model* m);
  void renderPanelStatic(Canvas& img, const PanelSpec& p, float s);
  void drawWidgetDynamic(Canvas& c, const Widget& w, const Module* m, float ox, float oy,
                         float s);
  void drawCables(Canvas& c);
  void drawCable(Canvas& c, float x0, float y0, float x1, float y1, uint32_t color,
                 float alpha);
  void drawFocus(Canvas& c);
  void drawTopBar(Canvas& c);
  void drawStatusBar(Canvas& c);
  void drawMenu(Canvas& c, Menu& m);
  void drawToast(Canvas& c);

  Engine& engine_;
  std::string dataDir_;
  std::string patchName_ = "autosave";

  int focusIndex_ = 0;   // module index in the rack
  int focusWidget_ = -1;
  bool editing_ = false;
  int zoom_ = 1;  // 0 overview, 1 normal, 2 close-up
  float scrollX_ = 0.f, scrollY_ = 0.f;
  bool cameraSnap_ = true;

  struct PendingCable {
    bool active = false;
    int moduleId = -1;
    bool output = false;
    int port = -1;
    uint32_t color = 0;
  } pending_;

  bool held_[BTN_COUNT] = {};
  float repeatTimer_[BTN_COUNT] = {};
  float axis_[AXIS_COUNT] = {};
  int aDownParam_ = -1;  // momentary button param held via A
  int aDownModule_ = -1;
  Param* stickAccParam_ = nullptr;
  float stickAcc_ = 0.f;

  std::vector<std::unique_ptr<Menu>> menus_;
  std::string toast_;
  float toastTime_ = 0.f;
  float time_ = 0.f;
  float autosaveTimer_ = 0.f;
  bool quit_ = false;

  std::map<std::pair<const Model*, int>, std::unique_ptr<Canvas>> panelCache_;
};

}  // namespace br
