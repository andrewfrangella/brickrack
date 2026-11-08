// BrickRack - GPL-3.0-or-later.

#pragma once

namespace br {

enum Button {
  BTN_UP,
  BTN_DOWN,
  BTN_LEFT,
  BTN_RIGHT,
  BTN_A,
  BTN_B,
  BTN_X,
  BTN_Y,
  BTN_L1,
  BTN_R1,
  BTN_L2,
  BTN_R2,
  BTN_START,
  BTN_SELECT,
  BTN_MENU,
  BTN_L3,
  BTN_R3,
  BTN_COUNT
};

enum Axis { AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, AXIS_COUNT };

const char* buttonName(Button b);

}  // namespace br
