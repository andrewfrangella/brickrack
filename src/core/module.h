// BrickRack - modular synthesizer for TrimUI Brick / Brick Pro (NextUI)
// Copyright (C) 2026 BrickRack contributors. GPL-3.0-or-later.
//
// Module base class. The model is deliberately close to VCV Rack's: every
// module is processed once per engine sample, reads input voltages written by
// cables, and writes output voltages. Voltages follow Eurorack conventions
// (1 V/oct pitch, 0/+5..10 V gates, +/-5 V audio).

#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace br {

class Engine;
struct Model;

struct ProcessArgs {
  float sampleRate;
  float sampleTime;
  Engine* engine;
  int64_t frame;  // absolute engine frame counter
};

enum class ParamType {
  Knob,      // continuous control
  Switch,    // discrete positions (labels)
  Button,    // momentary push button (1 while held)
  Toggle,    // latching on/off
};

struct Param {
  ParamType type = ParamType::Knob;
  float value = 0.f;
  float minValue = 0.f;
  float maxValue = 1.f;
  float defaultValue = 0.f;
  std::string name;
  std::string unit;
  // Display transform (as in VCV Rack):
  //   displayBase == 0: shown = value * displayMultiplier + displayOffset
  //   displayBase  > 0: shown = displayBase^value * displayMultiplier + displayOffset
  // e.g. an exponential cutoff knob 0..10 with base 2, multiplier 20 shows Hz.
  float displayBase = 0.f;
  float displayMultiplier = 1.f;
  float displayOffset = 0.f;
  int displayPrecision = 2;
  std::vector<std::string> labels;  // for switches
  bool hidden = false;              // not on panel; reachable from the module menu
  bool snap = false;                // integer values

  float get() const { return value; }
  void set(float v);
  std::string displayString() const;
  float normalized() const;
  void setNormalized(float n);
};

struct Port {
  float voltage = 0.f;
  bool connected = false;
  std::string name;

  float getVoltage() const { return voltage; }
  float getNormalVoltage(float normal) const { return connected ? voltage : normal; }
  void setVoltage(float v) { voltage = v; }
  bool isConnected() const { return connected; }
};

// A tiny key/value store used for module state persistence in patch files.
using StateMap = std::map<std::string, std::string>;

// Extra per-module actions shown in the module menu (Y button).
struct MenuAction {
  std::string label;
  std::function<std::string()> value;      // optional current value text
  std::function<void(int dir)> activate;   // dir = 0 (A), -1/+1 (left/right)
};

class Module {
 public:
  virtual ~Module() = default;

  enum { kNoLight = -1 };

  std::vector<Param> params;
  std::vector<Port> inputs;
  std::vector<Port> outputs;
  std::vector<float> lights;  // brightness 0..1, written by audio thread

  int id = -1;
  const Model* model = nullptr;

  virtual void process(const ProcessArgs& args) = 0;
  virtual void onReset() {}
  virtual void onSampleRateChange(float /*sampleRate*/) {}
  virtual void saveState(StateMap& /*state*/) const {}
  virtual void loadState(const StateMap& /*state*/) {}
  virtual void appendMenu(std::vector<MenuAction>& /*items*/) {}

  // Optional per-module text shown on a display widget (e.g. alphanumeric).
  virtual std::string displayText(int /*which*/) const { return std::string(); }

  void resetParams();

 protected:
  void config(int numParams, int numInputs, int numOutputs, int numLights);
  Param& configParam(int id, float minValue, float maxValue, float defaultValue,
                     const std::string& name, const std::string& unit = "",
                     float displayMultiplier = 1.f, float displayOffset = 0.f);
  Param& configSwitch(int id, float minValue, float maxValue, float defaultValue,
                      const std::string& name, std::vector<std::string> labels = {});
  Param& configButton(int id, const std::string& name);
  Param& configToggle(int id, const std::string& name, bool def = false);
  void configInput(int id, const std::string& name);
  void configOutput(int id, const std::string& name);
};

}  // namespace br
