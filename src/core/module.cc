// BrickRack - GPL-3.0-or-later.

#include "core/module.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace br {

void Param::set(float v) {
  if (snap) v = std::round(v);
  value = std::min(std::max(v, std::min(minValue, maxValue)), std::max(minValue, maxValue));
}

float Param::normalized() const {
  if (maxValue == minValue) return 0.f;
  return (value - minValue) / (maxValue - minValue);
}

void Param::setNormalized(float n) {
  n = std::min(std::max(n, 0.f), 1.f);
  set(minValue + n * (maxValue - minValue));
}

std::string Param::displayString() const {
  char buf[64];
  if (!labels.empty()) {
    int i = static_cast<int>(std::lround(value - minValue));
    i = std::min(std::max(i, 0), static_cast<int>(labels.size()) - 1);
    return labels[static_cast<size_t>(i)];
  }
  if (type == ParamType::Button || type == ParamType::Toggle) {
    return value > 0.5f ? "ON" : "OFF";
  }
  float shown = (displayBase > 0.f ? std::pow(displayBase, value) : value) * displayMultiplier +
                displayOffset;
  if (displayBase > 0.f && std::fabs(shown) >= 1000.f && unit == "Hz") {
    std::snprintf(buf, sizeof(buf), "%.2f kHz", shown / 1000.f);
    return buf;
  }
  if (snap) {
    std::snprintf(buf, sizeof(buf), "%d%s%s", static_cast<int>(std::lround(shown)),
                  unit.empty() ? "" : " ", unit.c_str());
  } else {
    std::snprintf(buf, sizeof(buf), "%.*f%s%s", displayPrecision, shown,
                  unit.empty() ? "" : " ", unit.c_str());
  }
  return buf;
}

void Module::config(int numParams, int numInputs, int numOutputs, int numLights) {
  params.assign(static_cast<size_t>(numParams), Param());
  inputs.assign(static_cast<size_t>(numInputs), Port());
  outputs.assign(static_cast<size_t>(numOutputs), Port());
  lights.assign(static_cast<size_t>(numLights), 0.f);
}

Param& Module::configParam(int id, float minValue, float maxValue, float defaultValue,
                           const std::string& name, const std::string& unit,
                           float displayMultiplier, float displayOffset) {
  Param& p = params[static_cast<size_t>(id)];
  p.type = ParamType::Knob;
  p.minValue = minValue;
  p.maxValue = maxValue;
  p.defaultValue = defaultValue;
  p.value = defaultValue;
  p.name = name;
  p.unit = unit;
  p.displayMultiplier = displayMultiplier;
  p.displayOffset = displayOffset;
  return p;
}

Param& Module::configSwitch(int id, float minValue, float maxValue, float defaultValue,
                            const std::string& name, std::vector<std::string> labels) {
  Param& p = configParam(id, minValue, maxValue, defaultValue, name);
  p.type = ParamType::Switch;
  p.snap = true;
  p.labels = std::move(labels);
  return p;
}

Param& Module::configButton(int id, const std::string& name) {
  Param& p = configParam(id, 0.f, 1.f, 0.f, name);
  p.type = ParamType::Button;
  return p;
}

Param& Module::configToggle(int id, const std::string& name, bool def) {
  Param& p = configParam(id, 0.f, 1.f, def ? 1.f : 0.f, name);
  p.type = ParamType::Toggle;
  p.snap = true;
  return p;
}

void Module::configInput(int id, const std::string& name) {
  inputs[static_cast<size_t>(id)].name = name;
}

void Module::configOutput(int id, const std::string& name) {
  outputs[static_cast<size_t>(id)].name = name;
}

void Module::resetParams() {
  for (Param& p : params) p.value = p.defaultValue;
}

}  // namespace br
