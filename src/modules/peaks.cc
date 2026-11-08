// BrickRack - GPL-3.0-or-later.
//
// Dual Trigger Converter: wraps the two Peaks processors
// (peaks/processors.cc: envelopes, LFOs, tap LFO, drums, mini sequencer,
// pulse shaper/randomizer, FM drum and the number station) and drives them
// like peaks.cc and ui.cc do: 48 kHz, 4-sample blocks, gate inputs OR'ed with
// the panel trigger buttons, the pots filtered and thresholded once per block,
// twin/split/expert edit modes.
//
// The firmware keeps its two processors in a global array; each module
// instance owns its own pair here.

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <string>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "peaks/gate_processor.h"
#include "peaks/io_buffer.h"
#include "peaks/processors.h"
#include "ui/canvas.h"

namespace br {
namespace {

using peaks::kBlockSize;
constexpr int kNumAdcChannels = 4;

// ui.h / ui.cc (the header includes the STM32 drivers, so the enums and
// tables are reproduced here).
enum SwitchIndex { SWITCH_TWIN_MODE, SWITCH_FUNCTION, SWITCH_GATE_TRIG_1, SWITCH_GATE_TRIG_2 };
enum EditMode { EDIT_MODE_TWIN, EDIT_MODE_SPLIT, EDIT_MODE_FIRST, EDIT_MODE_SECOND, EDIT_MODE_LAST };
enum Function {
  FUNCTION_ENVELOPE,
  FUNCTION_LFO,
  FUNCTION_TAP_LFO,
  FUNCTION_DRUM_GENERATOR,
  FUNCTION_MINI_SEQUENCER,
  FUNCTION_PULSE_SHAPER,
  FUNCTION_PULSE_RANDOMIZER,
  FUNCTION_FM_DRUM_GENERATOR,
  FUNCTION_LAST,
  FUNCTION_FIRST_ALTERNATE_FUNCTION = FUNCTION_MINI_SEQUENCER
};

const uint16_t kAdcThresholdUnlocked = 1 << (16 - 10);  // 10 bits
const uint16_t kAdcThresholdLocked = 1 << (16 - 8);     // 8 bits
const int32_t kLongPressDuration = 600;

const peaks::ProcessorFunction kFunctionTable[FUNCTION_LAST][2] = {
    {peaks::PROCESSOR_FUNCTION_ENVELOPE, peaks::PROCESSOR_FUNCTION_ENVELOPE},
    {peaks::PROCESSOR_FUNCTION_LFO, peaks::PROCESSOR_FUNCTION_LFO},
    {peaks::PROCESSOR_FUNCTION_TAP_LFO, peaks::PROCESSOR_FUNCTION_TAP_LFO},
    {peaks::PROCESSOR_FUNCTION_BASS_DRUM, peaks::PROCESSOR_FUNCTION_SNARE_DRUM},
    {peaks::PROCESSOR_FUNCTION_MINI_SEQUENCER, peaks::PROCESSOR_FUNCTION_MINI_SEQUENCER},
    {peaks::PROCESSOR_FUNCTION_PULSE_SHAPER, peaks::PROCESSOR_FUNCTION_PULSE_SHAPER},
    {peaks::PROCESSOR_FUNCTION_PULSE_RANDOMIZER, peaks::PROCESSOR_FUNCTION_PULSE_RANDOMIZER},
    {peaks::PROCESSOR_FUNCTION_FM_DRUM, peaks::PROCESSOR_FUNCTION_FM_DRUM},
};

const char* kFunctionNames[FUNCTION_LAST] = {
    "Envelope", "LFO", "Tap LFO", "Drums (bass/snare)",
    "Mini sequencer", "Pulse shaper", "Pulse randomizer", "FM drum"};
const char* kFunctionShort[FUNCTION_LAST] = {"ENV", "LFO", "TAP", "DRUM",
                                             "SEQ", "PULSE", "RAND", "FM DRUM"};

class DualTrigger : public Module {
 public:
  enum ParamIds {
    KNOB1_PARAM,
    KNOB2_PARAM,
    KNOB3_PARAM,
    KNOB4_PARAM,
    TWIN_BUTTON,
    FUNCTION_BUTTON,
    TRIG1_BUTTON,
    TRIG2_BUTTON,
    EDIT_MODE_PARAM,  // hidden
    FUNCTION1_PARAM,  // hidden
    FUNCTION2_PARAM,  // hidden
    SNAP_MODE_PARAM,  // hidden
    NUM_PARAMS
  };
  enum InputIds { GATE1_INPUT, GATE2_INPUT, NUM_INPUTS };
  enum OutputIds { OUT1_OUTPUT, OUT2_OUTPUT, NUM_OUTPUTS };
  enum LightIds { TWIN_LIGHT, FUNCTION_LIGHT, TRIG1_LIGHT = FUNCTION_LIGHT + 4, TRIG2_LIGHT,
                  NUM_LIGHTS };

  DualTrigger() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < 4; ++i) {
      configParam(KNOB1_PARAM + i, 0.f, 1.f, 0.5f, "Knob " + std::to_string(i + 1), "%", 100.f);
    }
    configButton(TWIN_BUTTON, "Split/twin mode (hold: expert mode)");
    configButton(FUNCTION_BUTTON, "Function (hold: alternate functions)");
    configButton(TRIG1_BUTTON, "Trigger 1");
    configButton(TRIG2_BUTTON, "Trigger 2");
    configSwitch(EDIT_MODE_PARAM, 0.f, 3.f, 0.f, "Edit mode",
                 {"Twin", "Split", "Expert: channel 1", "Expert: channel 2"}).hidden = true;
    std::vector<std::string> names(kFunctionNames, kFunctionNames + FUNCTION_LAST);
    configSwitch(FUNCTION1_PARAM, 0.f, 7.f, 0.f, "Function, channel 1", names).hidden = true;
    configSwitch(FUNCTION2_PARAM, 0.f, 7.f, 0.f, "Function, channel 2", names).hidden = true;
    configSwitch(SNAP_MODE_PARAM, 0.f, 1.f, 0.f, "Expert mode pots: catch up (snap)",
                 {"Off", "On"}).hidden = true;
    configInput(GATE1_INPUT, "Gate 1");
    configInput(GATE2_INPUT, "Gate 2");
    configOutput(OUT1_OUTPUT, "Out 1");
    configOutput(OUT2_OUTPUT, "Out 2");

    // peaks.cc Init() / Ui::Init() with fresh settings. The firmware's
    // processors are zero-initialised globals and Init() relies on that (e.g.
    // MultistageEnvelope reads shape_[] slots it never sets), so clear them.
    std::memset(static_cast<void*>(processors_), 0, sizeof(processors_));
    processors_[0].Init(0);
    processors_[1].Init(1);
    std::memset(&block_, 0, sizeof(block_));
    std::memset(outBuf_, 0, sizeof(outBuf_));
    std::memset(potValue_, 0, sizeof(potValue_));
    initUi();
  }

  void onReset() override {
    std::memset(pending_.potValue, 0, sizeof(pending_.potValue));
    pendingLoad_.store(true, std::memory_order_release);
  }

  // The pot values remembered for each channel in expert mode.
  void saveState(StateMap& s) const override {
    std::string v;
    for (int i = 0; i < 8; ++i) {
      if (i) v += ",";
      v += std::to_string(potValue_[i]);
    }
    s["potValues"] = v;
  }

  void loadState(const StateMap& s) override {
    auto it = s.find("potValues");
    std::memset(pending_.potValue, 0, sizeof(pending_.potValue));
    if (it != s.end()) {
      const char* p = it->second.c_str();
      for (int i = 0; i < 8 && *p; ++i) {
        char* end = nullptr;
        long v = std::strtol(p, &end, 10);
        pending_.potValue[i] = static_cast<uint8_t>(std::min(255L, std::max(0L, v)));
        p = (*end == ',') ? end + 1 : end;
      }
    }
    pendingLoad_.store(true, std::memory_order_release);
  }

  void appendMenu(std::vector<MenuAction>& items) override {
    // On the hardware: hold both buttons three times in a row.
    items.push_back({"Number station (easter egg)",
                     [this] { return std::string(numberStation() ? "ON" : "OFF"); },
                     [this](int) { numberStationRequest_.store(true); }});
  }

  bool numberStation() const {
    return processors_[0].function() == peaks::PROCESSOR_FUNCTION_NUMBER_STATION;
  }
  int channelFunction(int i) const { return function_[i]; }
  int editMode() const { return editMode_; }

  void process(const ProcessArgs& args) override {
    if (pendingLoad_.exchange(false, std::memory_order_acquire)) applyLoadedState();
    if (numberStationRequest_.exchange(false)) {
      processors_[0].set_function(peaks::PROCESSOR_FUNCTION_NUMBER_STATION);
      processors_[1].set_function(peaks::PROCESSOR_FUNCTION_NUMBER_STATION);
    }
    syncHiddenParams();

    uiPhase_ += args.sampleTime * 1000.f;
    while (uiPhase_ >= 1.f) {
      uiPhase_ -= 1.f;
      pollUi();
    }

    // TIM1_UP_IRQHandler(): one 48 kHz frame.
    uint32_t external = 0;
    for (int i = 0; i < 2; ++i) {
      gateTrigger_[i].process(inputs[static_cast<size_t>(GATE1_INPUT + i)].getVoltage());
      if (gateTrigger_[i].isHigh()) external |= 1u << i;
    }
    uint32_t buttons = (panelGateControl_[0] ? 1u : 0u) | (panelGateControl_[1] ? 2u : 0u);
    uint32_t gates = external | buttons;
    for (int i = 0; i < 2; ++i) {
      gateFlags_[i] = peaks::ExtractGateFlags(gateFlags_[i], gates & (1u << i));
    }
    // Channel 1 also sees channel 2's gate (used to reset the sequencer).
    block_.input[0][pos_] = static_cast<peaks::GateFlags>(
        gateFlags_[0] | (gateFlags_[1] << 4) | (buttons & 1 ? peaks::GATE_FLAG_FROM_BUTTON : 0));
    block_.input[1][pos_] = static_cast<peaks::GateFlags>(
        gateFlags_[1] | (buttons & 2 ? peaks::GATE_FLAG_FROM_BUTTON : 0));
    // DAC: 16-bit, inverted, full scale = +/-8 V.
    outputs[OUT1_OUTPUT].setVoltage(static_cast<float>(outBuf_[0][pos_]) / 32768.f * 8.f);
    outputs[OUT2_OUTPUT].setVoltage(static_cast<float>(outBuf_[1][pos_]) / 32768.f * 8.f);
    if (++pos_ == kBlockSize) {
      pos_ = 0;
      renderBlock();
    }
  }

 private:
  // peaks.cc Process().
  void renderBlock() {
    pollPots();
    for (int i = 0; i < 2; ++i) {
      processors_[i].Process(block_.input[i], outBuf_[i], kBlockSize);
      brightness_[i] = outBuf_[i][0];
    }
  }

  // --- ui.cc -------------------------------------------------------------

  void initUi() {
    std::fill(&adcLp_[0], &adcLp_[kNumAdcChannels], 0);
    std::fill(&adcValue_[0], &adcValue_[kNumAdcChannels], 0);
    std::fill(&adcThreshold_[0], &adcThreshold_[kNumAdcChannels], 0);
    std::fill(&snapped_[0], &snapped_[kNumAdcChannels], false);
    editMode_ = EDIT_MODE_TWIN;
    function_[0] = function_[1] = FUNCTION_ENVELOPE;
    changeControlMode();
    setFunction(0, function_[0]);
    setFunction(1, function_[1]);
    doublePressCounter_ = 0;
    publishHiddenParams();
  }

  // Ui::Init() when settings are found in storage.
  void applyLoadedState() {
    std::memcpy(potValue_, pending_.potValue, sizeof(potValue_));
    editMode_ = static_cast<EditMode>(clampi(params[EDIT_MODE_PARAM].value, 0, 3));
    function_[0] = static_cast<Function>(clampi(params[FUNCTION1_PARAM].value, 0, 7));
    function_[1] = static_cast<Function>(clampi(params[FUNCTION2_PARAM].value, 0, 7));
    if (editMode_ == EDIT_MODE_FIRST || editMode_ == EDIT_MODE_SECOND) {
      lockPots();
      for (uint8_t i = 0; i < 4; ++i) {
        processors_[0].set_parameter(i, static_cast<uint16_t>(potValue_[i]) << 8);
        processors_[1].set_parameter(i, static_cast<uint16_t>(potValue_[i + 4]) << 8);
      }
    }
    changeControlMode();
    setFunction(0, function_[0]);
    setFunction(1, function_[1]);
    publishHiddenParams();
  }

  static int clampi(float v, int lo, int hi) {
    return std::min(hi, std::max(lo, static_cast<int>(v + 0.5f)));
  }

  void publishHiddenParams() {
    params[EDIT_MODE_PARAM].set(static_cast<float>(editMode_));
    params[FUNCTION1_PARAM].set(static_cast<float>(function_[0]));
    params[FUNCTION2_PARAM].set(static_cast<float>(function_[1]));
    shownEditMode_ = editMode_;
    shownFunction_[0] = function_[0];
    shownFunction_[1] = function_[1];
  }

  // Applies edits made from the module menu.
  void syncHiddenParams() {
    int mode = clampi(params[EDIT_MODE_PARAM].value, 0, 3);
    if (mode != shownEditMode_) {
      editMode_ = static_cast<EditMode>(mode);
      if (editMode_ >= EDIT_MODE_FIRST) lockPots();
      if (editMode_ <= EDIT_MODE_SPLIT) {
        // Twin/split use one function for both channels.
        function_[0] = function_[1];
        setFunction(0, function_[1]);
      }
      changeControlMode();
      publishHiddenParams();
    }
    for (int i = 0; i < 2; ++i) {
      int f = clampi(params[static_cast<size_t>(FUNCTION1_PARAM + i)].value, 0, 7);
      if (f != shownFunction_[i]) {
        if (editMode_ <= EDIT_MODE_SPLIT) {
          setFunction(0, static_cast<Function>(f));
        } else {
          function_[i] = static_cast<Function>(f);
          processors_[i].set_function(kFunctionTable[f][i]);
        }
        publishHiddenParams();
      }
    }
  }

  void lockPots() {
    std::fill(&adcThreshold_[0], &adcThreshold_[kNumAdcChannels], kAdcThresholdLocked);
    std::fill(&snapped_[0], &snapped_[kNumAdcChannels], false);
  }

  Function function() const {
    return editMode_ == EDIT_MODE_SECOND ? function_[1] : function_[0];
  }

  void changeControlMode() {
    uint16_t parameters[4];
    for (int i = 0; i < 4; ++i) parameters[i] = static_cast<uint16_t>(adcValue_[i]);
    if (editMode_ == EDIT_MODE_SPLIT) {
      processors_[0].CopyParameters(&parameters[0], 2);
      processors_[1].CopyParameters(&parameters[2], 2);
      processors_[0].set_control_mode(peaks::CONTROL_MODE_HALF);
      processors_[1].set_control_mode(peaks::CONTROL_MODE_HALF);
    } else if (editMode_ == EDIT_MODE_TWIN) {
      processors_[0].CopyParameters(&parameters[0], 4);
      processors_[1].CopyParameters(&parameters[0], 4);
      processors_[0].set_control_mode(peaks::CONTROL_MODE_FULL);
      processors_[1].set_control_mode(peaks::CONTROL_MODE_FULL);
    } else {
      processors_[0].set_control_mode(peaks::CONTROL_MODE_FULL);
      processors_[1].set_control_mode(peaks::CONTROL_MODE_FULL);
    }
  }

  void setFunction(uint8_t index, Function f) {
    if (editMode_ == EDIT_MODE_SPLIT || editMode_ == EDIT_MODE_TWIN) {
      function_[0] = function_[1] = f;
      processors_[0].set_function(kFunctionTable[f][0]);
      processors_[1].set_function(kFunctionTable[f][1]);
    } else {
      function_[index] = f;
      processors_[index].set_function(kFunctionTable[f][index]);
    }
  }

  // Ui::PollPots(), once per block.
  void pollPots() {
    for (int i = 0; i < kNumAdcChannels; ++i) {
      int32_t raw = static_cast<int32_t>(clampf(params[static_cast<size_t>(KNOB1_PARAM + i)].value,
                                                0.f, 1.f) * 65535.f);
      adcLp_[i] = (raw + adcLp_[i] * 7) >> 3;
      int32_t value = adcLp_[i];
      int32_t current = adcValue_[i];
      if (value >= current + adcThreshold_[i] || value <= current - adcThreshold_[i] ||
          !adcThreshold_[i]) {
        onPotChanged(i, value);
        adcValue_[i] = value;
        adcThreshold_[i] = kAdcThresholdUnlocked;
      }
    }
  }

  void onPotChanged(int id, int32_t data) {
    const uint16_t d = static_cast<uint16_t>(data);
    switch (editMode_) {
      case EDIT_MODE_TWIN:
        processors_[0].set_parameter(static_cast<uint8_t>(id), d);
        processors_[1].set_parameter(static_cast<uint8_t>(id), d);
        potValue_[id] = static_cast<uint8_t>(d >> 8);
        break;
      case EDIT_MODE_SPLIT:
        if (id < 2) {
          processors_[0].set_parameter(static_cast<uint8_t>(id), d);
        } else {
          processors_[1].set_parameter(static_cast<uint8_t>(id - 2), d);
        }
        potValue_[id] = static_cast<uint8_t>(d >> 8);
        break;
      case EDIT_MODE_FIRST:
      case EDIT_MODE_SECOND: {
        int index = id + (editMode_ - EDIT_MODE_FIRST) * 4;
        peaks::Processors* p = &processors_[editMode_ - EDIT_MODE_FIRST];
        int delta = std::abs(static_cast<int>(potValue_[index]) - static_cast<int>(d >> 8));
        bool snapMode = params[SNAP_MODE_PARAM].value > 0.5f;
        if (!snapMode || snapped_[id] || delta <= 2) {
          p->set_parameter(static_cast<uint8_t>(id), d);
          potValue_[index] = static_cast<uint8_t>(d >> 8);
          snapped_[id] = true;
        }
        break;
      }
      case EDIT_MODE_LAST:
        break;
    }
  }

  // Ui::Poll() + DoEvents(), at 1 kHz.
  void pollUi() {
    ++ms_;
    for (int i = 0; i < 4; ++i) {
      bool pressed = params[static_cast<size_t>(TWIN_BUTTON + i)].value > 0.5f;
      if (pressed && !wasPressed_[i]) {
        onSwitchPressed(i);
        pressTime_[i] = ms_;
      }
      if (pressed && pressTime_[i] != 0 && i < SWITCH_GATE_TRIG_1) {
        int32_t pressedTime = static_cast<int32_t>(ms_ - pressTime_[i]);
        if (pressedTime > kLongPressDuration) {
          bool otherPressed = params[static_cast<size_t>(TWIN_BUTTON + 1 - i)].value > 0.5f;
          if (otherPressed) {
            ++doublePressCounter_;
            pressTime_[0] = pressTime_[1] = 0;
            if (doublePressCounter_ == 3) {
              doublePressCounter_ = 0;
              processors_[0].set_function(peaks::PROCESSOR_FUNCTION_NUMBER_STATION);
              processors_[1].set_function(peaks::PROCESSOR_FUNCTION_NUMBER_STATION);
            }
          } else {
            onSwitchReleased(i, pressedTime);
            pressTime_[i] = 0;  // Inhibit next release event
          }
        }
      }
      if (!pressed && wasPressed_[i] && pressTime_[i] != 0) {
        onSwitchReleased(i, static_cast<int32_t>(ms_ - pressTime_[i] + 1));
      }
      wasPressed_[i] = pressed;
    }
    refreshLeds();
  }

  void onSwitchPressed(int id) {
    if (id == SWITCH_GATE_TRIG_1) panelGateControl_[0] = true;
    if (id == SWITCH_GATE_TRIG_2) panelGateControl_[1] = true;
  }

  void onSwitchReleased(int id, int32_t data) {
    switch (id) {
      case SWITCH_TWIN_MODE:
        if (data > kLongPressDuration) {
          editMode_ = static_cast<EditMode>((editMode_ + EDIT_MODE_FIRST) % EDIT_MODE_LAST);
          function_[0] = function_[1];
          processors_[0].set_function(kFunctionTable[function_[0]][0]);
          processors_[1].set_function(kFunctionTable[function_[0]][1]);
          lockPots();
        } else {
          if (editMode_ <= EDIT_MODE_SPLIT) {
            editMode_ = static_cast<EditMode>(EDIT_MODE_SPLIT - editMode_);
          } else {
            editMode_ = static_cast<EditMode>(EDIT_MODE_SECOND - (editMode_ & 1));
            lockPots();
          }
        }
        changeControlMode();
        publishHiddenParams();
        break;
      case SWITCH_FUNCTION: {
        Function f = function();
        if (data > kLongPressDuration) {
          f = static_cast<Function>((f + FUNCTION_FIRST_ALTERNATE_FUNCTION) % FUNCTION_LAST);
        } else if (f <= FUNCTION_DRUM_GENERATOR) {
          f = static_cast<Function>((f + 1) & 3);
        } else {
          f = static_cast<Function>(((f + 1) & 3) + FUNCTION_FIRST_ALTERNATE_FUNCTION);
        }
        setFunction(static_cast<uint8_t>(editMode_ - EDIT_MODE_FIRST), f);
        publishHiddenParams();
        break;
      }
      case SWITCH_GATE_TRIG_1:
        panelGateControl_[0] = false;
        break;
      case SWITCH_GATE_TRIG_2:
        panelGateControl_[1] = false;
        break;
    }
  }

  void refreshLeds() {
    uint8_t flash = (ms_ >> 7) & 7;
    bool twin;
    switch (editMode_) {
      case EDIT_MODE_FIRST: twin = flash == 1; break;
      case EDIT_MODE_SECOND: twin = flash == 1 || flash == 3; break;
      default: twin = editMode_ & 1; break;
    }
    uint8_t pattern;
    if ((ms_ & 256) && function() >= FUNCTION_FIRST_ALTERNATE_FUNCTION) {
      pattern = 1 << 4;  // all function LEDs off
    } else {
      pattern = static_cast<uint8_t>(1 << (function() & 3));
    }
    int b[2];
    for (int i = 0; i < 2; ++i) {
      switch (function_[i]) {
        case FUNCTION_DRUM_GENERATOR:
        case FUNCTION_FM_DRUM_GENERATOR:
          b[i] = std::min(255, std::abs(static_cast<int>(brightness_[i])) >> 8);
          break;
        case FUNCTION_LFO:
        case FUNCTION_TAP_LFO:
        case FUNCTION_MINI_SEQUENCER: {
          int32_t brightness = int32_t(brightness_[i]) * 409 >> 8;
          brightness += 32768;
          brightness >>= 8;
          b[i] = std::min(255, std::max(0, static_cast<int>(brightness)));
          break;
        }
        default:
          b[i] = std::max(0, brightness_[i] >> 7);
          b[i] = b[i] & 0xff;  // uint8_t in the firmware
          break;
      }
    }
    if (numberStation()) {
      pattern = processors_[0].number_station().digit() ^ processors_[1].number_station().digit();
      b[0] = processors_[0].number_station().gate() ? 255 : 0;
      b[1] = processors_[1].number_station().gate() ? 255 : 0;
    }
    lights[TWIN_LIGHT] = twin ? 1.f : 0.f;
    for (int i = 0; i < 4; ++i) {
      lights[static_cast<size_t>(FUNCTION_LIGHT + i)] = (pattern & (1 << i)) ? 1.f : 0.f;
    }
    lights[TRIG1_LIGHT] = static_cast<float>(b[0]) / 255.f;
    lights[TRIG2_LIGHT] = static_cast<float>(b[1]) / 255.f;
  }

  peaks::Processors processors_[2];
  peaks::IOBuffer::Block block_;
  int16_t outBuf_[2][kBlockSize];
  size_t pos_ = 0;
  SchmittTrigger gateTrigger_[2];
  peaks::GateFlags gateFlags_[2] = {0, 0};
  int16_t brightness_[2] = {0, 0};

  // Ui state.
  EditMode editMode_ = EDIT_MODE_TWIN;
  Function function_[2] = {FUNCTION_ENVELOPE, FUNCTION_ENVELOPE};
  uint8_t potValue_[8];
  int32_t adcLp_[kNumAdcChannels];
  int32_t adcValue_[kNumAdcChannels];
  int32_t adcThreshold_[kNumAdcChannels];
  bool snapped_[kNumAdcChannels];
  bool panelGateControl_[2] = {false, false};
  uint32_t pressTime_[4] = {};
  bool wasPressed_[4] = {};
  int doublePressCounter_ = 0;
  float uiPhase_ = 0.f;
  uint32_t ms_ = 0;

  // Menu / patch-loading handshake with the UI thread.
  int shownEditMode_ = 0;
  int shownFunction_[2] = {0, 0};
  struct {
    uint8_t potValue[8];
  } pending_;
  std::atomic<bool> pendingLoad_{false};
  std::atomic<bool> numberStationRequest_{false};
};

void buildPanel(PanelSpec& p) {
  using M = DualTrigger;
  p.hp = 8;
  p.bg = 0xFFF3F2EE;
  p.ink = 0xFF1E1E20;
  const uint32_t led = 0xFFFF3B30;

  p.button(M::TWIN_BUTTON, 5.1f, 19.9f, 5.f);
  p.light(M::TWIN_LIGHT, 5.2f, 26.1f, led, 2.6f);
  p.label(8.0f, 26.1f, "SPLIT", 3.0f, kAlignLeft);
  p.button(M::FUNCTION_BUTTON, 5.1f, 32.5f, 5.f);
  const char* fn[4] = {"ENV", "LFO", "TAP", "DRUM"};
  const float fy[4] = {38.7f, 44.0f, 49.4f, 54.8f};
  for (int i = 0; i < 4; ++i) {
    p.light(M::FUNCTION_LIGHT + i, 5.2f, fy[i], led, 2.6f);
    p.label(8.0f, fy[i], fn[i], 3.0f, kAlignLeft);
  }

  const float ky[4] = {23.9f, 45.8f, 67.7f, 89.5f};
  for (int i = 0; i < 4; ++i) {
    p.knob(M::KNOB1_PARAM + i, 27.4f, ky[i], 10.f);
    p.label(27.4f, ky[i] - 8.6f, std::to_string(i + 1), 3.2f);
  }
  p.line(19.3f, 13.f, 19.3f, 98.f, 0.35f, 0xFFC4C4C2);

  p.button(M::TRIG1_BUTTON, 7.0f, 67.5f, 7.5f, "", 0xFFD8D9DB, M::TRIG1_LIGHT);
  p.label(7.0f, 75.0f, "TRIG1", 3.0f);
  p.input(M::GATE1_INPUT, 7.0f, 82.2f);
  p.button(M::TRIG2_BUTTON, 7.0f, 96.6f, 7.5f, "", 0xFFD8D9DB, M::TRIG2_LIGHT);
  p.label(7.0f, 104.3f, "TRIG2", 3.0f);
  p.input(M::GATE2_INPUT, 7.0f, 111.1f);

  const uint32_t dark = 0xFF5E5E60, white = 0xFFFFFFFF;
  p.box(16.6f, 101.9f, 10.0f, 14.8f, dark, 1.4f);
  p.box(28.2f, 101.9f, 10.0f, 14.8f, dark, 1.4f);
  p.output(M::OUT1_OUTPUT, 21.6f, 111.1f);
  p.output(M::OUT2_OUTPUT, 33.2f, 111.1f);
  p.label(21.6f, 104.6f, "OUT1", 2.9f, kAlignCenter, white);
  p.label(33.2f, 104.6f, "OUT2", 2.9f, kAlignCenter, white);

  // Function readout (both channels; handy in split/expert modes).
  p.display(1.5f, 58.4f, 17.0f, 3.6f, [](Canvas& c, float x, float y, float w, float h, float s,
                                         const Module* m) {
    const DualTrigger* d = static_cast<const DualTrigger*>(m);
    std::string t;
    if (d->numberStation()) {
      t = "NUMBERS";
    } else if (d->editMode() <= EDIT_MODE_SPLIT) {
      t = kFunctionShort[d->channelFunction(0)];
    } else {
      t = std::string(kFunctionShort[d->channelFunction(0)]) + "|" +
          kFunctionShort[d->channelFunction(1)];
    }
    c.text(x + w * 0.5f, y + h * 0.5f, t, std::max(12.f, 2.4f * s), 0xFF168A8C, kAlignCenter);
  });
}

Model model = [] {
  Model m;
  m.slug = "dual-trigger";
  m.name = "Dual Trigger Converter";
  m.shortName = "DUAL TRIGGER";
  m.basedOn = "Peaks";
  m.category = Category::Modulator;
  m.description = "Two channels of envelopes, LFOs, tap LFO, drums, sequencer and pulse tools.";
  m.create = [] { return new DualTrigger(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
