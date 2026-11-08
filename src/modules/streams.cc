// BrickRack - GPL-3.0-or-later.
//
// Dual Dynamics Gate: wraps the two Streams processors (envelope, vactrol,
// follower, compressor, filter controller and the Lorenz generator easter
// egg) exactly as streams.cc and ui.cc drive them: one sample at a time at
// the ADC rate (31.089 kHz), pots read through the firmware's 1 kHz
// thresholded events, buttons with short/long/double presses and the
// "secret handshake", the 8-LED meter/function display.
//
// On the hardware the firmware only computes two control signals per channel
// (a VCA gain DAC code and a PWM filter cutoff); the audio path is analog: a
// 2-pole low-pass filter and a VCA whose response is blended between linear
// and exponential by the RESPONSE knob, with the LEVEL CV input (through the
// LEVEL MOD attenuverter) summed into the VCA control. That analog stage is
// modelled here at the engine rate, using the calibration constants found in
// the firmware:
//   * linear response: gain = code / 32767 (unity at 32767, +6 dB at 65535),
//   * exponential response: 1.55 dB per 256 codes around unity (compressor.cc),
//     limited to +18 dB (gain.h),
//   * cutoff: exponential in the PWM code, ~10 Hz..20 kHz (model),
//   * LEVEL input normalled to +8 V, so LEVEL MOD alone opens the VCA (needed
//     in filter-controller mode, where the firmware leaves the gain DAC at 0).
// Inputs are 8 V full scale for the converters (kSchmittTriggerThreshold in
// gain.h is 3.33 V expressed on that scale).

#include <atomic>
#include <cmath>
#include <cstring>
#include <string>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "streams/audio_cv_meter.h"
#include "streams/gain.h"
#include "streams/processor.h"
#include "streams/resources.h"
#include "ui/canvas.h"

namespace br {
namespace {

constexpr float kNativeRate = 31089.f;  // drivers/adc.cc
constexpr int kNumChannels = 2;
constexpr int32_t kLongPressDuration = 1000;

// ui.h (the header includes the STM32 drivers).
enum DisplayMode { DISPLAY_MODE_FUNCTION, DISPLAY_MODE_MONITOR, DISPLAY_MODE_MONITOR_FUNCTION };
enum MonitorMode {
  MONITOR_MODE_EXCITE_IN,
  MONITOR_MODE_VCA_CV,
  MONITOR_MODE_AUDIO_IN,
  MONITOR_MODE_OUTPUT,
  MONITOR_MODE_LAST
};
enum Switch { SWITCH_MODE_1, SWITCH_MODE_2, SWITCH_MONITOR, SWITCH_LAST };

const char* kFunctionNames[streams::PROCESSOR_FUNCTION_LAST] = {
    "Envelope", "Vactrol", "Follower", "Compressor", "Filter controller", "Lorenz generator"};
const char* kFunctionShort[streams::PROCESSOR_FUNCTION_LAST] = {"ENV", "VACTROL", "FOLLOW",
                                                               "COMP", "FILTER", "LORENZ"};

// Analog part of one channel: 2-pole low-pass (TPT state-variable filter)
// followed by the VCA.
struct AnalogChannel {
  float ic1 = 0.f, ic2 = 0.f;
  float g = 0.5f, a1 = 0.f, a2 = 0.f, a3 = 0.f;
  float gain = 0.f;

  void setCutoff(float hz, float sampleRate) {
    hz = std::min(hz, sampleRate * 0.45f);
    const float k = 1.41421356f;  // Butterworth
    g = std::tan(static_cast<float>(M_PI) * hz / sampleRate);
    a1 = 1.f / (1.f + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
  }

  float process(float in) {
    float v3 = in - ic2;
    float v1 = a1 * ic1 + a2 * v3;
    float v2 = ic2 + a2 * ic1 + a3 * v3;
    ic1 = 2.f * v1 - ic1;
    ic2 = 2.f * v2 - ic2;
    return clampf(v2 * gain, -11.f, 11.f);  // op-amp rails
  }
};

class DualDynamicsGate : public Module {
 public:
  enum ParamIds {
    SHAPE1_PARAM,
    MOD1_PARAM,
    SHAPE2_PARAM,
    MOD2_PARAM,
    LEVEL_MOD1_PARAM,  // analog: LEVEL CV attenuverter
    LEVEL_MOD2_PARAM,
    RESPONSE1_PARAM,   // analog: VCA exponential (0) .. linear (1)
    RESPONSE2_PARAM,
    MODE1_BUTTON,
    MODE2_BUTTON,
    METER_BUTTON,
    FUNCTION1_PARAM,   // hidden
    FUNCTION2_PARAM,   // hidden
    ALTERNATE1_PARAM,  // hidden
    ALTERNATE2_PARAM,  // hidden
    LINKED_PARAM,      // hidden
    MONITOR_PARAM,     // hidden
    NUM_PARAMS
  };
  enum InputIds { EXCITE1_INPUT, IN1_INPUT, LEVEL1_INPUT, EXCITE2_INPUT, IN2_INPUT, LEVEL2_INPUT,
                  NUM_INPUTS };
  enum OutputIds { OUT1_OUTPUT, OUT2_OUTPUT, NUM_OUTPUTS };
  enum LightIds { LED_LIGHT = 0, NUM_LIGHTS = 8 * 3 };  // 8 LEDs (top to bottom), R G B

  DualDynamicsGate() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < 2; ++i) {
      std::string n = std::to_string(i + 1);
      configParam(SHAPE1_PARAM + 2 * i, 0.f, 1.f, 0.5f, "Shape " + n, "%", 100.f);
      configParam(MOD1_PARAM + 2 * i, 0.f, 1.f, 0.5f, "Mod " + n, "%", 100.f);
      configParam(LEVEL_MOD1_PARAM + i, -1.f, 1.f, 0.f, "Level mod " + n, "%", 100.f);
      configParam(RESPONSE1_PARAM + i, 0.f, 1.f, 0.5f, "Response " + n + " (exp..lin)", "%",
                  100.f);
      configButton(MODE1_BUTTON + i, "Function " + n + " (hold: alternate)");
      std::vector<std::string> names(kFunctionNames, kFunctionNames + 6);
      configSwitch(FUNCTION1_PARAM + i, 0.f, 5.f, 0.f, "Function " + n, names).hidden = true;
      configSwitch(ALTERNATE1_PARAM + i, 0.f, 1.f, 0.f, "Alternate function " + n + " (red)",
                   {"Off", "On"}).hidden = true;
      configInput(EXCITE1_INPUT + 3 * i, "Excite " + n);
      configInput(IN1_INPUT + 3 * i, "In " + n);
      configInput(LEVEL1_INPUT + 3 * i, "Level " + n);
      configOutput(OUT1_OUTPUT + i, "Out " + n);
    }
    configButton(METER_BUTTON, "Meter (monitor mode)");
    configSwitch(LINKED_PARAM, 0.f, 1.f, 0.f, "Linked channels", {"Off", "On"}).hidden = true;
    configSwitch(MONITOR_PARAM, 0.f, 3.f, 3.f, "Meter shows",
                 {"Excite in", "VCA CV", "Audio in", "Output"}).hidden = true;

    // Ui::Init() with fresh settings.
    // The firmware's DSP objects are zero-initialised globals: do the same
    // (Init() doesn't set every member).
    std::memset(static_cast<void*>(processor_), 0, sizeof(processor_));
    std::memset(static_cast<void*>(meter_), 0, sizeof(meter_));
    for (int i = 0; i < kNumChannels; ++i) {
      processor_[i].Init(static_cast<uint8_t>(i));
      meter_[i].Init();
      processor_[i].set_alternate(false);
      processor_[i].set_linked(false);
      processor_[i].set_function(streams::PROCESSOR_FUNCTION_ENVELOPE);
      displayMode_[i] = DISPLAY_MODE_MONITOR;
    }
    monitorMode_ = MONITOR_MODE_OUTPUT;
    publishState();
    onSampleRateChange(48000.f);
  }

  void onSampleRateChange(float sampleRate) override {
    sampleRate_ = sampleRate;
    step_ = kNativeRate / sampleRate;
  }

  void appendMenu(std::vector<MenuAction>& items) override {
    // On the hardware: a "secret handshake" on the two function buttons.
    items.push_back({"Lorenz generator (easter egg)", nullptr,
                     [this](int dir) { if (dir == 0) lorenzRequest_.store(true); }});
  }

  int displayFunction(int i) const { return processor_[i].function(); }
  bool displayAlternate(int i) const { return processor_[i].alternate(); }

  void process(const ProcessArgs& args) override {
    syncHiddenParams();
    if (lorenzRequest_.exchange(false)) startLorenz();

    // UI: SysTick at 4 kHz.
    uiPhase_ += args.sampleTime * 4000.f;
    while (uiPhase_ >= 1.f) {
      uiPhase_ -= 1.f;
      pollUi();
    }

    // Firmware Process() at the ADC rate; control outputs are interpolated.
    phase_ += step_;
    while (phase_ >= 1.f) {
      phase_ -= 1.f;
      nativeSample();
    }
    for (int i = 0; i < kNumChannels; ++i) {
      float code = prevGain_[i] + (curGain_[i] - prevGain_[i]) * phase_;
      float freq = prevFreq_[i] + (curFreq_[i] - prevFreq_[i]) * phase_;
      // PWM ripple filter (RC) on the cutoff control.
      freqLp_[i] += (freq - freqLp_[i]) * 0.12f;
      updateAnalog(i, code, freqLp_[i]);
      float in = inputs[static_cast<size_t>(IN1_INPUT + 3 * i)].getVoltage();
      outputs[static_cast<size_t>(OUT1_OUTPUT + i)].setVoltage(analog_[i].process(in));
    }
  }

 private:
  static int16_t adcSample(float volts) {
    int32_t v = static_cast<int32_t>(volts / 8.f * 32768.f);
    return static_cast<int16_t>(std::min(32767, std::max(-32768, v)));
  }

  // streams.cc Process().
  void nativeSample() {
    for (int i = 0; i < kNumChannels; ++i) {
      audio_[i] = adcSample(inputs[static_cast<size_t>(IN1_INPUT + 3 * i)].getVoltage());
      excite_[i] = adcSample(inputs[static_cast<size_t>(EXCITE1_INPUT + 3 * i)].getVoltage());
      uint16_t gain = 0;
      uint16_t frequency = 65535;
      processor_[i].Process(audio_[i], excite_[i], &gain, &frequency);
      prevGain_[i] = curGain_[i];
      prevFreq_[i] = curFreq_[i];
      curGain_[i] = static_cast<float>(gain);
      curFreq_[i] = static_cast<float>(frequency);
    }
  }

  // VCA control node: DAC code + LEVEL CV through the LEVEL MOD attenuverter.
  void updateAnalog(int i, float code, float frequency) {
    const Port& level = inputs[static_cast<size_t>(LEVEL1_INPUT + 3 * i)];
    float levelV = level.isConnected() ? clampf(level.getVoltage(), -8.f, 8.f) : 8.f;
    float total = code + params[static_cast<size_t>(LEVEL_MOD1_PARAM + i)].value * levelV / 8.f *
                             static_cast<float>(streams::kUnityGain);
    total = clampf(total, 0.f, 65535.f);
    float gLin = total / static_cast<float>(streams::kUnityGain);
    float db = 1.55f * (total - static_cast<float>(streams::kUnityGain)) / 256.f;
    float gExp = db < -120.f ? 0.f : std::pow(10.f, std::min(db, 18.f) / 20.f);
    float r = params[static_cast<size_t>(RESPONSE1_PARAM + i)].value;
    analog_[i].gain = gExp + (gLin - gExp) * r;
    vcaDb_[i] = 20.f * std::log10(std::max(analog_[i].gain, 1e-6f));
    // Cutoff: exponential in the PWM code.
    if (std::fabs(frequency - lastFreq_[i]) > 2.f) {
      lastFreq_[i] = frequency;
      analog_[i].setCutoff(10.f * std::exp2(11.f * frequency / 65535.f), sampleRate_);
    }
  }

  // --- ui.cc ---------------------------------------------------------------

  void publishState() {
    for (int i = 0; i < kNumChannels; ++i) {
      shownFunction_[i] = processor_[i].function();
      shownAlternate_[i] = processor_[i].alternate();
      params[static_cast<size_t>(FUNCTION1_PARAM + i)].set(static_cast<float>(shownFunction_[i]));
      params[static_cast<size_t>(ALTERNATE1_PARAM + i)].set(shownAlternate_[i] ? 1.f : 0.f);
    }
    shownLinked_ = processor_[0].linked();
    shownMonitor_ = monitorMode_;
    params[LINKED_PARAM].set(shownLinked_ ? 1.f : 0.f);
    params[MONITOR_PARAM].set(static_cast<float>(monitorMode_));
  }

  // Applies menu edits and patch loading of the hidden settings.
  void syncHiddenParams() {
    bool changed = false;
    bool linked = params[LINKED_PARAM].value > 0.5f;
    if (linked != shownLinked_) {
      for (int i = 0; i < kNumChannels; ++i) processor_[i].set_linked(linked);
      if (linked) link(0);
      changed = true;
    }
    int firstChanged = -1;
    for (int i = 0; i < kNumChannels; ++i) {
      int f = static_cast<int>(params[static_cast<size_t>(FUNCTION1_PARAM + i)].value + 0.5f);
      bool alt = params[static_cast<size_t>(ALTERNATE1_PARAM + i)].value > 0.5f;
      bool channelChanged = false;
      if (alt != shownAlternate_[i]) {
        processor_[i].set_alternate(alt);
        channelChanged = true;
      }
      if (f != shownFunction_[i]) {
        processor_[i].set_function(static_cast<streams::ProcessorFunction>(f));
        channelChanged = true;
      }
      if (channelChanged && firstChanged < 0) firstChanged = i;
    }
    if (firstChanged >= 0) {
      link(static_cast<uint8_t>(firstChanged));
      changed = true;
    }
    int m = static_cast<int>(params[MONITOR_PARAM].value + 0.5f);
    if (m != shownMonitor_) {
      monitorMode_ = static_cast<MonitorMode>(m);
      changed = true;
    }
    if (changed) {
      configureProcessors();
      publishState();
    }
  }

  void configureProcessors() {
    for (int i = 0; i < kNumChannels; ++i) processor_[i].Configure();
  }

  void startLorenz() {
    for (int i = 0; i < kNumChannels; ++i) {
      processor_[i].set_alternate(false);
      processor_[i].set_function(streams::PROCESSOR_FUNCTION_LORENZ_GENERATOR);
    }
    configureProcessors();
    publishState();
  }

  void link(uint8_t index) {
    if (processor_[0].linked()) {
      for (int i = 0; i < kNumChannels; ++i) {
        if (i != index) {
          displayMode_[i] = displayMode_[index];
          processor_[i].set_function(processor_[index].function());
          processor_[i].set_alternate(processor_[index].alternate());
        }
      }
    }
  }

  void pollUi() {
    ++divider_;
    if ((divider_ & 3) == 0) {
      ++ms_;
      bool events = false;
      for (int i = 0; i < SWITCH_LAST; ++i) {
        bool pressed = params[static_cast<size_t>(MODE1_BUTTON + i)].value > 0.5f;
        if (pressed && !wasPressed_[i]) {
          onSwitchPressed(i);
          pressTime_[i] = ms_;
          events = true;
        }
        if (pressed && pressTime_[i] != 0) {
          int32_t pressedTime = static_cast<int32_t>(ms_ - pressTime_[i]);
          if (pressedTime > kLongPressDuration) {
            onSwitchReleased(i, pressedTime);
            pressTime_[i] = 0;
            events = true;
          }
        }
        if (!pressed && wasPressed_[i] && pressTime_[i] != 0) {
          onSwitchReleased(i, static_cast<int32_t>(ms_ - pressTime_[i] + 1));
          pressTime_[i] = 0;
          events = true;
        }
        wasPressed_[i] = pressed;
      }
      // Pots: 16-bit, events when moving by more than 256 LSB.
      for (int i = 0; i < 4; ++i) {
        int32_t value = static_cast<int32_t>(
            clampf(params[static_cast<size_t>(SHAPE1_PARAM + i)].value, 0.f, 1.f) * 65535.f);
        if (value >= potValue_[i] + potThreshold_[i] || value <= potValue_[i] - potThreshold_[i] ||
            !potThreshold_[i]) {
          processor_[0].set_global(static_cast<uint16_t>(i), static_cast<uint16_t>(value));
          processor_[1].set_global(static_cast<uint16_t>(i), static_cast<uint16_t>(value));
          processor_[i >> 1].set_parameter(static_cast<uint16_t>(i & 1),
                                           static_cast<uint16_t>(value));
          potValue_[i] = value;
          potThreshold_[i] = 256;
          events = true;
          // The main loop handles events one at a time; configuring after
          // each one matters because set_global() clears the other
          // channel's dirty flag when the channels are not linked.
          configureProcessors();
        }
      }
      // DoEvents().
      if (events) lastEventMs_ = ms_;
      if (ms_ - lastEventMs_ > 1000) {
        lastEventMs_ = ms_;
        if (displayMode_[0] == DISPLAY_MODE_MONITOR_FUNCTION &&
            displayMode_[1] == DISPLAY_MODE_MONITOR_FUNCTION) {
          displayMode_[0] = displayMode_[1] = DISPLAY_MODE_MONITOR;
        }
        events = true;
      }
      if (events) configureProcessors();
    }
    paintLeds();
  }

  void onSwitchPressed(int id) {
    // Double press: link/unlink the two channels.
    if ((id == SWITCH_MODE_1 && pressTime_[SWITCH_MODE_2]) ||
        (id == SWITCH_MODE_2 && pressTime_[SWITCH_MODE_1])) {
      pressTime_[SWITCH_MODE_1] = pressTime_[SWITCH_MODE_2] = 0;
      bool linked = !processor_[0].linked();
      for (int i = 0; i < kNumChannels; ++i) {
        displayMode_[i] = DISPLAY_MODE_FUNCTION;
        processor_[i].set_linked(linked);
      }
      link(static_cast<uint8_t>(1 - id));
      publishState();
      return;
    }
    if (id == SWITCH_MONITOR) {
      if (displayMode_[0] == DISPLAY_MODE_MONITOR && displayMode_[1] == DISPLAY_MODE_MONITOR) {
        displayMode_[0] = displayMode_[1] = DISPLAY_MODE_MONITOR_FUNCTION;
      } else if (displayMode_[0] == DISPLAY_MODE_MONITOR_FUNCTION &&
                 displayMode_[1] == DISPLAY_MODE_MONITOR_FUNCTION) {
        monitorMode_ = static_cast<MonitorMode>((monitorMode_ + 1) % MONITOR_MODE_LAST);
        publishState();
      } else {
        displayMode_[0] = displayMode_[1] = DISPLAY_MODE_MONITOR;
      }
    }
  }

  void onSwitchReleased(int id, int32_t data) {
    // Secret handshake for the Lorenz generator easter egg.
    uint8_t code = static_cast<uint8_t>(id | (data >= kLongPressDuration ? 2 : 0));
    if ((secretHandshakeCounter_ & 3) == code) {
      ++secretHandshakeCounter_;
      if (secretHandshakeCounter_ == 16) {
        startLorenz();
        secretHandshakeCounter_ = 0;
        return;
      }
    } else {
      secretHandshakeCounter_ = 0;
    }

    if (data >= kLongPressDuration) {
      // (A long press on METER enters the offset calibration: not emulated.)
      if (id == SWITCH_MODE_1 || id == SWITCH_MODE_2) {
        processor_[id].set_alternate(!processor_[id].alternate());
        if (processor_[id].function() > streams::PROCESSOR_FUNCTION_COMPRESSOR) {
          processor_[id].set_function(streams::PROCESSOR_FUNCTION_ENVELOPE);
        }
        displayMode_[id] = DISPLAY_MODE_FUNCTION;
        int other = 1 - id;
        if (displayMode_[other] == DISPLAY_MODE_MONITOR_FUNCTION) {
          displayMode_[other] = DISPLAY_MODE_MONITOR;
        }
        link(static_cast<uint8_t>(id));
        publishState();
      }
    } else if (id == SWITCH_MODE_1 || id == SWITCH_MODE_2) {
      if (displayMode_[id] == DISPLAY_MODE_FUNCTION) {
        int index = processor_[id].function() + 1;
        int limit = processor_[id].alternate() ? streams::PROCESSOR_FUNCTION_FILTER_CONTROLLER
                                               : streams::PROCESSOR_FUNCTION_COMPRESSOR;
        if (index > limit) index = 0;
        processor_[id].set_function(static_cast<streams::ProcessorFunction>(index));
      } else {
        displayMode_[id] = DISPLAY_MODE_FUNCTION;
        int other = 1 - id;
        if (displayMode_[other] == DISPLAY_MODE_MONITOR_FUNCTION) {
          displayMode_[other] = DISPLAY_MODE_MONITOR;
        }
      }
      link(static_cast<uint8_t>(id));
      publishState();
    }
  }

  // --- LEDs (ui.cc PaintLeds() and drivers/leds.h) -------------------------

  void setLed(int index, int red, int green) {
    lights[static_cast<size_t>(3 * index)] = static_cast<float>(std::min(255, red)) / 255.f;
    lights[static_cast<size_t>(3 * index + 1)] = static_cast<float>(std::min(255, green)) / 255.f;
  }

  void paintBar(int start, int direction, int32_t db) {
    if (db < 0) return;
    if (db > 32767) db = 32767;
    db <<= 1;
    if (db >= 49152) {
      setLed(start, (db - 49152) >> 6, 0);
      setLed(start + direction, 255, 255);
      setLed(start + 2 * direction, 0, 255);
      setLed(start + 3 * direction, 0, 255);
    } else if (db >= 32768) {
      setLed(start + direction, (db - 32768) >> 6, (db - 32768) >> 6);
      setLed(start + 2 * direction, 0, 255);
      setLed(start + 3 * direction, 0, 255);
    } else if (db >= 16384) {
      setLed(start + 2 * direction, 0, (db - 16384) >> 6);
      setLed(start + 3 * direction, 0, 255);
    } else {
      setLed(start + 3 * direction, 0, db >> 6);
    }
  }

  void paintCv(int channel, int32_t cv) {
    int bank = channel * 4;
    bool flip = false;
    if (cv < 0) {
      flip = true;
      cv = -cv;
    }
    if (cv < 1024) cv = 0;
    if (cv > 32767) cv = 32767;
    for (int i = 0; i < 4; ++i) {
      int32_t residual = std::min(8191, std::max(0, cv - (3 - i) * 8192));
      if (flip) {
        setLed(bank + i, residual >> 5, 0);
      } else {
        setLed(bank + i, 0, residual >> 5);
      }
    }
  }

  // The VCA control node as the firmware's ADC sees it: 0 = fully open,
  // 65535 = about -99 dB (lut_2164_gain), times -2.
  int32_t gainSample(int channel) const {
    float raw = clampf(-vcaDb_[channel] / 99.f * 65536.f, 0.f, 65535.f);
    return -2 * static_cast<int32_t>(raw);
  }

  void paintAdaptive(int channel, int32_t sample, int32_t gain) {
    meter_[channel].Process(sample);
    if (meter_[channel].cv()) {
      sample = sample * streams::lut_2164_gain[std::min(255, -gain >> 9)] >> 15;
      paintCv(channel, sample * 5 >> 2);
    } else {
      paintBar(channel * 4, +1, streams::wav_db[meter_[channel].peak() >> 7] + gain);
    }
  }

  void paintMonitor(int channel) {
    switch (monitorMode_) {
      case MONITOR_MODE_EXCITE_IN:
        paintAdaptive(channel, excite_[channel], 0);
        break;
      case MONITOR_MODE_AUDIO_IN:
        paintAdaptive(channel, audio_[channel], 0);
        break;
      case MONITOR_MODE_VCA_CV:
        paintBar(channel * 4, +1, 32768 + gainSample(channel));
        break;
      case MONITOR_MODE_OUTPUT:
        if (processor_[channel].function() == streams::PROCESSOR_FUNCTION_COMPRESSOR) {
          paintBar(channel * 4 + 3, -1, -processor_[channel].gain_reduction());
        } else {
          paintAdaptive(channel, audio_[channel], gainSample(channel));
        }
        break;
      default:
        break;
    }
  }

  void paintLeds() {
    std::fill(lights.begin(), lights.end(), 0.f);
    for (int i = 0; i < kNumChannels; ++i) {
      int bank = i * 4;
      switch (displayMode_[i]) {
        case DISPLAY_MODE_FUNCTION: {
          bool alternate = processor_[i].alternate();
          int intensity = 255;
          if (processor_[i].linked()) {
            uint8_t phase = static_cast<uint8_t>(ms_ >> 1);
            phase = static_cast<uint8_t>(phase + i * 128);
            phase = phase < 128 ? phase : static_cast<uint8_t>(255 - phase);
            intensity = (phase * 224 >> 7) + 32;
            intensity = intensity * intensity >> 8;
          }
          int function = processor_[i].function();
          if (function == streams::PROCESSOR_FUNCTION_FILTER_CONTROLLER) {
            for (int j = 0; j < 4; ++j) {
              setLed(bank + j, alternate ? intensity : 0, alternate ? 0 : intensity);
            }
          } else if (function < streams::PROCESSOR_FUNCTION_LORENZ_GENERATOR) {
            setLed(bank + function, alternate ? intensity : 0, alternate ? 0 : intensity);
          } else {
            int index = (processor_[i].last_gain() >> 4) * 5 >> 4;
            if (index > 3) index = 3;
            int color = (processor_[i].last_frequency() - 128) * 2;
            if (color < 0) {
              if (color < -127) color = -127;
              setLed(bank + index, 255 + color * 2, 255);
            } else {
              if (color > 127) color = 127;
              setLed(bank + index, 255, 255 - color * 2);
            }
          }
          break;
        }
        case DISPLAY_MODE_MONITOR_FUNCTION: {
          int position = static_cast<int>(monitorMode_);
          setLed(position * 2, 255, 0);
          setLed(position * 2 + 1, 255, 0);
          break;
        }
        case DISPLAY_MODE_MONITOR:
          paintMonitor(i);
          break;
      }
    }
  }

  streams::Processor processor_[kNumChannels];
  streams::AudioCvMeter meter_[kNumChannels];
  AnalogChannel analog_[kNumChannels];
  int16_t audio_[kNumChannels] = {0, 0};
  int16_t excite_[kNumChannels] = {0, 0};
  float prevGain_[kNumChannels] = {}, curGain_[kNumChannels] = {};
  float prevFreq_[kNumChannels] = {65535.f, 65535.f}, curFreq_[kNumChannels] = {65535.f, 65535.f};
  float freqLp_[kNumChannels] = {65535.f, 65535.f};
  float lastFreq_[kNumChannels] = {-100.f, -100.f};
  float vcaDb_[kNumChannels] = {-120.f, -120.f};
  float sampleRate_ = 48000.f;
  float step_ = 0.65f;
  float phase_ = 0.f;

  // UI.
  DisplayMode displayMode_[kNumChannels];
  MonitorMode monitorMode_ = MONITOR_MODE_OUTPUT;
  int32_t potValue_[4] = {};
  int32_t potThreshold_[4] = {};
  uint32_t pressTime_[SWITCH_LAST] = {};
  bool wasPressed_[SWITCH_LAST] = {};
  uint8_t secretHandshakeCounter_ = 0;
  uint32_t divider_ = 0;
  uint32_t ms_ = 0;
  uint32_t lastEventMs_ = 0;
  float uiPhase_ = 0.f;

  int shownFunction_[kNumChannels] = {};
  bool shownAlternate_[kNumChannels] = {};
  bool shownLinked_ = false;
  int shownMonitor_ = MONITOR_MODE_OUTPUT;
  std::atomic<bool> lorenzRequest_{false};
};

void buildPanel(PanelSpec& p) {
  using M = DualDynamicsGate;
  p.hp = 12;
  p.bg = 0xFFF3F2EE;
  p.ink = 0xFF1E1E20;
  const uint32_t red = 0xFFC8414B, teal = 0xFF168A8C, white = 0xFFFFFFFF;
  const float lx = 10.9f, rx = 49.6f, cx = 30.2f;

  // Channel knobs.
  for (int i = 0; i < 2; ++i) {
    float x = i == 0 ? lx : rx;
    uint32_t c = i == 0 ? red : teal;
    p.knob(M::SHAPE1_PARAM + 2 * i, x, 20.8f, 10.f);
    p.label(x, 31.1f, "SHAPE", 3.2f);
    p.knob(M::MOD1_PARAM + 2 * i, x, 44.3f, 10.f);
    p.label(x, 54.5f, "MOD", 3.2f);
    p.knob(M::LEVEL_MOD1_PARAM + i, x, 67.7f, 8.f);
    p.box(x - 8.4f, 75.9f, 16.8f, 4.2f, c, 1.0f);
    p.label(x, 78.0f, "LEVEL MOD", 2.9f, kAlignCenter, white);
    p.label(i == 0 ? 3.6f : 57.0f, 59.1f, i == 0 ? "1" : "2", 3.2f, kAlignCenter, c);
  }

  // Function buttons, LED columns and legend.
  p.button(M::MODE1_BUTTON, 24.4f, 14.8f, 5.2f);
  p.button(M::MODE2_BUTTON, 36.0f, 14.8f, 5.2f);
  const float ly[4] = {21.6f, 27.4f, 33.2f, 38.9f};
  const char* legend[4] = {"ENV", "VAC", "FOL", "COMP"};
  for (int j = 0; j < 4; ++j) {
    int l1 = j, l2 = 4 + j;
    p.rgbLight(3 * l1, 3 * l1 + 1, 3 * l1 + 2, 24.4f, ly[j], 2.8f);
    p.rgbLight(3 * l2, 3 * l2 + 1, 3 * l2 + 2, 36.0f, ly[j], 2.8f);
    p.label(cx, ly[j], legend[j], 2.2f);
  }
  p.line(24.4f, 41.5f, 24.4f, 45.7f, 0.3f);
  p.line(36.0f, 41.5f, 36.0f, 45.7f, 0.3f);
  p.line(24.4f, 45.7f, 27.0f, 45.7f, 0.3f);
  p.line(33.4f, 45.7f, 36.0f, 45.7f, 0.3f);
  p.button(M::METER_BUTTON, cx, 45.7f, 4.6f);
  p.label(cx, 51.4f, "METER", 3.0f);

  // Response knobs (analog), exp..lin.
  auto arc = [&](float y, uint32_t c) {
    p.display(cx - 6.f, y - 6.f, 12.f, 12.f,
              [c](Canvas& cv, float x, float yy, float w, float h, float s, const Module*) {
                cv.arc(x + w * 0.5f, yy + h * 0.5f, 5.4f * s, 0.8f * static_cast<float>(M_PI),
                       2.2f * static_cast<float>(M_PI), std::max(1.5f, 0.45f * s), c);
              });
    p.label(cx - 6.2f, y + 4.6f, "EXP", 2.2f, kAlignCenter, c);
    p.label(cx + 6.2f, y + 4.6f, "LIN", 2.2f, kAlignCenter, c);
  };
  arc(60.9f, red);
  p.knob(M::RESPONSE1_PARAM, cx, 60.9f, 8.f);
  arc(75.4f, teal);
  p.knob(M::RESPONSE2_PARAM, cx, 75.4f, 8.f);

  p.line(3.2f, 85.5f, 57.6f, 85.5f, 0.6f, 0xFFC4C4C2);

  // Current function of each channel (the LEDs only show it after a press).
  for (int i = 0; i < 2; ++i) {
    float x = i == 0 ? lx : rx;
    p.display(x - 8.f, 10.0f, 16.f, 3.2f, [i](Canvas& c, float x0, float y0, float w, float h,
                                              float s, const Module* m) {
      const DualDynamicsGate* d = static_cast<const DualDynamicsGate*>(m);
      std::string t = kFunctionShort[d->displayFunction(i)];
      if (d->displayAlternate(i)) t += " ALT";
      c.text(x0 + w * 0.5f, y0 + h * 0.5f, t, std::max(12.f, 2.4f * s),
             d->displayAlternate(i) ? 0xFFC8414B : 0xFF168A8C, kAlignCenter);
    });
  }

  const float jx[4] = {8.4f, 22.9f, 37.5f, 52.2f};
  p.input(M::EXCITE1_INPUT, jx[0], 96.4f, "EXCITE", -6.6f);
  p.input(M::IN1_INPUT, jx[1], 96.4f, "IN", -6.6f);
  p.input(M::IN2_INPUT, jx[2], 96.4f, "IN", -6.6f);
  p.input(M::EXCITE2_INPUT, jx[3], 96.4f, "EXCITE", -6.6f);
  p.box(jx[0] - 6.0f, 102.2f, 12.0f, 4.0f, red, 1.0f);
  p.label(jx[0], 104.2f, "LEVEL", 2.9f, kAlignCenter, white);
  p.box(jx[3] - 6.0f, 102.2f, 12.0f, 4.0f, teal, 1.0f);
  p.label(jx[3], 104.2f, "LEVEL", 2.9f, kAlignCenter, white);
  p.input(M::LEVEL1_INPUT, jx[0], 110.8f);
  p.input(M::LEVEL2_INPUT, jx[3], 110.8f);
  for (int i = 0; i < 2; ++i) {
    float x = jx[1 + i];
    p.box(x - 4.95f, 102.2f, 9.9f, 14.5f, 0xFF5E5E60, 1.2f);
    p.label(x, 104.6f, "OUT", 2.9f, kAlignCenter, white);
    p.output(M::OUT1_OUTPUT + i, x, 110.8f);
  }
}

Model model = [] {
  Model m;
  m.slug = "dual-dynamics-gate";
  m.name = "Dual Dynamics Gate";
  m.shortName = "DYNAMICS GATE";
  m.basedOn = "Streams";
  m.category = Category::Utility;
  m.description = "Two VCA/low-pass gates driven by envelopes, vactrol, follower, compressor.";
  m.create = [] { return new DualDynamicsGate(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
