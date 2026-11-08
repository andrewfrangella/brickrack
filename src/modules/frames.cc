// BrickRack - GPL-3.0-or-later.
//
// Keyframer / Mixer: wraps the Frames keyframer and poly-LFO
// (frames/keyframer.cc, frames/poly_lfo.cc) and drives them like frames.cc
// and ui.cc do: the main loop evaluates the keyframer at its ~24 kHz refresh
// rate (the rate the poly-LFO increment table is computed for), the UI runs
// at 1 kHz (thresholded pot events, add/delete with long and very long
// presses, response/easing edit modes, the sequencer and poly-LFO easter
// eggs), and the frame-step trigger is generated like the 128 kHz timer
// interrupt does (1 ms pulses).
//
// The channel outputs on the hardware are 2164 VCAs controlled by the DAC
// codes. They are modelled with the converter law the firmware's lookup
// tables are built from (resources/lookup_tables.py: the control voltage is
// code / 65535 * 2.5 V and gain = 10^(-1.5 * V)). Unpatched channel inputs
// take the ALL input, which gets +10 V added when the OFFSET switch is on, so
// with nothing patched the outputs are 0..10 V CVs. MIX is the sum of the
// four channel outputs.
//
// Keyframes, channel settings and the easter-egg modes are saved in the patch
// (the hardware keeps them in flash).

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "frames/keyframer.h"
#include "frames/poly_lfo.h"
#include "ui/canvas.h"

namespace br {
namespace {

using frames::kMaxNumKeyframe;
constexpr int kNumCh = frames::kNumChannels;
constexpr float kRefreshRate = 24000.f;

// ui.h / ui.cc
enum SwitchIndex { SWITCH_ADD_FRAME, SWITCH_DELETE_FRAME, kNumSwitches };
enum UiMode {
  UI_MODE_SPLASH,
  UI_MODE_NORMAL,
  UI_MODE_SAVE_CONFIRMATION,
  UI_MODE_ERASE_CONFIRMATION,
  UI_MODE_EDIT_RESPONSE,
  UI_MODE_EDIT_EASING,
};
const uint16_t kAdcThreshold = 1 << (16 - 10);  // 10 bits
const int32_t kLongPressDuration = 800;
const int32_t kVeryLongPressDuration = 3000;
const uint16_t kKeyframeGridTolerance = 2048;
const int kFrameAdcChannel = 4;
const int kNumAdcChannels = 6;

const char* kEasingNames[6] = {"Step", "Linear", "Quartic in", "Quartic out", "Sine", "Bounce"};
const char* kEasingShort[6] = {"STEP", "LINEAR", "QUART IN", "QUART OUT", "SINE", "BOUNCE"};

// Snapshot of the persistent keyframer data, exchanged with the UI thread.
struct Snapshot {
  uint16_t numKeyframes = 0;
  uint16_t idCounter = 0;
  frames::Keyframe keyframes[kMaxNumKeyframe];
  bool polyLfo = false;
  bool sequencer = false;
};

class Keyframer : public Module {
 public:
  enum ParamIds {
    CHANNEL1_PARAM,
    FRAME_PARAM = CHANNEL1_PARAM + kNumCh,
    MODULATION_PARAM,
    ADD_BUTTON,
    DELETE_BUTTON,
    OFFSET_PARAM,
    EASING1_PARAM,                    // hidden
    RESPONSE1_PARAM = EASING1_PARAM + kNumCh,  // hidden
    NUM_PARAMS = RESPONSE1_PARAM + kNumCh
  };
  enum InputIds { ALL_INPUT, IN1_INPUT, FRAME_INPUT = IN1_INPUT + kNumCh, NUM_INPUTS };
  enum OutputIds { MIX_OUTPUT, OUT1_OUTPUT, FRAME_STEP_OUTPUT = OUT1_OUTPUT + kNumCh,
                   NUM_OUTPUTS };
  enum LightIds { CHANNEL1_LIGHT, KEYFRAME_LIGHT = CHANNEL1_LIGHT + kNumCh, RGB_LIGHT,
                  NUM_LIGHTS = RGB_LIGHT + 3 };

  Keyframer() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < kNumCh; ++i) {
      std::string n = std::to_string(i + 1);
      configParam(CHANNEL1_PARAM + i, 0.f, 1.f, 0.f, "Channel " + n, "%", 100.f);
      configSwitch(EASING1_PARAM + i, 0.f, 5.f, 1.f, "Channel " + n + " easing",
                   std::vector<std::string>(kEasingNames, kEasingNames + 6)).hidden = true;
      configParam(RESPONSE1_PARAM + i, 0.f, 1.f, 0.f, "Channel " + n + " response (lin..exp)",
                  "%", 100.f).hidden = true;
      configInput(IN1_INPUT + i, "In " + n);
      configOutput(OUT1_OUTPUT + i, "Out " + n);
    }
    configParam(FRAME_PARAM, 0.f, 1.f, 0.f, "Frame", "%", 100.f);
    configParam(MODULATION_PARAM, -1.f, 1.f, 1.f, "Frame modulation", "%", 100.f);
    configButton(ADD_BUTTON, "Add keyframe (hold: easing, 3 s: save)");
    configButton(DELETE_BUTTON, "Delete keyframe (hold: response, 3 s: clear all)");
    configSwitch(OFFSET_PARAM, 0.f, 1.f, 1.f, "+10 V offset on ALL", {"Off", "On"});
    configInput(ALL_INPUT, "All channels");
    configInput(FRAME_INPUT, "Frame modulation");
    configOutput(MIX_OUTPUT, "Mix");
    configOutput(FRAME_STEP_OUTPUT, "Frame step trigger");

    // frames.cc Init(). Keyframer::Init() does nothing in TEST builds (it
    // only loads flash), so apply the firmware's defaults here.
    // The firmware's DSP objects are zero-initialised globals: do the same
    // (Init() doesn't set every member).
    std::memset(static_cast<void*>(&keyframer_), 0, sizeof(keyframer_));
    std::memset(static_cast<void*>(&polyLfo_), 0, sizeof(polyLfo_));
    keyframer_.Init();
    keyframer_.Clear();
    for (int i = 0; i < kNumCh; ++i) {
      keyframer_.mutable_settings(static_cast<uint8_t>(i))->easing_curve =
          frames::EASING_CURVE_LINEAR;
      keyframer_.mutable_settings(static_cast<uint8_t>(i))->response = 0;
      keyframer_.set_immediate(static_cast<uint8_t>(i), 0);
    }
    keyframer_.Save(0);
    keyframer_.Calibrate(32767);
    polyLfo_.Init();
    std::memset(adcValue_, 0, sizeof(adcValue_));
    std::memset(adcFilteredValue_, 0, sizeof(adcFilteredValue_));
    findNearestKeyframe();
    onSampleRateChange(48000.f);
    // Evaluate once now, so that the FRAME STEP edge detector below starts from
    // the real position / nearest keyframe instead of the never-evaluated
    // ones (which made it fire a spurious pulse at startup).
    refresh();
    std::copy(curGain_, curGain_ + kNumCh, prevGain_);
    previousPosition_ = keyframer_.position();
    previousNearestKeyframe_ = keyframer_.nearest_keyframe();
  }

  void onSampleRateChange(float sampleRate) override {
    controlStep_ = kRefreshRate / sampleRate;
    pulseSamples_ = static_cast<int>(sampleRate / 1000.f);  // 128 ticks at 128 kHz
  }

  void onReset() override {
    pending_.numKeyframes = 0;
    pending_.idCounter = 0;
    pending_.polyLfo = pending_.sequencer = false;
    pendingLoad_.store(true, std::memory_order_release);
  }

  void saveState(StateMap& s) const override {
    Snapshot snap;
    // Seqlock: retry if the audio thread edited the keyframes meanwhile.
    for (int attempt = 0; attempt < 100; ++attempt) {
      uint32_t s1 = editSeq_.load(std::memory_order_acquire);
      if (s1 & 1) continue;
      snap.numKeyframes = keyframer_.num_keyframes();
      for (int i = 0; i < snap.numKeyframes && i < kMaxNumKeyframe; ++i) {
        snap.keyframes[i] = keyframer_.keyframe(static_cast<uint16_t>(i));
      }
      snap.idCounter = idCounter_;
      snap.polyLfo = polyLfoMode_;
      snap.sequencer = sequencerMode_;
      if (editSeq_.load(std::memory_order_acquire) == s1) break;
    }
    std::string text = std::to_string(snap.numKeyframes);
    char buf[64];
    for (int i = 0; i < snap.numKeyframes; ++i) {
      const frames::Keyframe& k = snap.keyframes[i];
      std::snprintf(buf, sizeof(buf), " %u:%u:%u:%u:%u:%u", k.timestamp, k.id, k.values[0],
                    k.values[1], k.values[2], k.values[3]);
      text += buf;
    }
    s["keyframes"] = text;
    s["idCounter"] = std::to_string(snap.idCounter);
    s["polyLfo"] = snap.polyLfo ? "1" : "0";
    s["sequencer"] = snap.sequencer ? "1" : "0";
  }

  void loadState(const StateMap& s) override {
    pending_.numKeyframes = 0;
    auto it = s.find("keyframes");
    if (it != s.end()) {
      const char* p = it->second.c_str();
      char* end = nullptr;
      long n = std::strtol(p, &end, 10);
      p = end;
      for (long i = 0; i < n && i < kMaxNumKeyframe; ++i) {
        unsigned v[6];
        int used = 0;
        if (std::sscanf(p, " %u:%u:%u:%u:%u:%u%n", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                        &used) != 6) {
          break;
        }
        p += used;
        frames::Keyframe& k = pending_.keyframes[pending_.numKeyframes++];
        k.timestamp = static_cast<uint16_t>(v[0]);
        k.id = static_cast<uint16_t>(v[1]);
        for (int c = 0; c < kNumCh; ++c) k.values[c] = static_cast<uint16_t>(v[2 + c]);
      }
    }
    it = s.find("idCounter");
    pending_.idCounter = it != s.end() ? static_cast<uint16_t>(std::atoi(it->second.c_str()))
                                       : pending_.numKeyframes;
    it = s.find("polyLfo");
    pending_.polyLfo = it != s.end() && it->second == "1";
    it = s.find("sequencer");
    pending_.sequencer = it != s.end() && it->second == "1";
    pendingLoad_.store(true, std::memory_order_release);
  }

  void appendMenu(std::vector<MenuAction>& items) override {
    // Hardware: press DELETE 10 times with the frame knob fully CCW.
    items.push_back({"Poly-LFO mode (easter egg)",
                     [this] { return std::string(polyLfoMode_ ? "ON" : "OFF"); },
                     [this](int) { menuToggle_.fetch_or(1); }});
    // Hardware: press ADD 5 times on an existing keyframe.
    items.push_back({"Sequencer mode (FRAME input steps)",
                     [this] { return std::string(sequencerMode_ ? "ON" : "OFF"); },
                     [this](int) { menuToggle_.fetch_or(2); }});
  }

  // For the panel readout.
  UiMode uiMode() const { return mode_; }
  int activeChannel() const { return activeChannel_; }
  bool polyLfoMode() const { return polyLfoMode_; }
  bool sequencerMode() const { return sequencerMode_; }
  int numKeyframes() const { return keyframer_.num_keyframes(); }
  int activeKeyframe() const { return activeKeyframe_; }
  int easing(int i) const { return keyframer_.mutable_settings(static_cast<uint8_t>(i)).easing_curve; }

  void process(const ProcessArgs& args) override {
    if (pendingLoad_.exchange(false, std::memory_order_acquire)) applyLoadedState();
    int toggles = menuToggle_.exchange(0);
    if (toggles & 1) polyLfoMode_ = !polyLfoMode_;
    if (toggles & 2) sequencerMode_ = !sequencerMode_;
    syncSettingsParams();

    uiPhase_ += args.sampleTime * 1000.f;
    while (uiPhase_ >= 1.f) {
      uiPhase_ -= 1.f;
      pollUi();
    }

    controlPhase_ += controlStep_;
    while (controlPhase_ >= 1.f) {
      controlPhase_ -= 1.f;
      refresh();
    }

    // TIM1 interrupt: frame step trigger.
    int16_t position = keyframer_.position();
    if (previousPosition_ != position) {
      previousPosition_ = position;
      if (canFireTrigger_) {
        pulseCounter_ = pulseSamples_;
        canFireTrigger_ = false;
      }
    }
    int16_t nearest = keyframer_.nearest_keyframe();
    if (previousNearestKeyframe_ != nearest) {
      previousNearestKeyframe_ = nearest;
      canFireTrigger_ = true;
    }
    outputs[FRAME_STEP_OUTPUT].setVoltage(pulseCounter_ > 0 ? 10.f : 0.f);
    if (pulseCounter_ > 0) --pulseCounter_;

    // Analog part: VCAs and mix.
    float all = inputs[ALL_INPUT].getVoltage() + (params[OFFSET_PARAM].value > 0.5f ? 10.f : 0.f);
    float mix = 0.f;
    for (int i = 0; i < kNumCh; ++i) {
      float g = prevGain_[i] + (curGain_[i] - prevGain_[i]) * controlPhase_;
      float out = inputs[static_cast<size_t>(IN1_INPUT + i)].getNormalVoltage(all) * g;
      outputs[static_cast<size_t>(OUT1_OUTPUT + i)].setVoltage(railLimit(out));
      mix += out;
    }
    outputs[MIX_OUTPUT].setVoltage(railLimit(mix));
  }

 private:
  // The output stages run from +-12 V rails: soft-limit to about +-10.5 V
  // (linear up to 8.5 V). The ALL input plus the +10 V OFFSET reaches 20 V.
  static float railLimit(float x) {
    const float a = std::fabs(x);
    if (a <= 8.5f) return x;
    const float y = 8.5f + 2.f * std::tanh((a - 8.5f) * 0.5f);
    return x < 0.f ? -y : y;
  }

  // 2164 law the firmware's tables assume.
  static float vcaGain(uint16_t dacCode12) {
    float control = static_cast<float>(dacCode12 << 4) / 65535.f;
    return std::pow(10.f, -3.75f * control);
  }

  // One pass of the firmware's main loop (frames.cc main()).
  void refresh() {
    int32_t frame = frameAdc();
    int32_t frameModulation = (frameModulationAdc() - keyframer_.dc_offset_frame_modulation()) * 2;
    frame += frameModulation;
    uint16_t codes[kNumCh];
    if (polyLfoMode_) {
      // A negative frame (pot near 0, negative FRAME CV) makes
      // PolyLfo::FrequencyToPhaseIncrement read lut_increments[-1] and shift
      // by a negative amount. The ADC clamps above bound the top end.
      polyLfo_.Render(std::min<int32_t>(2 * 65535, std::max<int32_t>(0, frame)));
      for (int i = 0; i < kNumCh; ++i) codes[i] = polyLfo_.dac_code(static_cast<uint8_t>(i));
    } else {
      frame = std::min<int32_t>(65535, std::max<int32_t>(0, frame));
      if (sequencerMode_) {
        // Detect a trigger on the FRAME input.
        if (frameModulation < 21845) triggerDetectorArmed_ = true;
        if (frameModulation > 43690 && triggerDetectorArmed_) {
          triggerDetectorArmed_ = false;
          ++sequencerStep_;
        }
        if (sequencerStep_ >= keyframer_.num_keyframes()) sequencerStep_ = 0;
        frame = keyframer_.keyframe(sequencerStep_).timestamp;
      }
      keyframer_.Evaluate(static_cast<uint16_t>(frame));
      for (int i = 0; i < kNumCh; ++i) codes[i] = keyframer_.dac_code(static_cast<uint8_t>(i));
    }
    for (int i = 0; i < kNumCh; ++i) {
      prevGain_[i] = curGain_[i];
      curGain_[i] = vcaGain(codes[i]);
    }
  }

  // ADC readings: pots are 16-bit; the FRAME CV input is bipolar +/-5 V
  // around the calibrated 0 V code (32767), and goes through the MODULATION
  // attenuverter.
  int32_t frameAdc() const {
    return static_cast<int32_t>(clampf(params[FRAME_PARAM].value, 0.f, 1.f) * 65535.f);
  }
  int32_t frameModulationAdc() const {
    float v = inputs[FRAME_INPUT].getVoltage() * params[MODULATION_PARAM].value;
    return static_cast<int32_t>(clampf(32767.f + v / 5.f * 32768.f, 0.f, 65535.f));
  }

  void applyLoadedState() {
    editSeq_.fetch_add(1, std::memory_order_acq_rel);
    keyframer_.Clear();
    for (int i = 0; i < pending_.numKeyframes; ++i) {
      frames::Keyframe& k = pending_.keyframes[i];
      keyframer_.AddKeyframe(k.timestamp, k.values);
    }
    // Restore the ids (they pick the keyframe colours) and the id counter
    // modulo the palette size.
    for (int i = 0; i < keyframer_.num_keyframes(); ++i) {
      frames::Keyframe* k = keyframer_.mutable_keyframe(static_cast<uint16_t>(i));
      for (int j = 0; j < pending_.numKeyframes; ++j) {
        if (pending_.keyframes[j].timestamp == k->timestamp) k->id = pending_.keyframes[j].id;
      }
    }
    idCounter_ = static_cast<uint16_t>(keyframer_.num_keyframes());
    alignIdCounter(pending_.idCounter);
    polyLfoMode_ = pending_.polyLfo;
    sequencerMode_ = pending_.sequencer;
    editSeq_.fetch_add(1, std::memory_order_acq_rel);
    findNearestKeyframe();
    syncWithPots();
  }

  // The keyframer's id counter is private; advance it (by adding and
  // removing a throwaway keyframe) until it matches modulo 8.
  void alignIdCounter(uint16_t target) {
    if (keyframer_.num_keyframes() >= kMaxNumKeyframe) return;
    uint16_t spare = 0;
    while (keyframer_.FindNearestKeyframe(spare, 1) != -1) ++spare;
    uint16_t zero[kNumCh] = {0, 0, 0, 0};
    while ((idCounter_ & 7) != (target & 7)) {
      keyframer_.AddKeyframe(spare, zero);
      keyframer_.RemoveKeyframe(spare);
      ++idCounter_;
    }
  }

  // Hidden easing/response params <-> keyframer settings.
  void syncSettingsParams() {
    for (int i = 0; i < kNumCh; ++i) {
      frames::ChannelSettings* s = keyframer_.mutable_settings(static_cast<uint8_t>(i));
      int e = static_cast<int>(params[static_cast<size_t>(EASING1_PARAM + i)].value + 0.5f);
      int r = static_cast<int>(params[static_cast<size_t>(RESPONSE1_PARAM + i)].value * 255.f + 0.5f);
      if (e != shownEasing_[i]) s->easing_curve = static_cast<frames::EasingCurve>(e);
      if (r != shownResponse_[i]) s->response = static_cast<uint8_t>(r);
      publishSettings(i);
    }
  }

  void publishSettings(int i) {
    const frames::ChannelSettings& s = *keyframer_.mutable_settings(static_cast<uint8_t>(i));
    if (s.easing_curve != shownEasing_[i]) {
      shownEasing_[i] = s.easing_curve;
      params[static_cast<size_t>(EASING1_PARAM + i)].set(static_cast<float>(s.easing_curve));
    }
    if (s.response != shownResponse_[i]) {
      shownResponse_[i] = s.response;
      params[static_cast<size_t>(RESPONSE1_PARAM + i)].set(static_cast<float>(s.response) / 255.f);
    }
  }

  // --- ui.cc, polled at 1 kHz ----------------------------------------------

  void findNearestKeyframe() {
    activeKeyframe_ = keyframer_.FindNearestKeyframe(static_cast<uint16_t>(frameAdc()),
                                                     kKeyframeGridTolerance);
  }

  void syncWithPots() {
    if (!keyframer_.num_keyframes()) {
      for (int i = 0; i < kNumCh; ++i) {
        keyframer_.set_immediate(static_cast<uint8_t>(i), adcFilteredValue_[i]);
      }
    }
  }

  void pollUi() {
    ++ms_;
    for (int i = 0; i < kNumSwitches; ++i) {
      bool pressed = params[static_cast<size_t>(ADD_BUTTON + i)].value > 0.5f;
      if (pressed && !wasPressed_[i]) {
        lastEventMs_ = ms_;
        pressTime_[i] = ms_;
        detectVeryLongPress_[i] = false;
      }
      if (pressed) {
        int32_t pressedTime = static_cast<int32_t>(ms_ - pressTime_[i]);
        if (!detectVeryLongPress_[i]) {
          if (pressedTime > kLongPressDuration) {
            onSwitchReleased(i, pressedTime);
            detectVeryLongPress_[i] = true;
          }
        } else if (pressedTime > kVeryLongPressDuration) {
          onSwitchReleased(i, pressedTime);
          detectVeryLongPress_[i] = false;
          pressTime_[i] = 0;
        }
      }
      if (!pressed && wasPressed_[i] && pressTime_[i] != 0 && !detectVeryLongPress_[i]) {
        onSwitchReleased(i, static_cast<int32_t>(ms_ - pressTime_[i] + 1));
        pressTime_[i] = 0;
        detectVeryLongPress_[i] = false;
      }
      wasPressed_[i] = pressed;
    }

    for (int i = 0; i <= kFrameAdcChannel; ++i) {
      int32_t raw = i == kFrameAdcChannel
                        ? frameAdc()
                        : static_cast<int32_t>(clampf(params[static_cast<size_t>(CHANNEL1_PARAM + i)].value,
                                                      0.f, 1.f) * 65535.f);
      int32_t value = (31 * adcFilteredValue_[i] + raw) >> 5;
      adcFilteredValue_[i] = static_cast<uint16_t>(value);
      int32_t current = adcValue_[i];
      if (value >= current + kAdcThreshold || value <= current - kAdcThreshold) {
        adcValue_[i] = static_cast<uint16_t>(value);
        onPotChanged(i, static_cast<uint16_t>(value));
      }
    }

    // DoEvents() idle handling.
    if (ms_ - lastEventMs_ > 500) {
      lastEventMs_ = ms_;
      if (mode_ == UI_MODE_SPLASH || mode_ == UI_MODE_SAVE_CONFIRMATION ||
          mode_ == UI_MODE_ERASE_CONFIRMATION) {
        mode_ = UI_MODE_NORMAL;
      }
      secretHandshakeCounter_ = 0;
    }
    updateLeds();
  }

  void beginEdit() { editSeq_.fetch_add(1, std::memory_order_acq_rel); }
  void endEdit() { editSeq_.fetch_add(1, std::memory_order_acq_rel); }

  void onSwitchReleased(int id, int32_t data) {
    lastEventMs_ = ms_;
    beginEdit();
    switch (id) {
      case SWITCH_ADD_FRAME:
        if (data > kVeryLongPressDuration) {
          // Save to flash: the patch keeps everything, show the confirmation.
          mode_ = UI_MODE_SAVE_CONFIRMATION;
        } else if (data > kLongPressDuration) {
          if (!polyLfoMode_) {
            mode_ = UI_MODE_EDIT_EASING;
            activeChannel_ = -1;
          }
        } else if (mode_ == UI_MODE_NORMAL && !polyLfoMode_) {
          if (activeKeyframe_ == -1) {
            if (keyframer_.AddKeyframe(static_cast<uint16_t>(frameAdc()), &adcValue_[0])) {
              ++idCounter_;
            }
          } else {
            ++secretHandshakeCounter_;
            if (secretHandshakeCounter_ >= 5) sequencerMode_ = !sequencerMode_;
          }
          findNearestKeyframe();
        } else {
          mode_ = UI_MODE_NORMAL;
        }
        break;

      case SWITCH_DELETE_FRAME:
        if (frameAdc() < 128) {
          --secretHandshakeCounter_;
          if (secretHandshakeCounter_ <= -10) {
            polyLfoMode_ = !polyLfoMode_;
            secretHandshakeCounter_ = 0;
          }
        }
        if (data > kVeryLongPressDuration) {
          keyframer_.Clear();
          idCounter_ = 0;
          findNearestKeyframe();
          syncWithPots();
          polyLfoMode_ = false;
          mode_ = UI_MODE_ERASE_CONFIRMATION;
        } else if (data > kLongPressDuration) {
          if (!polyLfoMode_) {
            mode_ = UI_MODE_EDIT_RESPONSE;
            activeChannel_ = -1;
          }
        } else if (mode_ == UI_MODE_NORMAL && !polyLfoMode_) {
          if (activeKeyframe_ != -1) {
            keyframer_.RemoveKeyframe(
                keyframer_.keyframe(static_cast<uint16_t>(activeKeyframe_)).timestamp);
          }
          findNearestKeyframe();
          syncWithPots();
        } else {
          mode_ = UI_MODE_NORMAL;
        }
        break;
    }
    endEdit();
  }

  void onPotChanged(int id, uint16_t data) {
    lastEventMs_ = ms_;
    if (polyLfoMode_) {
      switch (id) {
        case 0: polyLfo_.set_shape(data); break;
        case 1: polyLfo_.set_shape_spread(data); break;
        case 2: polyLfo_.set_spread(data); break;
        case 3: polyLfo_.set_coupling(data); break;
      }
      return;
    }
    if (id < kNumCh) {
      if (mode_ == UI_MODE_NORMAL || mode_ == UI_MODE_SPLASH) {
        if (activeKeyframe_ != -1) {
          beginEdit();
          keyframer_.mutable_keyframe(static_cast<uint16_t>(activeKeyframe_))->values[id] = data;
          endEdit();
        } else {
          keyframer_.set_immediate(static_cast<uint8_t>(id), data);
        }
      } else if (mode_ == UI_MODE_EDIT_RESPONSE) {
        activeChannel_ = id;
        keyframer_.mutable_settings(static_cast<uint8_t>(id))->response =
            static_cast<uint8_t>(data >> 8);
        publishSettings(id);
      } else if (mode_ == UI_MODE_EDIT_EASING) {
        activeChannel_ = id;
        keyframer_.mutable_settings(static_cast<uint8_t>(id))->easing_curve =
            static_cast<frames::EasingCurve>(data * 6 >> 16);
        publishSettings(id);
      }
    } else if (id == kFrameAdcChannel) {
      findNearestKeyframe();
    }
  }

  void setRgb(float r, float g, float b) {
    lights[RGB_LIGHT] = r;
    lights[RGB_LIGHT + 1] = g;
    lights[RGB_LIGHT + 2] = b;
  }

  void setChannelLed(int i, int value) {
    lights[static_cast<size_t>(CHANNEL1_LIGHT + i)] = static_cast<float>(value & 0xff) / 255.f;
  }

  void updateLeds() {
    switch (mode_) {
      case UI_MODE_SPLASH:
        animationCounter_ = static_cast<uint16_t>(animationCounter_ + 128);
        for (int i = 0; i < kNumCh; ++i) {
          setChannelLed(i, (animationCounter_ + (3 - i) * 16384) >> 8);
        }
        setRgb(1.f, 1.f, 1.f);
        lights[KEYFRAME_LIGHT] = 0.f;
        break;

      case UI_MODE_SAVE_CONFIRMATION:
      case UI_MODE_ERASE_CONFIRMATION:
        animationCounter_ = static_cast<uint16_t>(animationCounter_ - 256);
        for (int i = 0; i < kNumCh; ++i) {
          setChannelLed(i, (animationCounter_ + i * 16384) >> 8);
        }
        if (mode_ == UI_MODE_SAVE_CONFIRMATION) {
          setRgb(0.f, 1.f, 0.f);
        } else {
          setRgb(1.f, 0.f, 0.f);
        }
        break;

      case UI_MODE_NORMAL:
        for (int i = 0; i < kNumCh; ++i) {
          setChannelLed(i, keyframer_.level(static_cast<uint8_t>(i)) >> 8);
        }
        {
          const uint8_t* c = keyframer_.color();
          setRgb(c[0] / 255.f, c[1] / 255.f, c[2] / 255.f);
        }
        if (activeKeyframe_ == -1) {
          lights[KEYFRAME_LIGHT] = 0.f;
        } else {
          // The firmware PWMs the LED (16 steps) by the distance to the keyframe.
          int32_t distance = frameAdc() -
                             keyframer_.keyframe(static_cast<uint16_t>(activeKeyframe_)).timestamp;
          distance = std::min(distance * distance >> 18, int32_t(15));
          lights[KEYFRAME_LIGHT] = static_cast<float>(16 - distance) / 16.f;
        }
        if (polyLfoMode_) {
          for (int i = 0; i < kNumCh; ++i) setChannelLed(i, polyLfo_.level(static_cast<uint8_t>(i)));
          const uint8_t* c = polyLfo_.color();
          setRgb(c[0] / 255.f, c[1] / 255.f, c[2] / 255.f);
          lights[KEYFRAME_LIGHT] = polyLfo_.level(0) > 128 ? 1.f : 0.f;
        }
        break;

      case UI_MODE_EDIT_RESPONSE:
      case UI_MODE_EDIT_EASING: {
        animationCounter_ = static_cast<uint16_t>(animationCounter_ + 48);
        for (int i = 0; i < kNumCh; ++i) setChannelLed(i, activeChannel_ == i ? 255 : 0);
        uint16_t brightness =
            activeChannel_ == -1
                ? 65535
                : keyframer_.SampleAnimation(static_cast<uint8_t>(activeChannel_),
                                             animationCounter_, mode_ == UI_MODE_EDIT_EASING);
        float b = static_cast<float>(brightness) / 65536.f;
        if (mode_ == UI_MODE_EDIT_EASING) {
          setRgb(b, 16.f / 255.f * b, 32.f / 255.f * b);
        } else {
          setRgb(16.f / 255.f * b, 192.f / 255.f * b, 32.f / 255.f * b);
        }
        break;
      }
    }
  }

  frames::Keyframer keyframer_;
  frames::PolyLfo polyLfo_;

  // Main loop / trigger state.
  bool triggerDetectorArmed_ = false;
  uint16_t sequencerStep_ = 0;
  int16_t previousPosition_ = -2;
  int16_t previousNearestKeyframe_ = -2;
  bool canFireTrigger_ = false;
  int pulseCounter_ = 0;
  int pulseSamples_ = 48;
  float controlStep_ = 0.5f;
  float controlPhase_ = 0.f;
  float prevGain_[kNumCh] = {}, curGain_[kNumCh] = {};

  // UI state.
  UiMode mode_ = UI_MODE_SPLASH;
  uint16_t adcValue_[kNumAdcChannels];
  uint16_t adcFilteredValue_[kNumAdcChannels];
  uint32_t pressTime_[kNumSwitches] = {};
  bool detectVeryLongPress_[kNumSwitches] = {};
  bool wasPressed_[kNumSwitches] = {};
  int16_t activeKeyframe_ = -1;
  int activeChannel_ = -1;
  int secretHandshakeCounter_ = 0;
  uint16_t animationCounter_ = 0;
  bool polyLfoMode_ = false;
  bool sequencerMode_ = false;
  uint16_t idCounter_ = 0;  // mirrors the keyframer's private counter
  float uiPhase_ = 0.f;
  uint32_t ms_ = 0;
  uint32_t lastEventMs_ = 0;
  int shownEasing_[kNumCh] = {-1, -1, -1, -1};
  int shownResponse_[kNumCh] = {-1, -1, -1, -1};

  // UI thread handshakes.
  mutable std::atomic<uint32_t> editSeq_{0};
  Snapshot pending_;
  std::atomic<bool> pendingLoad_{false};
  std::atomic<int> menuToggle_{0};
};

void buildPanel(PanelSpec& p) {
  using M = Keyframer;
  p.hp = 18;
  p.bg = 0xFFF3F2EE;
  p.ink = 0xFF1E1E20;
  const uint32_t teal = 0xFF168A8C, white = 0xFFFFFFFF, grey = 0xFFB8B8B6;
  const float cx[4] = {11.3f, 34.2f, 57.0f, 79.6f};

  for (int i = 0; i < 4; ++i) {
    p.label(cx[i], 14.3f, std::to_string(i + 1), 3.4f);
    p.knob(M::CHANNEL1_PARAM + i, cx[i], 24.1f, 10.f);
    p.light(M::CHANNEL1_LIGHT + i, cx[i], 35.4f, 0xFF35E06A, 2.6f);
  }

  // Edit buttons and keyframe LED.
  p.button(M::ADD_BUTTON, 11.3f, 46.1f, 8.f);
  p.line(15.6f, 46.1f, 19.6f, 46.1f, 0.4f);
  p.label(20.3f, 46.1f, "ADD", 2.9f, kAlignLeft);
  p.button(M::DELETE_BUTTON, 11.3f, 62.7f, 8.f);
  p.line(15.6f, 62.7f, 19.6f, 62.7f, 0.4f);
  p.label(20.3f, 62.7f, "DEL", 2.9f, kAlignLeft);
  p.label(11.3f, 71.4f, "EDIT", 3.2f);
  p.light(M::KEYFRAME_LIGHT, 22.8f, 54.4f, 0xFFFFE9A8, 4.4f);

  // Frame knob, RGB frame LED and modulation attenuverter.
  p.rgbLight(M::RGB_LIGHT, M::RGB_LIGHT + 1, M::RGB_LIGHT + 2, 45.5f, 42.3f, 3.0f);
  p.knob(M::FRAME_PARAM, 45.5f, 55.2f, 16.f);
  p.label(45.5f, 75.2f, "FRAME", 3.4f);
  p.label(73.6f, 45.7f, "-", 3.4f);
  p.label(80.6f, 45.7f, "+", 3.4f);
  p.knob(M::MODULATION_PARAM, 77.1f, 54.4f, 8.f);
  p.box(67.6f, 63.6f, 19.0f, 4.4f, teal, 1.0f);
  p.label(77.1f, 65.8f, "MODULATION", 2.9f, kAlignCenter, white);

  // +10 V offset switch for the ALL input.
  p.toggleSwitch(M::OFFSET_PARAM, 9.2f, 82.6f);
  p.line(13.0f, 82.6f, 16.6f, 82.6f, 0.4f);
  p.label(17.2f, 82.6f, "+10V OFFSET", 2.9f, kAlignLeft);
  p.line(36.8f, 82.6f, 86.0f, 82.6f, 0.6f, grey);

  const float jx[6] = {9.2f, 23.9f, 38.4f, 52.8f, 67.3f, 81.7f};
  p.label(jx[0] + 3.4f, 89.4f, "ALL", 2.9f);
  for (int i = 0; i < 4; ++i) p.label(jx[1 + i], 89.4f, std::to_string(i + 1), 3.0f);
  p.box(jx[5] - 5.0f, 87.3f, 10.0f, 4.2f, teal, 1.0f);
  p.label(jx[5], 89.4f, "FRAME", 2.9f, kAlignCenter, white);
  p.input(M::ALL_INPUT, jx[0], 96.2f);
  for (int i = 0; i < 4; ++i) p.input(M::IN1_INPUT + i, jx[1 + i], 96.2f);
  p.input(M::FRAME_INPUT, jx[5], 96.2f);

  p.box(2.5f, 102.3f, 86.2f, 14.2f, 0xFF5E5E60, 1.4f);
  p.label(jx[0], 104.7f, "MIX", 2.9f, kAlignCenter, white);
  for (int i = 0; i < 4; ++i) {
    p.label(jx[1 + i], 104.7f, std::to_string(i + 1), 3.0f, kAlignCenter, white);
  }
  p.label(jx[5], 104.7f, "FR. STEP", 2.9f, kAlignCenter, white);
  p.output(M::MIX_OUTPUT, jx[0], 111.1f);
  for (int i = 0; i < 4; ++i) p.output(M::OUT1_OUTPUT + i, jx[1 + i], 111.1f);
  p.output(M::FRAME_STEP_OUTPUT, jx[5], 111.1f);

  // Status readout: edit modes, easter eggs, keyframe count.
  p.display(29.f, 66.0f, 33.f, 3.6f, [](Canvas& c, float x, float y, float w, float h, float s,
                                         const Module* m) {
    const Keyframer* k = static_cast<const Keyframer*>(m);
    std::string t;
    if (k->polyLfoMode()) {
      t = "POLY LFO";
    } else if (k->uiMode() == UI_MODE_EDIT_EASING) {
      t = k->activeChannel() < 0
              ? std::string("EDIT EASING")
              : "EASING " + std::to_string(k->activeChannel() + 1) + ": " +
                    kEasingShort[k->easing(k->activeChannel())];
    } else if (k->uiMode() == UI_MODE_EDIT_RESPONSE) {
      t = k->activeChannel() < 0 ? std::string("EDIT RESPONSE")
                                 : "RESPONSE " + std::to_string(k->activeChannel() + 1);
    } else {
      t = std::to_string(k->numKeyframes()) + " KEYFRAMES";
      if (k->sequencerMode()) t += "  SEQ";
    }
    c.text(x + w * 0.5f, y + h * 0.5f, t, std::max(12.f, 2.5f * s), 0xFF168A8C, kAlignCenter);
  });
}

Model model = [] {
  Model m;
  m.slug = "keyframer";
  m.name = "Keyframer / Mixer";
  m.shortName = "KEYFRAMER";
  m.basedOn = "Frames";
  m.category = Category::Modulator;
  m.description = "Four-channel keyframe animator and VCA mixer, with a poly-LFO mode.";
  m.create = [] { return new Keyframer(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
