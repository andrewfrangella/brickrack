// BrickRack - GPL-3.0-or-later.
//
// Segment Generator: wraps the Stages segment generators
// (stages/segment_generator.cc) and the "ouroboros" harmonic oscillator, driven
// the way stages.cc, cv_reader.cc, chain_state.cc and ui.cc do on the
// hardware: 31.25 kHz, 8-sample blocks.
//
// Chaining: the firmware's ChainState groups segments by gate patching (a
// patched gate input starts a group, the following unpatched segments join
// it) and also exchanges that state with neighbouring modules over a serial
// bus. Only the local part is reproduced here (a chain of one module): the
// grouping, loop and button logic below follow chain_state.cc line by line.
// Ouroboros mode (the firmware enters it when a module's chain loops back
// onto itself) is a hidden setting.
//
// Rate conversion: inputs (gates) are sampled at the 32 kHz-ish native clock
// like the hardware's GPIO reads; outputs are linearly interpolated between
// native samples. A windowed-sinc converter would ring on every gate edge and
// step (Stages outputs lots of them) and add ~1 ms of latency to gates.

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "stages/io_buffer.h"
#include "stages/oscillator.h"
#include "stages/segment_generator.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/hysteresis_quantizer.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/gate_flags.h"

namespace br {
namespace {

using stages::kBlockSize;
using stages::kNumChannels;
using stmlib::GateFlags;

constexpr int kNumCh = static_cast<int>(kNumChannels);

// chain_state.cc
constexpr uint16_t kUnpatchedInputDelay = 2000;
constexpr int16_t kLongPressDuration = 800;
constexpr int16_t kVeryLongPressDuration = 3000;
// ui.cc (ouroboros mode)
constexpr int kOuroborosLongPress = 1000;

const float kOuroborosRatios[] = {0.25f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f,
                                  4.0f,  5.0f, 6.0f, 8.0f, 8.0f};

enum LoopStatus { LOOP_STATUS_NONE, LOOP_STATUS_START, LOOP_STATUS_END, LOOP_STATUS_SELF };
enum Request {
  REQUEST_NONE,
  REQUEST_SET_ALT_SEGMENT_TYPE = 0xfd,
  REQUEST_SET_SEGMENT_TYPE = 0xfe,
  REQUEST_SET_LOOP = 0xff
};
struct RequestPacket {
  uint8_t request;
  uint8_t argument[4];
};
struct Loop {
  int8_t start;
  int8_t end;
};
struct ParameterBinding {
  size_t generator;
  size_t source;
  size_t destination;
};

class SegmentGen : public Module {
 public:
  enum ParamIds {
    SHAPE_PARAM = 0,
    LEVEL_PARAM = SHAPE_PARAM + kNumCh,      // the sliders
    TYPE_BUTTON = LEVEL_PARAM + kNumCh,
    TYPE_STATE = TYPE_BUTTON + kNumCh,       // hidden: segment type
    LOOP_STATE = TYPE_STATE + kNumCh,        // hidden: loop bit
    OUROBOROS_PARAM = LOOP_STATE + kNumCh,   // hidden
    NUM_PARAMS
  };
  enum InputIds { CV_INPUT = 0, GATE_INPUT = CV_INPUT + kNumCh, NUM_INPUTS = GATE_INPUT + kNumCh };
  enum OutputIds { ENVELOPE_OUTPUT = 0, NUM_OUTPUTS = kNumCh };
  enum LightIds {
    TYPE_LIGHT = 0,                          // R, G, B per segment
    SLIDER_LIGHT = TYPE_LIGHT + 3 * kNumCh,
    OUT_LIGHT = SLIDER_LIGHT + kNumCh,       // green, red per output
    NUM_LIGHTS = OUT_LIGHT + 2 * kNumCh
  };

  SegmentGen() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    for (int i = 0; i < kNumCh; ++i) {
      std::string n = std::to_string(i + 1);
      configParam(SHAPE_PARAM + i, 0.f, 1.f, 0.5f, "Shape/time " + n, "%", 100.f);
      configParam(LEVEL_PARAM + i, 0.f, 1.f, 0.5f, "Time/level " + n, "%", 100.f);
      configButton(TYPE_BUTTON + i, "Type " + n + " (hold: loop, hold 3 s: alt)");
      configSwitch(TYPE_STATE + i, 0.f, 3.f, 0.f, "Segment " + n + " type",
                   {"Ramp", "Step", "Hold", "Alt"}).hidden = true;
      configSwitch(LOOP_STATE + i, 0.f, 1.f, 0.f, "Segment " + n + " loop", {"Off", "On"})
          .hidden = true;
      configInput(CV_INPUT + i, "CV " + n);
      configInput(GATE_INPUT + i, "Gate " + n);
      configOutput(ENVELOPE_OUTPUT + i, "Out " + n);
    }
    configSwitch(OUROBOROS_PARAM, 0.f, 1.f, 0.f, "Ouroboros (harmonic oscillator)",
                 {"Off", "On"}).hidden = true;

    // stages.cc Init(). The firmware objects are zero-initialised globals on
    // the hardware; clear them so Init() never sees heap garbage.
    std::memset(static_cast<void*>(noteQuantizer_), 0, sizeof(noteQuantizer_));
    std::memset(static_cast<void*>(generator_), 0, sizeof(generator_));
    std::memset(static_cast<void*>(oscillator_), 0, sizeof(oscillator_));
    for (int i = 0; i < kNumCh; ++i) {
      noteQuantizer_[i].Init(13, 0.03f, false);
      generator_[i].Init(&noteQuantizer_[i]);
      oscillator_[i].Init();
    }
    std::fill(&noGate_[0], &noGate_[kBlockSize], stmlib::GATE_FLAG_LOW);
    std::memset(&block_, 0, sizeof(block_));
    std::memset(out_, 0, sizeof(out_));
    std::memset(outBuf_, 0, sizeof(outBuf_));
    // chain_state.cc Init(): flags 0xf0 (unknown module index) so that the
    // first update marks every channel dirty.
    std::fill(&flags_[0], &flags_[kNumCh], uint8_t(0xf0));
    // The firmware starts these at 0, which makes every input look patched
    // for 2 s after the boot-time neighbour discovery. Start "unpatched"
    // instead so that a freshly added module doesn't glitch.
    std::fill(&unpatchCounter_[0], &unpatchCounter_[kNumCh], kUnpatchedInputDelay);
    request_.request = REQUEST_NONE;
    onSampleRateChange(48000.f);
  }

  void onSampleRateChange(float sampleRate) override {
    step_ = stages::kSampleRate / sampleRate;
  }

  void process(const ProcessArgs& args) override {
    uiPhase_ += args.sampleTime * 1000.f;
    while (uiPhase_ >= 1.f) {
      uiPhase_ -= 1.f;
      pollUi();
    }

    phase_ += step_;
    while (phase_ >= 1.f) {
      phase_ -= 1.f;
      nativeSample();
    }
    for (int c = 0; c < kNumCh; ++c) {
      float v = prevOut_[c] + (curOut_[c] - prevOut_[c]) * phase_;
      outputs[static_cast<size_t>(ENVELOPE_OUTPUT + c)].setVoltage(v);
    }
  }

  // For the panel.
  uint8_t configuration(int i) const {
    int type = static_cast<int>(params[static_cast<size_t>(TYPE_STATE + i)].value + 0.5f);
    bool loop = params[static_cast<size_t>(LOOP_STATE + i)].value > 0.5f;
    return static_cast<uint8_t>((type & 3) | (loop ? 4 : 0));
  }

 private:
  bool ouroboros() const { return params[OUROBOROS_PARAM].value > 0.5f; }
  void setConfiguration(int i, uint8_t c) {
    params[static_cast<size_t>(TYPE_STATE + i)].set(static_cast<float>(c & 3));
    params[static_cast<size_t>(LOOP_STATE + i)].set((c & 4) ? 1.f : 0.f);
  }
  stages::segment::Configuration segmentConfiguration(int i) const {
    stages::segment::Configuration c;
    c.loop = flags_[i] & 0x04;
    c.type = stages::segment::Type(flags_[i] & 0x03);
    return c;
  }
  bool inputPatched(size_t i) const { return flags_[i] & 0x08; }

  // One native sample: FillBuffer() of the firmware.
  void nativeSample() {
    for (int c = 0; c < kNumCh; ++c) {
      gateTrigger_[c].process(inputs[static_cast<size_t>(GATE_INPUT + c)].getVoltage());
      gateFlags_[c] = stmlib::ExtractGateFlags(gateFlags_[c], gateTrigger_[c].isHigh());
      block_.input[c][pos_] = gateFlags_[c];
      prevOut_[c] = curOut_[c];
      curOut_[c] = outBuf_[c][pos_];
    }
    if (++pos_ == kBlockSize) {
      pos_ = 0;
      readCv();
      if (ouroboros()) {
        processOuroboros();
      } else {
        processSegments();
      }
    }
  }

  // cv_reader.cc Read(). CV inputs are calibrated so that 8 V = 1 unit (the
  // DAC/ADC loopback calibration makes inputs and outputs use the same unit),
  // which gives 1 V/oct in the oscillator modes.
  void readCv() {
    // The pots are multiplexed: one pot is converted per block.
    ONE_POLE(lpPot_[potIndex_], params[static_cast<size_t>(SHAPE_PARAM + potIndex_)].value, 0.1f);
    potIndex_ = (potIndex_ + 1) % kNumCh;
    for (int i = 0; i < kNumCh; ++i) {
      float cv = clampf(inputs[static_cast<size_t>(CV_INPUT + i)].getVoltage(), -10.f, 10.f) / 8.f;
      ONE_POLE(lpCv_[i], cv, 0.7f);
      ONE_POLE(lpCv2_[i], lpCv_[i], 0.7f);
      ONE_POLE(lpSlider_[i], params[static_cast<size_t>(LEVEL_PARAM + i)].value, 0.025f);
      float combined = lpCv2_[i] + lpSlider_[i];
      CONSTRAIN(combined, -1.0f, 1.999995f);
      block_.pot[i] = lpPot_[i];
      block_.cv_slider[i] = combined;
      block_.input_patched[i] = inputs[static_cast<size_t>(GATE_INPUT + i)].isConnected();
    }
  }

  // DAC: code = level * -32263 + 32768, clipped to 0..65531; 1 unit = 8 V.
  static float dacVolts(float level) {
    return clampf(level, -32763.f / 32263.f, 32768.f / 32263.f) * 8.f;
  }

  // stages.cc Process(), with ChainState::Update() for a chain of one module.
  void processSegments() {
    if (!configured_) {
      // Stands in for the neighbour discovery phase: configure right away.
      updateLocalState();
      configure();
      configured_ = true;
    }
    switch (counter_ & 0x3) {
      case 0:
        pollSwitches();
        updateLocalState();
        break;
      case 1:
        handleRequest();
        break;
      case 2:
        break;  // pot/CV/slider exchange with remote modules
      case 3:
        configure();
        break;
    }
    // BindLocalParameters().
    for (size_t i = 0; i < numBindings_; ++i) {
      const ParameterBinding& m = binding_[i];
      generator_[m.generator].set_segment_parameters(
          static_cast<int>(m.destination), block_.cv_slider[m.source], block_.pot[m.source]);
    }
    std::fill(&out_[0], &out_[kBlockSize], lastSample_);
    ++counter_;

    for (int channel = 0; channel < kNumCh; ++channel) {
      bool ledState = generator_[channel].Process(
          block_.input_patched[channel] ? block_.input[channel] : noGate_, out_, kBlockSize);
      if (ledState) sliderLedCounter_[channel] = 5;
      for (size_t i = 0; i < kBlockSize; ++i) outBuf_[channel][i] = dacVolts(out_[i].value);
    }
  }

  // stages.cc ProcessOuroboros().
  void processOuroboros() {
    using namespace stages;
    const size_t size = kBlockSize;
    const float coarse = (block_.cv_slider[0] - 0.5f) * 96.0f;
    const float fine = block_.pot[0] * 2.0f - 1.0f;
    // cv_slider spans -1..2, so coarse reaches +-144 semitones, past the ends
    // of lut_pitch_ratio_high. Constrain like SegmentGenerator::ProcessOscillator.
    float semitones = coarse + fine;
    CONSTRAIN(semitones, -128.0f, 127.0f);
    const float f0 = stmlib::SemitonesToRatio(semitones) * 261.6255f / kSampleRate;

    float sum[kBlockSize];
    float thisChannel[kBlockSize];
    std::fill(&sum[0], &sum[size], 0.0f);

    for (int channel = kNumCh - 1; channel >= 0; --channel) {
      const float harmonic = block_.pot[channel] * 9.999f;
      MAKE_INTEGRAL_FRACTIONAL(harmonic);
      harmonic_fractional = 8.0f * (harmonic_fractional - 0.5f) + 0.5f;
      CONSTRAIN(harmonic_fractional, 0.0f, 1.0f);
      const float ratio = channel == 0 ? 1.0f
                                       : stmlib::Crossfade(kOuroborosRatios[harmonic_integral],
                                                           kOuroborosRatios[harmonic_integral + 1],
                                                           harmonic_fractional);
      ONE_POLE(channelAmplitude_[channel],
               channel == 0 ? 1.0f : std::max(block_.cv_slider[channel] - 0.01f, 0.0f), 0.2f);
      const float amplitude = channelAmplitude_[channel];

      bool trigger = false;
      for (size_t i = 0; i < size; ++i) {
        trigger = trigger || (block_.input[channel][i] & stmlib::GATE_FLAG_RISING);
      }
      if (trigger || !block_.input_patched[channel]) {
        channelEnvelope_[channel] = 1.0f;
      } else {
        channelEnvelope_[channel] *= 0.999f;
      }
      if (channelEnvelope_[channel] * amplitude > 0.02f) sliderLedCounter_[channel] = 1;
      const float f = f0 * ratio;

      uint8_t waveshape = configuration(channel);
      Oscillator& osc = oscillator_[channel];
      switch (waveshape) {
        case 0: osc.Render<OSCILLATOR_SHAPE_SINE>(f, 0.5f, thisChannel, size); break;
        case 1: osc.Render<OSCILLATOR_SHAPE_TRIANGLE>(f, 0.5f, thisChannel, size); break;
        case 2:
        case 3: osc.Render<OSCILLATOR_SHAPE_SQUARE>(f, 0.5f, thisChannel, size); break;
        case 4: osc.Render<OSCILLATOR_SHAPE_SAW>(f, 0.5f, thisChannel, size); break;
        case 5: osc.Render<OSCILLATOR_SHAPE_SQUARE>(f, 0.75f, thisChannel, size); break;
        case 6:
        case 7: osc.Render<OSCILLATOR_SHAPE_SQUARE>(f, 0.9f, thisChannel, size); break;
      }

      stmlib::ParameterInterpolator am(&previousAmplitude_[channel],
                                       amplitude * amplitude * channelEnvelope_[channel], size);
      for (size_t i = 0; i < size; ++i) sum[i] += thisChannel[i] * am.Next();

      const float gain = channel == 0 ? 0.2f : 0.66f;
      const float* source = channel == 0 ? sum : thisChannel;
      for (size_t i = 0; i < size; ++i) outBuf_[channel][i] = dacVolts(source[i] * gain);
    }
  }

  // --- chain_state.cc, for a chain of one module (index 0, size 1) --------

  void updateLocalState() {
    patchedMask_ = 0;
    for (int i = 0; i < kNumCh; ++i) {
      if (block_.input_patched[i]) {
        unpatchCounter_[i] = 0;
      } else if (unpatchCounter_[i] < kUnpatchedInputDelay) {
        ++unpatchCounter_[i];
      }
      bool patched = unpatchCounter_[i] < kUnpatchedInputDelay;
      uint8_t newFlags = static_cast<uint8_t>(configuration(i) | (patched ? 0x08 : 0));
      dirty_[i] = newFlags != flags_[i];
      flags_[i] = newFlags;
      if (patched) patchedMask_ |= static_cast<uint8_t>(1 << i);
    }
  }

  void setLoopStatus(int channel, int segment, Loop loop) {
    if (segment == loop.start) {
      loopStatus_[channel] = segment == loop.end ? LOOP_STATUS_SELF : LOOP_STATUS_START;
    } else if (segment == loop.end) {
      loopStatus_[channel] = LOOP_STATUS_END;
    } else {
      loopStatus_[channel] = LOOP_STATUS_NONE;
    }
  }

  void configure() {
    const size_t lastChannel = kNumChannels;
    size_t lastPatchedChannel = kNumChannels;  // rx_last_patched_channel_ of module 0
    Loop lastLoop = {-1, -1};
    numBindings_ = 0;
    stages::segment::Configuration config[kNumChannels];

    for (size_t i = 0; i < kNumChannels; ++i) {
      size_t channel = i;
      if (!inputPatched(i)) {
        if (channel > lastPatchedChannel) {
          // Slave channel: extends the chain of segments started earlier.
          int segment = static_cast<int>(channel - lastPatchedChannel);
          generator_[i].ConfigureSlave(segment);
          setLoopStatus(static_cast<int>(i), segment, lastLoop);
        } else {
          // Free-running channel.
          stages::segment::Configuration c = segmentConfiguration(static_cast<int>(i));
          generator_[i].ConfigureSingleSegment(false, c);
          binding_[numBindings_++] = {i, i, 0};
          loopStatus_[i] = c.loop ? LOOP_STATUS_SELF : LOOP_STATUS_NONE;
        }
      } else {
        lastPatchedChannel = channel;
        int numSegments = 0;
        bool addMoreSegments = true;
        bool dirty = false;
        lastLoop.start = -1;
        lastLoop.end = -1;
        while (addMoreSegments) {
          stages::segment::Configuration c = segmentConfiguration(static_cast<int>(channel));
          config[numSegments] = c;
          dirty |= dirty_[channel];
          if (c.loop) {
            if (lastLoop.start == -1) lastLoop.start = static_cast<int8_t>(numSegments);
            lastLoop.end = static_cast<int8_t>(numSegments);
          }
          binding_[numBindings_++] = {i, i + static_cast<size_t>(numSegments),
                                      static_cast<size_t>(numSegments)};
          ++channel;
          ++numSegments;
          addMoreSegments = channel < lastChannel && !inputPatched(channel);
        }
        if (dirty || numSegments != generator_[i].num_segments()) {
          // The alt mode is only available for single segments.
          if (numSegments != 1) {
            for (int j = 0; j < numSegments; ++j) {
              if (config[j].type == stages::segment::TYPE_ALT) {
                config[j].type = stages::segment::TYPE_RAMP;
              }
            }
          }
          generator_[i].Configure(true, config, numSegments);
        }
        setLoopStatus(static_cast<int>(i), 0, lastLoop);
      }
    }
  }

  RequestPacket makeLoopChangeRequest(size_t loopStart, size_t loopEnd) {
    size_t channelIndex = 0;
    size_t groupStart = 0;
    size_t groupEnd = kNumChannels;
    bool inconsistentLoop = false;

    uint8_t patched = patchedMask_;
    for (size_t j = 0; j < kNumChannels; ++j) {
      if (patched & 1) {
        if (channelIndex <= loopStart) {
          groupStart = channelIndex;
        } else if (channelIndex >= loopEnd) {
          groupEnd = std::min(groupEnd, channelIndex);
        }
        // No patched channel between the loop start and the loop end.
        if (channelIndex > loopStart && channelIndex < loopEnd) inconsistentLoop = true;
      }
      patched >>= 1;
      ++channelIndex;
    }
    // No loop spanning several channels in the first group of unpatched channels.
    if (groupStart == 0 && !(patchedMask_ & 1)) {
      if (loopStart != loopEnd) {
        inconsistentLoop = true;
      } else {
        groupStart = groupEnd = loopStart = loopEnd;
      }
    }
    // A loop can end on a patched channel only in a single-channel group.
    if (groupEnd == loopEnd && groupStart != groupEnd) inconsistentLoop = true;

    RequestPacket result;
    if (inconsistentLoop) {
      result.request = REQUEST_NONE;
    } else {
      result.request = REQUEST_SET_LOOP;
      result.argument[0] = static_cast<uint8_t>(groupStart);
      result.argument[1] = static_cast<uint8_t>(loopStart);
      result.argument[2] = static_cast<uint8_t>(loopEnd);
      result.argument[3] = static_cast<uint8_t>(groupEnd);
    }
    return result;
  }

  void pollSwitches() {
    request_.request = REQUEST_NONE;
    size_t firstPressed = kNumChannels;
    for (size_t j = 0; j < kNumChannels; ++j) {
      bool pressed = params[TYPE_BUTTON + j].value > 0.5f;
      if (pressed) {
        if (switchPressTime_[j] != -1) {
          ++switchPressTime_[j];
          if (firstPressed != kNumChannels) {
            // Simultaneously pressing a pair of buttons.
            request_ = makeLoopChangeRequest(firstPressed, j);
            switchPressTime_[firstPressed] = -1;
            switchPressTime_[j] = -1;
          } else if (switchPressTime_[j] == kLongPressDuration) {
            request_ = makeLoopChangeRequest(j, j);
          } else if (switchPressTime_[j] > kVeryLongPressDuration) {
            request_ = makeLoopChangeRequest(j, j);
            const bool validLoop = request_.request != REQUEST_NONE;
            const bool singleSegment = request_.argument[0] == request_.argument[3] ||
                                       request_.argument[0] == (request_.argument[3] - 1);
            if (validLoop && singleSegment) request_.request = REQUEST_SET_ALT_SEGMENT_TYPE;
            switchPressTime_[j] = -1;
          } else {
            firstPressed = j;
          }
        }
      } else {
        if (switchPressTime_[j] > 5 && switchPressTime_[j] < kLongPressDuration) {
          request_.request = REQUEST_SET_SEGMENT_TYPE;
          request_.argument[0] = static_cast<uint8_t>(j);
        }
        switchPressTime_[j] = 0;
      }
    }
  }

  void handleRequest() {
    if (request_.request == REQUEST_NONE) return;
    for (int i = 0; i < kNumCh; ++i) {
      uint8_t c = configuration(i);
      uint8_t typeBits = c & 0x3;
      uint8_t loopBit = c & 0x4;
      size_t channel = static_cast<size_t>(i);
      if (request_.request == REQUEST_SET_ALT_SEGMENT_TYPE) {
        if (channel == request_.argument[0]) {
          typeBits = 0x3;
          loopBit = 0x4;
        }
      } else if (request_.request == REQUEST_SET_SEGMENT_TYPE) {
        if (channel == request_.argument[0]) typeBits = (typeBits + 1) % 3;
      } else if (request_.request == REQUEST_SET_LOOP) {
        uint8_t newLoopBit = loopBit;
        if (channel >= request_.argument[0] && channel < request_.argument[3]) {
          newLoopBit = 0x0;
          typeBits = typeBits % 3;
        }
        if (channel == request_.argument[1] || channel == request_.argument[2]) {
          if (request_.argument[1] == request_.argument[2]) {
            newLoopBit = static_cast<uint8_t>(0x4 - loopBit);
          } else {
            newLoopBit = 0x4;
          }
        }
        loopBit = newLoopBit;
      }
      uint8_t n = typeBits | loopBit;
      if (n != c) setConfiguration(i, n);
    }
    // In the firmware the request is consumed by the next exchange.
    request_.request = REQUEST_NONE;
  }

  // --- ui.cc, polled at 1 kHz ------------------------------------------------

  void pollUi() {
    ++ms_;
    if (ouroboros()) {
      for (int i = 0; i < kNumCh; ++i) {
        // Sanitize segment type in Ouroboros mode to disable alt type.
        uint8_t c = configuration(i);
        uint8_t sane = static_cast<uint8_t>((c & 0x4) | ((c & 0x3) % 3));
        if (sane != c) setConfiguration(i, sane);
        if (params[static_cast<size_t>(TYPE_BUTTON + i)].value > 0.5f) {
          if (pressTime_[i] != -1) ++pressTime_[i];
          if (pressTime_[i] > kOuroborosLongPress) {
            setConfiguration(i, static_cast<uint8_t>((sane & 0x3) | (4 - (sane & 0x4))));
            pressTime_[i] = -1;
          }
        } else {
          if (pressTime_[i] > 0) {
            setConfiguration(i, static_cast<uint8_t>((((sane & 0x3) + 1) % 3) | (sane & 0x4)));
          }
          pressTime_[i] = 0;
        }
      }
    }
    updateLeds();
  }

  uint8_t fadePattern(int shift, uint8_t phase) const {
    uint8_t x = static_cast<uint8_t>(ms_ >> shift);
    x = static_cast<uint8_t>(x + phase);
    x &= 0x1f;
    return x <= 0x10 ? x : static_cast<uint8_t>(0x1f - x);
  }

  void updateLeds() {
    // LED colours as (red, green): green, yellow, red.
    static const float kPalette[3][2] = {{0.f, 1.f}, {1.f, 1.f}, {1.f, 0.f}};
    const uint8_t fades[4] = {0xf, fadePattern(4, 0), fadePattern(4, 0x0f), fadePattern(4, 0x08)};
    for (int i = 0; i < kNumCh; ++i) {
      uint8_t c = configuration(i);
      int type = c & 0x3;
      int loopStatus = loopStatus_[i];
      if (ouroboros()) loopStatus = (c & 0x4) ? LOOP_STATUS_SELF : LOOP_STATUS_NONE;
      int brightness = fades[loopStatus];
      int color = type % 3;
      if (loopStatus == LOOP_STATUS_SELF && type == 3) {
        // Rainbow pattern for a looping alt segment.
        brightness = fadePattern(5, static_cast<uint8_t>(i));
        uint8_t rainbow = fadePattern(9, static_cast<uint8_t>(i * 2));
        int high = (rainbow >> 2) & 0x3;
        color = high == 3 ? 1 : high;
      }
      // The firmware PWMs the LED at 1/16 steps; show the duty cycle.
      float b = brightness == 0 ? 0.f : std::min(1.f, (brightness + 1) / 16.f);
      lights[static_cast<size_t>(TYPE_LIGHT + 3 * i)] = kPalette[color][0] * b;
      lights[static_cast<size_t>(TYPE_LIGHT + 3 * i + 1)] = kPalette[color][1] * b;
      lights[static_cast<size_t>(TYPE_LIGHT + 3 * i + 2)] = 0.f;
      lights[static_cast<size_t>(SLIDER_LIGHT + i)] = sliderLedCounter_[i] ? 1.f : 0.f;
      if (sliderLedCounter_[i]) --sliderLedCounter_[i];
      // Output LEDs follow the output voltage (hardware).
      float v = outputs[static_cast<size_t>(ENVELOPE_OUTPUT + i)].getVoltage() / 8.f;
      lights[static_cast<size_t>(OUT_LIGHT + 2 * i)] = std::max(0.f, v);
      lights[static_cast<size_t>(OUT_LIGHT + 2 * i + 1)] = std::max(0.f, -v);
    }
  }

  // DSP.
  stmlib::HysteresisQuantizer2 noteQuantizer_[kNumChannels];
  stages::SegmentGenerator generator_[kNumChannels];
  stages::Oscillator oscillator_[kNumChannels];
  stages::IOBuffer::Block block_;
  stages::SegmentGenerator::Output out_[kBlockSize];
  GateFlags noGate_[kBlockSize];
  float outBuf_[kNumChannels][kBlockSize];

  // cv_reader.cc state.
  float lpPot_[kNumChannels] = {};
  float lpSlider_[kNumChannels] = {};
  float lpCv_[kNumChannels] = {};
  float lpCv2_[kNumChannels] = {};
  int potIndex_ = 0;

  // ProcessOuroboros() state.
  float channelAmplitude_[kNumChannels] = {};
  float channelEnvelope_[kNumChannels] = {};
  float previousAmplitude_[kNumChannels] = {};

  // ChainState (single module).
  bool configured_ = false;
  uint32_t counter_ = 0;
  uint8_t flags_[kNumChannels];
  bool dirty_[kNumChannels] = {};
  uint8_t patchedMask_ = 0;
  uint16_t unpatchCounter_[kNumChannels] = {};
  int loopStatus_[kNumChannels] = {};
  int16_t switchPressTime_[kNumChannels] = {};
  RequestPacket request_;
  size_t numBindings_ = 0;
  ParameterBinding binding_[kNumChannels];
  // rx_last_sample_: what the module on the left sends; module 0 has none.
  stages::SegmentGenerator::Output lastSample_ = {0.f, 0.f, 0};

  // UI.
  float uiPhase_ = 0.f;
  uint32_t ms_ = 0;
  int pressTime_[kNumChannels] = {};
  int sliderLedCounter_[kNumChannels] = {};

  // Native clock and I/O.
  float step_ = 0.65f;
  float phase_ = 0.f;
  size_t pos_ = 0;
  SchmittTrigger gateTrigger_[kNumChannels];
  GateFlags gateFlags_[kNumChannels] = {};
  float prevOut_[kNumChannels] = {};
  float curOut_[kNumChannels] = {};
};

void buildPanel(PanelSpec& p) {
  using M = SegmentGen;
  p.hp = 14;
  p.bg = 0xFFF3F2EE;
  p.ink = 0xFF1E1E20;
  const uint32_t teal = 0xFF168A8C, ochre = 0xFFD39F2B, red = 0xFFC8414B;
  const uint32_t grey = 0xFFA9ABAE;
  const float xs[6] = {6.8f, 18.2f, 29.7f, 41.1f, 52.6f, 64.0f};
  auto dot = [&](float x, float y, uint32_t c) { p.box(x - 1.0f, y - 1.0f, 2.0f, 2.0f, c, 1.0f); };
  // Bracket joining the outer columns, with a caption in the middle.
  auto bracket = [&](float y0, float y, float gapL, float gapR) {
    p.line(xs[0], y0, xs[0], y, 0.35f, grey);
    p.line(xs[0], y, gapL, y, 0.35f, grey);
    p.line(gapR, y, xs[5], y, 0.35f, grey);
    p.line(xs[5], y, xs[5], y0, 0.35f, grey);
  };

  bracket(20.6f, 23.5f, 20.6f, 50.2f);
  dot(23.6f, 23.5f, teal);
  dot(26.4f, 23.5f, ochre);
  p.label(36.0f, 23.5f, "SHAPE/TIME", 3.0f);
  dot(47.1f, 23.5f, red);

  for (int i = 0; i < 6; ++i) {
    float x = xs[i];
    p.knob(M::SHAPE_PARAM + i, x, 15.6f, 8.f);
    p.rgbLight(M::TYPE_LIGHT + 3 * i, M::TYPE_LIGHT + 3 * i + 1, M::TYPE_LIGHT + 3 * i + 2, x,
               28.4f, 2.6f);
    p.button(M::TYPE_BUTTON + i, x, 34.6f, 5.4f);
    p.light(M::SLIDER_LIGHT + i, x, 41.4f, 0xFF35E06A, 2.2f);
    // Slider: slot graphic behind a knob.
    p.box(x - 0.6f, 44.0f, 1.2f, 25.5f, 0xFFCFCFCC, 0.6f);
    p.knob(M::LEVEL_PARAM + i, x, 56.8f, 9.5f);
    p.input(M::CV_INPUT + i, x, 81.9f);
    p.input(M::GATE_INPUT + i, x, 96.1f);
  }
  // Type legend under the sliders.
  p.label(18.2f, 71.5f, "RAMP", 2.4f, kAlignCenter, teal);
  p.label(29.7f, 71.5f, "STEP", 2.4f, kAlignCenter, ochre);
  p.label(41.1f, 71.5f, "HOLD", 2.4f, kAlignCenter, red);

  bracket(78.6f, 74.9f, 22.4f, 50.4f);
  dot(24.4f, 74.9f, teal);
  p.label(35.4f, 74.9f, "TIME/LEVEL", 3.0f);
  dot(45.8f, 74.9f, ochre);
  dot(48.4f, 74.9f, red);

  bracket(92.8f, 89.6f, 30.0f, 41.4f);
  p.label(35.7f, 89.6f, "GATE", 3.0f);

  p.box(1.6f, 102.4f, 67.6f, 14.8f, 0xFF5E5E60, 1.4f);
  for (int i = 0; i < 6; ++i) {
    p.output(M::ENVELOPE_OUTPUT + i, xs[i], 111.2f);
    p.bicolorLight(M::OUT_LIGHT + 2 * i, M::OUT_LIGHT + 2 * i + 1, xs[i] - 2.4f, 105.4f,
                   0xFF35E06A, 0xFFFF3B30, 2.0f);
    p.label(xs[i] + 1.6f, 105.4f, std::to_string(i + 1), 2.9f, kAlignCenter, 0xFFFFFFFF);
  }
}

Model model = [] {
  Model m;
  m.slug = "segment-generator";
  m.name = "Segment Generator";
  m.shortName = "SEGMENT GEN";
  m.basedOn = "Stages";
  m.category = Category::Envelope;
  m.description = "Six chainable ramp/step/hold segments: envelopes, LFOs, sequencers, S&H.";
  m.create = [] { return new SegmentGen(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
