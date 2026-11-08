// BrickRack - GPL-3.0-or-later.
//
// Macro Oscillator: wraps the Braids MacroOscillator (braids/macro_oscillator.cc)
// at its native 96 kHz in 24-sample blocks. renderBlock() is braids.cc's
// RenderBlock(): AD envelope, META shape selection, quantizer, auto-trigger,
// VCO drift, trigger delay, sync buffer, then the sample-rate / bit-depth
// reduction, VCA and signature waveshaper. The settings from Braids' encoder
// menu are hidden params (module menu); the encoder itself is the SHAPE knob,
// and the 4-character display shows the shape, or a setting for a moment
// after it changes.

#include <algorithm>
#include <cmath>
#include <cstring>

#include "braids/envelope.h"
#include "braids/macro_oscillator.h"
#include "braids/quantizer.h"
#include "braids/quantizer_scales.h"
#include "braids/signature_waveshaper.h"
#include "braids/vco_jitter_source.h"
#include "core/model.h"
#include "core/module.h"
#include "core/resampler.h"
#include "ui/canvas.h"

namespace br {
namespace {

constexpr int kNumShapes = braids::MACRO_OSC_SHAPE_LAST - 1;  // QPSK is the last in the menu
// Pitch ADC codes per volt (about 408.5): XTND's 4.5 pitch units per code
// (settings.h adc_to_pitch) come out as 4.5 * 4096 / 15401 times the
// calibrated 12 * 128 units per volt.
constexpr float kPitchAdcPerVolt = 12.f * 128.f * 4096.f / 15401.f;

// 4-character display codes (settings.cc algo_values; waveform glyphs of the
// hardware font replaced by ASCII), plus the "paques" question mark.
const char* kShapeCodes[kNumShapes + 1] = {
    "CSAW", "MRPH", "SAWQ", "FOLD", "BUZZ", "SUBQ", "SUB/", "SYNQ", "SYN/", "//x3",
    "SQx3", "/\\x3", "SIx3", "RING", "SWRM", "COMB", "TOY*", "ZLPF", "ZPKF", "ZBPF",
    "ZHPF", "VOSM", "VOWL", "VFOF", "HARM", "FM  ", "FBFM", "WTFM", "PLUK", "BOWD",
    "BLOW", "FLUT", "BELL", "DRUM", "KICK", "CYMB", "SNAR", "WTBL", "WMAP", "WLIN",
    "WTx4", "NOIS", "TWNQ", "CLKN", "CLOU", "PRTC", "QPSK", "????"};

const char* kShapeNames[kNumShapes] = {
    "CSAW (CS-80 saw)",     "Morph tri/saw/sq/pulse", "Saw / square",       "Sine-triangle fold",
    "Buzz",                 "Square + sub",          "Saw + sub",           "Square sync",
    "Saw sync",             "Triple saw",            "Triple square",       "Triple triangle",
    "Triple sine",          "Triple ring mod",       "Saw swarm",           "Saw comb",
    "Toy",                  "Filter LP",             "Filter peak",         "Filter BP",
    "Filter HP",            "VOSIM",                 "Vowel",               "Vowel FOF",
    "Harmonics",            "FM",                    "Feedback FM",         "Chaotic feedback FM",
    "Plucked",              "Bowed",                 "Blown",               "Fluted",
    "Struck bell",          "Struck drum",           "Kick",                "Cymbal",
    "Snare",                "Wavetables",            "Wave map",            "Wave line",
    "Paraphonic wavetables", "Filtered noise",       "Twin peaks noise",    "Clocked noise",
    "Granular cloud",       "Particle noise",        "Digital modulation",
};

const char* kOffOn[] = {"OFF ", "ON  "};
const char* kBits[] = {"2BIT", "3BIT", "4BIT", "6BIT", "8BIT", "12B ", "16B "};
const char* kRates[] = {"4KHZ", "8KHZ", "16K ", "24K ", "32K ", "48K ", "96K "};
const char* kTrigSrc[] = {"EXT.", "AUTO"};
const char* kTrigDelay[] = {"NONE", "125u", "250u", "500u", "1ms ", "2ms ", "4ms "};
const char* kZeroToFifteen[] = {"   0", "   1", "   2", "   3", "   4", "   5", "   6", "   7",
                                "   8", "   9", "  10", "  11", "  12", "  13", "  14", "  15"};
const char* kRange[] = {"EXT.", "FREE", "XTND", "440 ", "LFO "};
const char* kOctave[] = {"-2  ", "-1  ", "0   ", "1   ", "2   "};
const char* kScales[] = {
    "OFF ", "SEMI", "IONI", "DORI", "PHRY", "LYDI", "MIXO", "AEOL", "LOCR", "BLU+",
    "BLU-", "PEN+", "PEN-", "FOLK", "JAPA", "GAME", "GYPS", "ARAB", "FLAM", "WHOL",
    "PYTH", "EB/4", "E /4", "EA/4", "BHAI", "GUNA", "MARW", "SHRI", "PURV", "BILA",
    "YAMA", "KAFI", "BHIM", "DARB", "RAGE", "KHAM", "MIMA", "PARA", "RANG", "GANG",
    "KAME", "PAKA", "NATB", "KAUN", "BAIR", "BTOD", "CHAN", "KTOD", "JOGE"};
const char* kNotes[] = {"C   ", "Db  ", "D   ", "Eb  ", "E   ", "F   ",
                        "Gb  ", "G   ", "Ab  ", "A   ", "Bb  ", "B   "};
const char* kIntensity[] = {"OFF ", "   1", "   2", "   3", "FULL"};

// braids.cc tables.
const uint16_t kBitReductionMasks[] = {0xc000, 0xe000, 0xf000, 0xf800, 0xff00, 0xfff0, 0xffff};
const uint16_t kDecimationFactors[] = {24, 12, 6, 4, 3, 2, 1};

std::vector<std::string> labels(const char* const* values, int n) {
  std::vector<std::string> v;
  for (int i = 0; i < n; ++i) {
    std::string s(values[i]);
    while (!s.empty() && s.back() == ' ') s.pop_back();
    while (!s.empty() && s.front() == ' ') s.erase(s.begin());
    v.push_back(s);
  }
  return v;
}

class MacroOsc : public Module {
 public:
  enum ParamIds {
    SHAPE_PARAM,  // the encoder
    FINE_PARAM,
    COARSE_PARAM,
    FM_PARAM,
    TIMBRE_PARAM,
    MODULATION_PARAM,
    COLOR_PARAM,
    // Hidden settings, in the order of Braids' menu.
    META_PARAM,
    BITS_PARAM,
    RATE_PARAM,
    TRIG_SOURCE_PARAM,
    TRIG_DELAY_PARAM,
    AD_ATTACK_PARAM,
    AD_DECAY_PARAM,
    AD_FM_PARAM,
    AD_TIMBRE_PARAM,
    AD_COLOR_PARAM,
    AD_VCA_PARAM,
    PITCH_RANGE_PARAM,
    PITCH_OCTAVE_PARAM,
    QUANTIZER_SCALE_PARAM,
    QUANTIZER_ROOT_PARAM,
    VCO_FLATTEN_PARAM,
    VCO_DRIFT_PARAM,
    SIGNATURE_PARAM,
    NUM_PARAMS
  };
  enum InputIds { TRIG_INPUT, PITCH_INPUT, FM_INPUT, TIMBRE_INPUT, COLOR_INPUT, NUM_INPUTS };
  enum OutputIds { OUT_OUTPUT, NUM_OUTPUTS };
  enum LightIds { NUM_LIGHTS };

  MacroOsc() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configSwitch(SHAPE_PARAM, 0.f, kNumShapes - 1, 0.f, "Shape",
                 std::vector<std::string>(kShapeNames, kShapeNames + kNumShapes));
    configParam(FINE_PARAM, -1.f, 1.f, 0.f, "Fine frequency", "semitones");
    configParam(COARSE_PARAM, -4.f, 4.f, 0.f, "Coarse frequency", "semitones", 12.f);
    configParam(FM_PARAM, -1.f, 1.f, 0.f, "FM amount", "%", 100.f);
    configParam(TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Timbre", "%", 100.f);
    configParam(MODULATION_PARAM, -1.f, 1.f, 0.f, "Timbre CV amount", "%", 100.f);
    configParam(COLOR_PARAM, 0.f, 1.f, 0.5f, "Color", "%", 100.f);

    hidden(configSwitch(META_PARAM, 0, 1, 0, "META: FM CV selects shape", labels(kOffOn, 2)));
    hidden(configSwitch(BITS_PARAM, 0, 6, 6, "BITS: bit depth", labels(kBits, 7)));
    hidden(configSwitch(RATE_PARAM, 0, 6, 6, "RATE: sample rate", labels(kRates, 7)));
    hidden(configSwitch(TRIG_SOURCE_PARAM, 0, 1, 0, "TSRC: trigger source", labels(kTrigSrc, 2)));
    hidden(configSwitch(TRIG_DELAY_PARAM, 0, 6, 1, "TDLY: trigger delay", labels(kTrigDelay, 7)));
    hidden(configSwitch(AD_ATTACK_PARAM, 0, 15, 0, "ATT: AD attack", labels(kZeroToFifteen, 16)));
    hidden(configSwitch(AD_DECAY_PARAM, 0, 15, 5, "DEC: AD decay", labels(kZeroToFifteen, 16)));
    hidden(configSwitch(AD_FM_PARAM, 0, 15, 0, "FM: AD -> pitch", labels(kZeroToFifteen, 16)));
    hidden(configSwitch(AD_TIMBRE_PARAM, 0, 15, 0, "TIM: AD -> timbre", labels(kZeroToFifteen, 16)));
    hidden(configSwitch(AD_COLOR_PARAM, 0, 15, 0, "COL: AD -> color", labels(kZeroToFifteen, 16)));
    hidden(configSwitch(AD_VCA_PARAM, 0, 1, 0, "VCA: AD -> amplitude", labels(kOffOn, 2)));
    hidden(configSwitch(PITCH_RANGE_PARAM, 0, 4, 0, "RANG: pitch range", labels(kRange, 5)));
    hidden(configSwitch(PITCH_OCTAVE_PARAM, 0, 4, 2, "OCTV: octave", labels(kOctave, 5)));
    hidden(configSwitch(QUANTIZER_SCALE_PARAM, 0, 48, 0, "QNTZ: quantizer scale", labels(kScales, 49)));
    hidden(configSwitch(QUANTIZER_ROOT_PARAM, 0, 11, 0, "ROOT: quantizer root", labels(kNotes, 12)));
    hidden(configSwitch(VCO_FLATTEN_PARAM, 0, 1, 0, "FLAT: VCO flattening", labels(kOffOn, 2)));
    hidden(configSwitch(VCO_DRIFT_PARAM, 0, 4, 0, "DRFT: VCO drift", labels(kIntensity, 5)));
    hidden(configSwitch(SIGNATURE_PARAM, 0, 4, 0, "SIGN: signature waveshaper", labels(kIntensity, 5)));

    configInput(TRIG_INPUT, "Trigger / sync");
    configInput(PITCH_INPUT, "V/Oct");
    configInput(FM_INPUT, "FM");
    configInput(TIMBRE_INPUT, "Timbre");
    configInput(COLOR_INPUT, "Color");
    configOutput(OUT_OUTPUT, "Out");

    // Firmware objects are zero-initialised globals: do the same.
    std::memset(static_cast<void*>(&osc_), 0, sizeof(osc_));
    std::memset(static_cast<void*>(&envelope_), 0, sizeof(envelope_));
    std::memset(static_cast<void*>(&quantizer_), 0, sizeof(quantizer_));
    std::memset(static_cast<void*>(&ws_), 0, sizeof(ws_));
    std::memset(static_cast<void*>(&jitter_), 0, sizeof(jitter_));
    osc_.Init();
    quantizer_.Init();
    envelope_.Init();
    ws_.Init(0x2A1D5C3Bu);  // the hardware seeds this with the MCU's unique id
    jitter_.Init();
    for (int i = 0; i < NUM_PARAMS; ++i) shown_[i] = params[i].value;
    adapter_.init(48000.f, kNativeRate, kBlockSize);
  }

  void onSampleRateChange(float sampleRate) override {
    adapter_.init(sampleRate, kNativeRate, kBlockSize);
  }

  void process(const ProcessArgs& args) override {
    float in[1] = {inputs[TRIG_INPUT].getVoltage()};
    float out[1];
    adapter_.process(in, out, [&](const float* nin, float* nout, int n) {
      renderBlock(nin, nout, n);
    });
    outputs[OUT_OUTPUT].setVoltage(out[0] * 5.f);

    // Like the hardware's MODE_EDIT: briefly show a setting after it changes.
    if (firstRun_) {  // don't flash settings restored from a patch
      firstRun_ = false;
      for (int i = 0; i < NUM_PARAMS; ++i) shown_[i] = params[i].value;
    }
    if ((args.frame & 255) == 0) {
      for (int i = META_PARAM; i < NUM_PARAMS; ++i) {
        if (params[i].value != shown_[i]) {
          shown_[i] = params[i].value;
          displayParam_ = i;
          displayUntil_ = args.frame + static_cast<int64_t>(args.sampleRate * 1.5f);
        }
      }
      if (args.frame > displayUntil_) displayParam_ = -1;
    }
  }

  void saveState(StateMap& s) const override { s["paques"] = paques_ ? "1" : "0"; }
  void loadState(const StateMap& s) override {
    auto it = s.find("paques");
    if (it != s.end()) paques_ = it->second == "1";
  }
  void onReset() override { paques_ = false; }
  void appendMenu(std::vector<MenuAction>& items) override {
    items.push_back({"Paques (marquee \"49\" easter egg)",
                     [this] { return std::string(paques_ ? "ON" : "OFF"); },
                     [this](int) { paques_ = !paques_; }});
  }

  // Text for the 4-character display.
  const char* displayText4() const {
    int p = displayParam_;
    if (p >= 0) {
      int v = static_cast<int>(params[p].value + 0.5f);
      switch (p) {
        case META_PARAM: case AD_VCA_PARAM: case VCO_FLATTEN_PARAM: return kOffOn[v];
        case BITS_PARAM: return kBits[v];
        case RATE_PARAM: return kRates[v];
        case TRIG_SOURCE_PARAM: return kTrigSrc[v];
        case TRIG_DELAY_PARAM: return kTrigDelay[v];
        case PITCH_RANGE_PARAM: return kRange[v];
        case PITCH_OCTAVE_PARAM: return kOctave[v];
        case QUANTIZER_SCALE_PARAM: return kScales[v];
        case QUANTIZER_ROOT_PARAM: return kNotes[v];
        case VCO_DRIFT_PARAM: case SIGNATURE_PARAM: return kIntensity[v];
        default: return kZeroToFifteen[v];
      }
    }
    return kShapeCodes[activeShape_];
  }

 private:
  static constexpr float kNativeRate = 96000.f;
  static constexpr int kBlockSize = 24;

  static void hidden(Param& p) { p.hidden = true; }
  int setting(int id) const { return static_cast<int>(params[id].value + 0.5f); }

  // braids.cc: RenderBlock(), plus the trigger handling of TIM1_UP_IRQHandler.
  void renderBlock(const float* nin, float* nout, int n) {
    // Trigger input: rising edges feed the sync buffer and, after the trigger
    // delay (counted in 8-sample ADC scan cycles), strike the oscillator.
    const int trigDelay = setting(TRIG_DELAY_PARAM);
    bool triggerFlag = false;
    for (int i = 0; i < n; ++i) {
      bool edge = trigger_.process(nin[i]);
      syncBuffer_[i] = edge ? 1 : 0;
      if (edge) triggerDelay_ = (trigDelay ? (1 << trigDelay) : 0) * 8 + 1;
      if (autoTrig_) {
        triggerDelay_ = (trigDelay ? (1 << trigDelay) : 0) * 8 + 1;
        autoTrig_ = false;
      }
      if (triggerDelay_ > 0 && --triggerDelay_ == 0) triggerFlag = true;
    }

    envelope_.Update(setting(AD_ATTACK_PARAM) * 8, setting(AD_DECAY_PARAM) * 8);
    uint32_t adValue = envelope_.Render();

    const bool meta = setting(META_PARAM) != 0;
    const float fm = params[FM_PARAM].value * inputs[FM_INPUT].getVoltage();
    if (paques_) {
      activeShape_ = braids::MACRO_OSC_SHAPE_QUESTION_MARK;
    } else if (meta) {
      // FM CV offsets the selected shape (scaling of the VCV Rack port).
      int shape = setting(SHAPE_PARAM) +
                  static_cast<int>(std::round(fm / 10.f * braids::MACRO_OSC_SHAPE_LAST_ACCESSIBLE_FROM_META));
      activeShape_ = std::min(std::max(shape, 0),
                              static_cast<int>(braids::MACRO_OSC_SHAPE_LAST_ACCESSIBLE_FROM_META));
    } else {
      activeShape_ = setting(SHAPE_PARAM);
    }
    osc_.set_shape(static_cast<braids::MacroOscillatorShape>(activeShape_));

    // Timbre and color: pot + CV (summed in the analog domain on the hardware)
    // plus internal AD modulation.
    float timbre = params[TIMBRE_PARAM].value +
                   params[MODULATION_PARAM].value * inputs[TIMBRE_INPUT].getVoltage() / 5.f;
    float color = params[COLOR_PARAM].value + inputs[COLOR_INPUT].getVoltage() / 5.f;
    int32_t parameters[2] = {static_cast<int32_t>(clampf(timbre, 0.f, 1.f) * 32767.f),
                             static_cast<int32_t>(clampf(color, 0.f, 1.f) * 32767.f)};
    for (int i = 0; i < 2; ++i) {
      int adMod = setting(i == 0 ? AD_TIMBRE_PARAM : AD_COLOR_PARAM);
      parameters[i] += static_cast<int32_t>(adValue * adMod >> 5);
      parameters[i] = std::min(std::max(parameters[i], int32_t(0)), int32_t(32767));
    }
    // RenderWaveLine() reads wave_line[(2 * timbre >> 10) + 1] from a 64-entry
    // table: entry 64 for timbre >= 32256. Stop just below.
    if (activeShape_ == braids::MACRO_OSC_SHAPE_WAVE_LINE) {
      parameters[0] = std::min(parameters[0], int32_t(32255));
    }
    osc_.set_parameters(static_cast<int16_t>(parameters[0]), static_cast<int16_t>(parameters[1]));

    // Pitch (Settings::adc_to_pitch, in 1/128 semitones). The coarse knob is
    // summed with the V/Oct CV in the analog domain on the hardware.
    const int range = setting(PITCH_RANGE_PARAM);
    // The pitch ADC spans codes 0..4095 with 0 V at 1638 and 4096 * 15401 /
    // (4.5 * 12 * 128 * 4096) V per code (see the XTND scale below), i.e.
    // about -4.0..+6.0 V. Clamp like the hardware: beyond it the quantizer
    // (int16 maths) can pick codebook_[-1].
    float volts = clampf(inputs[PITCH_INPUT].getVoltage() + params[COARSE_PARAM].value,
                         -1638.f / kPitchAdcPerVolt, 2457.f / kPitchAdcPerVolt);
    int32_t pitch;
    if (range == braids::PITCH_RANGE_440) {
      pitch = 69 << 7;
    } else if (range == braids::PITCH_RANGE_EXTENDED) {
      // (adc - 1638) * 9 / 2 instead of the calibrated 1 V/Oct scale.
      pitch = (60 << 7) + static_cast<int32_t>(volts * kPitchAdcPerVolt * 4.5f);
    } else {
      pitch = (60 << 7) + static_cast<int32_t>(volts * 12.f * 128.f);
    }
    int scale = setting(QUANTIZER_SCALE_PARAM);
    if (scale != currentScale_) {
      currentScale_ = scale;
      quantizer_.Configure(braids::scales[scale]);
    }
    pitch = quantizer_.Process(pitch, (60 + setting(QUANTIZER_ROOT_PARAM)) << 7);
    if (!meta && range != braids::PITCH_RANGE_440) {
      pitch += static_cast<int32_t>(fm * 12.f * 128.f);
    }
    // Auto-trigger on pitch changes.
    int32_t pitchDelta = pitch - previousPitch_;
    if (setting(TRIG_SOURCE_PARAM) && (pitchDelta >= 0x40 || -pitchDelta >= 0x40)) {
      autoTrig_ = true;
    }
    previousPitch_ = pitch;

    pitch += jitter_.Render(static_cast<int32_t>(setting(VCO_DRIFT_PARAM)));
    pitch += static_cast<int32_t>(params[FINE_PARAM].value * 128.f);
    pitch += static_cast<int32_t>(adValue * setting(AD_FM_PARAM) >> 7);
    pitch = std::min(std::max(pitch, int32_t(0)), int32_t(16383));
    if (setting(VCO_FLATTEN_PARAM)) {
      pitch = stmlib::Interpolate88(braids::lut_vco_detune, static_cast<uint16_t>(pitch << 2));
    }
    // Settings::pitch_transposition().
    int32_t transposition = range == braids::PITCH_RANGE_LFO ? int32_t(-36 * 128) : 0;
    transposition += (setting(PITCH_OCTAVE_PARAM) - 2) * 12 * 128;
    // The firmware doesn't clamp pitch + transposition: OCTV +2 takes it past
    // 16383 and FLUT's lut_flute_body_filter[pitch >> 7] (128 entries)
    // overruns; below 0 it reads before the table. Keep 0..16383, except that
    // the LFO range's -36 semitones are meant to go below 0 (allowed there for
    // every shape but FLUT).
    int32_t minPitch = range == braids::PITCH_RANGE_LFO &&
                               activeShape_ != braids::MACRO_OSC_SHAPE_FLUTED
                           ? int32_t(-32768)
                           : int32_t(0);
    osc_.set_pitch(static_cast<int16_t>(
        std::min(std::max(pitch + transposition, minPitch), int32_t(16383))));

    if (triggerFlag) {
      osc_.Strike();
      envelope_.Trigger(braids::ENV_SEGMENT_ATTACK);
    }

    const bool adActive = setting(AD_VCA_PARAM) || setting(AD_TIMBRE_PARAM) ||
                          setting(AD_COLOR_PARAM) || setting(AD_FM_PARAM);
    if (adActive) std::memset(syncBuffer_, 0, sizeof(syncBuffer_));

    osc_.Render(syncBuffer_, renderBuffer_, static_cast<size_t>(n));

    // Sample rate and bit reduction, VCA, signature.
    int16_t heldSample = 0;
    size_t decimationFactor = kDecimationFactors[setting(RATE_PARAM)];
    uint16_t bitMask = kBitReductionMasks[setting(BITS_PARAM)];
    int32_t gain = setting(AD_VCA_PARAM) ? static_cast<int32_t>(adValue) : 65535;
    uint16_t signature = static_cast<uint16_t>(setting(SIGNATURE_PARAM) * setting(SIGNATURE_PARAM) * 4095);
    for (size_t i = 0; i < static_cast<size_t>(n); ++i) {
      if ((i % decimationFactor) == 0) {
        heldSample = static_cast<int16_t>(renderBuffer_[i] & bitMask);
      }
      int16_t sample = static_cast<int16_t>(heldSample * gainLp_ >> 16);
      gainLp_ = static_cast<uint16_t>(gainLp_ + ((gain - gainLp_) >> 4));
      int16_t warped = static_cast<int16_t>(ws_.Transform(sample));
      nout[i] = stmlib::Mix(sample, warped, signature) / 32768.f;
    }
  }

  braids::MacroOscillator osc_;
  braids::Envelope envelope_;
  braids::Quantizer quantizer_;
  braids::SignatureWaveshaper ws_;
  braids::VcoJitterSource jitter_;
  BlockAdapter<1, 1> adapter_;
  uint8_t syncBuffer_[kBlockSize];
  int16_t renderBuffer_[kBlockSize];
  SchmittTrigger trigger_;
  int triggerDelay_ = 0;
  bool autoTrig_ = false;
  int32_t previousPitch_ = 0;
  uint16_t gainLp_ = 0;
  int currentScale_ = -1;
  int activeShape_ = 0;
  bool paques_ = false;

  float shown_[NUM_PARAMS];
  int displayParam_ = -1;
  bool firstRun_ = true;
  int64_t displayUntil_ = 0;
};

void buildPanel(PanelSpec& p) {
  p.hp = 16;
  p.bg = 0xFFF2F1EC;
  p.ink = 0xFF1E1E20;
  p.title = "MACRO OSC";
  const uint32_t red = 0xFFC8414B;
  const uint32_t teal = 0xFF168A8C;

  // Alphanumeric display + EDIT encoder (shape).
  p.display(4.7f, 18.4f, 50.1f, 18.6f, [](Canvas& c, float x, float y, float w, float h, float s,
                                          const Module* m) {
    c.fillRoundRect(x, y, w, h, 1.5f * s, 0xFF1A1B1E);
    const char* text = static_cast<const MacroOsc*>(m)->displayText4();
    const float cell = (w - 4.f * s) / 4.f;
    const float size = std::max(14.f, 12.f * s);
    for (int i = 0; i < 4 && text[i]; ++i) {
      float cx = x + 2.f * s + cell * (i + 0.5f);
      c.fillRoundRect(cx - cell * 0.42f, y + 2.f * s, cell * 0.84f, h - 4.f * s, 0.8f * s,
                      0xFF24262A);
      const char ch[2] = {text[i], 0};
      c.text(cx, y + h * 0.5f, ch, size, 0xFFAFD22C, kAlignCenter);
    }
  });
  p.line(55.4f, 27.2f, 61.6f, 27.2f, 0.4f, red);
  p.knob(MacroOsc::SHAPE_PARAM, 66.9f, 27.2f, 9.f);
  p.label(66.9f, 37.9f, "EDIT", 3.0f);

  p.knob(MacroOsc::FINE_PARAM, 13.6f, 54.2f, 9.f);
  p.label(13.6f, 65.7f, "FINE", 3.0f);
  p.knob(MacroOsc::COARSE_PARAM, 40.3f, 54.2f, 9.f);
  p.label(40.3f, 65.7f, "COARSE", 3.0f);
  p.label(62.4f, 45.6f, "-", 3.2f);
  p.label(71.4f, 45.6f, "+", 3.2f);
  p.knob(MacroOsc::FM_PARAM, 66.9f, 54.2f, 9.f);
  p.label(66.9f, 65.7f, "FM", 3.0f);

  p.knob(MacroOsc::TIMBRE_PARAM, 13.6f, 81.1f, 9.f);
  p.label(35.8f, 72.0f, "-", 3.2f);
  p.label(44.8f, 72.0f, "+", 3.2f);
  p.knob(MacroOsc::MODULATION_PARAM, 40.3f, 81.1f, 9.f);
  p.knob(MacroOsc::COLOR_PARAM, 66.9f, 81.1f, 9.f);
  p.box(5.0f, 90.6f, 17.2f, 4.2f, teal, 1.0f);
  p.label(13.6f, 92.7f, "TIMBRE", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.box(29.6f, 90.6f, 21.4f, 4.2f, teal, 1.0f);
  p.label(40.3f, 92.7f, "MODULATION", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.box(58.3f, 90.6f, 17.2f, 4.2f, red, 1.0f);
  p.label(66.9f, 92.7f, "COLOR", 2.7f, kAlignCenter, 0xFFFFFFFF);

  p.line(3.f, 97.6f, 78.3f, 97.6f, 0.4f, 0xFFC4C4C4);

  p.input(MacroOsc::TRIG_INPUT, 7.4f, 111.0f, "TRIG", -7.4f);
  p.input(MacroOsc::PITCH_INPUT, 19.8f, 111.0f, "V/OCT", -7.4f);
  p.input(MacroOsc::FM_INPUT, 32.6f, 111.0f, "FM", -7.4f);
  p.box(41.0f, 101.6f, 9.2f, 4.0f, teal, 1.0f);
  p.label(45.6f, 103.6f, "TIMBRE", 2.5f, kAlignCenter, 0xFFFFFFFF);
  p.input(MacroOsc::TIMBRE_INPUT, 45.6f, 111.0f);
  p.box(53.4f, 101.6f, 9.2f, 4.0f, red, 1.0f);
  p.label(58.0f, 103.6f, "COLOR", 2.5f, kAlignCenter, 0xFFFFFFFF);
  p.input(MacroOsc::COLOR_INPUT, 58.0f, 111.0f);
  p.box(67.6f, 100.6f, 11.6f, 16.2f, 0xFF5E5E60, 1.2f);
  p.label(73.4f, 103.6f, "OUT", 2.7f, kAlignCenter, 0xFFFFFFFF);
  p.output(MacroOsc::OUT_OUTPUT, 73.4f, 111.0f);
}

Model model = [] {
  Model m;
  m.slug = "macro-osc";
  m.name = "Macro Oscillator";
  m.shortName = "MACRO OSC";
  m.basedOn = "Braids";
  m.category = Category::Oscillator;
  m.description = "47 shapes: analog, sync, FM, physical models, drums, wavetables, noise.";
  m.create = [] { return new MacroOsc(); };
  m.buildPanel = buildPanel;
  return m;
}();

BR_REGISTER_MODEL(model);

}  // namespace
}  // namespace br
