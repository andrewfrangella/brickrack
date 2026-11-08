# Writing a BrickRack module

BrickRack wraps the original Mutable Instruments firmware DSP (in the
`third_party/eurorack` submodule) **without modifying it**. A module is one
`.cc` file plus an optional `.cmake` fragment in `src/modules/`. Both are picked up
automatically by `CMakeLists.txt` (file globs), so adding a module never touches
shared files.

`src/modules/plaits.cc` is the reference wrapper. `src/modules/brick_io.cc` shows
simpler, original modules.

## Files

* `src/modules/<name>.cmake`: appends firmware sources to `BR_MI_SOURCES`:
  ```cmake
  list(APPEND BR_MI_SOURCES
    ${EURORACK_DIR}/rings/dsp/part.cc
    ${EURORACK_DIR}/rings/resources.cc)
  ```
  Only add DSP sources, never `drivers/`, `bootloader/`, `ui.cc` or the firmware's `main` file.
  stmlib's `dsp/units.cc`, `dsp/atan.cc` and `utils/random.cc` are already included.
  Everything compiles with `-DTEST`, which is what the firmware's own desktop tests use.
* `src/modules/<name>.cc`: the wrapper, panel and model registration.
* `tests/patches/<slug>.brp`: a small test patch that makes the module produce output.

## Module API (`src/core/module.h`)

```cpp
class MyModule : public br::Module {
 public:
  enum ParamIds { FREQ_PARAM, NUM_PARAMS };
  enum InputIds { VOCT_INPUT, NUM_INPUTS };
  enum OutputIds { OUT_OUTPUT, NUM_OUTPUTS };
  enum LightIds { NUM_LIGHTS };
  MyModule() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    configParam(FREQ_PARAM, -4.f, 4.f, 0.f, "Frequency", "semitones", 12.f);
    configInput(VOCT_INPUT, "V/Oct");
    configOutput(OUT_OUTPUT, "Out");
  }
  void process(const br::ProcessArgs& args) override { ... }  // once per engine sample
};
```

* The engine runs at **48 kHz, one sample at a time**. Cables have a one-sample delay.
* `params[i].value` is written by the UI thread. Read it in `process()`.
* `configSwitch(id, min, max, def, name, {labels})` gives a discrete param (snaps to
  integers). `configButton` is momentary (1 while held). `configToggle` latches.
* Only `buildPanel` decides what appears on the panel. A param with no widget is still
  editable from the module menu (Y button) and saved in patches. Mark such params
  `param.hidden = true` (shown with a `*` in the menu). Use this for the firmware's
  "hidden" settings, such as Plaits' LPG colour/decay or Rings' polyphony.
* `appendMenu(items)` adds custom actions to the module menu (mode toggles and so on). The
  lambdas run on the UI thread: only flip simple members that `process()` reads.
* `saveState` / `loadState` persist non-param state as string key/values.
* `onReset()` is called after params are reset to defaults.
* `process()` must be real-time safe: no allocation, locks, file I/O or printing.
  Allocate big buffers as class members or in the constructor.

### Voltage conventions (Eurorack / VCV Rack)

* Pitch: 1 V/oct, 0 V = C4 (MIDI 60). The firmware usually wants semitones: `note = 60 + V*12`.
* Audio: ±5 V. Firmware `int16` output → `x / 32768.f * 5.f`. Firmware float ±1 → `* 5.f`.
* Gates/triggers: 0/+10 V out. Read with `br::SchmittTrigger` from `src/core/resampler.h`
  (thresholds 0.7/1.7 V).
* Unipolar CV 0..+8 V, bipolar ±5 V. Follow the scalings the firmware's `cv_scaler.cc` /
  `settings.cc` calibration produce. The VCV Rack "Audible Instruments" port uses the
  same conventions, so matching its scalings is fine (this repo is GPL-3.0-or-later).
* `input.isConnected()` replaces the hardware normalisation probes ("patched" flags).

### Native sample rates: `br::BlockAdapter` (`src/core/resampler.h`)

Run the firmware DSP at its **own** rate and block size: Clouds/Elements 32 kHz,
Warps/Braids 96 kHz, Tides 2 62.5 kHz, and so on. The adapter does windowed-sinc
conversion both ways, so pitch tables and filter coefficients stay correct.

Don't use it for gate/CV-heavy modules. The sinc filter rings (about 8% overshoot) on
steps and adds about 40 samples of latency. Marbles, Stages, Streams and Frames instead
clock their firmware at its native rate from the engine rate: sample inputs, then hold
or linearly interpolate outputs (see `marbles.cc`, `stages.cc`). For audio:

```cpp
br::BlockAdapter<2, 2> adapter_;     // 2 audio channels in, 2 out
// constructor / onSampleRateChange:
adapter_.init(48000.f, 32000.f, 32); // engine rate, native rate, native block size
// process():
float in[2] = {inputs[IN_L].getVoltage() / 5.f, ...};
float out[2];
adapter_.process(in, out, [&](const float* nin, float* nout, int n) {
  // nin/nout are interleaved native-rate frames (n == block size).
  // Read CV params/inputs here (block rate), call the firmware Process().
});
outputs[OUT_L].setVoltage(out[0] * 5.f);
```

`BlockAdapter<0, N>` works for generators with no audio input. If the native rate is
48 kHz the resamplers are bypassed and only block buffering remains. Plaits shows
the hand-rolled version of that. Audio-rate inputs (audio in, FM, sync) go through
the adapter. CV can be sampled once per block inside the render lambda, which is
what the firmware does.

## Panel (`src/core/model.h`)

Coordinates are **millimetres** on a real Eurorack faceplate (128.5 mm tall,
1 HP = 5.08 mm), measured from the top-left. Lay controls out like the original
hardware. The original panel artwork is in
`third_party/eurorack/<module>/hardware_design/panel/*.ai`. These are PDF-compatible;
on macOS, `sips -s format png file.ai --out file.png` renders one. To convert,
mm = px * 128.5 / image height in px. Keep the same HP as the hardware where it fits. The screen shows about
38 HP at normal zoom.

```cpp
void buildPanel(br::PanelSpec& p) {
  p.hp = 14;
  p.bg = 0xFFD9DCDE; p.ink = 0xFF2B2B2E;      // ARGB
  p.knob(FREQ_PARAM, x, y, 13.f, "FREQUENCY");  // diameter mm; label auto-placed above
  p.knob(ATTEN_PARAM, x, y, 6.5f);              // small knob / attenuverter (< 9 mm = light grey)
  p.button(MODE_PARAM, x, y, 5.f, "MODE", 0, MODE_LIGHT);  // optional light inside
  p.toggleSwitch(RANGE_PARAM, x, y, "RANGE");   // 2/3-position switch
  p.input(VOCT_INPUT, x, y, "V/OCT");           // label 6 mm above by default
  p.output(OUT_OUTPUT, x, y, "OUT");
  p.light(LIGHT_ID, x, y, 0xFF35E06A, 2.5f);
  p.bicolorLight(G_LIGHT, R_LIGHT, x, y, 0xFF35E06A, 0xFFFF3B30);
  p.rgbLight(R, G, B, x, y);
  p.label(x, y, "TEXT", 3.0f);                  // size in mm, centred
  p.line(x0, y0, x1, y1, 0.3f, color);          // panel graphics
  p.box(x, y, w, h, 0xFF5E5E60, 1.2f);          // e.g. dark output area
  p.display(x, y, w, h, [](br::Canvas& c, float x, float y, float w, float h, float s,
                           const br::Module* m) { ... });  // custom drawing, pixels; s = px/mm
}
```

* Size guide: big knob 13 mm, medium 10 mm, small 6.5-8 mm, button 5-6 mm,
  jack 6.2 mm (fixed), LED 2.5-3 mm, label text 2.6-3.4 mm. Leave at least 1.5 mm
  between widgets. Labels render at ≥ 13 px.
* Font: Barlow Condensed. Stick to ASCII plus common Latin-1. Arrows and symbols such as
  ← ∿ are **not** in the font.
* All focusable widgets (knobs, buttons, switches, jacks) are reached with the D-pad by
  spatial navigation. Keep a clean grid where you can.
* Widget order in `p.widgets` doesn't matter for indices. Params, ports and lights are
  referenced by their enum ids.
* Use `p.display` for alphanumeric displays (Braids), LED rings and similar. `Canvas` has
  `fillRect`, `fillRoundRect`, `fillCircle`, `strokeCircle`, `line`, `polyline`, `arc`,
  `text(x, y, str, sizePx, color, align)`.

## Naming (trademark policy)

"Mutable Instruments" is a registered trademark. The eurorack repo's guidelines say
the name should not be used on derivative works and recommend against keeping the
original module names. So:

* `Model::name` / `shortName` is a **descriptive** name ("Resonator", "Texture Synthesizer").
* `Model::basedOn` holds the original module name ("Rings"). It is shown as "after Rings"
  in the browser, as attribution.
* Never put "Mutable Instruments" or MI logos on a panel or in UI text.
* Wrappers that copy or reproduce firmware code must keep the upstream copyright line
  ("Portions derived from ... Copyright Emilie Gillet, MIT License" or
  GPL-3.0-or-later for the AVR firmwares).

## Registration

```cpp
namespace br { namespace {
// ... class + buildPanel ...
Model model = [] {
  Model m;
  m.slug = "resonator";             // stable id used in patch files; never change it
  m.name = "Resonator";
  m.shortName = "RESONATOR";        // panel title
  m.basedOn = "Rings";
  m.category = Category::Resonator; // Oscillator, Resonator, Effect, Filter, Modulator,
                                    // Envelope, Sequencer, Utility, IO
  m.description = "One line shown in the module browser.";
  m.create = [] { return new MyModule(); };
  m.buildPanel = buildPanel;
  return m;
}();
BR_REGISTER_MODEL(model);
}}
```

## Building and testing (desktop)

```sh
cmake -S . -B build-<yourname> -DCMAKE_BUILD_TYPE=Release
cmake --build build-<yourname> -j8
./build-<yourname>/brickrack --list-modules
# Screenshot (1024x768 PNG of the device UI). --focus = module index, --zoom 0/1/2,
# --widget N focuses widget N, --menu module opens the module menu.
./build-<yourname>/brickrack --data /tmp/x --patch tests/patches/<slug>.brp --focus 0 --screenshot out.png
# Offline audio render (prints peak/RMS). --gate-every pulses the BRICK CONTROL L2 gate.
./build-<yourname>/brickrack --data /tmp/x --patch tests/patches/<slug>.brp --render-wav out.wav --seconds 5
./build-<yourname>/brickrack --data /tmp/x --patch tests/patches/<slug>.brp --bench --seconds 10
```

Patch file format (`*.brp`). Port and param indices are your enum values. Module ids
are arbitrary positive integers:

```
brickrack-patch 1
volume 0.8
module 1 step-sequencer
module 2 resonator
param 0 0.5
state someKey someValue
module 3 audio-out
cable 1 1 1 2 0 F2B630
```

`cable <id> <outModuleId> <outPort> <inModuleId> <inPort> <colour hex>` (RRGGBB or
AARRGGBB; saved files use 8 digits). The
step-sequencer outputs are 0 = Pitch, 1 = Gate, 2 = Clock. brick-control outputs:
0 = stick X, 1 = stick Y, 2 = L2 gate (pulsed by `--gate-every`), 3 = R2.
audio-out inputs: 0 = Left, 1 = Right.

Check: the render is not silent and reports `non-finite frames 0` (the master bus is
soft-clipped, so the peak is always ≤ 1 and can't reveal NaN), the CPU figure from `--bench`
is sensible (the device's Cortex-A53 is about 10-15x slower than an Apple M-series core),
and the screenshot looks right. Open the PNG and look at it.
