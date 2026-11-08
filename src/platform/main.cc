// BrickRack - modular synthesizer for TrimUI Brick / Brick Pro (NextUI, tg5040)
// Copyright (C) 2026 BrickRack contributors. GPL-3.0-or-later.
//
// Platform layer: SDL2 window/audio/input, plus headless tools:
//   brickrack [--demo | --patch file|demo] [--input-debug]   (interactive)
//   brickrack --screenshot out.png [--patch file] [--focus N] [--widget W]
//             [--zoom Z] [--menu main|module|browser] [--frames N]
//   brickrack --render-wav out.wav [--patch file] [--seconds S]
//             [--gate-every S]   (pulses the L2 controller gate)
//   brickrack --list-modules
//   brickrack --bench [--patch file] [--seconds S]
//   brickrack --bench-ui [--zoom Z]
//   brickrack --patch demo --save-patch out.brp
// Common: --data DIR (patch/autosave folder; default $BRICKRACK_HOME or
// <binary dir>/data).

#include <SDL.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "core/engine.h"
#include "core/model.h"
#include "ui/app.h"
#include "ui/canvas.h"
#include "ui/font.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBIW_WINDOWS_UTF8
#include "stb_image_write.h"

using namespace br;

namespace {

constexpr float kSampleRate = 48000.f;

struct Options {
  std::string screenshot;
  std::string renderWav;
  std::string patch;
  std::string menu;
  std::string dataDir;
  std::string savePatch;
  int focus = 0;
  int widget = -1;
  int zoom = 1;
  int frames = 4800;
  float seconds = 4.f;
  float gateEvery = 0.f;
  bool listModules = false;
  bool bench = false;
  bool benchUi = false;
  bool inputDebug = false;
  bool demo = false;
};

std::string defaultDataDir(const char* argv0) {
  if (const char* env = std::getenv("BRICKRACK_HOME")) return env;
  std::string exe = argv0;
  size_t slash = exe.find_last_of('/');
  if (slash != std::string::npos) return exe.substr(0, slash) + "/data";
  return "./data";
}

Options parseArgs(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : std::string(); };
    if (a == "--screenshot") o.screenshot = next();
    else if (a == "--render-wav") o.renderWav = next();
    else if (a == "--patch") o.patch = next();
    else if (a == "--menu") o.menu = next();
    else if (a == "--data") o.dataDir = next();
    else if (a == "--save-patch") o.savePatch = next();
    else if (a == "--focus") o.focus = std::atoi(next().c_str());
    else if (a == "--widget") o.widget = std::atoi(next().c_str());
    else if (a == "--zoom") o.zoom = std::atoi(next().c_str());
    else if (a == "--frames") o.frames = std::atoi(next().c_str());
    else if (a == "--seconds") o.seconds = static_cast<float>(std::atof(next().c_str()));
    else if (a == "--gate-every") o.gateEvery = static_cast<float>(std::atof(next().c_str()));
    else if (a == "--list-modules") o.listModules = true;
    else if (a == "--bench") o.bench = true;
    else if (a == "--bench-ui") o.benchUi = true;
    else if (a == "--input-debug") o.inputDebug = true;
    else if (a == "--demo") o.demo = true;
  }
  if (o.dataDir.empty()) o.dataDir = defaultDataDir(argv[0]);
  return o;
}

bool setupPatch(Engine& engine, App& app, const Options& o) {
  if (!o.patch.empty()) {
    if (o.patch == "demo") {
      app.buildDefaultPatch();
      return true;
    }
    std::string err;
    if (!engine.loadPatch(o.patch, &err)) {
      std::fprintf(stderr, "error: %s\n", err.c_str());
      return false;
    }
    return true;
  }
  app.buildDefaultPatch();
  return true;
}

void writeWav(const std::string& path, const std::vector<float>& data, int rate) {
  FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return;
  auto w32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
  auto w16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
  uint32_t bytes = static_cast<uint32_t>(data.size() * 2);
  std::fwrite("RIFF", 1, 4, f);
  w32(36 + bytes);
  std::fwrite("WAVEfmt ", 1, 8, f);
  w32(16);
  w16(1);
  w16(2);
  w32(static_cast<uint32_t>(rate));
  w32(static_cast<uint32_t>(rate * 4));
  w16(4);
  w16(16);
  std::fwrite("data", 1, 4, f);
  w32(bytes);
  for (float x : data) {
    int v = static_cast<int>(std::lround(std::max(-1.f, std::min(1.f, x)) * 32767.f));
    w16(static_cast<uint16_t>(static_cast<int16_t>(v)));
  }
  std::fclose(f);
}

void runOffline(Engine& engine, float seconds, float gateEvery, std::vector<float>* out) {
  const int block = 256;
  int total = static_cast<int>(seconds * kSampleRate);
  std::vector<float> buf(block * 2);
  int gatePeriod = gateEvery > 0.f ? static_cast<int>(gateEvery * kSampleRate) : 0;
  for (int pos = 0; pos < total; pos += block) {
    if (gatePeriod) {
      int phase = pos % gatePeriod;
      engine.controller().gate[0].store(phase < gatePeriod / 4);
    }
    engine.process(buf.data(), nullptr, 0, block);
    if (out) out->insert(out->end(), buf.begin(), buf.end());
  }
}

// Lock-free single-producer/single-consumer ring for microphone samples.
struct MicRing {
  static constexpr int kSize = 1 << 15;
  float data[kSize];
  std::atomic<int> head{0}, tail{0};
  void push(const float* x, int n) {
    int h = head.load(std::memory_order_relaxed);
    for (int i = 0; i < n; ++i) {
      int nh = (h + 1) & (kSize - 1);
      if (nh == tail.load(std::memory_order_acquire)) break;
      data[h] = x[i];
      h = nh;
    }
    head.store(h, std::memory_order_release);
  }
  int pop(float* x, int n) {
    int t = tail.load(std::memory_order_relaxed);
    int got = 0;
    while (got < n && t != head.load(std::memory_order_acquire)) {
      x[got++] = data[t];
      t = (t + 1) & (kSize - 1);
    }
    tail.store(t, std::memory_order_release);
    return got;
  }
};

struct AudioContext {
  Engine* engine;
  MicRing mic;
  std::atomic<bool> micOpen{false};
  std::vector<float> micScratch;
};

void audioCallback(void* user, Uint8* stream, int len) {
  auto* ctx = static_cast<AudioContext*>(user);
  int frames = len / static_cast<int>(sizeof(float) * 2);
  float* out = reinterpret_cast<float*>(stream);
  const float* in = nullptr;
  if (ctx->micOpen.load()) {
    if (static_cast<int>(ctx->micScratch.size()) < frames) ctx->micScratch.assign(frames, 0.f);
    int got = ctx->mic.pop(ctx->micScratch.data(), frames);
    std::fill(ctx->micScratch.begin() + got, ctx->micScratch.begin() + frames, 0.f);
    in = ctx->micScratch.data();
  }
  ctx->engine->process(out, in, in ? 1 : 0, frames);
}

void captureCallback(void* user, Uint8* stream, int len) {
  auto* ctx = static_cast<AudioContext*>(user);
  ctx->mic.push(reinterpret_cast<const float*>(stream), len / static_cast<int>(sizeof(float)));
}

bool engineWantsMic(const Engine& engine) {
  for (const Module* m : engine.modules()) {
    if (m->model->slug == "mic-in") return true;
  }
  return false;
}

Button mapKey(SDL_Keycode k) {
  switch (k) {
    case SDLK_UP: return BTN_UP;
    case SDLK_DOWN: return BTN_DOWN;
    case SDLK_LEFT: return BTN_LEFT;
    case SDLK_RIGHT: return BTN_RIGHT;
    case SDLK_z: case SDLK_SPACE: return BTN_A;
    case SDLK_x: case SDLK_LCTRL: return BTN_B;
    case SDLK_c: case SDLK_LSHIFT: return BTN_X;
    case SDLK_v: case SDLK_LALT: return BTN_Y;
    case SDLK_q: case SDLK_PAGEUP: return BTN_L1;
    case SDLK_e: case SDLK_PAGEDOWN: return BTN_R1;
    case SDLK_1: return BTN_L2;
    case SDLK_3: return BTN_R2;
    case SDLK_RETURN: return BTN_START;
    case SDLK_TAB: case SDLK_RCTRL: return BTN_SELECT;
    case SDLK_ESCAPE: case SDLK_HOME: return BTN_MENU;
    default: return BTN_COUNT;
  }
}

Button mapControllerButton(Uint8 b) {
  // TrimUI's SDL reports a Nintendo-style layout: the physical A (east) button
  // arrives as SDL's "B". Same mapping as the proven GrainBrick native build.
  switch (b) {
    case SDL_CONTROLLER_BUTTON_DPAD_UP: return BTN_UP;
    case SDL_CONTROLLER_BUTTON_DPAD_DOWN: return BTN_DOWN;
    case SDL_CONTROLLER_BUTTON_DPAD_LEFT: return BTN_LEFT;
    case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: return BTN_RIGHT;
    case SDL_CONTROLLER_BUTTON_B: return BTN_A;
    case SDL_CONTROLLER_BUTTON_A: return BTN_B;
    case SDL_CONTROLLER_BUTTON_Y: return BTN_X;
    case SDL_CONTROLLER_BUTTON_X: return BTN_Y;
    case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: return BTN_L1;
    case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: return BTN_R1;
    case SDL_CONTROLLER_BUTTON_START: return BTN_START;
    case SDL_CONTROLLER_BUTTON_BACK: return BTN_SELECT;
    case SDL_CONTROLLER_BUTTON_GUIDE: return BTN_MENU;
    case SDL_CONTROLLER_BUTTON_LEFTSTICK: return BTN_L3;
    case SDL_CONTROLLER_BUTTON_RIGHTSTICK: return BTN_R3;
    default: return BTN_COUNT;
  }
}

Button mapJoyButton(Uint8 b) {
  switch (b) {
    case 1: return BTN_A;
    case 0: return BTN_B;
    case 3: return BTN_X;
    case 2: return BTN_Y;
    case 4: return BTN_L1;
    case 5: return BTN_R1;
    case 6: return BTN_SELECT;
    case 7: return BTN_START;
    case 8: return BTN_MENU;
    case 9: return BTN_L3;
    case 10: return BTN_R3;
    case 11: return BTN_L2;
    case 12: return BTN_R2;
    default: return BTN_COUNT;
  }
}

int runHeadless(const Options& o) {
  Engine engine(kSampleRate);
  App app(engine, o.dataDir);
  if (!setupPatch(engine, app, o)) return 1;
  if (!o.savePatch.empty()) {
    if (!engine.savePatch(o.savePatch)) {
      std::fprintf(stderr, "could not write %s\n", o.savePatch.c_str());
      return 1;
    }
    if (o.screenshot.empty() && o.renderWav.empty() && !o.bench && !o.benchUi) return 0;
  }

  if (!o.renderWav.empty()) {
    std::vector<float> data;
    auto t0 = std::chrono::steady_clock::now();
    runOffline(engine, o.seconds, o.gateEvery, &data);
    float el = std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
    writeWav(o.renderWav, data, static_cast<int>(kSampleRate));
    float peak = 0.f;
    double rms = 0.0;
    for (float x : data) {
      peak = std::max(peak, std::fabs(x));
      rms += x * x;
    }
    rms = std::sqrt(rms / std::max<size_t>(1, data.size()));
    std::printf("rendered %.1fs in %.2fs (%.1fx realtime), peak %.3f rms %.4f, "
                "non-finite frames %lld -> %s\n",
                o.seconds, el, o.seconds / el, peak, rms,
                static_cast<long long>(engine.nonFiniteFrames()), o.renderWav.c_str());
    return 0;
  }
  if (o.bench) {
    auto t0 = std::chrono::steady_clock::now();
    runOffline(engine, o.seconds, o.gateEvery, nullptr);
    float el = std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
    std::printf("bench: %.1fs of audio in %.3fs = %.1f%% of one core\n", o.seconds, el,
                100.f * el / o.seconds);
    return 0;
  }

  if (o.benchUi) {
    // Renders 240 frames while walking the focus across the rack.
    Canvas canvas(App::kScreenW, App::kScreenH);
    app.setZoom(o.zoom);
    std::vector<float> scratch(512);
    auto t0 = std::chrono::steady_clock::now();
    const int frames = 240;
    for (int i = 0; i < frames; ++i) {
      if (i % 20 == 0) app.buttonEvent(BTN_RIGHT, true), app.buttonEvent(BTN_RIGHT, false);
      engine.process(scratch.data(), nullptr, 0, 256);
      app.update(1.f / 60.f);
      app.render(canvas);
    }
    float el = std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
    std::printf("bench-ui: %.2f ms/frame (zoom %d)\n", 1000.f * el / frames, o.zoom);
    return 0;
  }

  // Screenshot: run the engine briefly so lights and displays show activity.
  runOffline(engine, static_cast<float>(o.frames) / kSampleRate, 0.25f, nullptr);
  app.setZoom(o.zoom);
  app.focusModule(o.focus, o.widget);
  if (o.menu == "main") app.openMainMenuForTest();
  else if (o.menu == "module") app.openModuleMenuForTest();
  else if (o.menu == "browser") app.openBrowserForTest();
  app.update(0.016f);
  Canvas canvas(App::kScreenW, App::kScreenH);
  app.render(canvas);
  std::vector<uint8_t> rgb(static_cast<size_t>(App::kScreenW * App::kScreenH * 3));
  const uint32_t* px = canvas.pixels();
  for (int i = 0; i < App::kScreenW * App::kScreenH; ++i) {
    rgb[static_cast<size_t>(3 * i)] = static_cast<uint8_t>(px[i] >> 16);
    rgb[static_cast<size_t>(3 * i + 1)] = static_cast<uint8_t>(px[i] >> 8);
    rgb[static_cast<size_t>(3 * i + 2)] = static_cast<uint8_t>(px[i]);
  }
  if (!stbi_write_png(o.screenshot.c_str(), App::kScreenW, App::kScreenH, 3, rgb.data(),
                      App::kScreenW * 3)) {
    std::fprintf(stderr, "could not write %s\n", o.screenshot.c_str());
    return 1;
  }
  std::printf("wrote %s\n", o.screenshot.c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options o = parseArgs(argc, argv);

  Font font;
  if (!font.loadEmbedded()) {
    std::fprintf(stderr, "font load failed\n");
    return 1;
  }
  Canvas::setFont(&font);

  if (o.listModules) {
    for (const Model* m : allModels()) {
      std::printf("%-22s %-28s %3dHP  %s\n", m->slug.c_str(), m->name.c_str(), m->panel().hp,
                  m->basedOn.c_str());
    }
    return 0;
  }
  if (!o.screenshot.empty() || !o.renderWav.empty() || o.bench || o.benchUi ||
      !o.savePatch.empty())
    return runHeadless(o);

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK | SDL_INIT_TIMER) !=
      0) {
    std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }
  // Audio separately: without it the rack still runs (silently).
  if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
    std::fprintf(stderr, "SDL audio init failed: %s (continuing silently)\n", SDL_GetError());
  }
  SDL_ShowCursor(SDL_DISABLE);

  SDL_Window* win = SDL_CreateWindow("BrickRack", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                     App::kScreenW, App::kScreenH, SDL_WINDOW_SHOWN);
  if (!win) {
    std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_Renderer* ren =
      SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!ren) ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
  if (!ren) {
    std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_RenderSetLogicalSize(ren, App::kScreenW, App::kScreenH);
  SDL_Texture* tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                       App::kScreenW, App::kScreenH);

  Engine engine(kSampleRate);
  App app(engine, o.dataDir);
  if (!o.patch.empty()) setupPatch(engine, app, o);
  else if (o.demo) app.buildDefaultPatch();
  else app.loadStartupPatch();

  AudioContext actx;
  actx.engine = &engine;
  actx.micScratch.assign(16384, 0.f);  // no allocation on the audio thread
  int bufferFrames = 1024;
  if (const char* b = std::getenv("BRICKRACK_BUFFER")) bufferFrames = std::max(64, std::atoi(b));
  SDL_AudioSpec want{}, have{};
  want.freq = static_cast<int>(kSampleRate);
  want.format = AUDIO_F32SYS;
  want.channels = 2;
  want.samples = static_cast<Uint16>(bufferFrames);
  want.callback = audioCallback;
  want.userdata = &actx;
  SDL_AudioDeviceID dev = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
  if (!dev) {
    std::fprintf(stderr, "SDL_OpenAudioDevice failed: %s (continuing silently)\n", SDL_GetError());
  } else {
    std::printf("audio: %d Hz, %d ch, %d frames\n", have.freq, have.channels, have.samples);
    SDL_PauseAudioDevice(dev, 0);
  }
  SDL_AudioDeviceID capDev = 0;

  SDL_GameController* pad = nullptr;
  SDL_Joystick* joy = nullptr;
  auto openPad = [&]() {
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
      if (SDL_IsGameController(i)) {
        pad = SDL_GameControllerOpen(i);
        if (pad) {
          std::printf("controller: %s\n", SDL_GameControllerName(pad));
          return;
        }
      }
    }
    if (SDL_NumJoysticks() > 0) {
      joy = SDL_JoystickOpen(0);
      if (joy) std::printf("joystick: %s\n", SDL_JoystickName(joy));
    }
  };
  openPad();

  Canvas canvas(App::kScreenW, App::kScreenH);
  bool running = true;
  auto last = std::chrono::steady_clock::now();
  bool trigL = false, trigR = false;
  uint8_t hatState = 0;
  float micCheckTimer = 0.f;
  float micRetryDelay = 0.f;  // backoff after a failed open

  while (running && !app.wantsQuit()) {
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
      if (o.inputDebug) {
        if (ev.type == SDL_JOYBUTTONDOWN) std::printf("joy button %d\n", ev.jbutton.button);
        if (ev.type == SDL_JOYAXISMOTION && std::abs(ev.jaxis.value) > 8000)
          std::printf("joy axis %d = %d\n", ev.jaxis.axis, ev.jaxis.value);
        if (ev.type == SDL_JOYHATMOTION) std::printf("joy hat %d\n", ev.jhat.value);
        if (ev.type == SDL_CONTROLLERBUTTONDOWN) std::printf("pad button %d\n", ev.cbutton.button);
        if (ev.type == SDL_KEYDOWN) std::printf("key %d\n", ev.key.keysym.sym);
        std::fflush(stdout);
      }
      switch (ev.type) {
        case SDL_QUIT: running = false; break;
        case SDL_KEYDOWN:
        case SDL_KEYUP: {
          if (ev.key.repeat) break;
          bool down = ev.type == SDL_KEYDOWN;
          Button b = mapKey(ev.key.keysym.sym);
          if (b != BTN_COUNT) app.buttonEvent(b, down);
          // Desktop: IJKL = right stick, WASD-ish TFGH = left stick.
          SDL_Keycode k = ev.key.keysym.sym;
          float v = down ? 1.f : 0.f;
          if (k == SDLK_i) app.axisEvent(AXIS_RY, -v);
          if (k == SDLK_k) app.axisEvent(AXIS_RY, v);
          if (k == SDLK_j) app.axisEvent(AXIS_RX, -v);
          if (k == SDLK_l) app.axisEvent(AXIS_RX, v);
          if (k == SDLK_t) app.axisEvent(AXIS_LY, -v);
          if (k == SDLK_g) app.axisEvent(AXIS_LY, v);
          if (k == SDLK_f) app.axisEvent(AXIS_LX, -v);
          if (k == SDLK_h) app.axisEvent(AXIS_LX, v);
          break;
        }
        case SDL_CONTROLLERDEVICEADDED:
          if (!pad && !joy) openPad();
          break;
        case SDL_CONTROLLERDEVICEREMOVED:
          if (pad && ev.cdevice.which ==
                         SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(pad))) {
            SDL_GameControllerClose(pad);
            pad = nullptr;
            openPad();
          }
          break;
        case SDL_CONTROLLERBUTTONDOWN:
        case SDL_CONTROLLERBUTTONUP: {
          Button b = mapControllerButton(ev.cbutton.button);
          if (b != BTN_COUNT) app.buttonEvent(b, ev.type == SDL_CONTROLLERBUTTONDOWN);
          break;
        }
        case SDL_CONTROLLERAXISMOTION: {
          float v = ev.caxis.value / 32767.f;
          switch (ev.caxis.axis) {
            case SDL_CONTROLLER_AXIS_LEFTX: app.axisEvent(AXIS_LX, v); break;
            case SDL_CONTROLLER_AXIS_LEFTY: app.axisEvent(AXIS_LY, v); break;
            case SDL_CONTROLLER_AXIS_RIGHTX: app.axisEvent(AXIS_RX, v); break;
            case SDL_CONTROLLER_AXIS_RIGHTY: app.axisEvent(AXIS_RY, v); break;
            case SDL_CONTROLLER_AXIS_TRIGGERLEFT: {
              bool d = trigL ? v > 0.3f : v > 0.6f;  // hysteresis
              if (d != trigL) app.buttonEvent(BTN_L2, d);
              trigL = d;
              break;
            }
            case SDL_CONTROLLER_AXIS_TRIGGERRIGHT: {
              bool d = trigR ? v > 0.3f : v > 0.6f;
              if (d != trigR) app.buttonEvent(BTN_R2, d);
              trigR = d;
              break;
            }
            default: break;
          }
          break;
        }
        case SDL_JOYBUTTONDOWN:
        case SDL_JOYBUTTONUP:
          if (!pad) {
            Button b = mapJoyButton(ev.jbutton.button);
            if (b != BTN_COUNT) app.buttonEvent(b, ev.type == SDL_JOYBUTTONDOWN);
          }
          break;
        case SDL_JOYAXISMOTION:
          if (!pad) {
            float v = ev.jaxis.value / 32767.f;
            switch (ev.jaxis.axis) {
              case 0: app.axisEvent(AXIS_LX, v); break;
              case 1: app.axisEvent(AXIS_LY, v); break;
              case 3: app.axisEvent(AXIS_RX, v); break;
              case 4: app.axisEvent(AXIS_RY, v); break;
              case 2: {
                bool d = v > 0.f;
                if (d != trigL) app.buttonEvent(BTN_L2, d);
                trigL = d;
                break;
              }
              case 5: {
                bool d = v > 0.f;
                if (d != trigR) app.buttonEvent(BTN_R2, d);
                trigR = d;
                break;
              }
              default: break;
            }
          }
          break;
        case SDL_JOYHATMOTION:
          if (!pad) {
            uint8_t h = ev.jhat.value;
            const struct { uint8_t mask; Button b; } dirs[] = {
                {SDL_HAT_UP, BTN_UP}, {SDL_HAT_DOWN, BTN_DOWN},
                {SDL_HAT_LEFT, BTN_LEFT}, {SDL_HAT_RIGHT, BTN_RIGHT}};
            for (auto& d : dirs) {
              bool now = (h & d.mask) != 0, before = (hatState & d.mask) != 0;
              if (now != before) app.buttonEvent(d.b, now);
            }
            hatState = h;
          }
          break;
        default: break;
      }
    }

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last).count();
    last = now;
    dt = std::min(dt, 0.1f);
    app.update(dt);

    // Open the microphone only while a MIC IN module is in the rack (keeping
    // capture open permanently can upset the TrimUI ALSA driver).
    micCheckTimer += dt;
    if (micRetryDelay > 0.f) micRetryDelay -= dt;
    if (micCheckTimer > 0.5f) {
      micCheckTimer = 0.f;
      bool want = engineWantsMic(engine);
      if (!want) micRetryDelay = 0.f;  // a newly added MIC IN retries at once
      if (want && !capDev && micRetryDelay <= 0.f) {
        SDL_AudioSpec cw{}, ch{};
        cw.freq = static_cast<int>(kSampleRate);
        cw.format = AUDIO_F32SYS;
        cw.channels = 1;
        cw.samples = static_cast<Uint16>(bufferFrames);
        cw.callback = captureCallback;
        cw.userdata = &actx;
        capDev = SDL_OpenAudioDevice(nullptr, 1, &cw, &ch, 0);
        if (capDev) {
          actx.mic.tail.store(actx.mic.head.load());  // drop stale samples
          actx.micOpen.store(true);
          SDL_PauseAudioDevice(capDev, 0);
          std::printf("mic: opened %d Hz\n", ch.freq);
        } else {
          std::printf("mic: unavailable (%s), retrying in 30 s\n", SDL_GetError());
          micRetryDelay = 30.f;
        }
      } else if (!want && capDev) {
        actx.micOpen.store(false);
        SDL_CloseAudioDevice(capDev);
        capDev = 0;
      }
    }

    app.render(canvas);
    SDL_UpdateTexture(tex, nullptr, canvas.pixels(), App::kScreenW * 4);
    SDL_RenderClear(ren);
    SDL_RenderCopy(ren, tex, nullptr, nullptr);
    SDL_RenderPresent(ren);

    // Cap at ~60 fps if vsync is unavailable.
    float frameTime =
        std::chrono::duration<float>(std::chrono::steady_clock::now() - now).count();
    if (frameTime < 0.016f) SDL_Delay(static_cast<Uint32>((0.016f - frameTime) * 1000.f));
  }

  app.saveAutosave();
  if (capDev) SDL_CloseAudioDevice(capDev);
  if (dev) SDL_CloseAudioDevice(dev);
  SDL_DestroyTexture(tex);
  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(win);
  SDL_Quit();
  return 0;
}
