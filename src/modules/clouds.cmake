# Texture Synthesizer (after Mutable Instruments Clouds)
set(BR_CLOUDS_SOURCES
  ${EURORACK_DIR}/clouds/dsp/correlator.cc
  ${EURORACK_DIR}/clouds/dsp/granular_processor.cc
  ${EURORACK_DIR}/clouds/dsp/mu_law.cc
  ${EURORACK_DIR}/clouds/dsp/pvoc/frame_transformation.cc
  ${EURORACK_DIR}/clouds/dsp/pvoc/phase_vocoder.cc
  ${EURORACK_DIR}/clouds/dsp/pvoc/stft.cc
  ${EURORACK_DIR}/clouds/resources.cc)
list(APPEND BR_MI_SOURCES ${BR_CLOUDS_SOURCES})

set(BR_CLOUDS_FIX_DIR ${CMAKE_BINARY_DIR}/generated/clouds_fix)

# Upstream regression: two 2023 "duplicate assignment" clean-ups (fbb53ba and
# 0e3756f) each removed one of the two `done_ = false;` lines in
# clouds/dsp/window.h Window::Start(), and the merge dropped both. Without it a
# WSOLA window never restarts, so the "stretch" playback mode is silent. The
# shipped firmware had the line. We leave the submodule untouched and compile
# against a configure-time copy of window.h with the line restored; -iquote
# directories are searched before -I ones for #include "clouds/dsp/window.h".
set(BR_CLOUDS_WINDOW_H ${EURORACK_DIR}/clouds/dsp/window.h)
file(READ ${BR_CLOUDS_WINDOW_H} BR_CLOUDS_WINDOW_SRC)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BR_CLOUDS_WINDOW_H})
set(BR_CLOUDS_WINDOW_BROKEN "    phase_ = 0;\n    regenerated_ = false;\n    envelope_phase_increment_")
set(BR_CLOUDS_WINDOW_FIXED "    phase_ = 0;\n    done_ = false;\n    regenerated_ = false;\n    envelope_phase_increment_")
string(FIND "${BR_CLOUDS_WINDOW_SRC}" "${BR_CLOUDS_WINDOW_BROKEN}" BR_CLOUDS_WINDOW_POS)
string(FIND "${BR_CLOUDS_WINDOW_SRC}" "${BR_CLOUDS_WINDOW_FIXED}" BR_CLOUDS_WINDOW_FIXED_POS)
if(NOT BR_CLOUDS_WINDOW_POS EQUAL -1)
  string(REPLACE "${BR_CLOUDS_WINDOW_BROKEN}" "${BR_CLOUDS_WINDOW_FIXED}"
    BR_CLOUDS_WINDOW_SRC "${BR_CLOUDS_WINDOW_SRC}")
elseif(BR_CLOUDS_WINDOW_FIXED_POS EQUAL -1)
  # Neither the broken text nor the fixed one: upstream changed, re-check.
  message(FATAL_ERROR "clouds.cmake: clouds/dsp/window.h changed upstream; "
    "re-check the Window::Start() done_ fix.")
endif()
file(WRITE ${BR_CLOUDS_FIX_DIR}/clouds/dsp/window.h.tmp "${BR_CLOUDS_WINDOW_SRC}")
configure_file(${BR_CLOUDS_FIX_DIR}/clouds/dsp/window.h.tmp
               ${BR_CLOUDS_FIX_DIR}/clouds/dsp/window.h COPYONLY)

# Grain::RenderEnvelope() reads lut_window (4097 entries) at gain = 1 (the top
# of every grain's envelope), i.e. entries 4096 and 4097, the latter with zero
# weight, and below entry 0 once the phase passes 2 (gain < 0). Harmless on the
# hardware, but they are reads outside the table (and stop AddressSanitizer
# runs). Same approach: a copy of grain.h with the lookup index kept in [0, 1).
set(BR_CLOUDS_GRAIN_H ${EURORACK_DIR}/clouds/dsp/grain.h)
file(READ ${BR_CLOUDS_GRAIN_H} BR_CLOUDS_GRAIN_SRC)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BR_CLOUDS_GRAIN_H})
set(BR_CLOUDS_GRAIN_BROKEN "window = stmlib::Interpolate(lut_window, gain, 4096.0f);")
string(FIND "${BR_CLOUDS_GRAIN_SRC}" "${BR_CLOUDS_GRAIN_BROKEN}" BR_CLOUDS_GRAIN_POS)
if(BR_CLOUDS_GRAIN_POS EQUAL -1)
  message(FATAL_ERROR "clouds.cmake: clouds/dsp/grain.h changed upstream; "
    "re-check the lut_window index fix.")
endif()
string(REPLACE "${BR_CLOUDS_GRAIN_BROKEN}"
  "window = stmlib::Interpolate(lut_window, gain < 0.0f ? 0.0f : (gain < 0.99999994f ? gain : 0.99999994f), 4096.0f);  // BrickRack"
  BR_CLOUDS_GRAIN_SRC "${BR_CLOUDS_GRAIN_SRC}")
file(WRITE ${BR_CLOUDS_FIX_DIR}/clouds/dsp/grain.h.tmp "${BR_CLOUDS_GRAIN_SRC}")
configure_file(${BR_CLOUDS_FIX_DIR}/clouds/dsp/grain.h.tmp
               ${BR_CLOUDS_FIX_DIR}/clouds/dsp/grain.h COPYONLY)

# OBJECT_DEPENDS: the compiler's dependency files still point at the
# submodule's headers in an existing build directory, so rebuild explicitly
# whenever a patched copy changes.
set_source_files_properties(${BR_CLOUDS_SOURCES} ${CMAKE_CURRENT_LIST_DIR}/clouds.cc
  PROPERTIES COMPILE_OPTIONS "-iquote;${BR_CLOUDS_FIX_DIR}"
  OBJECT_DEPENDS "${BR_CLOUDS_FIX_DIR}/clouds/dsp/window.h;${BR_CLOUDS_FIX_DIR}/clouds/dsp/grain.h")
