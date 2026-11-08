# Texture Synthesizer 2 (after Mutable Instruments Beads)
set(BR_BEADS_SOURCES
  ${EURORACK_DIR}/beads/dsp/delay_clock.cc
  ${EURORACK_DIR}/beads/dsp/delay_controller.cc
  ${EURORACK_DIR}/beads/dsp/mu_law.cc
  ${EURORACK_DIR}/beads/dsp/processor.cc
  ${EURORACK_DIR}/beads/dsp/synth.cc
  ${EURORACK_DIR}/beads/dsp/trigger_generator.cc
  ${EURORACK_DIR}/beads/resources.cc)
list(APPEND BR_MI_SOURCES ${BR_BEADS_SOURCES})

# Everything is built with -DTEST (the firmware's desktop-test configuration),
# which enables a printf() in WavetableGranularPlayer whenever a grain is
# stolen. That would print from the audio thread. We leave the submodule
# untouched and compile synth.cc (and the wrapper, which sees the same inline
# code) against a configure-time copy of the header without that debug block,
# i.e. exactly what the firmware build compiles. -iquote directories are
# searched before -I ones for quoted includes.
set(BR_BEADS_WGP_H ${EURORACK_DIR}/beads/dsp/wavetable_granular_player.h)
file(READ ${BR_BEADS_WGP_H} BR_BEADS_WGP_SRC)
string(REGEX REPLACE
  "#ifdef TEST\n[^#]*printf\\(\"stealing[^#]*#endif  // TEST\n" ""
  BR_BEADS_WGP_FIXED "${BR_BEADS_WGP_SRC}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BR_BEADS_WGP_H})
if(NOT BR_BEADS_WGP_FIXED STREQUAL BR_BEADS_WGP_SRC)
  set(BR_BEADS_FIX_DIR ${CMAKE_BINARY_DIR}/generated/beads_fix)
  file(WRITE ${BR_BEADS_FIX_DIR}/beads/dsp/wavetable_granular_player.h.tmp "${BR_BEADS_WGP_FIXED}")
  configure_file(${BR_BEADS_FIX_DIR}/beads/dsp/wavetable_granular_player.h.tmp
                 ${BR_BEADS_FIX_DIR}/beads/dsp/wavetable_granular_player.h COPYONLY)
  set_source_files_properties(${EURORACK_DIR}/beads/dsp/synth.cc ${CMAKE_CURRENT_LIST_DIR}/beads.cc
    PROPERTIES COMPILE_OPTIONS "-iquote;${BR_BEADS_FIX_DIR}")
else()
  # Nothing removed: fine if the debug printf is gone upstream, otherwise the
  # text changed and the regex needs updating.
  string(FIND "${BR_BEADS_WGP_SRC}" "printf(\"stealing" BR_BEADS_WGP_PRINTF_POS)
  if(NOT BR_BEADS_WGP_PRINTF_POS EQUAL -1)
    message(FATAL_ERROR "beads.cmake: beads/dsp/wavetable_granular_player.h changed "
      "upstream; re-check the removal of the TEST printf.")
  endif()
endif()
