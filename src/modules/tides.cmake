# Tidal Modulator (after Mutable Instruments Tides, 2015)

# Generator::ComputeCutoffFrequency() returns up to 65536 + (pitch-dependent
# extra), and its callers read lut_cutoff[(frequency >> 7) + 1] from a
# 513-entry table: entry 513 (one past the end) at the default SMOOTHNESS and
# anything past it at high pitches. We leave the submodule untouched and
# compile a configure-time copy of generator.cc with the result clamped to
# 65535 (it includes its headers through -I).
set(BR_TIDES_GENERATOR_CC ${EURORACK_DIR}/tides/generator.cc)
file(READ ${BR_TIDES_GENERATOR_CC} BR_TIDES_GENERATOR_SRC)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BR_TIDES_GENERATOR_CC})
set(BR_TIDES_CUTOFF_BROKEN
  "  frequency += 32768;\n  if (frequency < 0) {\n    frequency = 0;\n  }\n  return frequency;")
set(BR_TIDES_CUTOFF_FIXED
  "  frequency += 32768;\n  if (frequency < 0) {\n    frequency = 0;\n  }\n  if (frequency > 65535) {  // BrickRack: stay inside lut_cutoff\n    frequency = 65535;\n  }\n  return frequency;")
string(FIND "${BR_TIDES_GENERATOR_SRC}" "${BR_TIDES_CUTOFF_BROKEN}" BR_TIDES_CUTOFF_POS)
if(BR_TIDES_CUTOFF_POS EQUAL -1)
  message(FATAL_ERROR "tides.cmake: tides/generator.cc changed upstream; "
    "re-check the lut_cutoff index fix.")
endif()
string(REPLACE "${BR_TIDES_CUTOFF_BROKEN}" "${BR_TIDES_CUTOFF_FIXED}"
  BR_TIDES_GENERATOR_SRC "${BR_TIDES_GENERATOR_SRC}")
set(BR_TIDES_FIX_DIR ${CMAKE_BINARY_DIR}/generated/tides_fix)
file(WRITE ${BR_TIDES_FIX_DIR}/generator.cc.tmp "${BR_TIDES_GENERATOR_SRC}")
configure_file(${BR_TIDES_FIX_DIR}/generator.cc.tmp
               ${BR_TIDES_FIX_DIR}/tides/generator.cc COPYONLY)

set(BR_TIDES_SOURCES
  ${BR_TIDES_FIX_DIR}/tides/generator.cc
  ${EURORACK_DIR}/tides/resources.cc)
list(APPEND BR_MI_SOURCES ${BR_TIDES_SOURCES})

# Tides (2015) and Tides 2 both live in `namespace tides` and both define
# tides::lookup_table_table (with different types), which would clash at link
# time and risk ODR violations. Compile the 2015 code, and its wrapper, with
# the namespace renamed. Quoted #include paths are not affected by the macro.
set_source_files_properties(${BR_TIDES_SOURCES} ${CMAKE_CURRENT_LIST_DIR}/tides.cc
  PROPERTIES COMPILE_DEFINITIONS "tides=tides2015")
