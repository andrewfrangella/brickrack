# Meta Modulator (after Mutable Instruments Warps)
file(GLOB WARPS_DSP CONFIGURE_DEPENDS ${EURORACK_DIR}/warps/dsp/*.cc)
list(REMOVE_ITEM WARPS_DSP ${EURORACK_DIR}/warps/dsp/modulator.cc)
list(APPEND BR_MI_SOURCES ${WARPS_DSP} ${EURORACK_DIR}/warps/resources.cc)

# Modulator::Xmod<ALGORITHM_FOLD> indexes lut_bipolar_fold (4097 entries) with
# an unbounded value: at exact full scale it reads one float past the end, and
# a runaway carrier reads arbitrarily far (segfault). We leave the submodule
# untouched and compile a configure-time copy of modulator.cc whose fold index
# is constrained to the table (it includes its headers through -I).
set(BR_WARPS_MODULATOR_CC ${EURORACK_DIR}/warps/dsp/modulator.cc)
file(READ ${BR_WARPS_MODULATOR_CC} BR_WARPS_MODULATOR_SRC)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BR_WARPS_MODULATOR_CC})
set(BR_WARPS_FOLD_BROKEN
  "  const float kScale = 2048.0f / ((1.0f + 1.0f + 0.25f) * 1.02f);\n  return Interpolate(lut_bipolar_fold + 2048, sum, kScale);")
set(BR_WARPS_FOLD_FIXED
  "  const float kScale = 2048.0f / ((1.0f + 1.0f + 0.25f) * 1.02f);\n  CONSTRAIN(sum, -2047.0f / kScale, 2047.0f / kScale);  // BrickRack: stay in the table\n  return Interpolate(lut_bipolar_fold + 2048, sum, kScale);")
string(FIND "${BR_WARPS_MODULATOR_SRC}" "${BR_WARPS_FOLD_BROKEN}" BR_WARPS_FOLD_POS)
if(BR_WARPS_FOLD_POS EQUAL -1)
  message(FATAL_ERROR "warps.cmake: warps/dsp/modulator.cc changed upstream; "
    "re-check the ALGORITHM_FOLD table index fix.")
endif()
string(REPLACE "${BR_WARPS_FOLD_BROKEN}" "${BR_WARPS_FOLD_FIXED}"
  BR_WARPS_MODULATOR_SRC "${BR_WARPS_MODULATOR_SRC}")
set(BR_WARPS_FIX_DIR ${CMAKE_BINARY_DIR}/generated/warps_fix)
file(WRITE ${BR_WARPS_FIX_DIR}/modulator.cc.tmp "${BR_WARPS_MODULATOR_SRC}")
configure_file(${BR_WARPS_FIX_DIR}/modulator.cc.tmp
               ${BR_WARPS_FIX_DIR}/warps/dsp/modulator.cc COPYONLY)
list(APPEND BR_MI_SOURCES ${BR_WARPS_FIX_DIR}/warps/dsp/modulator.cc)
