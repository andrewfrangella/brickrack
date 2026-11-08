# Shared by Segment Generator (stages.cmake) and Tidal Modulator 2
# (tides2.cmake): both include this file, which defines BR_RAMP_EXTRACTOR_CC.
# It is also picked up by the fragment glob, hence the include guard.
include_guard(GLOBAL)

# Firmware bug: after Reset() the current pulse has total_duration 0, so a
# rising gate on the very first sample the extractor sees (e.g. a Stages
# single-segment PLL whose gate is already high) gives period = 0. In the
# audio-rate path expected_phase becomes +Inf and
# `while (expected_phase >= 1.0f) expected_phase -= 1.0f;` never ends; in the
# LFO path frequency_ becomes +Inf. We leave the submodule untouched and compile
# a configure-time copy of ramp_extractor.cc with the period clamped to >= 1
# sample (it includes its header through -I, so nothing else changes).
set(BR_RAMP_EXTRACTOR_SRC_CC ${EURORACK_DIR}/tides2/ramp/ramp_extractor.cc)
file(READ ${BR_RAMP_EXTRACTOR_SRC_CC} BR_RAMP_EXTRACTOR_SRC)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BR_RAMP_EXTRACTOR_SRC_CC})
set(BR_RAMP_EXTRACTOR_BROKEN "float period = float(p.total_duration);")
set(BR_RAMP_EXTRACTOR_FIXED "float period = float(std::max(p.total_duration, uint32_t(1)));")
string(FIND "${BR_RAMP_EXTRACTOR_SRC}" "${BR_RAMP_EXTRACTOR_BROKEN}" BR_RAMP_EXTRACTOR_POS)
string(FIND "${BR_RAMP_EXTRACTOR_SRC}" "${BR_RAMP_EXTRACTOR_FIXED}" BR_RAMP_EXTRACTOR_FIXED_POS)
if(NOT BR_RAMP_EXTRACTOR_POS EQUAL -1)
  string(REPLACE "${BR_RAMP_EXTRACTOR_BROKEN}" "${BR_RAMP_EXTRACTOR_FIXED}"
    BR_RAMP_EXTRACTOR_SRC "${BR_RAMP_EXTRACTOR_SRC}")
  set(BR_RAMP_EXTRACTOR_DIR ${CMAKE_BINARY_DIR}/generated/ramp_extractor_fix)
  file(WRITE ${BR_RAMP_EXTRACTOR_DIR}/ramp_extractor.cc.tmp "${BR_RAMP_EXTRACTOR_SRC}")
  configure_file(${BR_RAMP_EXTRACTOR_DIR}/ramp_extractor.cc.tmp
                 ${BR_RAMP_EXTRACTOR_DIR}/tides2/ramp/ramp_extractor.cc COPYONLY)
  set(BR_RAMP_EXTRACTOR_CC ${BR_RAMP_EXTRACTOR_DIR}/tides2/ramp/ramp_extractor.cc)
elseif(NOT BR_RAMP_EXTRACTOR_FIXED_POS EQUAL -1)
  set(BR_RAMP_EXTRACTOR_CC ${BR_RAMP_EXTRACTOR_SRC_CC})  # fixed upstream
else()
  message(FATAL_ERROR "ramp_extractor.cmake: tides2/ramp/ramp_extractor.cc changed "
    "upstream; re-check the zero-period fix.")
endif()
