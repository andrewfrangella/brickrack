# Tidal Modulator 2 (after Mutable Instruments Tides, 2018)
list(APPEND BR_MI_SOURCES
  ${EURORACK_DIR}/tides2/poly_slope_generator.cc
  ${EURORACK_DIR}/tides2/resources.cc)
# Patched copy of tides2/ramp/ramp_extractor.cc (see ramp_extractor.cmake).
include(${CMAKE_CURRENT_LIST_DIR}/ramp_extractor.cmake)
list(APPEND BR_MI_SOURCES ${BR_RAMP_EXTRACTOR_CC})
