# Segment Generator (after Mutable Instruments Stages)
list(APPEND BR_MI_SOURCES
  ${EURORACK_DIR}/stages/segment_generator.cc
  ${EURORACK_DIR}/stages/resources.cc)
# Patched copy of tides2/ramp/ramp_extractor.cc (see ramp_extractor.cmake).
include(${CMAKE_CURRENT_LIST_DIR}/ramp_extractor.cmake)
list(APPEND BR_MI_SOURCES ${BR_RAMP_EXTRACTOR_CC})
