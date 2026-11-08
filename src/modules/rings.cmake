# Resonator (after Mutable Instruments Rings)
file(GLOB RINGS_DSP CONFIGURE_DEPENDS ${EURORACK_DIR}/rings/dsp/*.cc)
list(APPEND BR_MI_SOURCES ${RINGS_DSP} ${EURORACK_DIR}/rings/resources.cc)
