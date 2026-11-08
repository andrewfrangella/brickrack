# Macro Oscillator 2 (after Mutable Instruments Plaits)
file(GLOB_RECURSE PLAITS_DSP CONFIGURE_DEPENDS ${EURORACK_DIR}/plaits/dsp/*.cc)
list(APPEND BR_MI_SOURCES ${PLAITS_DSP} ${EURORACK_DIR}/plaits/resources.cc)
