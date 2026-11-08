# Modal Synthesizer (after Mutable Instruments Elements)
file(GLOB ELEMENTS_DSP CONFIGURE_DEPENDS ${EURORACK_DIR}/elements/dsp/*.cc)
list(APPEND BR_MI_SOURCES ${ELEMENTS_DSP} ${EURORACK_DIR}/elements/resources.cc)
