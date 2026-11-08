# Dual Trigger Converter (after Mutable Instruments Peaks)
list(APPEND BR_MI_SOURCES
  ${EURORACK_DIR}/peaks/drums/bass_drum.cc
  ${EURORACK_DIR}/peaks/drums/fm_drum.cc
  ${EURORACK_DIR}/peaks/drums/high_hat.cc
  ${EURORACK_DIR}/peaks/drums/snare_drum.cc
  ${EURORACK_DIR}/peaks/modulations/lfo.cc
  ${EURORACK_DIR}/peaks/modulations/multistage_envelope.cc
  ${EURORACK_DIR}/peaks/number_station/number_station.cc
  ${EURORACK_DIR}/peaks/pulse_processor/pulse_randomizer.cc
  ${EURORACK_DIR}/peaks/pulse_processor/pulse_shaper.cc
  ${EURORACK_DIR}/peaks/processors.cc
  ${EURORACK_DIR}/peaks/resources.cc)
