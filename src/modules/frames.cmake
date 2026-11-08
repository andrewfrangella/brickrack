# Keyframer / Mixer (after Mutable Instruments Frames)
list(APPEND BR_MI_SOURCES
  ${EURORACK_DIR}/frames/poly_lfo.cc
  ${EURORACK_DIR}/frames/resources.cc)

# Keyframer::Evaluate() gets scale = 65536 when the frame sits exactly on a
# keyframe (FindKeyframe() is a lower_bound; always the case in sequencer
# mode), and Easing() then reads entry 1025 of its 1025-entry curve tables.
# We leave the submodule untouched and compile a configure-time copy of
# keyframer.cc with the scale clamped to 65535 (it includes its headers
# through -I).
set(BR_FRAMES_KEYFRAMER_CC ${EURORACK_DIR}/frames/keyframer.cc)
file(READ ${BR_FRAMES_KEYFRAMER_CC} BR_FRAMES_KEYFRAMER_SRC)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BR_FRAMES_KEYFRAMER_CC})
set(BR_FRAMES_SCALE_BROKEN "      scale /= (b.timestamp - a.timestamp);\n")
set(BR_FRAMES_SCALE_FIXED
  "      scale /= (b.timestamp - a.timestamp);\n      if (scale > 65535) scale = 65535;  // BrickRack: stay inside the easing tables\n")
string(FIND "${BR_FRAMES_KEYFRAMER_SRC}" "${BR_FRAMES_SCALE_BROKEN}" BR_FRAMES_SCALE_POS)
if(BR_FRAMES_SCALE_POS EQUAL -1)
  message(FATAL_ERROR "frames.cmake: frames/keyframer.cc changed upstream; "
    "re-check the easing table index fix.")
endif()
string(REPLACE "${BR_FRAMES_SCALE_BROKEN}" "${BR_FRAMES_SCALE_FIXED}"
  BR_FRAMES_KEYFRAMER_SRC "${BR_FRAMES_KEYFRAMER_SRC}")
set(BR_FRAMES_FIX_DIR ${CMAKE_BINARY_DIR}/generated/frames_fix)
file(WRITE ${BR_FRAMES_FIX_DIR}/keyframer.cc.tmp "${BR_FRAMES_KEYFRAMER_SRC}")
configure_file(${BR_FRAMES_FIX_DIR}/keyframer.cc.tmp
               ${BR_FRAMES_FIX_DIR}/frames/keyframer.cc COPYONLY)
list(APPEND BR_MI_SOURCES ${BR_FRAMES_FIX_DIR}/frames/keyframer.cc)
