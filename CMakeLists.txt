cmake_minimum_required(VERSION 3.16)
project(BrickRack VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(BRICKRACK_DEVICE "Build for TrimUI Brick / Brick Pro (tg5040, aarch64)" OFF)

set(EURORACK_DIR ${CMAKE_SOURCE_DIR}/third_party/eurorack)
if(NOT EXISTS ${EURORACK_DIR}/plaits/dsp/voice.h)
  message(FATAL_ERROR "third_party/eurorack is missing: run `git submodule update --init --recursive`")
endif()

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SDL2 sdl2)
endif()
if(NOT SDL2_FOUND)
  find_package(SDL2 REQUIRED)
  set(SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_DIRS})
  set(SDL2_LIBRARIES SDL2::SDL2)
endif()

# ---------------------------------------------------------------------------
# Embedded font
set(FONT_FILE ${CMAKE_SOURCE_DIR}/third_party/fonts/BarlowCondensed-SemiBold.ttf)
set(FONT_HEADER ${CMAKE_BINARY_DIR}/generated/font_data.h)
add_custom_command(
  OUTPUT ${FONT_HEADER}
  COMMAND ${CMAKE_COMMAND} -DINPUT=${FONT_FILE} -DOUTPUT=${FONT_HEADER}
          -P ${CMAKE_SOURCE_DIR}/cmake/embed.cmake
  DEPENDS ${FONT_FILE} ${CMAKE_SOURCE_DIR}/cmake/embed.cmake
  COMMENT "Embedding font")

# ---------------------------------------------------------------------------
# Original Mutable Instruments DSP code (unmodified, from the submodule).
# Each module wrapper ships a <name>.cmake fragment that appends the firmware
# sources it needs to BR_MI_SOURCES.
set(BR_MI_SOURCES
  ${EURORACK_DIR}/stmlib/dsp/units.cc
  ${EURORACK_DIR}/stmlib/dsp/atan.cc
  ${EURORACK_DIR}/stmlib/utils/random.cc)

file(GLOB BR_MODULE_FRAGMENTS CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/modules/*.cmake)
foreach(fragment ${BR_MODULE_FRAGMENTS})
  include(${fragment})
endforeach()
list(REMOVE_DUPLICATES BR_MI_SOURCES)

add_library(mi_dsp STATIC ${BR_MI_SOURCES})
target_include_directories(mi_dsp SYSTEM PUBLIC ${EURORACK_DIR})
target_compile_definitions(mi_dsp PUBLIC TEST)
target_compile_options(mi_dsp PUBLIC -include ${CMAKE_SOURCE_DIR}/src/core/mi_prelude.h)
# The firmware is written for 32-bit ARM GCC 4.x; silence its warnings.
target_compile_options(mi_dsp PRIVATE -w -fno-strict-aliasing)

# ---------------------------------------------------------------------------
# Application
file(GLOB BR_MODULE_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/modules/*.cc)

add_executable(brickrack
  src/core/module.cc
  src/core/model.cc
  src/core/engine.cc
  src/ui/canvas.cc
  src/ui/font.cc
  src/ui/app.cc
  src/platform/main.cc
  ${BR_MODULE_SOURCES}
  ${FONT_HEADER})

target_include_directories(brickrack PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_BINARY_DIR}/generated
  ${CMAKE_SOURCE_DIR}/third_party/stb
  ${SDL2_INCLUDE_DIRS})
target_link_directories(brickrack PRIVATE ${SDL2_LIBRARY_DIRS})
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(brickrack PRIVATE mi_dsp ${SDL2_LIBRARIES} Threads::Threads)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # glibc < 2.34 keeps pthreads in libpthread. With --as-needed it could be
  # dropped, and std::mutex would silently become a no-op. Force it.
  target_link_libraries(brickrack PRIVATE -Wl,--no-as-needed pthread -Wl,--as-needed)
endif()
target_compile_options(brickrack PRIVATE -Wall -Wno-unused-parameter)

# Fast math like the firmware's build, but keep NaN/Inf checks working.
set(BR_OPT_FLAGS -O2 -ffast-math -fno-finite-math-only)
if(BRICKRACK_DEVICE)
  # Allwinner A133P: quad Cortex-A53.
  list(APPEND BR_OPT_FLAGS -mcpu=cortex-a53)
endif()
target_compile_options(mi_dsp PRIVATE ${BR_OPT_FLAGS})
target_compile_options(brickrack PRIVATE ${BR_OPT_FLAGS})

# Unit test for the sample-rate adapter.
add_executable(resampler_test tests/resampler_test.cc)
target_include_directories(resampler_test PRIVATE ${CMAKE_SOURCE_DIR}/src)

if(BRICKRACK_DEVICE)
  # Avoid depending on the device's libstdc++ version.
  target_link_options(brickrack PRIVATE -static-libstdc++ -static-libgcc)
endif()
