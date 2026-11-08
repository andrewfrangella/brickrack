// Force-included when compiling the Mutable Instruments firmware sources (and
// wrappers that include them). The firmware was written against older
// toolchains that pulled these headers in transitively; modern GCC does not.
// This lets us build the firmware unmodified.
#pragma once
#ifdef __cplusplus
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#endif
