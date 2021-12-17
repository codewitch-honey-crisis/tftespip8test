#pragma once
#include <Arduino.h>
#if defined(ESP32)
   #define OPTIMIZE_ESP32
   #define OPTIMIZE_DMA
#endif
#if defined(__AVR__)
    #define OPTIMIZE_AVR
#endif
#define FORCE_INLINE __attribute((always_inline))
namespace arduino {
    enum struct tft_io_type {
        spi = 0,
        i2c = 1,
        parallel8 = 2
    };
}
