#pragma once
#include <Arduino.h>
#include "tft_io.hpp"
namespace arduino {
    template<int8_t PinDC, int8_t PinRst, int8_t PinBL, typename Bus, uint8_t SoftResetCommand = 0x01>
    struct tft_driver {
        constexpr static const int8_t pin_dc = PinDC;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const int8_t pin_bl = PinBL;
        constexpr static const uint8_t swrst_command = SoftResetCommand;
        using bus = Bus;
        static bool initialize() {
            if(bus::initialize()) {
                
                if(pin_dc>-1) {
                    pinMode(pin_dc, OUTPUT);
                    digitalWrite(pin_dc, HIGH);
                }
                if(pin_rst>-1) {
                    pinMode(pin_rst, OUTPUT);
                    // TODO: is this necessary? Should it be LOW?
                    digitalWrite(pin_rst, HIGH);
                }
                //bus::end_write();
                reset();
                return true;
            }
            return false;
        }
        static void deinitialize() {
            bus::deinitialize();
        }
        static void reset() {
            if(pin_rst > -1) {
                digitalWrite(pin_rst, HIGH);
                delay(5);
                digitalWrite(pin_rst, LOW);
                delay(20);
                digitalWrite(pin_rst, HIGH);
            } else {
                send_command(swrst_command);
            }
            // wait for reset to complete
            delay(150);

        }
        static void send_command(uint8_t command) {
            bus::begin_write();
            dc_command();
            bus::write_raw8(command);
            dc_data();
            bus::end_write();
        }
        static void send_data8(uint8_t data) {
            bus::begin_write();
            dc_data();
            bus::write_raw8(data);
            bus::cs_low(); // allow more hold time for low VDI rail
            bus::end_write();
        }
        static uint8_t recv_command8(uint8_t command, uint8_t index) {
            uint8_t result = 0;
            send_command(command);
            bus::direction(INPUT);
            bus::cs_low();
            while(index--) result = bus::read_raw8();
            bus::direction(OUTPUT);
            bus::cs_high();
            return result;
        }
        static uint8_t recv_command16(uint8_t command, uint8_t index) {
            uint8_t result = recv_command8(command, index) << 8;
            result |= recv_command8(command, index);
            return result;
        }
        static uint8_t recv_command32(uint8_t command, uint8_t index) {
            uint8_t result = recv_command8(command, index) << 24;
            result |= recv_command8(command, index) << 16;
            result |= recv_command8(command, index) << 8;
            result |= recv_command8(command, index);
            return result;
        }
        inline static void dc_command() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_dc>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_dc - 32)&31));
            } else if(pin_dc>-1) {
                GPIO.out_w1tc = (1 << (pin_dc&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_dc,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void dc_data() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_dc>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_dc - 32)&31));
            } else if(pin_dc>-1) {
                GPIO.out_w1ts = (1 << (pin_dc &31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_dc,HIGH);
#endif // !OPTIMIZE_ESP32
        }
    };
}