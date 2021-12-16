#pragma once
#if !defined (SUPPORT_TRANSACTIONS)
  #define SUPPORT_TRANSACTIONS
#endif
#include <Arduino.h>
#include <SPI.h>
// The parallel code only works on the ESP32. The generic code for Arduino that is in place is non-functional in my tests. I'm not sure why, but it might be a timing issue.
#if defined(ESP32)
    #define OPTIMIZE_ESP32
    #define OPTIMIZE_DMA
#endif
#if defined(__AVR__)
    #define OPTIMIZE_AVR
#endif
#ifdef ESP32
    #include "soc/spi_reg.h"
    #include "driver/spi_master.h"
#endif
#define FORCE_INLINE __attribute((always_inline))
namespace arduino {
    enum struct tft_io_type {
        spi = 0,
        i2c = 1,
        parallel8 = 2
    };
    template<uint8_t SpiHost,
        int8_t PinCS, 
        int8_t PinMosi, 
        int8_t PinMiso, 
        int8_t PinSClk, 
        uint8_t SpiMode = 0, 
        uint32_t SpiWriteSpeed=26*1000*1000, 
        uint32_t SpiReadSpeed=20*1000*1000, 
        bool SdaRead = (PinMiso < 0)
#ifdef OPTIMIZE_DMA
    , size_t DmaSize = 4120
    , uint8_t DmaChannel = 
    #ifdef ESP32
        1
    #else
    0
    #endif // ESP32
#endif // OPTIMIZE_DMA
    >
    struct tft_spi {
        constexpr static const tft_io_type type = tft_io_type::spi;
        constexpr static const bool readable = PinMiso > -1 || SdaRead;
        constexpr static const bool sda_read = PinMiso < 0 && SdaRead;
        constexpr static const size_t dma_size =
#ifdef OPTIMIZE_DMA
        DmaSize
#else
        0
#endif
        ;
constexpr static const uint8_t dma_channel =
#ifdef OPTIMIZE_DMA
        DmaChannel
#else
        0
#endif
        ;
        constexpr static const uint8_t spi_host = SpiHost;
        constexpr static const uint8_t spi_mode = SpiMode;
        constexpr static const uint32_t spi_write_speed = SpiWriteSpeed;
        constexpr static const uint32_t spi_read_speed = SpiReadSpeed;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_mosi = PinMosi;
        constexpr static const int8_t pin_miso = PinMiso;
        constexpr static const int8_t pin_sclk = PinSClk;
    private:
        static SPIClass spi;
        static bool lock_transaction;
        static bool in_transaction;
        static bool locked;
#ifdef OPTIMIZE_ESP32
        // Volatile for register reads:
        volatile static uint32_t* _spi_cmd;
        volatile static uint32_t* _spi_user;
        // Register writes only:
        volatile static uint32_t* _spi_mosi_dlen;
        volatile static uint32_t* _spi_w;
        #ifdef OPTIMIZE_DMA
        static spi_device_handle_t dma_hal;
        static uint8_t spi_busy_check;
        #endif
#else
        static uint32_t cs_pin_mask, sclk_pin_mask;
#endif // !OPTIMIZE_ESP32
    public:
        static bool initialize() {
            pinMode(pin_cs,OUTPUT);
            digitalWrite(pin_cs,HIGH);
            spi.begin(pin_sclk,pin_miso,pin_mosi,-1);
            lock_transaction = false;
            in_transaction = false;
            locked = true;
            return true;
        }
        static void deinitialize() {
            lock_transaction = false;
            in_transaction = false;
            locked = true;
        }
        static bool initialize_dma() {
#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            if(dma_size>0) {
                esp_err_t ret;
                spi_bus_config_t buscfg = {
                    .mosi_io_num = pin_mosi,
                    .miso_io_num = pin_miso,
                    .sclk_io_num = pin_sclk,
                    .quadwp_io_num = -1,
                    .quadhd_io_num = -1,
                    .max_transfer_sz = dma_size,
                    .flags = 0,
                    .intr_flags = 0
                };
                spi_device_interface_config_t devcfg = {
                    .command_bits = 0,
                    .address_bits = 0,
                    .dummy_bits = 0,
                    .mode = spi_mode,
                    .duty_cycle_pos = 0,
                    .cs_ena_pretrans = 0,
                    .cs_ena_posttrans = 0,
                    .clock_speed_hz = spi_write_speed,
                    .input_delay_ns = 0,
                    .spics_io_num = -1,
                    .flags = SPI_DEVICE_NO_DUMMY, //0,
                    .queue_size = 1,
                    .pre_cb = 0,
                    // callback to handle DMA chaining: (not working)
                    .post_cb = 0 //dma_chain_callback
                };
                ret = spi_bus_initialize((spi_host_device_t)spi_host, &buscfg, dma_channel);
                ESP_ERROR_CHECK(ret);
                ret = spi_bus_add_device((spi_host_device_t)spi_host, &devcfg, &dma_hal);
                ESP_ERROR_CHECK(ret);
                spi_busy_check = 0;

            }
#endif // defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            return true;
        }
        static void deinitialize_dma() {
#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            if(dma_size>0) {
                spi_bus_remove_device(dma_hal);
                spi_bus_free((spi_host_device_t)spi_host);
            }
#endif // defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)

        }
        inline static void start_transaction() FORCE_INLINE {
            in_transaction = true;
        }
        inline static void end_transaction() FORCE_INLINE {
            in_transaction = lock_transaction;
        }
        static void write_raw(const uint8_t* data, size_t length) {
            while(length--) {
                write_raw8(*data++);
            }
        }
        static void write_raw_dma(const uint8_t* data,size_t length) {
#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            if(dma_size == 0) {
                write_raw(data,length);
            } else {
                if(length==0) return;
                dma_wait();
                static spi_transaction_t trans;
                memset(&trans,0,sizeof(spi_transaction_t));
                trans.user=(void*)length;
                trans.tx_buffer = data;
                
                trans.length = ((length>dma_size)?dma_size:length) * 8;
                trans.flags = 0;
                esp_err_t ret = spi_device_queue_trans(dma_hal,&trans,portMAX_DELAY);
                assert(ret==ESP_OK);
                ++spi_busy_check;
            }
#else // !defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
            write_raw(data,length);
#endif // !defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
        }
        inline static void write_raw8(uint8_t value) FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            *_spi_mosi_dlen = 7;
            *_spi_w = value;
            *_spi_cmd = SPI_USR;
            while (*_spi_cmd & SPI_USR);
#elif defined(OPTIMIZE_AVR)
            SPDR=(C); 
            while (!(SPSR&_BV(SPIF)));
#else // !OPTIMIZE_ESP32
            spi.transfer(value);
#endif // !OPTIMIZE_ESP32
        }
        static void write_raw8_repeat(uint16_t value, size_t count) {
#ifdef OPTIMIZE_ESP32
            volatile uint32_t* spi_w = _spi_w;
            uint32_t val32 = (value<<24) | (value <<16) | (value<<8) | value;
            uint32_t i = 0;
            uint32_t rem = count & 0x1F;
            count =  count - rem;

            // Start with partial buffer pixels
            if (rem)
            {
            while (*_spi_cmd&SPI_USR);
            for (i=0; i < rem; ++i) *spi_w++ = val32;
            *_spi_mosi_dlen = (rem << 3) - 1;
            *_spi_cmd = SPI_USR;
            if (!count) return; //{while (*_spi_cmd&SPI_USR); return; }
            i = i>>1; while(i++<8) *spi_w++ = val32;
            }

            while (*_spi_cmd&SPI_USR);
            if (!rem) while (i++<8) *spi_w++ = val32;
            *_spi_mosi_dlen =  511;

            // End with full buffer to maximise useful time for downstream code
            while(count)
            {
            while (*_spi_cmd&SPI_USR);
            *_spi_cmd = SPI_USR;
                count -= 32;
            }
#else // !OPTIMIZE_ESP32
            while(count--) write_raw8(value);
#endif // !OPTIMIZE_ESP32
        }
        inline static void write_raw16(uint16_t value) FORCE_INLINE {
#if defined(OPTIMIZE_ESP32)
            *_spi_mosi_dlen = 15;
            *_spi_w = (value<<8)|(value>>8);
            *_spi_cmd = SPI_USR;
            while (*_spi_cmd & SPI_USR);
#elif defined(__AVR__)
            // AVR does not have 16 bit write
            write_raw8(value>>8);
            write_raw8(value);
#else // !OPTIMIZE_ESP32
            spi.transfer16(value);
#endif // !OPTIMIZE_ESP32
        }
        static void write_raw16_repeat(uint16_t value, size_t count) {
#ifdef OPTIMIZE_ESP32
            volatile uint32_t* spi_w = _spi_w;
            uint32_t val32 = (value<<8 | value >>8)<<16 | (value<<8 | value >>8);  
            uint32_t i = 0;
            uint32_t rem = count & 0x1F;
            count =  count - rem;

            // Start with partial buffer pixels
            if (rem)
            {
            while (*_spi_cmd&SPI_USR);
            for (i=0; i < rem; i+=2) *spi_w++ = val32;
            *_spi_mosi_dlen = (rem << 4) - 1;
            *_spi_cmd = SPI_USR;
            if (!count) return; //{while (*_spi_cmd&SPI_USR); return; }
            i = i>>1; while(i++<16) *spi_w++ = val32;
            }

            while (*_spi_cmd&SPI_USR);
            if (!rem) while (i++<16) *spi_w++ = val32;
            *_spi_mosi_dlen =  511;

            // End with full buffer to maximise useful time for downstream code
            while(count)
            {
            while (*_spi_cmd&SPI_USR);
            *_spi_cmd = SPI_USR;
                count -= 32;
            }
#else // !OPTIMIZE_ESP32
            while(count--) {
                write_raw16(value);
            }
#endif // !OPTIMIZE_ESP32

        }
        inline static void write_raw32(uint32_t value) FORCE_INLINE {
            write_raw16(value>>16);
            write_raw16(value);
        }
        static uint8_t read_raw8() {
#ifdef OPTIMIZE_ESP32
            return spi.transfer(0);
#else // !OPTIMIZE_ESP32
            if(sda_read) {
                uint8_t  ret = 0;
                for (uint8_t i = 0; i < 8; i++) {  // read results
                    ret <<= 1;
                    sclk_low();
                    if (digitalRead(pin_mosi)) ret |= 1;
                    sclk_high();
                }
                return ret;
            } else {
                return spi.transfer(0);
            }
#endif // !OPTIMIZE_ESP32
        }
        static void begin_write() {
            if (locked) {
                locked = false; // Flag to show SPI access now unlocked
                spi.beginTransaction(SPISettings(spi_write_speed, MSBFIRST, spi_mode)); // RP2040 SDK -> 68us delay!
                cs_low();
#ifdef OPTIMIZE_ESP32
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI;
                }
#endif // OPTIMIZE_ESP32
            }
        }
        static void end_write() {
            dma_wait();
            if(!in_transaction) {
                if(!locked) {
                    locked = true;
#ifdef OPTIMIZE_ESP32
                    while (*_spi_cmd&SPI_USR);
#endif // OPTIMIZE_ESP32
                    cs_high();
                    spi.endTransaction();
                }
#ifdef OPTIMIZE_ESP32
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN;
                }
#endif // OPTIMIZE_ESP32
            }
            
        }
        
        static void begin_read() {
            dma_wait();
            if (locked) {
                locked = false;
                spi.beginTransaction(SPISettings(spi_read_speed, MSBFIRST, spi_mode)); // RP2040 SDK -> 68us delay!
                cs_low();
            }
#ifdef OPTIMIZE_ESP32
            if(spi_mode==1||spi_mode==2) {
                *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN | SPI_CK_OUT_EDGE;
            } else {
                *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN;
            }
#endif // OPTIMIZE_ESP32
        }
        static void end_read() {
            if(!in_transaction) {
                if(!locked) {
                    locked = true;
                    cs_high();
                    spi.endTransaction();
                }
#ifdef OPTIMIZE_ESP32
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI;
                }
#endif // OPTIMIZE_ESP32
            }
        }
        
        inline static void direction(uint8_t direction) FORCE_INLINE {
            if(direction==INPUT) {
                begin_sda_read();
            } else {
                end_sda_read();
            }
        }
        static void dma_wait() {
            if(dma_size>0) {
#ifdef OPTIMIZE_ESP32
    #ifdef OPTIMIZE_DMA
                if(!spi_busy_check) return;
                spi_transaction_t* rtrans;
                esp_err_t ret;
                for (int i = 0; i < spi_busy_check; ++i) {
                    ret = spi_device_get_trans_result(dma_hal, &rtrans, portMAX_DELAY);
                    assert(ret == ESP_OK);
                }
                spi_busy_check = 0;
    #endif // OPTIMIZE_DMA
#endif // OPTIMIZE_ESP32
            }
        }
        static bool dma_busy() {
            if(dma_size>0) {
#ifdef OPTIMIZE_ESP32
    #ifdef OPTIMIZE_DMA
                if(!spi_busy_check) return false;
                spi_transaction_t* rtrans;
                esp_err_t ret;
                uint8_t checks = spi_busy_check;
                for (int i = 0; i < checks; ++i) {
                    ret = spi_device_get_trans_result(dma_hal, &rtrans, portMAX_DELAY);
                    if(ret == ESP_OK) spi_busy_check--;
                }
                if(spi_busy_check == 0) return false;
                return true;
    #endif // OPTIMIZE_DMA
#endif // OPTIMIZE_ESP32
            }
            return false;
        }
        inline static void cs_low() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_cs>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_cs - 32)&31));
            } else if(pin_cs>-1) {
                GPIO.out_w1tc = (1 << (pin_cs&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void cs_high() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_cs>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_cs - 32)&31));
            } else if(pin_cs>-1) {
                GPIO.out_w1ts = (1 << (pin_cs&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,HIGH);
#endif // !OPTIMIZE_ESP32
        }
        inline static void sclk_low() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_sclk>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_sclk - 32)&31));
            } else if(pin_sclk>-1) {
                GPIO.out_w1tc = (1 << (pin_sclk&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_sclk,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void sclk_high() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_sclk>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_sclk - 32)&31));
            } else if(pin_sclk>-1) {
                GPIO.out_w1ts = (1 << (pin_sclk&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,HIGH);
#endif // !OPTIMIZE_ESP32
        }
    private:
/*
#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
        static void IRAM_ATTR dma_chain_callback(spi_transaction_t* ptrans) {
            size_t read = (ptrans->length/8);
            size_t remaining = ((size_t)ptrans->user)-read;
            if(remaining>0) {
                static spi_transaction_t trans;
                memset(&trans,0,sizeof(spi_transaction_t));
                size_t ns = (remaining>dma_size)?dma_size:remaining;
                trans.user=(void*)remaining;
                trans.tx_buffer = ((const uint8_t*)ptrans->tx_buffer)+read;
                trans.length = ns * 8;
                trans.flags = 0;
                esp_err_t ret = spi_device_queue_trans(dma_hal,&trans,portMAX_DELAY);
                assert(ret==ESP_OK);
            }
        }
#endif // defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
*/
        static void begin_sda_read() {
            if(sda_read) {
#ifdef OPTIMIZE_ESP32
                pinMatrixOutDetach(pin_mosi, false, false);
                pinMode(pin_mosi, INPUT);
                pinMatrixInAttach(pin_mosi, VSPIQ_IN_IDX, false);
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN;
                }
#endif // OPTIMIZE_ESP32
            }
            cs_low();
        }
        static void end_sda_read() {
            if(sda_read) {
#ifdef OPTIMIZE_ESP32
                pinMode(pin_mosi, OUTPUT);
                pinMatrixOutAttach(pin_mosi, VSPID_OUT_IDX, false, false);
                pinMode(pin_miso, INPUT);
                pinMatrixInAttach(pin_miso, VSPIQ_IN_IDX, false);
                if(spi_mode==1||spi_mode==2) {
                    *_spi_user = SPI_USR_MOSI | SPI_CK_OUT_EDGE;
                } else {
                    *_spi_user = SPI_USR_MOSI;
                }
#endif // OPTIMIZE_ESP32
            }
        }
    };
    
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > bool tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::lock_transaction = false;

template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > bool tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::in_transaction = false;

    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > bool tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::locked = true;

#ifdef OPTIMIZE_ESP32
template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > volatile uint32_t* tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::_spi_cmd = (volatile uint32_t*)(SPI_CMD_REG(SpiHost));

template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > volatile uint32_t* tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::_spi_user = (volatile uint32_t*)(SPI_USER_REG(SpiHost));

template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode,uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed,  bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > volatile uint32_t* tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::_spi_mosi_dlen = (volatile uint32_t*)(SPI_MOSI_DLEN_REG(SpiHost));

template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > volatile uint32_t* tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::_spi_w = (volatile uint32_t*)(SPI_W0_REG(SpiHost));
#endif // OPTIMIZE_ESP32

#ifndef OPTIMIZE_ESP32
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > uint32_t tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::cs_pin_mask = digitalPinToBitMask(pin_cs);
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > uint32_t tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::sclk_pin_mask = digitalPinToBitMask(pin_sclk);
#endif

    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead
#ifdef OPTIMIZE_DMA
    , size_t DmaSize
    , uint8_t DmaChannel
#endif
    > SPIClass tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead
#ifdef OPTIMIZE_DMA
    ,DmaSize
    ,DmaChannel
#endif
    >::spi(SpiHost);

#if defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead, size_t DmaSize, uint8_t DmaChannel> uint8_t tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead,DmaSize,DmaChannel>::spi_busy_check = 0;
    template<uint8_t SpiHost,int8_t PinCS, int8_t PinMosi, int8_t PinMiso, int8_t PinSClk, uint8_t SpiMode, uint32_t SpiWriteSpeed, uint32_t SpiReadSpeed, bool SdaRead, size_t DmaSize, uint8_t DmaChannel> spi_device_handle_t tft_spi<SpiHost,PinCS,PinMosi,PinMiso,PinSClk,SpiMode,SpiWriteSpeed,SpiReadSpeed,SdaRead,DmaSize,DmaChannel>::dma_hal = {0};
#endif // defined(OPTIMIZE_ESP32) && defined(OPTIMIZE_DMA)

    template<int8_t PinCS, 
        int8_t PinWR, 
        int8_t PinRD,
        int8_t PinD0,
        int8_t PinD1,
        int8_t PinD2,
        int8_t PinD3,
        int8_t PinD4,
        int8_t PinD5,
        int8_t PinD6,
        int8_t PinD7>
    struct tft_p8 {
        constexpr static const tft_io_type type = tft_io_type::parallel8;
        constexpr static const bool readable = true;
        constexpr static const size_t dma_size = 0;
        constexpr static const int8_t pin_cs = PinCS;
        constexpr static const int8_t pin_wr = PinWR;
        constexpr static const int8_t pin_rd = PinRD;
        constexpr static const int8_t pin_d0 = PinD0;
        constexpr static const int8_t pin_d1 = PinD1;
        constexpr static const int8_t pin_d2 = PinD2;
        constexpr static const int8_t pin_d3 = PinD3;
        constexpr static const int8_t pin_d4 = PinD4;
        constexpr static const int8_t pin_d5 = PinD5;
        constexpr static const int8_t pin_d6 = PinD6;
        constexpr static const int8_t pin_d7 = PinD7;
#ifdef OPTIMIZE_ESP32
    private:
        constexpr static const bool has_data_low_pins = (pin_d0>=0 && pin_d0<32) || 
                                                        (pin_d1>=0 && pin_d1<32) ||
                                                        (pin_d2>=0 && pin_d2<32) ||
                                                        (pin_d3>=0 && pin_d3<32) ||
                                                        (pin_d4>=0 && pin_d4<32) ||
                                                        (pin_d5>=0 && pin_d5<32) ||
                                                        (pin_d6>=0 && pin_d6<32) ||
                                                        (pin_d7>=0 && pin_d7<32);
        constexpr static const bool has_data_high_pins = (pin_d0>31) || 
                                                        (pin_d1>31) || 
                                                        (pin_d2>31) || 
                                                        (pin_d3>31) || 
                                                        (pin_d4>31) || 
                                                        (pin_d5>31) || 
                                                        (pin_d6>31) || 
                                                        (pin_d7>31);
        constexpr static uint32_t get_dir_mask_low() {
            uint32_t result = 0;
            if(pin_d0>-1 && pin_d0<32) {
                result |= 1<<pin_d0;
            }
            if(pin_d1>-1 && pin_d1<32) {
                result |= 1<<pin_d1;
            }
            if(pin_d2>-1 && pin_d2<32) {
                result |= 1<<pin_d2;
            }
            if(pin_d3>-1 && pin_d3<32) {
                result |= 1<<pin_d3;
            }
            if(pin_d4>-1 && pin_d4<32) {
                result |= 1<<pin_d4;
            }
            if(pin_d5>-1 && pin_d5<32) {
                result |= 1<<pin_d5;
            }
            if(pin_d6>-1 && pin_d6<32) {
                result |= 1<<pin_d6;
            }
            if(pin_d7>-1 && pin_d7<32) {
                result |= 1<<pin_d7;
            }
            return result;
        }
        constexpr static uint32_t get_dir_mask_high() {
            uint32_t result = 0;
            if(pin_d0>31) {
                result |= 1<<((pin_d0-32)&31);
            }
            if(pin_d1>31) {
                result |= 1<<((pin_d1-32)&31);
            }
            if(pin_d2>31) {
                result |= 1<<((pin_d2-32)&31);
            }
            if(pin_d3>31) {
                result |= 1<<((pin_d3-32)&31);
            }
            if(pin_d4>31) {
                result |= 1<<((pin_d4-32)&31);
            }
            if(pin_d5>31) {
                result |= 1<<((pin_d5-32)&31);
            }
            if(pin_d6>31) {
                result |= 1<<((pin_d6-32)&31);
            }
            if(pin_d7>31) {
                result |= 1<<((pin_d7-32)&31);
            }
            return result;
        }
        
        constexpr static const uint32_t dir_mask_low = get_dir_mask_low();
        constexpr static const uint32_t dir_mask_high = get_dir_mask_high();
        
        inline static uint32_t clr_mask_low() FORCE_INLINE {
            if(pin_wr>31) {
                uint32_t result = dir_mask_low;
                wr_low();
                return result;
            } else if(pin_wr>-1) {
                return (dir_mask_low | 1<<pin_wr);
            }
            return 0;
        }
        inline static uint32_t clr_mask_high() FORCE_INLINE {
            if(pin_wr>31) {
                return (dir_mask_high | 1<<((pin_wr-32)&31));
            } else if(pin_wr>-1) {
                uint32_t result = dir_mask_high;
                wr_low();
                return result;
            }
            return 0;
        }
        static uint32_t xset_mask_low[256*has_data_low_pins];
        static uint32_t xset_mask_high[256*has_data_high_pins];
#endif // OPTIMIZE_ESP32
    public:
        static bool initialize() {
            if(pin_cs > -1) {
                pinMode(pin_cs,OUTPUT);
                digitalWrite(pin_cs,HIGH);
            }
            if(pin_wr > -1) {
                pinMode(pin_wr,OUTPUT);
                digitalWrite(pin_wr,HIGH);
            }
            if(pin_rd > -1) {
                pinMode(pin_rd,OUTPUT);
                digitalWrite(pin_rd,HIGH);
            }
            if(pin_d0 > -1) {
                pinMode(pin_d0,OUTPUT);
                digitalWrite(pin_d0,HIGH);
            }
            if(pin_d1 > -1) {
                pinMode(pin_d1,OUTPUT);
                digitalWrite(pin_d1,HIGH);
            }
            if(pin_d2 > -1) {
                pinMode(pin_d2,OUTPUT);
                digitalWrite(pin_d2,HIGH);
            }
            if(pin_d3 > -1) {
                pinMode(pin_d3,OUTPUT);
                digitalWrite(pin_d3,HIGH);
            }
            if(pin_d4 > -1) {
                pinMode(pin_d4,OUTPUT);
                digitalWrite(pin_d4,HIGH);
            }
            if(pin_d5 > -1) {
                pinMode(pin_d5,OUTPUT);
                digitalWrite(pin_d5,HIGH);
            }
            if(pin_d6 > -1) {
                pinMode(pin_d6,OUTPUT);
                digitalWrite(pin_d6,HIGH);
            }
            if(pin_d7 > -1) {
                pinMode(pin_d7,OUTPUT);
                digitalWrite(pin_d7,HIGH);
            }
#ifdef OPTIMIZE_ESP32
            if(has_data_low_pins) {
                for (int32_t c = 0; c<256; c++) {
                    xset_mask_low[c] = 0;
                    if ( pin_d0>-1 && pin_d0<32 && (c & 0x01)) xset_mask_low[c] |= (1 << pin_d0);
                    if ( pin_d1>-1 && pin_d1<32 && (c & 0x02) ) xset_mask_low[c] |= (1 << pin_d1);
                    if ( pin_d2>-1 && pin_d2<32 && (c & 0x04) ) xset_mask_low[c] |= (1 << pin_d2);
                    if ( pin_d3>-1 && pin_d3<32 && (c & 0x08) ) xset_mask_low[c] |= (1 << pin_d3);
                    if ( pin_d4>-1 && pin_d4<32 && (c & 0x10) ) xset_mask_low[c] |= (1 << pin_d4);
                    if ( pin_d5>-1 && pin_d5<32 && (c & 0x20) ) xset_mask_low[c] |= (1 << pin_d5);
                    if ( pin_d6>-1 && pin_d6<32 && (c & 0x40) ) xset_mask_low[c] |= (1 << pin_d6);
                    if ( pin_d7>-1 && pin_d7<32 && (c & 0x80) ) xset_mask_low[c] |= (1 << pin_d7);
                }
            }
            if(has_data_high_pins) {
                for (int32_t c = 0; c<256; c++) {
                    xset_mask_high[c] = 0;
                    if ( pin_d0>31 && (c & 0x01)) xset_mask_high[c] |= (1 << ((pin_d0-32)&31));
                    if ( pin_d1>31 && (c & 0x02) ) xset_mask_high[c] |= (1 << ((pin_d1-32)&31));
                    if ( pin_d2>31 && (c & 0x04) ) xset_mask_high[c] |= (1 << ((pin_d2-32)&31));
                    if ( pin_d3>31 && (c & 0x08) ) xset_mask_high[c] |= (1 << ((pin_d3-32)&31));
                    if ( pin_d4>31 && (c & 0x10) ) xset_mask_high[c] |= (1 << ((pin_d4-32)&31));
                    if ( pin_d5>31 && (c & 0x20) ) xset_mask_high[c] |= (1 << ((pin_d5-32)&31));
                    if ( pin_d6>31 && (c & 0x40) ) xset_mask_high[c] |= (1 << ((pin_d6-32)&31));
                    if ( pin_d7>31 && (c & 0x80) ) xset_mask_high[c] |= (1 << ((pin_d7-32)&31));
                }
            }
#endif // OPTIMIZE_ESP32
            // Set to output once again in case ESP8266 D6 (MISO) is used for CS
            if(pin_cs>-1) {
                pinMode(pin_cs, OUTPUT);
                digitalWrite(pin_cs, HIGH); // Chip select high (inactive)
            }
            return true;
        }
        static void deinitialize() {

        }
        inline static bool initialize_dma() FORCE_INLINE { return true; }
        inline static void deinitialize_dma() FORCE_INLINE {}
        inline static void start_transaction() FORCE_INLINE {
         
        }
        inline static void end_transaction() FORCE_INLINE {
         
        }
        static uint8_t read_raw8() {
            rd_low();
            uint8_t b = 0xAA;
#ifdef OPTIMIZE_ESP32
            if(has_data_low_pins && has_data_high_pins) {
                
                // 3x for bus access stabilization
                uint32_t pins_l = gpio_input_get();
                pins_l = gpio_input_get();
                pins_l = gpio_input_get();
                uint32_t pins_h = gpio_input_get_high();
                if(pin_d0>31) {
                    b = (((pins_h>>((pin_d0-32)&31))&1)<<0);
                } else if(pin_d0>-1) {
                    b = (((pins_l>>(pin_d0))&1)<<0);
                } else {
                    b=0;
                }
                if(pin_d1>31) {
                    b |= (((pins_h>>((pin_d1-32)&31))&1)<<1);
                } else if(pin_d1>-1) {
                    b |= (((pins_l>>(pin_d1))&1)<<1);
                }
                if(pin_d2>31) {
                    b |= (((pins_h>>((pin_d2-32)&31))&1)<<2);
                } else if(pin_d2>-1) {
                    b |= (((pins_l>>(pin_d2))&1)<<2);
                }
                if(pin_d3>31) {
                    b |= (((pins_h>>((pin_d3-32)&31))&1)<<3);
                } else if(pin_d3>-1) {
                    b |= (((pins_l>>(pin_d3))&1)<<3);
                }
                if(pin_d4>31) {
                    b |= (((pins_h>>((pin_d4-32)&31))&1)<<4);
                } else if(pin_d4>-1) {
                    b |= (((pins_l>>((pin_d4)&31))&1)<<4);
                }
                if(pin_d5>31) {
                    b |= (((pins_h>>((pin_d5-32)&31))&1)<<5);
                } else if(pin_d5>-1) {
                    b |= (((pins_l>>(pin_d5))&1)<<5);
                }
                if(pin_d6>31) {
                    b |= (((pins_h>>((pin_d6-32)&31))&1)<<6);
                } else if(pin_d6>-1) {
                    b |= (((pins_l>>(pin_d6))&1)<<6);
                }
                if(pin_d7>31) {
                    b |= (((pins_h>>((pin_d7-32)&31))&1)<<7);
                } else if(pin_d7>-1) {
                    b |= (((pins_l>>(pin_d7))&1)<<7);
                }
            } else if(has_data_low_pins) {
                uint32_t pins_l = gpio_input_get();
                pins_l = gpio_input_get();
                pins_l = gpio_input_get();
                if(pin_d0>-1) {
                    b = (((pins_l>>(pin_d0))&1)<<0);
                } else {
                    b=0;
                }
                if(pin_d1>-1) {
                    b |= (((pins_l>>(pin_d1))&1)<<1);
                }
                if(pin_d2>-1) {
                    b |= (((pins_l>>(pin_d2))&1)<<2);
                }
                if(pin_d3>-1) {
                    b |= (((pins_l>>(pin_d3))&1)<<3);
                }
                if(pin_d4>-1) {
                    b |= (((pins_l>>(pin_d4))&1)<<4);
                }
                if(pin_d5>-1) {
                    b |= (((pins_l>>(pin_d5))&1)<<5);
                }
                if(pin_d6>-1) {
                    b |= (((pins_l>>(pin_d6))&1)<<6);
                }
                if(pin_d7>-1) {
                    b |= (((pins_l>>(pin_d7))&1)<<7);
                }
            } else {
                uint32_t pins_h = gpio_input_get_high();
                pins_h = gpio_input_get_high();
                pins_h = gpio_input_get_high();
                if(pin_d0>-1) {
                    b = (((pins_h>>((pin_d0-32)&31))&1)<<0);
                } else {
                    b=0;
                }
                if(pin_d1>-1) {
                    b |= (((pins_h>>((pin_d1-32)&31))&1)<<1);
                }
                if(pin_d2>-1) {
                    b |= (((pins_h>>((pin_d2-32)&31))&1)<<2);
                }
                if(pin_d3>-1) {
                    b |= (((pins_h>>((pin_d3-32)&31))&1)<<3);
                }
                if(pin_d4>-1) {
                    b |= (((pins_h>>((pin_d4-32)&31))&1)<<4);
                }
                if(pin_d5>-1) {
                    b |= (((pins_h>>((pin_d5-32)&31))&1)<<5);
                }
                if(pin_d6>-1) {
                    b |= (((pins_h>>((pin_d6-32)&31))&1)<<6);
                }
                if(pin_d7>-1) {
                    b |= (((pins_h>>((pin_d7-32)&31))&1)<<7);
                }
            }
#else // !OPTIMIZE_ESP32
            if(pin_d0>-1) {
                b=digitalRead(pin_d0);
            } else {
                b=0;
            }
            if(pin_d1>-1) {
                b|=digitalRead(pin_d1)<<1;
            }
            if(pin_d2>-1) {
                b|=digitalRead(pin_d2)<<2;
            }
            if(pin_d3>-1) {
                b|=digitalRead(pin_d3)<<3;
            }
            if(pin_d4>-1) {
                b|=digitalRead(pin_d4)<<4;
            }
            if(pin_d5>-1) {
                b|=digitalRead(pin_d5)<<5;
            }
            if(pin_d6>-1) {
                b|=digitalRead(pin_d6)<<6;
            }
            if(pin_d7>-1) {
                b|=digitalRead(pin_d7)<<7;
            }
#endif // !OPTIMIZE_ESP32
            rd_high();
            return b;
        }
        inline static void write_raw8(uint8_t value) FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[value];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[value];
            } 
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_d0,value&1);
            digitalWrite(pin_d1,(value>>1)&1);
            digitalWrite(pin_d2,(value>>2)&1);
            digitalWrite(pin_d3,(value>>3)&1);
            digitalWrite(pin_d4,(value>>4)&1);
            digitalWrite(pin_d5,(value>>5)&1);
            digitalWrite(pin_d6,(value>>6)&1);
            digitalWrite(pin_d7,(value>>7)&1);
#endif // !OPTIMIZE_ESP32
            wr_high();
        }
        inline static bool dma_busy() FORCE_INLINE { return false; }
        inline static void dma_wait() FORCE_INLINE { }
        static void write_raw8_repeat(uint8_t value, size_t count) {
            write_raw8(value);
            while(--count) {
                wr_low(); wr_high();
            }
        }
        static void write_raw(const uint8_t* buffer, size_t length) {
            if(length) {
#if 1
                uint8_t old=*buffer;
                write_raw8(*buffer++);
                while(--length) {
                    if(old==*buffer) {
                        wr_low(); wr_high();
                    } else {
                        write_raw8(*buffer);
                    }
                    old = *buffer++;
                }
#else
                while(length--) {
                    write_raw8(*buffer++);
                }
#endif
            }
        }
        inline static void write_raw_dma(const uint8_t* data,size_t length) FORCE_INLINE {
            write_raw(data,length);
        }
        
        inline static void write_raw16(uint16_t value) FORCE_INLINE {
            uint8_t b = uint8_t(value >> 8);
#ifdef OPTIMIZE_ESP32
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
            b=value;
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_d0,b&1);
            digitalWrite(pin_d1,(b>>1)&1);
            digitalWrite(pin_d2,(b>>2)&1);
            digitalWrite(pin_d3,(b>>3)&1);
            digitalWrite(pin_d4,(b>>4)&1);
            digitalWrite(pin_d5,(b>>5)&1);
            digitalWrite(pin_d6,(b>>6)&1);
            digitalWrite(pin_d7,(b>>7)&1);
            wr_high();
            b=value;
            digitalWrite(pin_d0,b&1);
            digitalWrite(pin_d1,(b>>1)&1);
            digitalWrite(pin_d2,(b>>2)&1);
            digitalWrite(pin_d3,(b>>3)&1);
            digitalWrite(pin_d4,(b>>4)&1);
            digitalWrite(pin_d5,(b>>5)&1);
            digitalWrite(pin_d6,(b>>6)&1);
            digitalWrite(pin_d7,(b>>7)&1);
#endif // !OPTIMIZE_ESP32
            wr_high();
        }
        static void write_raw16_repeat(uint16_t value, size_t count) {
            write_raw16(value);
            if((value >> 8) == (value & 0x00FF)) {
                while(--count) {
                    wr_low(); wr_high();
                    wr_low(); wr_high();
                }
            } else {
                while(--count) {
                    write_raw16(value);
                }
            }
        }
        inline static void write_raw32(uint32_t value) FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            uint8_t b = uint8_t(value >> 24);
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
            b=uint8_t(value>>16);
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
            b = uint8_t(value >> 8);
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
            b=value;
            if(has_data_low_pins) {
                GPIO.out_w1tc = clr_mask_low(); GPIO.out_w1ts = xset_mask_low[b];
            }
            if(has_data_high_pins) {
                GPIO.out1_w1tc.val = clr_mask_high(); GPIO.out1_w1ts.val = xset_mask_high[b];
            } 
            wr_high();
#else // !OPTIMIZE_ESP32
            write_raw16(value>>16);
            write_raw16(value);
#endif // !OPTIMIZE_ESP32
        }
        static void write_raw32_repeat(uint32_t value, size_t count) {
            while(count--) {
                write_raw32(value);
            }
        }
        inline static void pin_mode(int8_t pin, uint8_t mode) FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin<32) {
                if(mode==INPUT) {
                    GPIO.enable_w1tc = ((uint32_t)1 << pin);
                } else {
                    GPIO.enable_w1ts = ((uint32_t)1 << pin);
                }
            } else {
                if(mode==INPUT) {
                    GPIO.enable1_w1tc.val = ((uint32_t)1 << ((pin-32)&31));
                } else {
                    GPIO.enable1_w1ts.val = ((uint32_t)1 << ((pin-32)&31));
                }
            }
            ESP_REG(DR_REG_IO_MUX_BASE + esp32_gpioMux[pin].reg) // Register lookup
                = ((uint32_t)2 << FUN_DRV_S)                        // Set drive strength 2
                | (FUN_IE)                                          // Input enable
                | ((uint32_t)2 << MCU_SEL_S);                       // Function select 2
            GPIO.pin[pin].val = 1;  
#else // !OPTIMIZE_ESP32
        if(mode==INPUT) {
            digitalWrite(pin,LOW);
            pinMode(pin,INPUT);
        } else {
            digitalWrite(pin,HIGH);
            pinMode(pin,OUTPUT);
        }
#endif // !OPTIMIZE_ESP32
        }
        inline static void begin_write() FORCE_INLINE {
            cs_low();
        }
        inline static void end_write() FORCE_INLINE {
            cs_high();
        }
        inline static void begin_read() FORCE_INLINE {
            cs_low();
        }
        inline static void end_read() FORCE_INLINE {
            cs_high();
        }
        static void direction(uint8_t mode) {
            pin_mode(pin_d0,mode);
            pin_mode(pin_d1,mode);
            pin_mode(pin_d2,mode);
            pin_mode(pin_d3,mode);
            pin_mode(pin_d4,mode);
            pin_mode(pin_d5,mode);
            pin_mode(pin_d6,mode);
            pin_mode(pin_d7,mode);
        }
        inline static void cs_low() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_cs>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_cs - 32)&31));
            } else if(pin_cs>-1) {
                GPIO.out_w1tc = (1 << (pin_cs&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void cs_high() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_cs>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_cs - 32)&31));
            } else if(pin_cs>-1) {
                GPIO.out_w1ts = (1 << (pin_cs&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_cs,HIGH);
#endif // !OPTIMIZE_ESP32
        }
        
        inline static void wr_low() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_wr>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_wr - 32)&31));
            } else if(pin_wr>-1) {
                GPIO.out_w1tc = (1 << (pin_wr&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_wr,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void wr_high() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_wr>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_wr - 32)&31));
            } else if(pin_wr>-1) {
                GPIO.out_w1ts = (1 << (pin_wr&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_wr,HIGH);
#endif // !OPTIMIZE_ESP32
        }

        inline static void rd_low() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_rd>31) {
                GPIO.out1_w1tc.val = (1 << ((pin_rd - 32)&31));
            } else if(pin_rd>-1) {
                GPIO.out_w1tc = (1 << (pin_rd &31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_rd,LOW);
#endif // !OPTIMIZE_ESP32
        }
        inline static void rd_high() FORCE_INLINE {
#ifdef OPTIMIZE_ESP32
            if(pin_rd>31) {
                GPIO.out1_w1ts.val = (1 << ((pin_rd - 32)&31));
            } else if(pin_rd>-1) {
                GPIO.out_w1ts = (1 << (pin_rd&31));
            }
#else // !OPTIMIZE_ESP32
            digitalWrite(pin_rd,HIGH);
#endif // !OPTIMIZE_ESP32
        }

    };
#ifdef OPTIMIZE_ESP32
    template<int8_t PinCS, 
            int8_t PinWR, 
            int8_t PinRD,
            int8_t PinD0,
            int8_t PinD1,
            int8_t PinD2,
            int8_t PinD3,
            int8_t PinD4,
            int8_t PinD5,
            int8_t PinD6,
            int8_t PinD7> 
            uint32_t tft_p8<PinCS,
                                PinWR,
                                PinRD,
                                PinD0,
                                PinD1,
                                PinD2,
                                PinD3,
                                PinD4,
                                PinD5,
                                PinD6,
                                PinD7>::xset_mask_low[256*tft_p8<PinCS,
                                PinWR,
                                PinRD,
                                PinD0,
                                PinD1,
                                PinD2,
                                PinD3,
                                PinD4,
                                PinD5,
                                PinD6,
                                PinD7>::has_data_low_pins];
    template<int8_t PinCS, 
            int8_t PinWR, 
            int8_t PinRD,
            int8_t PinD0,
            int8_t PinD1,
            int8_t PinD2,
            int8_t PinD3,
            int8_t PinD4,
            int8_t PinD5,
            int8_t PinD6,
            int8_t PinD7> 
            uint32_t tft_p8<PinCS,
                                PinWR,
                                PinRD,
                                PinD0,
                                PinD1,
                                PinD2,
                                PinD3,
                                PinD4,
                                PinD5,
                                PinD6,
                                PinD7>::xset_mask_high[256*tft_p8<PinCS,
                                PinWR,
                                PinRD,
                                PinD0,
                                PinD1,
                                PinD2,
                                PinD3,
                                PinD4,
                                PinD5,
                                PinD6,
                                PinD7>::has_data_high_pins];
#endif // OPTIMIZE_ESP32
}