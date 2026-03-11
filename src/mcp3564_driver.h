/**
 * @file mcp3564_driver.h
 * @author Pavel Tshonek
 * @date 2026
 * @brief MCP3564RT driver for ESP32-S3 - Direct Arduino port
 * * Praca dyplomowa: ZASTOSOWANIE WYBRANYCH METOD UCZENIA MASZYNOWEGO DO WYKRYWANIA USZKODZEŃ NAPĘDU ELEKTRYCZNEGO DRONA
 * * Instytucja: Politechnika Poznańska
 * * Plik zawiera kluczowe parametry systemu pomiarowego:
 * - Mapowanie pinów magistrali SPI2 (FSPI) dla ESP32-S3.
 * - Stałe fizyczne układu: bocznik 3mOhm, wzmocnienie INA240 (20V/V).
 * - Definicje rejestrów konfiguracyjnych (OSR, SCAN, CONFIG)
 */

#ifndef MCP3564_DRIVER_H
#define MCP3564_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Pin Configuration (SPI2/FSPI)
#define MCP3564_CS_PIN     10
#define MCP3564_MISO_PIN   13
#define MCP3564_MOSI_PIN   11
#define MCP3564_CLK_PIN    12
#define MCP3564_IRQ_PIN    4

// Hardware Configuration (from Arduino code)
#define MCLK_INTERNAL      true
// #define OSR_SETTING        OSR_128
#define OSR_SETTING        OSR_32
#define SHUNT_R            0.003f      // 3 mOhm
#define INA240_GAIN        20.0f       // INA240A1

// Calculated
#define CURRENT_SENSE_GAIN (1.0f / (SHUNT_R * INA240_GAIN))

// SPI Configuration
#define SPI_SPEED          20000000    // 20 MHz

// MCP3564 Commands and Addresses
#define ADDR               0b01000000
#define FAST_CMD           0b00000000
#define CONV               0b00101000
#define STBY               0b00101100
#define RESET_CMD          0b00111000
#define INC_WRITE          0b00000010
#define READ               0b00000001
#define INC_READ           0b00000011

#define CONFIG0_ADDR       0b00000100
#define CONFIG1_ADDR       0b00001000
#define CONFIG2_ADDR       0b00001100
#define CONFIG3_ADDR       0b00010000
#define IRQ_ADDR           0b00010100
#define MUX_ADDR           0b00011000
#define SCAN_ADDR          0b00011100
#define ADC_REG            0b00000000

// OSR Settings
typedef enum {
    OSR_32    = 0x00,
    OSR_64    = 0x01,
    OSR_128   = 0x02,
    OSR_256   = 0x03,
    OSR_512   = 0x04,
    OSR_1024  = 0x05,
    OSR_2048  = 0x06,
    OSR_4096  = 0x07,
    OSR_8192  = 0x08,
    OSR_16384 = 0x09,
    OSR_20480 = 0x0A,
    OSR_24576 = 0x0B,
    OSR_40960 = 0x0C,
    OSR_49152 = 0x0D,
    OSR_81920 = 0x0E,
    OSR_98304 = 0x0F
} osr_t;

// Channel IDs
typedef enum {
    SE_0    = 0x00,  // CH0 - INA240 #3
    SE_1    = 0x01,  // CH1 - GND
    SE_2    = 0x02,  // CH2 - INA240 #2
    SE_3    = 0x03,  // CH3 - GND
    SE_4    = 0x04,  // CH4 - INA240 #1
    SE_5    = 0x05,  // CH5 - GND
    SE_6    = 0x06,  // CH6 - INA240 #0
    SE_7    = 0x07,  // CH7 - GND
    DIFF_A  = 0x08,
    DIFF_B  = 0x09,
    DIFF_C  = 0x0A,
    DIFF_D  = 0x0B,
    TEMP    = 0x0C,
    AVDD    = 0x0D,
    VCM     = 0x0E,
    OFFSET  = 0x0F
} channel_t;

// Configuration Registers (matching Arduino exactly)
#define CONFIG0            (0b11010010 | (MCLK_INTERNAL << 5))
#define CONFIG1            (0b00000000 | (OSR_SETTING << 2))
#define CONFIG2            0b10001011
#define CONFIG3            0b11110000
#define IRQ_CONFIG         0b01110110
#define MUX_CONFIG         0b01111000
// #define SCAN_CONFIG        0b010000000000000011111111  // All 8 channels SE_0 to SE_7
#define SCAN_CONFIG        0x000055UL // 0b01010101 (Tylko 4 używane kanały prądowe)
// Function prototypes
esp_err_t mcp3564_init(void);
esp_err_t mcp3564_reset(void);
esp_err_t mcp3564_start_conversion(void);
esp_err_t mcp3564_stop_conversion(void);
uint32_t mcp3564_read_adc_raw(void);
void mcp3564_process_adc_data(uint32_t adc_data);
void mcp3564_calibrate_offsets(void); 

float mcp3564_get_current(uint8_t ina_channel);  // 0=INA0(CH6), 1=INA1(CH4), 2=INA2(CH2), 3=INA3(CH0)

extern spi_device_handle_t spi_handle;

#endif 