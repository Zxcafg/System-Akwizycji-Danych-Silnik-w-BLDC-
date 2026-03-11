/**
 * @file mcp3564_driver.c
 * @author Pavel Tshonek
 * @date 2026
 * @brief MCP3564RT driver implementation
 * * Praca dyplomowa: ZASTOSOWANIE WYBRANYCH METOD UCZENIA MASZYNOWEGO DO WYKRYWANIA USZKODZEŃ NAPĘDU ELEKTRYCZNEGO DRONA
 * * Instytucja: Politechnika Poznańska
 * * Implementacja w języku C z wykorzystaniem frameworka ESP-IDF.
 * Plik zawiera funkcje obsługi magistrali SPI, konfigurację rejestrów układu,
 * procedurę kalibracji offsetów, 
 * logikę przeliczania surowych danych z ADC na wartości prądu w amperach (A).
 */

#include "mcp3564_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MCP3564";

spi_device_handle_t spi_handle = NULL;

static volatile float current_values[4] = {0.0f};

static int32_t channel_offsets[8] = {0};
static bool calibration_done = false;

/**
 * @brief Calibrate ADC offsets
 */
void mcp3564_calibrate_offsets(void) {
    ESP_LOGI(TAG, "=== ADC CALIBRATION START ===");
    ESP_LOGI(TAG, "Current MUST be 0A on all channels!");
    
    int32_t sums[8] = {0};
    uint32_t counts[8] = {0};
    const int SAMPLES = 100; 
    
    ESP_LOGI(TAG, "Collecting %d samples per channel...", SAMPLES);
    
    for (int i = 0; i < SAMPLES * 8; i++) { 
        uint32_t adc_data = mcp3564_read_adc_raw();
        
        if (adc_data == 0) {
            ESP_LOGW(TAG, "Empty ADC read, skipping...");
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        uint8_t channel_id = (adc_data & 0xF0000000) >> 28;
        int32_t raw_value = (int32_t)(adc_data & 0x00FFFFFF);
        
        if (raw_value & 0x00800000) {
            raw_value |= 0xFF000000;
        }
        
        if (channel_id < 8) {
            sums[channel_id] += raw_value;
            counts[channel_id]++;
            
            if ((i % 50) == 0) {
                ESP_LOGI(TAG, "Progress: %d/%d samples", i, SAMPLES * 8);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    ESP_LOGI(TAG, "Calibration results:");
    for (int i = 0; i < 8; i++) {
        if (counts[i] > 0) {
            channel_offsets[i] = sums[i] / counts[i];
            ESP_LOGI(TAG, "  CH%d: offset=%6ld (0x%06lX) from %lu samples", 
                     i, channel_offsets[i], channel_offsets[i] & 0x00FFFFFF, counts[i]);
        } else {
            ESP_LOGW(TAG, "  CH%d: NO DATA! (Offset set to 0)", i);
            channel_offsets[i] = 0; 
        }
    }
    
    calibration_done = true;
    ESP_LOGI(TAG, "=== CALIBRATION COMPLETED ===");
}

/**
 * @brief Initialize SPI and MCP3564
 */
esp_err_t mcp3564_init(void) {
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing MCP3564 (Arduino port)");

    spi_bus_config_t buscfg = {
        .miso_io_num = MCP3564_MISO_PIN,
        .mosi_io_num = MCP3564_MOSI_PIN,
        .sclk_io_num = MCP3564_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_SPEED,
        .mode = 0,  
        .spics_io_num = MCP3564_CS_PIN,
        .queue_size = 7,
        .flags = 0,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MCP3564_IRQ_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "SPI initialized at %d Hz", SPI_SPEED);
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = mcp3564_reset();
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t tx_data[10];
    tx_data[0] = ADDR | CONFIG0_ADDR | INC_WRITE;
    tx_data[1] = CONFIG0;
    tx_data[2] = CONFIG1;
    tx_data[3] = CONFIG2;
    tx_data[4] = CONFIG3;
    tx_data[5] = IRQ_CONFIG;
    tx_data[6] = MUX_CONFIG;
    
    // SCAN_CONFIG - 3 bajty
    tx_data[7] = (SCAN_CONFIG >> 16) & 0xFF; // MSB
    tx_data[8] = (SCAN_CONFIG >> 8) & 0xFF;  // Middle Byte
    tx_data[9] = SCAN_CONFIG & 0xFF;         // LSB (0x55)

    spi_transaction_t t = {
        .length = 80,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
    };

    ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config write failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "MCP3564 configured:");
    ESP_LOGI(TAG, "  CONFIG0: 0x%02X (Internal CLK)", CONFIG0);
    ESP_LOGI(TAG, "  CONFIG1: 0x%02X (OSR_32)", CONFIG1);
    // Logowanie nowej, skróconej konfiguracji
    ESP_LOGI(TAG, "  SCAN: 0x%06lX (CH0, CH2, CH4, CH6 only)", (uint32_t)SCAN_CONFIG); 
    ESP_LOGI(TAG, "  Shunt: %.1f mOhm, Gain: %.0f V/V", SHUNT_R * 1000, INA240_GAIN);

    return ESP_OK;
}

/**
 * @brief Reset device
 */
esp_err_t mcp3564_reset(void) {
    uint8_t cmd = ADDR | RESET_CMD | FAST_CMD;
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device reset OK");
    }
    return ret;
}

/**
 * @brief Start conversion
 */
esp_err_t mcp3564_start_conversion(void) {
    uint8_t cmd = ADDR | CONV | FAST_CMD;
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Conversion started");
    }
    return ret;
}

/**
 * @brief Stop conversion
 */
esp_err_t mcp3564_stop_conversion(void) {
    uint8_t cmd = ADDR | STBY | FAST_CMD;
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Conversion stopped");
    }
    return ret;
}

/**
 * @brief Read raw ADC data
 */
uint32_t mcp3564_read_adc_raw(void) {
    uint8_t tx_data[5] = {0};
    uint8_t rx_data[5] = {0};
    
    tx_data[0] = ADDR | ADC_REG | READ;

    spi_transaction_t t = {
        .length = 40,  // 5 bytes
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    if (spi_device_transmit(spi_handle, &t) != ESP_OK) {
        return 0;
    }

    uint32_t adc_data = ((uint32_t)rx_data[1] << 24) | 
                        ((uint32_t)rx_data[2] << 16) | 
                        ((uint32_t)rx_data[3] << 8) | 
                        ((uint32_t)rx_data[4]);

    return adc_data;
}

/**
 * @brief Process ADC reading - NO AVERAGING, DIRECT MEASUREMENT
 */
void mcp3564_process_adc_data(uint32_t adc_data) {
    uint8_t channel_id = (adc_data & 0xF0000000) >> 28;
    
    int32_t raw_value = (int32_t)(adc_data & 0x00FFFFFF);
    
    if (raw_value & 0x00800000) {
        raw_value |= 0xFF000000;
    }
    
    int32_t calibrated_value = raw_value;
    if (calibration_done && channel_id < 8) {
        calibrated_value = raw_value - channel_offsets[channel_id];
    }
    
    const float VREF_ADC = 2.4f;  
    const float ADC_FULL_SCALE = 8388607.0f;  // 2^23 - 1 (24-bit signed max)
    
    // Only process the 4 connected channels (SE_0, SE_2, SE_4, SE_6)
    if (channel_id == SE_0 || channel_id == SE_2 || 
        channel_id == SE_4 || channel_id == SE_6) {
        
        // Map channel to INA index
        uint8_t ina_idx;
        switch (channel_id) {
            case SE_0: ina_idx = 3; break;  // INA240 #3
            case SE_2: ina_idx = 2; break;  // INA240 #2
            case SE_4: ina_idx = 1; break;  // INA240 #1
            case SE_6: ina_idx = 0; break;  // INA240 #0
            default: return;
        }
        
        // Calculate voltage at ADC input
        float voltage = VREF_ADC * ((float)calibrated_value / ADC_FULL_SCALE);
        
        // Calculate current: I = V_adc / (R_shunt * Gain)
        float current = voltage / (SHUNT_R * INA240_GAIN);
        
        current_values[ina_idx] = current;
        
    }
}

/**
 * @brief Get current measurement for specific INA240
 */
float mcp3564_get_current(uint8_t ina_channel) {
    if (ina_channel > 3) return 0.0f;
    return current_values[ina_channel];
}