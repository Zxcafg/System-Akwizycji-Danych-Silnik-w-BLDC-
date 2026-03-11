/**
 * @file main.cpp
 * @author Pavel Tshonek
 * @date 2026
 * @brief Połączenie: Pomiary prądu z MCP3564 (SPI2/FSPI) i zapis do pliku na karcie SD (SPI3/SUBSPI).
 * * Praca dyplomowa: ZASTOSOWANIE WYBRANYCH METOD UCZENIA MASZYNOWEGO DO WYKRYWANIA USZKODZEŃ NAPĘDU ELEKTRYCZNEGO DRONA
 * * Instytucja: Politechnika Poznańska
 * * Plik zawiera pełną implementację logiki obsługi przetwornika analogowo-cyfrowego:
 * - Inicjalizację magistrali SPI2 (FSPI) i konfigurację rejestrów układu.
 * - Procedurę automatycznej kalibracji offsetu
 * - Odczyt i dekodowanie 24-bitowych danych w trybie SCAN.
 * - Przeliczanie surowych wartości na prąd (A) z uwzględnieniem wzmocnienia układów INA240 oraz rezystancji boczników (3 mOhm).
 */

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <time.h> 
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "mcp3564_driver.h"


#define PIN_NUM_MOSI_SD 35 // (SUBSPID)
#define PIN_NUM_MISO_SD 37 // (SUBSPIQ)
#define PIN_NUM_CLK_SD  36 // (SUBSPICLK)
#define PIN_NUM_CS_SD   38 // (SUBSPICS1)
#define SPI_HOST_SD     SPI3_HOST
#define MOUNT_POINT     "/sdcard"
#define LOG_FILE_PATH   MOUNT_POINT "/data_log.csv"


static const char *TAG = "MAIN";
static QueueHandle_t adc_queue = NULL;
static sdmmc_card_t *card = NULL; 
static int log_attempts = 0; 


void sd_card_unmount() {
    if (card) {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        ESP_LOGI(TAG, "Karta SD odmontowana.");
        spi_bus_free(SPI_HOST_SD);
        ESP_LOGI(TAG, "Magistrala SPI3 zwolniona.");
        card = NULL;
    }
}


bool sd_card_init_and_mount()
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Inicjalizacja i montowanie karty SD...");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = true
    };

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI_SD,
        .miso_io_num = PIN_NUM_MISO_SD,
        .sclk_io_num = PIN_NUM_CLK_SD,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

    ret = spi_bus_initialize(SPI_HOST_SD, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Nie udało się zainicjować magistrali SPI3 dla SD. Błąd: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Magistrala SPI3 dla SD zainicjowana.");


    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI_HOST_SD; 
    host.max_freq_khz = 10000; 

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)PIN_NUM_CS_SD;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BŁĄD MONOWANIA KARTY SD (%s).", esp_err_to_name(ret));
        spi_bus_free(SPI_HOST_SD);
        card = NULL;
        return false;
    }

    ESP_LOGI(TAG, "System plików zamontowany pomyślnie w %s", MOUNT_POINT);
    sdmmc_card_print_info(stdout, card);
    
    struct stat st;
    const char *header = "Timestamp_us,INA0_A,INA1_A,INA2_A,INA3_A\n";
    
    bool file_exists = (stat(LOG_FILE_PATH, &st) == 0);

    FILE *f = fopen(LOG_FILE_PATH, file_exists ? "a" : "w");

    if (f == NULL) {
        ESP_LOGE(TAG, "BŁĄD PLIKU: Nie udało się otworzyć %s do zapisu/dopisywania.", LOG_FILE_PATH);
        ESP_LOGE(TAG, "Błąd systemu: %s", strerror(errno)); 
    } else {
        if (!file_exists || st.st_size == 0) {
 
            int written = fprintf(f, "%s", header);
            if (written > 0) {
                ESP_LOGI(TAG, "Utworzono nowy plik logu z nagłówkiem CSV. Rozmiar: %d bajtów.", written);
            } else {
                 ESP_LOGE(TAG, "BŁĄD PLIKU: Nie udało się zapisać nagłówka do pliku! Błąd: %s", strerror(errno));
            }
        } else {
            ESP_LOGI(TAG, "Plik logu, już istnieje.");
        }
        fclose(f);
    }

    return true;
}



static void IRAM_ATTR mcp3564_irq_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t dummy = 1;
    
    xQueueSendFromISR(adc_queue, &dummy, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void adc_task(void *pvParameter) {
    uint32_t notification;
    
    esp_err_t ret = mcp3564_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP3564 init failed! ZATRZYMANO.");
        vTaskDelete(NULL);
        return;
    }
    
    gpio_set_direction(MCP3564_IRQ_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(MCP3564_IRQ_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MCP3564_IRQ_PIN, mcp3564_irq_handler, NULL);
    
    mcp3564_start_conversion();
    
    mcp3564_calibrate_offsets();
    
    ESP_LOGI(TAG, "ADC - ok, Rozpoczęto logowanie do SD.");

    while (1) {
        if (xQueueReceive(adc_queue, &notification, portMAX_DELAY) == pdPASS) {
            
            uint32_t adc_data = mcp3564_read_adc_raw();
            
            if (adc_data == 0) continue;
            
            mcp3564_process_adc_data(adc_data);
            
            uint8_t channel_id = (adc_data & 0xF0000000) >> 28;

            if (channel_id == SE_0) { 
                if (card != NULL) {
                    
                    uint64_t timestamp_us = esp_timer_get_time();
                    
                    const char *mode = (log_attempts < 10) ? "w" : "a";
                    
                    FILE *f = fopen(LOG_FILE_PATH, mode);
                    
                    if (f == NULL) {
                        ESP_LOGE(TAG, "BŁĄD ZAPISU: Nie udało się otworzyć %s w trybie '%s'. Błąd: %s", 
                                LOG_FILE_PATH, mode, strerror(errno));
                    } else {
                        if (strcmp(mode, "w") == 0) {
                             fprintf(f, "Timestamp_us,INA0_A,INA1_A,INA2_A,INA3_A\n");
                        }
                        

                        int written = fprintf(f, 
                                "%llu,%.6f,%.6f,%.6f,%.6f\n",
                                timestamp_us,
                                mcp3564_get_current(0), // INA0 (CH6)
                                mcp3564_get_current(1), // INA1 (CH4)
                                mcp3564_get_current(2), // INA2 (CH2)
                                mcp3564_get_current(3)  // INA3 (CH0)
                        );
                        
                        fflush(f);
                        fclose(f);
                        
                        if (written > 0) {
                            ESP_LOGI(TAG, "Zapisano %d bajtów do SD. Próba: %d, Tryb: '%s'", written, log_attempts + 1, mode);
                        } else {
                            ESP_LOGE(TAG, "BŁĄD ZAPISU: fprintf zwrócił 0 lub błąd! Błąd: %s", strerror(errno));
                        }
                    }
                    
                    log_attempts++;
                }
                // Wypisywanie danych w konsoli
                // printf("LOG (Console): %.6fA, %.6fA, %.6fA, %.6fA\n",
                //        mcp3564_get_current(0), 
                //        mcp3564_get_current(1), 
                //        mcp3564_get_current(2), 
                //        mcp3564_get_current(3)
                // );
            }
        }
    }
}

void app_main(void) {
    adc_queue = xQueueCreate(20, sizeof(uint32_t));
    if (adc_queue == NULL) {
        ESP_LOGE(TAG, "Queue creation failed!");
        return;
    }
    
    if (sd_card_init_and_mount()) {
        ESP_LOGI(TAG, "SD Card gotowa do logowania.");
    } else {
        ESP_LOGE(TAG, "SD Card NIEDOSTĘPNA.");
    }
    
    xTaskCreate(&adc_task, "adc_task", 16384, NULL, 5, NULL);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}