/*
 * ESP32 LoRa Flight Computer
 * Copyright (C) 2025 Club de Cohetería Ursa Crux
 * 
 * No lo hacemos porque sea fácil, pero porque es difícil.
 *
 * Este programa es software libre: puedes redistribuirlo y/o modificarlo
 * bajo los términos de la Licencia Pública General GNU publicada por
 * la Free Software Foundation, ya sea la versión 3 de la Licencia, o
 * cualquier versión posterior.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>

// Librerías para tarjeta SD
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

// Librería de LoRa
#include "lora.h"

#define UART_PORT       UART_NUM_0
#define UART_BAUD_RATE  115200
#define BUFFER_SIZE     256
#define INPUT_DATA_SIZE 16
#define PACKET_POOL     32 // Número de paquetes preasignados

// Pins de tarjeta SD
#define SD_MISO         19
#define SD_MOSI         23
#define SD_SCK          18
#define SD_CS           5

// Guardar los paquetes por cada x número
#define SD_FLUSH_EVERY  15

#define MAGIC_NUMBER    0x69 // Número utilizado para reconocer el inicio de un paquete y verificar integridad

static const char *TAG = "UART_LORA";

// Implementación de CRC16-CCITT
uint16_t CRC16(uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

// Determinar el tamaño de los datos en el paquete dependiendo del RSSI (fuerza de la señal)
int get_payload_size(void) {
    int rssi = lora_packet_rssi();
    if (rssi > -90) {
        return 48;  // Señal fuerte: 16 bytes
    } else if (rssi > -105) {
        return 32;  // Señal media: 12 bytes
    } else {
        return 16;   // Señal debil: 8 bytes
    }
}

typedef struct {
    uint8_t descriptor;
    uint8_t raw_data[16];
} uart_frame_t; // Estructura de los datos que recibimos.

typedef struct {
    uint8_t buffer[1 + 1 + 4 + INPUT_DATA_SIZE + 2]; // Número mágico + tipo de paquete + marca de tiempo + datos + CRC16
    size_t length; // Tamaño actual
    uint8_t heap;
} packet_t; // Paquete PHUC 

// Colas
static QueueHandle_t uart_queue;
static QueueHandle_t free_pool;
static QueueHandle_t sd_queue;

// Pool de paquetes preasignados
static packet_t packet_pool[PACKET_POOL];

// Inicialización tarjeta SD
void init_sd() {
    esp_err_t ret;

    // Configuración del bus SPI
    spi_bus_config_t bus_config = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };

    ret = spi_bus_initialize(VSPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando bus SPI (%s)", esp_err_to_name(ret));
        return;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = VSPI_HOST;

    // Configuración del dispositivo SD
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 5;
    slot_config.host_id = VSPI_HOST;

    // Configuración de montaje FAT
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 10,
    };

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error Montado SD (%s)", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Tarjeta SD montada correctamente");
}

// Guardar binarios en la tarjeta SD
void save_packet_sd_binary(packet_t* pkt, FILE* f) {
    if(pkt && f) {
        if(pkt->length <= sizeof(pkt->buffer)) {
            fwrite(pkt->buffer, 1, pkt->length, f);
            fflush(f);
            ESP_LOGI(TAG, "Paquete guardado en SD (%zu bytes)", pkt->length);
        } else {
            ESP_LOGE(TAG, "Paquete demasiado grande, no se guarda");
        }
    }
}

// Trabajo del UART (USB)
void uart_task(void *args) {
    uart_frame_t frame;
    packet_t *pkt;

    while(1) {
        // Lee el input completo de 17 bytes (1 byte: descriptor del contenido + 16 bytes: datos)
        int len = uart_read_bytes(UART_PORT, (uint8_t*)&frame, sizeof(frame), pdMS_TO_TICKS(15));
        if (len == sizeof(frame)) {
            // Tomamos un paquete libre del pool
            if (xQueueReceive(free_pool, &pkt, portMAX_DELAY) == pdTRUE && pkt != NULL) {
                // Rellena datos del paquete
                pkt->length = 0;
                pkt->buffer[pkt->length++] = MAGIC_NUMBER;
                pkt->buffer[pkt->length++] = frame.descriptor;

                uint32_t time_stamp = (uint32_t)esp_timer_get_time();
                if (pkt->length + sizeof(time_stamp) <= sizeof(pkt->buffer)){
                    memcpy(&pkt->buffer[pkt->length], &time_stamp, sizeof(time_stamp)); // Copiamos los bytes de la marca de tiempo al paquete
                    pkt->length += sizeof(time_stamp); // Nos movemos el largo de la marca de tiempo a través del paquete
                }
                // Copiamos todos los datos por ahora. El tamaño real será determinado por la función correspondiente al LoRa
                if(pkt->length + INPUT_DATA_SIZE <= sizeof(pkt->buffer)){
                    memcpy(&pkt->buffer[pkt->length], frame.raw_data, INPUT_DATA_SIZE);
                    pkt->length += INPUT_DATA_SIZE;
                }

                // Evitamos conflictos en el buffer con una copia del paquete
                packet_t *pkt_sd = malloc(sizeof(packet_t));
                if(pkt_sd != NULL) {
                    memcpy(pkt_sd, pkt, sizeof(packet_t));
                    if(xQueueSend(sd_queue, &pkt_sd, pdMS_TO_TICKS(10)) != pdTRUE) {
                        ESP_LOGW(TAG, "sd_queue llena, liberando duplicado");
                        free(pkt_sd);
                    }
                } else {
                    // No hay memoria, enviamos el original a SD
                    if(xQueueSend(sd_queue, &pkt, pdMS_TO_TICKS(10)) != pdTRUE) {
                        ESP_LOGW(TAG, "sd_queue llena, paquete perdido");
                    } else {
                        pkt = NULL; // LoRa no lo enviará, prioridad a SD
                    }
                }

                // Enviamos al LoRa si aún existe
                if(pkt != NULL) {
                    if(xQueueSend(uart_queue, &pkt, pdMS_TO_TICKS(10)) != pdTRUE) {
                        ESP_LOGW(TAG, "uart_queue llena, paquete perdido");
                        xQueueSend(free_pool, &pkt, pdMS_TO_TICKS(10)); // devolvemos al pool
                    }
                }
            } else {
                ESP_LOGW(TAG, "No hay paquetes libres. Entrada descartada");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
} 

// Trabajo del LoRa
// Modificamos la tarea LoRa para acumular entradas de 16 bytes en paquetes de 48 bytes de payload
void lora_task(void *args) {
    packet_t *pkt;

    uint8_t temp_buffer[48];  // Acumulamos hasta 48 bytes
    size_t temp_len = 0;
    uint8_t current_descriptor = 0;
    uint32_t current_timestamp = 0;

    while (1) {
        if (xQueueReceive(uart_queue, &pkt, portMAX_DELAY) == pdTRUE){
            uint8_t descriptor = pkt->buffer[1];
            uint8_t *raw_data = &pkt->buffer[6]; // Los datos parten después del header original
            size_t offset_data = 0;
            size_t total_data = INPUT_DATA_SIZE; // 16 bytes por entrada

            // Guardamos el descriptor y timestamp del primer fragmento
            if (temp_len == 0) {
                current_descriptor = descriptor;
                current_timestamp = (uint32_t)esp_timer_get_time();
            }

            while (offset_data < total_data) {
                size_t remaining = total_data - offset_data;
                size_t copy_size = (remaining > (sizeof(temp_buffer) - temp_len)) ? 
                                    (sizeof(temp_buffer) - temp_len) : remaining;
                memcpy(&temp_buffer[temp_len], &raw_data[offset_data], copy_size);
                temp_len += copy_size;
                offset_data += copy_size;

                // Si llenamos 48 bytes, enviamos paquete
                if (temp_len == sizeof(temp_buffer)) {
                    size_t packet_offset = 0;

                    pkt->buffer[packet_offset++] = MAGIC_NUMBER;
                    pkt->buffer[packet_offset++] = current_descriptor;
                    memcpy(&pkt->buffer[packet_offset], &current_timestamp, sizeof(current_timestamp));
                    packet_offset += sizeof(current_timestamp);

                    memcpy(&pkt->buffer[packet_offset], temp_buffer, temp_len);
                    packet_offset += temp_len;

                    uint16_t crc = CRC16(pkt->buffer, packet_offset);
                    memcpy(&pkt->buffer[packet_offset], &crc, sizeof(crc));
                    packet_offset += sizeof(crc);

                    pkt->length = packet_offset;
                    lora_send_packet(pkt->buffer, pkt->length);
                    ESP_LOGI(TAG, "Paquete mandado: desc=0x%02X, payload=%zu, timestamp=%u",
                             current_descriptor, temp_len, current_timestamp);

                    // Reiniciamos buffer temporal
                    temp_len = 0;
                }
            }

            // Devolver buffer al pool
            xQueueSend(free_pool, &pkt, portMAX_DELAY);
        }
    }
}


void sd_task(void* args) {
    packet_t* pkt;
    FILE* sd_file = NULL;
    char filename[32];

    // Abrir un archivo único desde el inicio
    snprintf(filename, sizeof(filename), "/sdcard/data_log.bin");
    sd_file = fopen(filename, "wb");
    if (!sd_file) {
        ESP_LOGE(TAG, "No se pudo abrir %s", filename);
        vTaskDelete(NULL); // terminar tarea si no hay SD
        return;
    }
    ESP_LOGI(TAG, "Archivo SD abierto: %s", filename);

    while (1) {
        if (xQueueReceive(sd_queue, &pkt, portMAX_DELAY) == pdTRUE) {

            // Verificar si es paquete de "shutdown" (payload lleno de 0xFF)
            bool shutdown_packet = true;
            for (size_t i = 6; i < pkt->length - 2; i++) {
                if (pkt->buffer[i] != 0xFF) {
                    shutdown_packet = false;
                    break;
                }
            }

            if (shutdown_packet) {
                if (sd_file) {
                    fclose(sd_file);
                    sd_file = NULL;
                    ESP_LOGW(TAG, "Archivo cerrado por paquete de SHUTDOWN");
                }
                // Liberar memoria del paquete
                if (pkt < &packet_pool[0] || pkt > &packet_pool[PACKET_POOL - 1]) {
                    free(pkt);
                }
                // Aquí podrías mandar al ESP32 a deep sleep si quieres
                // esp_deep_sleep_start();
                continue; // saltar al siguiente ciclo
            }

            // Guardar paquete en SD
            if (sd_file) {
                fwrite(pkt->buffer, 1, pkt->length, sd_file);
                fflush(sd_file);
                ESP_LOGI(TAG, "Paquete guardado en SD (%zu bytes)", pkt->length);
            }

            // Liberar memoria si el paquete fue duplicado
            if (pkt != NULL && (pkt < &packet_pool[0] || pkt > &packet_pool[PACKET_POOL - 1])) {
                free(pkt);
            }
        }
    }
}



void app_main(void) {
    ESP_LOGI(TAG, "Iniciando sístema...");

    // Inicialización de UART (8n1)
    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
    };

    // Instalación del driver para usar UART para I/O (no buffer para tx)
    uart_driver_install(UART_PORT, BUFFER_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);

    // Inicialización del módulo LoRa
    if (lora_init() == 0) {
        ESP_LOGW(TAG, "Error: No se pudo inicializar el LoRa\n");
        return;
    } else {
        ESP_LOGI(TAG, "LoRa inicializado correctamente");
    }
    lora_set_frequency(433E6); // 433 MHz
    lora_set_spreading_factor(9);
    lora_set_bandwidth(7);
    lora_set_tx_power(16); // Poder de 16dBm (aprox. 25mW)
    lora_enable_crc(); // Habilitar el CRC a nivel hardware. (Es separado al CRC del paquete)
    lora_set_coding_rate(5); // FEC

    // Inicialización SD
    init_sd();

    // Creamos las colas
    uart_queue = xQueueCreate(PACKET_POOL, sizeof(packet_t*));
    free_pool = xQueueCreate(PACKET_POOL, sizeof(packet_t*));
    sd_queue = xQueueCreate(PACKET_POOL, sizeof(packet_t*));

    if (!uart_queue || !free_pool || !sd_queue) {
        ESP_LOGE(TAG, "Error creando colas");
        return;
    }
    
    // Llenamos la pool libre con buffers preasignados
    for (int i = 0; i < PACKET_POOL; i++) {
        packet_t *pkt_ptr = &packet_pool[i];
        if (xQueueSend(free_pool, &pkt_ptr, 0) != pdTRUE) {
            ESP_LOGW(TAG, "No se pudo enviar buffer %d al free_pool", i);
        }
    }
    ESP_LOGI(TAG, "Pool de buffers inicializado");

    ESP_LOGI(TAG, "Creando tareas...");
    // Iniciamos las tareas
    xTaskCreate(uart_task, "UART Taks", 4096, NULL, 10, NULL);
    xTaskCreate(lora_task, "LoRa Task", 8192, NULL, 9, NULL);
    xTaskCreate(sd_task, "SD Task", 4096, NULL, 8, NULL);
    ESP_LOGI(TAG, "Tareas creadas. Esperando...");
}
