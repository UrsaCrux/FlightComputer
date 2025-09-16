#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdint.h>

// Librería de LoRa
#include "lora.h"

#define UART_PORT       UART_NUM_0
#define UART_BAUD_RATE  115200
#define BUFFER_SIZE     256
#define INPUT_DATA_SIZE 16
#define PACKET_POOL     10 // Número de paquetes preasignados


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
        return 16;  // Señal fuerte: 16 bytes
    } else if (rssi > -105) {
        return 12;  // Señal media: 12 bytes
    } else {
        return 8;   // Señal debil: 8 bytes
    }
}

typedef struct {
    uint8_t descriptor;
    uint8_t raw_data[16];
} uart_frame_t; // Estructura de los datos que recibimos.

typedef struct {
    uint8_t buffer[1 + 1 + 4 + INPUT_DATA_SIZE + 2]; // Número mágico + tipo de paquete + marca de tiempo + datos + CRC16
    size_t length; // Tamaño actual
} packet_t; // Paquete PHUC 

// Colas
static QueueHandle_t uart_queue;
static QueueHandle_t free_pool;

// Pool de paquetes preasignados
static packet_t packet_pool[PACKET_POOL];

// Trabajo del UART (USB)
void uart_task(void *args) {
    uart_frame_t frame;
    packet_t *pkt;

    while(1) {
        // Lee el input completo de 17 bytes (1 byte: descriptor del contenido + 16 bytes: datos)
        int len = uart_read_bytes(UART_PORT, (uint8_t*)&frame, sizeof(frame), pdMS_TO_TICKS(15));
        if (len == sizeof(frame)) {
            // Tomamos un paquete libre del pool
            if (xQueueReceive(free_pool, &pkt, portMAX_DELAY) == pdTRUE) {
                // Rellena datos del paquete
                pkt->length = 0;
                pkt->buffer[pkt->length++] = MAGIC_NUMBER;
                pkt->buffer[pkt->length++] = frame.descriptor;

                uint32_t time_stamp = (uint32_t)esp_timer_get_time();
                memcpy(&pkt->buffer[pkt->length], &time_stamp, sizeof(time_stamp)); // Copiamos los bytes de la marca de tiempo al paquete
                pkt->length += sizeof(time_stamp); // Nos movemos el largo de la marca de tiempo a través del paquete

                // Copiamos todos los datos por ahora. El tamaño real será determinado por la función correspondiente al LoRa
                memcpy(&pkt->buffer[pkt->length], frame.raw_data, INPUT_DATA_SIZE);
                pkt->length += INPUT_DATA_SIZE;

                // Mandamos el puntero del paquete al trabajo del LoRa
                xQueueSend(uart_queue, &pkt, portMAX_DELAY);
            } else {
                ESP_LOGW(TAG, "No hay paquetes libres. Entrada descartada");
            }
        }
    }
} 

// Trabajo del LoRa
void lora_task(void *args) {
    packet_t *pkt;

    while (1) {
        if (xQueueReceive(uart_queue, &pkt, portMAX_DELAY) == pdTRUE){
            uint8_t descriptor = pkt->buffer[1];
            uint8_t *raw_data = &pkt->buffer[6]; // Los datos parten despues de 6 bytes

            // Determina el tamaño de los datos
            int payload_size = get_payload_size();

            size_t total_data = INPUT_DATA_SIZE;
            size_t offset_data = 0;

            // Dividimos los datos 
            while(offset_data < total_data) {
                size_t chunk_size = (total_data - offset_data > payload_size) ? payload_size : (total_data - offset_data);
                size_t offset = 0;

                // Header del paquete
                pkt->buffer[offset++] = MAGIC_NUMBER;
                pkt->buffer[offset++] = descriptor;

                // Copiamos la marca de tiempo
                uint32_t time_stamp = (uint32_t)esp_timer_get_time();
                memcpy(&pkt->buffer[offset], &time_stamp, sizeof(time_stamp));
                offset += sizeof(time_stamp);

                // Copiamos los datos
                memcpy(&pkt-> buffer[offset], &raw_data[offset_data], chunk_size);
                offset += chunk_size;

                // Computamos el CRC16 para todo menos el CRC mismo
                uint16_t crc = CRC16(pkt->buffer, offset);
                memcpy(&pkt->buffer[offset], &crc, sizeof(crc)); // Copiamos el CRC al paquete

                // Actualizamos el largo del paquete
                pkt->length = offset;

                // Mandamos el paquete
                lora_send_packet(pkt->buffer, pkt->length);

                ESP_LOGI(TAG, "Sent chunk: desc=0x%02X, payload=%zu, ts=%u, offset=%zu",
                         descriptor, chunk_size, time_stamp, offset_data);
                
                // Nos movemos al siguiente segmento
                offset_data += chunk_size;
            }
        
            // Devuelve el buffer del paquete al pool libre después de mandar todos los segmentos.
            xQueueSend(free_pool, &pkt, portMAX_DELAY);
        }
    }
}

void app_main(void) {
    // Inicialización de UART (8n1)
    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT, &uart_config);
    // Instalación del driver para usar UART para I/O (no buffer para tx)
    uart_driver_install(UART_PORT, BUFFER_SIZE, 0, 0, NULL, 0);

    // Inicialización del módulo LoRa
    lora_init();
    lora_set_frequency(433000000); // 433 MHz
    lora_set_tx_power(14); // Poder de 14dBm (aprox. 25mW)
    lora_enable_crc(); // Habilitar el CRC a nivel hardware. (Es separado al CRC del paquete)
    lora_set_coding_rate(5); // FEC

    // Creamos las colas
    uart_queue = xQueueCreate(PACKET_POOL, sizeof(packet_t*));
    free_pool = xQueueCreate(PACKET_POOL, sizeof(packet_t*));
    
    // Llenamos la pool libre con buffers preasignados
    for(int i = 0; i < PACKET_POOL; i++) {
        xQueueSend(free_pool, &packet_pool[i], 0);
    }

    // Iniciamos las tareas
    xTaskCreate(uart_task, "UART Taks", 4096, NULL, 10, NULL);
    xTaskCreate(lora_task, "LoRa Task", 8192, NULL, 9, NULL);
}
