#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lora.h"

#define UART_PORT       UART_NUM_0
#define UART_BAUD       115200
#define BUFFER_SIZE     256
#define DIO0_PIN        2
#define MAGIC_NUMBER    0x69

static SemaphoreHandle_t dio0_sem;
static const char *TAG = "LoRa_RX";

// ISR para el DIO0
static void IRAM_ATTR dio0_isr(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(dio0_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// Tarea de RX del LoRa
void lora_rx_task(void* arg) {
    uint8_t buffer[BUFFER_SIZE];
    while (1) {
        // Espera a que el ISR indique que hay un paquete
        if (xSemaphoreTake(dio0_sem, portMAX_DELAY) == pdTRUE) {
            int len = lora_receive_packet(buffer, sizeof(buffer));
            if (len > 0 && buffer[0] == MAGIC_NUMBER) {
                ESP_LOGI(TAG, "Paquete recibido (%d bytes)", len);

                // Mostrar en hexadecimal
                printf("HEX: ");
                for (int i = 0; i < len; i++) {
                    printf("%02X", buffer[i]);
                }
                printf("\n");

                uart_write_bytes(UART_PORT, (const char*)buffer, len);
            } else {
                ESP_LOGW(TAG, "Paquete inválido o sin MAGIC_NUMBER");
            }
            // Volver a poner el LoRa en modo continuo
            lora_receive();
        }
        vTaskDelay(1);
    }
}

void app_main(void) {
    // Crear semáforo binario
    dio0_sem = xSemaphoreCreateBinary();
    if (!dio0_sem) {
        ESP_LOGE(TAG, "Error creando semáforo");
        return;
    }

    // Configurar UART
    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT, &uart_cfg);
    uart_driver_install(UART_PORT, BUFFER_SIZE * 2, 0, 0, NULL, 0);

    // Inicializar LoRa
    if (lora_init() == 0) {
        ESP_LOGE(TAG, "No se pudo inicializar LoRa");
        return;
    }
    lora_set_frequency(433E6);
    lora_set_spreading_factor(9);
    lora_set_bandwidth(7);
    lora_set_tx_power(16);
    lora_enable_crc();
    lora_set_dio_mapping(0, 0); // DIO0 -> RxDone
    lora_set_coding_rate(5);
    lora_receive(); // Modo RX continuo

    // Configurar GPIO para DIO0
    gpio_config_t io_cfg = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << DIO0_PIN,
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(DIO0_PIN, dio0_isr, NULL);

    // Crear tarea de recepción LoRa
    xTaskCreate(lora_rx_task, "LoRa_RX_Task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Sistema LoRa-UART inicializado. Esperando paquetes...");
}
