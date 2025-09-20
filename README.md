# Flight Computer
Derivado del computador de vuelo principal para el testeo de velocidad y fidelidad de los paquetes enviados mediante LoRa.

Capacidades:
- Lectura de datos UART a 115200 baudios desde un computador u otros.
- Entrada de 16 bytes con 1 byte extra de descripción.
- Generación de paquetes de tamaño variable (máximo de 24 bytes) dependiendo de la intensidad de la señal (RSSI).
- División de segmentos grandes de datos en múltiples paquetes asegurando que no haya perdida de datos.
- Cada paquete LoRa tiene: (PHUC)
   - Número mágico (1 byte)
   - Descriptor (1 byte)
   - Marca de tiempo en microsegundos (4 bytes)
   - Payload (longitud variable)
   - CRC16 para detección de errores (2 bytes)
- Uso de colas de FreeRTOS y buffers preasignados para comunicación segura entre tareas y tiempo determinista
- Seguridad robusta, sin asignación dinámica de memoria en el flujo de datos.

# Hardware
- Placa de desarrollo ESP32 (genérica; idealmente de más de un núcleo)
- Módulo LoRa SX1278 (433MHz)
- Fuente de datos UART-USB (dispositivo enviando tramas de 16 bytes + 1 byte descriptor)

## Pines recomendados:
| Módulo                    | Señal   | Pin ESP32 | Notas                          |
| ------------------------- | ------- | --------- | ------------------------------ |
| SD Card (SPI, VSPI\_HOST) | MISO    | 19        | Entrada desde SD               |
|                           | MOSI    | 23        | Salida hacia SD                |
|                           | SCK     | 18        | Reloj SPI                      |
|                           | CS      | 5         | Chip Select                    |
| LoRa SX127x (HSPI\_HOST)  | MISO    | 12        | Entrada desde LoRa             |
|                           | MOSI    | 13        | Salida hacia LoRa              |
|                           | SCK     | 14        | Reloj SPI                      |
|                           | CS (NSS)| 15        | Chip Select                    |
|                           | DIO0    | 2         | Interrupción RX/TX             |
|                           | RESET   | 4         | Pin de reset del módulo LoRa   |
| UART                      | TX / RX | UART0     | Para comunicación con PC / USB |

Los pines de LoRa son configurables mediante:
```bash
idf.py menuconfig
```

# Requisitos
- ESP-IDF v5.5.1 o compatible
- Librería SX127x LoRa (incluida en [componentes](components/))
- FreeRTOS (incluido en ESP-IDF)

# Formato de datos
Entrada UART: 17 bytes
| Campo      | Tamaño   | Descripción                 |
| ---------- | -------- | --------------------------- |
| Descriptor | 1 byte   | Descriptor del tipo de dato |
| Datos      | 16 bytes | Datos brutos                |

Paquete LoRa: Variable (máx. 56 bytes)
| Campo      | Tamaño         | Descripción                     |
| ---------- | -------------- | ------------------------------- |
| Magic      | 1 byte         | `0x69`                          |
| Descriptor | 1 byte         | Igual que el descriptor de UART |
| Timestamp  | 4 bytes        | Tiempo en microsegundos         |
| Payload    | hasta 48 bytes | (3 tramas UART de 16 bytes)     |
| CRC16      | 2 bytes        | Detección de errores            |

### Paquete especial de cierre (guardado de archivos SD)
- Payload completo en 0xFF
- Indica al sistema que se debe cerrar el archivo en la SD y preparar un apagado seguro

# Licencia
Este programa es software libre bajo la Licencia Pública General GNU. Más información [aquí](LICENSE)

Hecho con mucho café por [domi](https://github.com/domigc)
