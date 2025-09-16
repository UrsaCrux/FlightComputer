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
Paquete LoRa: Variable
| Campo      | Tamaño        | Descripción                               |
| ---------- | ------------- | ----------------------------------------- |
| Magic      | 1 byte        | `0x69`                                    |
| Descriptor | 1 byte        | Igual que el descriptor de UAR            |
| Timestamp  | 4 bytes       | Tiempo en microsegundos                   |
| Payload    | 8/12/16 bytes | Determinado por RSSI (puede ser menos)    |
| CRC16      | 2 bytes       | Checksum CRC16 del paquete                |

# Licencia
Este programa es software libre bajo la Licencia Pública General GNU. Más información [aquí](LICENSE)

Hecho con mucho café por [domi](https://github.com/domigc)
