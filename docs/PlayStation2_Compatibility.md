# Compatibilidad con PlayStation 2: Implementación y Protocolo

## Introducción

Este documento describe la implementación de la compatibilidad con PlayStation 2 (PS2) en el PicoGamepadConverter. La PS2 utiliza un protocolo de comunicación basado en SPI modificado para comunicarse con los controladores, que es compatible con el protocolo original de PlayStation 1 (PSX) pero con algunas extensiones adicionales.

## Arquitectura General

La implementación se divide en dos partes principales:

1. **Modo Host (PSX Host)**: Para leer controladores PSX/PS2 conectados al Pico
2. **Modo Dispositivo (PSX Device)**: Para emular un controlador PSX/PS2 hacia una consola

## Protocolo de Comunicación PSX/PS2

### Características del Protocolo

El protocolo PSX/PS2 utiliza una variación del SPI con las siguientes características:

- **Velocidad de reloj**: ~250 kHz (la PS2 puede usar hasta 2.5 MHz)
- **Modo SPI**: Modo 3 (CPOL=1, CPHA=1)
- **Orden de bits**: MSB primero
- **Voltaje**: 3.3V

### Pines del Protocolo

```
PIN_DAT = 19    // Línea de datos (Controller → Console)
PIN_CMD = 20    // Línea de comandos (Console → Controller)
PIN_SEL = 21    // Chip Select (activo bajo)
PIN_CLK = 22    // Reloj
PIN_ACK = 26    // Acknowledge (Controller → Console)
```

### Secuencia de Comunicación

1. **Inicio**: La consola baja SEL (Chip Select)
2. **Comando**: La consola envía comandos por CMD sincronizados con CLK
3. **Respuesta**: El controlador responde por DAT y pulsa ACK
4. **Final**: La consola sube SEL para terminar la transacción

## Implementación del Modo Dispositivo

### Estructura de Datos del Controlador

```c
typedef struct __attribute((packed, aligned(1)))
{
    uint8_t buttons1;    // Botones D-Pad y controles
    uint8_t buttons2;    // Botones de acción (X, O, △, ▢, L1, L2, R1, R2)
    uint8_t lx;         // Stick izquierdo X
    uint8_t ly;         // Stick izquierdo Y
    uint8_t rx;         // Stick derecho X
    uint8_t ry;         // Stick derecho Y
    uint8_t l2;         // Presión L2 (modo analógico con presión)
    uint8_t r2;         // Presión R2 (modo analógico con presión)
} PSXInputState;
```

### Modos de Operación

#### 1. Modo Digital (0x41)
- Solo reporta botones digitales (presionado/no presionado)
- Paquete de 3 bytes: [0x5A, buttons1, buttons2]

#### 2. Modo Analógico (0x73)
- Incluye sticks analógicos
- Paquete de 7 bytes: [0x5A, buttons1, buttons2, rx, ry, lx, ly]

#### 3. Modo Analógico con Presión (0x79)
- Incluye presión sensible para todos los botones
- Paquete de 19 bytes con datos de presión para cada botón

### Comandos PSX/PS2 Implementados

| Comando | Valor | Descripción |
|---------|--------|-------------|
| CMD_POLL | 0x42 | Leer estado del controlador |
| CMD_CONFIG | 0x43 | Entrar/salir del modo configuración |
| CMD_ANALOG_SWITCH | 0x44 | Cambiar entre modo digital/analógico |
| CMD_STATUS | 0x45 | Obtener estado del controlador |
| CMD_CONST_46 | 0x46 | Comando de constantes (específico PS2) |
| CMD_CONST_47 | 0x47 | Comando de constantes (específico PS2) |
| CMD_CONST_4C | 0x4C | Comando de constantes (específico PS2) |
| CMD_ENABLE_RUMBLE | 0x4D | Configurar motores de vibración |
| CMD_POLL_CONFIG | 0x4F | Configurar polling de presión |

### Programas PIO para SPI

La implementación utiliza tres programas PIO (Programmable I/O) para manejar el protocolo:

#### 1. cmd_reader (Lector de Comandos)
```assembly
; Espera SEL bajo y lee bits de CMD en flancos de subida de CLK
wait 0 gpio PIN_SEL     ; esperar SEL bajo
set x, 7                ; contador de bits
wait 0 gpio PIN_CLK     ; esperar flanco de bajada
wait 1 gpio PIN_CLK     ; esperar flanco de subida
in pins 1               ; muestrear 1 bit de CMD
```

#### 2. dat_writer (Escritor de Datos)
```assembly
; Envía datos por DAT usando open-drain y genera ACK
set pindirs, 0 side 0   ; liberar línea DAT (Hi-Z)
wait 0 gpio PIN_SEL     ; esperar SEL bajo
pull side 0             ; obtener dato del FIFO
nop side 1 [5]          ; generar pulso ACK
```

#### 3. dat_reader (Lector de Datos - para modo host)
```assembly
; Lee datos de DAT para el modo host
wait 0 gpio PIN_SEL     ; esperar SEL bajo
wait 1 gpio PIN_CLK     ; flancos de subida
in pins 1               ; muestrear DAT
```

### Manejo de Interrupciones

El sistema utiliza interrupciones GPIO para detectar:

1. **Flanco de subida en SEL**: Indica fin de transacción
2. **Reinicio de máquinas de estado PIO**: Prepara para nueva transacción
3. **Reset de Core1**: Maneja casos de transacciones incompletas

```c
void __time_critical_func(sel_isr_callback()) {
    // Confirmar interrupción GPIO
    io_bank0_hw->intr[PIN_SEL / 8] = GPIO_IRQ_EDGE_RISE << (4 * (PIN_SEL % 8));
    // Reiniciar máquinas de estado PIO
    restart_pio_sm();
}
```

## Implementación del Modo Host

### Lectura de Controladores PSX

El modo host utiliza DMA para leer datos de controladores conectados:

```c
void __not_in_flash_func(dma_handler)() {
    // Limpiar interrupción DMA
    dma_hw->ints0 = 1u << dma_chan;
    
    // Reconfigurar transferencia DMA
    dma_channel_set_read_addr(dma_chan, &psx_pio->rxf[sm], false);
    dma_channel_set_write_addr(dma_chan, &data_psx[0], false);
    dma_channel_set_trans_count(dma_chan, N_BYTES, false);
    
    // Llamar callback con datos recibidos
    (*callback_psx)(data_psx);
    
    // Enviar próximo comando
    pio_sm_put_blocking(psx_pio, sm, cmd);
}
```

## Detalles Técnicos Específicos de PS2

### Diferencias con PSX Original

1. **Velocidad de reloj**: PS2 puede usar hasta 2.5 MHz vs ~250 kHz de PSX
2. **Comandos adicionales**: PS2 introduce comandos 0x46, 0x47, 0x4C específicos
3. **Modo de presión**: Soporte completo para sensores de presión en todos los botones
4. **Detección de tiempo**: PS2 es más estricta con los tiemings

### Configuración de Reloj PIO

```c
#define SLOW_CLKDIV 50  // 125MHz ÷ 50 = 2.5 MHz máximo
```

Esta configuración permite operar tanto con PSX (250 kHz) como con PS2 (2.5 MHz).

### Manejo de Open-Drain

Las líneas DAT y ACK utilizan configuración open-drain:

```c
// Configurar pin para open-drain (salida baja pero inicialmente como entrada)
pio_sm_set_pins_with_mask(pio, sm, 0, 1 << PIN_DAT);
pio_sm_set_consecutive_pindirs(pio, sm, PIN_DAT, 1, false);
```

## Mapeo de Botones

### Botones Digitales (buttons1)
```c
#define UP    0b00010000
#define RIGHT 0b00100000  
#define DOWN  0b01000000
#define LEFT  0b10000000
// Bits 0-3: SELECT, L3, R3, START
```

### Botones de Acción (buttons2)
```c
#define L2    0b00000001
#define R2    0b00000010
#define L1    0b00000100
#define R1    0b00001000
#define TRI   0b00010000  // Triángulo
#define CIR   0b00100000  // Círculo
#define X     0b01000000  // X
#define SQU   0b10000000  // Cuadrado
```

## Implementación de Vibración (Rumble)

La PS2 soporta dos motores de vibración:

1. **Motor derecho (pequeño)**: Vibración rápida (0x00 = ON, otros = OFF)
2. **Motor izquierdo (grande)**: Vibración variable (0x00-0xFF)

```c
void processEnableRumble() {
    for(uint8_t i = 0; i < 7; i++) {
        if(i == 0) {
            SEND(0x5A);
            RECV_CMD();
        } else {
            SEND(motorBytes[i - 1]);
            motorBytes[i - 1] = RECV_CMD();  // Recibir nueva configuración
        }
    }
}
```

## Consideraciones de Rendimiento

### Funciones Time-Critical

```c
void __time_critical_func(restart_pio_sm)()
void __time_critical_func(sel_isr_callback())
```

Estas funciones se ejecutan desde RAM para garantizar tiempos de respuesta consistentes.

### Multicore

- **Core 0**: Maneja la lógica principal y comunicación USB
- **Core 1**: Ejecuta el bucle principal del protocolo PSX (`psx_device_main()`)

## Diagrama de Flujo de Comunicación

```
Console                    Controller (Pico)
   |                            |
   |-------- SEL LOW ---------->|
   |                            |
   |-- CMD (0x01) ------------->|
   |<----------- DAT (Mode) ----|
   |                            |<-- ACK pulse
   |                            |
   |-- CMD (0x42) ------------->|
   |<----------- DAT (0x5A) ----|
   |                            |<-- ACK pulse
   |                            |
   |-- CMD (data) ------------->|
   |<----------- DAT (buttons1)-|
   |                            |<-- ACK pulse
   |                            |
   |-- CMD (data) ------------->|
   |<----------- DAT (buttons2)-|
   |                            |<-- ACK pulse
   |                            |
   |-------- SEL HIGH ---------->|
```

## Conclusión

La implementación de compatibilidad con PS2 en el PicoGamepadConverter utiliza las capacidades PIO del RP2040 para manejar el protocolo SPI modificado de manera eficiente. El diseño modular permite tanto leer controladores PSX/PS2 existentes como emular un controlador completo hacia una consola, soportando todos los modos de operación y características avanzadas como vibración y sensores de presión.

La compatibilidad total con PS2 incluye el manejo de las extensiones específicas de PS2 mientras mantiene retrocompatibilidad completa con el protocolo PSX original.