# PlayStation 2 - Referencia Técnica Detallada

## Análisis del Protocolo SPI Modificado

### Timing del Protocolo

El protocolo PSX/PS2 tiene requisitos de timing específicos:

```
     SEL  ___     _______________________________________________     ___
             \___|                                               |___/

     CLK  _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
           \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/

     CMD  ----< B0 >< B1 >< B2 >< B3 >< B4 >< B5 >< B6 >< B7 >---------

     DAT  --------< B0 >< B1 >< B2 >< B3 >< B4 >< B5 >< B6 >< B7 >-----

     ACK  ________________________________     ____________________
                                          |___|
```

### Características Temporales

- **Frecuencia de reloj PSX**: ~250 kHz (4 μs por ciclo)
- **Frecuencia de reloj PS2**: hasta 2.5 MHz (400 ns por ciclo)
- **Duración ACK**: ~2-10 μs
- **Setup time**: datos válidos antes del flanco de reloj
- **Hold time**: datos válidos después del flanco de reloj

## Implementación PIO Detallada

### Programa cmd_reader Explicado

```assembly
.program cmd_reader
; Entrada: PIN_CMD (pin 0)
; Función: Lee comandos de la consola sincronizado con reloj

sel_high:
    wait 0 gpio PIN_SEL     ; [1] Esperar que SEL baje (inicio transacción)
    set x, 7                ; [2] Cargar contador de bits (8 bits total)

.wrap_target                ; [3] Inicio del bucle principal
    wait 0 gpio PIN_CLK     ; [4] Esperar flanco de bajada de reloj  
    wait 1 gpio PIN_CLK     ; [5] Esperar flanco de subida de reloj
    in pins 1               ; [6] Leer 1 bit del pin CMD y meterlo en ISR
.wrap                       ; [7] Volver al inicio del bucle

; Notas:
; - Autopush configurado para 8 bits
; - Los datos se shiftan a la derecha en ISR
; - FIFO RX unido para máxima capacidad de buffer
```

### Programa dat_writer Explicado

```assembly
.program dat_writer
.side_set 1 pindirs         ; Sideset controla dirección de pin ACK

; Configuración open-drain para DAT y ACK
set pindirs, 0 side 0       ; [1] DAT como entrada (Hi-Z), ACK como entrada
wait 0 gpio PIN_SEL side 0  ; [2] Esperar SEL bajo, mantener ACK Hi-Z

.wrap_target
pull side 0                 ; [3] Obtener byte del FIFO TX (bloquea si vacío)
nop side 1 [5]             ; [4] ACK bajo por 6 ciclos (señal de reconocimiento)
set x, 7 side 0 [5]        ; [5] ACK alto, cargar contador bits, delay 6 ciclos

sendbit:
    wait 1 gpio PIN_CLK side 0  ; [6] Esperar CLK alto
    wait 0 gpio PIN_CLK side 0  ; [7] Esperar CLK bajo (momento de cambiar datos)
    out pindirs 1 side 0        ; [8] Sacar 1 bit: 0=input(Hi-Z=1), 1=output(Low=0)
    jmp x-- sendbit side 0      ; [9] Decrementar contador, continuar si >0
.wrap

; Explicación open-drain:
; - Bit 0: Pin como entrada → Hi-Z → línea pulled-up → lógico '1'  
; - Bit 1: Pin como salida low → fuerza '0' → lógico '0'
; - Permite multiple-drive sin cortocircuitos
```

### Configuración de Velocidad

```c
// Cálculo del divisor de reloj
#define SLOW_CLKDIV 50

// Para sistema a 125 MHz:
// Frecuencia PIO = 125 MHz / 50 = 2.5 MHz
// Esto permite compatibilidad con PS2 (hasta 2.5 MHz) y PSX (250 kHz)

static inline void cmd_reader_program_init(PIO pio, uint sm, uint offset) {
    pio_sm_config c = cmd_reader_program_get_default_config(offset);
    
    // Configurar divisor de reloj
    sm_config_set_clkdiv_int_frac(&c, SLOW_CLKDIV, 0x00);
    
    // Resto de configuración...
}
```

## Estados del Controlador PSX/PS2

### Máquina de Estados del Modo

```
    [DIGITAL]
        |
        | CMD_ANALOG_SWITCH (0x44) con data=0x01
        v
    [ANALOG] ←--→ [ANALOG_PRESSURE]
        |              ^
        |              | CMD_POLL_CONFIG (0x4F)
        |              | con configuración de presión
        | CMD_ANALOG_SWITCH (0x44) con data=0x00
        v
    [DIGITAL]
```

### Modo Configuración

```
[NORMAL_MODE] --→ CMD_CONFIG(0x43) con config=1 --→ [CONFIG_MODE]
      ^                                                    |
      |                                                    |
      ←-- CMD_CONFIG(0x43) con config=0 ←--←--←--←--←--←--
```

## Detalles de Implementación de Comandos

### CMD_POLL (0x42) - Lectura de Estado

```c
void processPoll() {
    config = false;  // Salir del modo configuración
    
    switch(mode) {
        case MODE_DIGITAL: {
            uint8_t buf[3] = { 
                0x5A,                    // Signature byte
                inputState->buttons1,    // D-pad + START/SELECT/L3/R3
                inputState->buttons2     // Botones acción + shoulders
            };
            
            for(uint8_t i = 0; i < 3; i++) {
                SEND(buf[i]);           // Enviar dato
                processRumble(i, RECV_CMD()); // Procesar rumble si aplicable
            }
            break;
        }
        
        case MODE_ANALOG: {
            uint8_t buf[7] = { 
                0x5A,                    // Signature
                inputState->buttons1,    // Botones digitales
                inputState->buttons2,    // Botones digitales
                inputState->rx,         // Stick derecho X (0x00=izq, 0xFF=der)
                inputState->ry,         // Stick derecho Y (0x00=arr, 0xFF=aba)
                inputState->lx,         // Stick izquierdo X
                inputState->ly          // Stick izquierdo Y
            };
            
            for(uint8_t i = 0; i < 7; i++) {
                SEND(buf[i]);
                processRumble(i, RECV_CMD());
            }
            break;
        }
        
        case MODE_ANALOG_PRESSURE: {
            // Incluye presión para cada botón (0x00=máxima, 0xFF=sin presión)
            uint8_t buf[19] = { 
                0x5A,
                inputState->buttons1, inputState->buttons2,  // Botones digitales
                inputState->rx, inputState->ry,             // Sticks
                inputState->lx, inputState->ly,
                // Presión botones D-pad (invertida: 0x00=presionado, 0xFF=libre)
                (inputState->buttons1 & RIGHT) ? 0x00 : 0xFF,
                (inputState->buttons1 & LEFT)  ? 0x00 : 0xFF,
                (inputState->buttons1 & UP)    ? 0x00 : 0xFF,
                (inputState->buttons1 & DOWN)  ? 0x00 : 0xFF,
                // Presión botones acción
                (inputState->buttons2 & TRI) ? 0x00 : 0xFF,   // △
                (inputState->buttons2 & CIR) ? 0x00 : 0xFF,   // ○
                (inputState->buttons2 & X)   ? 0x00 : 0xFF,   // ✕
                (inputState->buttons2 & SQU) ? 0x00 : 0xFF,   // ▢
                (inputState->buttons2 & L1)  ? 0x00 : 0xFF,   // L1
                (inputState->buttons2 & R1)  ? 0x00 : 0xFF,   // R1
                inputState->l2,              // L2 presión analógica
                inputState->r2               // R2 presión analógica
            };
            
            for(uint8_t i = 0; i < 19; i++) {
                SEND(buf[i]);
                processRumble(i, RECV_CMD());
            }
            break;
        }
    }
}
```

### CMD_ENABLE_RUMBLE (0x4D) - Configuración de Vibración

```c
void processEnableRumble() {
    if (!config) return;  // Solo en modo configuración
    
    for(uint8_t i = 0; i < 7; i++) {
        if(i == 0) {
            SEND(0x5A);           // Signature
            RECV_CMD();           // Ignorar respuesta
        } else {
            SEND(motorBytes[i-1]); // Enviar configuración actual
            motorBytes[i-1] = RECV_CMD(); // Recibir nueva configuración
        }
    }
    
    // motorBytes[0]: Motor pequeño (0x00=ON, otros=OFF)
    // motorBytes[1]: Motor grande (0x00-0xFF intensidad)
    // motorBytes[2-5]: Configuración adicional
}
```

## Manejo de Interrupciones y Sincronización

### ISR de SEL (Chip Select)

```c
void __time_critical_func(sel_isr_callback()) {
    // Optimización: confirmar IRQ inline para velocidad
    check_gpio_param(PIN_SEL);
    io_bank0_hw->intr[PIN_SEL / 8] = GPIO_IRQ_EDGE_RISE << (4 * (PIN_SEL % 8));
    
    // Reiniciar máquinas de estado
    restart_pio_sm();
}

void __time_critical_func(restart_pio_sm)() {
    // Detener máquinas de estado
    pio_set_sm_mask_enabled(psx_device_pio, 
                           1 << smCmdReader | 1 << smDatWriter, false);
    
    // Reiniciar program counter
    pio_restart_sm_mask(psx_device_pio, 1 << smCmdReader | 1 << smDatWriter);
    pio_sm_exec(psx_device_pio, smCmdReader, pio_encode_jmp(offsetCmdReader));
    pio_sm_exec(psx_device_pio, smDatWriter, pio_encode_jmp(offsetDatWriter));
    
    // Limpiar FIFOs
    pio_sm_clear_fifos(psx_device_pio, smCmdReader);
    pio_sm_drain_tx_fifo(psx_device_pio, smDatWriter);
    
    // Reiniciar Core1 (maneja transacciones incompletas)
    multicore_reset_core1();
    multicore_launch_core1(core1_function);
    
    // Reactivar máquinas de estado sincronizadamente
    pio_enable_sm_mask_in_sync(psx_device_pio, 1 << smCmdReader | 1 << smDatWriter);
}
```

### Arquitectura Multicore

```
Core 0 (Control Principal):
- Inicialización del sistema
- Comunicación USB/HID
- Manejo de interrupciones GPIO
- Conversión de datos entre formatos

Core 1 (Protocolo PSX):
- Bucle principal psx_device_main()
- Procesamiento de comandos PSX
- Gestión de estado del controlador
- Comunicación con máquinas PIO
```

## Patrones de Datos de Ejemplo

### Controlador en Reposo (Digital)
```
CMD: [0x01] [0x42] [0x00] 
DAT: [0x41] [0x5A] [0xFF] [0xFF]
     ^mode  ^sig   ^btn1  ^btn2
```

### Controlador con Botón X Presionado (Digital)
```
CMD: [0x01] [0x42] [0x00]
DAT: [0x41] [0x5A] [0xFF] [0xBF]  // bit 6 de buttons2 = 0
     ^mode  ^sig   ^btn1  ^btn2
```

### Controlador Analógico con Stick Movido
```
CMD: [0x01] [0x42] [0x00] [0x00] [0x00] [0x00] [0x00]
DAT: [0x73] [0x5A] [0xFF] [0xFF] [0x80] [0x40] [0x80] [0x80]
     ^mode  ^sig   ^btn1  ^btn2  ^RX    ^RY    ^LX    ^LY
```

## Consideraciones de Hardware

### Configuración de Pines

```c
void init_pio() {
    // Configurar todos los pines como entrada inicialmente
    gpio_set_dir(PIN_DAT, false);  // Entrada (será controlado por PIO)
    gpio_set_dir(PIN_CMD, false);  // Entrada 
    gpio_set_dir(PIN_SEL, false);  // Entrada
    gpio_set_dir(PIN_CLK, false);  // Entrada
    gpio_set_dir(PIN_ACK, false);  // Entrada (será controlado por PIO)
    
    // Deshabilitar pull-ups/downs (PS2 tiene sus propias resistencias)
    gpio_disable_pulls(PIN_DAT);
    gpio_disable_pulls(PIN_CMD);
    gpio_disable_pulls(PIN_SEL);
    gpio_disable_pulls(PIN_CLK);
    gpio_disable_pulls(PIN_ACK);
    
    // Configurar velocidad y corriente para pines críticos
    gpio_set_slew_rate(PIN_DAT, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(PIN_DAT, GPIO_DRIVE_STRENGTH_12MA);
}
```

### Esquema de Conexión

```
PS2 Controller Port     Raspberry Pi Pico
┌─────────────────┐    ┌─────────────────┐
│  1: DATA        │────│ GPIO 19 (PIN_DAT)│
│  2: COMMAND     │────│ GPIO 20 (PIN_CMD)│  
│  3: +7.5V       │    │                 │
│  4: GND         │────│ GND             │
│  5: +3.3V       │────│ 3V3             │
│  6: ATT         │────│ GPIO 21 (PIN_SEL)│
│  7: CLK         │────│ GPIO 22 (PIN_CLK)│
│  8: N/C         │    │                 │
│  9: ACK         │────│ GPIO 26 (PIN_ACK)│
└─────────────────┘    └─────────────────┘
```

## Debugging y Diagnóstico

### Monitoreo del Protocolo

```c
// Función para debug del protocolo
void debug_psx_transaction(uint8_t cmd, uint8_t* response, uint8_t len) {
    printf("CMD: 0x%02X -> ", cmd);
    for(int i = 0; i < len; i++) {
        printf("0x%02X ", response[i]);
    }
    printf("\n");
}

// Medición de timing con GPIO toggle
#define DEBUG_PIN 28

void toggle_debug_pin() {
    static bool state = false;
    gpio_put(DEBUG_PIN, state);
    state = !state;
}
```

### Errores Comunes y Soluciones

1. **Transacciones incompletas**: 
   - Causa: SEL sube antes de completar todos los bytes
   - Solución: Reset automático de Core1 en ISR

2. **Timing incorrecto**:
   - Causa: Divisor de reloj PIO incorrecto
   - Solución: Ajustar SLOW_CLKDIV según la consola

3. **Datos corruptos**:
   - Causa: Interferencia o niveles de voltaje incorrectos
   - Solución: Verificar conexiones y alimentación

## Optimizaciones de Rendimiento

### Funciones Time-Critical en RAM

```c
// Estas funciones se colocan en RAM para acceso más rápido
void __time_critical_func(sel_isr_callback())
void __time_critical_func(restart_pio_sm())
void __not_in_flash_func(kbd_irq())
void __not_in_flash_func(dma_handler())
```

### Uso Eficiente de FIFO

```c
// Unir FIFOs para mayor capacidad de buffer
sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);  // cmd_reader
sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);  // dat_writer
```

Esta documentación técnica proporciona una comprensión profunda de cómo funciona la implementación del protocolo PSX/PS2, permitiendo su adaptación y mejora para otros proyectos.