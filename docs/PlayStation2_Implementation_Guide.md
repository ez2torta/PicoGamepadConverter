# Guía de Implementación PlayStation 2

## Guía Práctica para Implementar Compatibilidad PSX/PS2

Este documento proporciona ejemplos prácticos y código listo para usar para implementar compatibilidad con PlayStation 2 en otros proyectos basados en RP2040.

## Implementación Básica Paso a Paso

### Paso 1: Configuración Inicial del Proyecto

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(psx_controller)

pico_sdk_init()

add_executable(psx_controller
    main.c
    psx_device.c
    psxSPI.pio
)

target_link_libraries(psx_controller 
    pico_stdlib
    hardware_pio
    hardware_dma
    pico_multicore
)

pico_add_extra_outputs(psx_controller)
pico_generate_pio_header(psx_controller ${CMAKE_CURRENT_LIST_DIR}/psxSPI.pio)
```

### Paso 2: Estructura de Datos Base

```c
// psx_types.h
#ifndef PSX_TYPES_H
#define PSX_TYPES_H

#include <stdint.h>

// Estructura del estado del controlador
typedef struct __attribute__((packed, aligned(1))) {
    uint8_t buttons1;    // D-pad, START, SELECT, L3, R3
    uint8_t buttons2;    // X, O, △, ▢, L1, L2, R1, R2  
    uint8_t lx;         // Stick izquierdo X (0x00=izq, 0x80=centro, 0xFF=der)
    uint8_t ly;         // Stick izquierdo Y (0x00=arr, 0x80=centro, 0xFF=aba)
    uint8_t rx;         // Stick derecho X
    uint8_t ry;         // Stick derecho Y
    uint8_t l2;         // Presión L2 (0x00=máx, 0xFF=min)
    uint8_t r2;         // Presión R2
} PSXControllerState;

// Definiciones de botones
#define PSX_BTN_SELECT  0x01
#define PSX_BTN_L3      0x02
#define PSX_BTN_R3      0x04
#define PSX_BTN_START   0x08
#define PSX_BTN_UP      0x10
#define PSX_BTN_RIGHT   0x20
#define PSX_BTN_DOWN    0x40
#define PSX_BTN_LEFT    0x80

#define PSX_BTN_L2      0x01
#define PSX_BTN_R2      0x02
#define PSX_BTN_L1      0x04
#define PSX_BTN_R1      0x08
#define PSX_BTN_TRIANGLE 0x10
#define PSX_BTN_CIRCLE  0x20
#define PSX_BTN_X       0x40
#define PSX_BTN_SQUARE  0x80

// Modos del controlador
typedef enum {
    PSX_MODE_DIGITAL = 0x41,
    PSX_MODE_ANALOG = 0x73,
    PSX_MODE_ANALOG_PRESSURE = 0x79,
    PSX_MODE_CONFIG = 0xF3
} PSXControllerMode;

// Comandos PSX
typedef enum {
    PSX_CMD_POLL = 0x42,
    PSX_CMD_CONFIG = 0x43,
    PSX_CMD_ANALOG_SWITCH = 0x44,
    PSX_CMD_STATUS = 0x45,
    PSX_CMD_ENABLE_RUMBLE = 0x4D,
    PSX_CMD_POLL_CONFIG = 0x4F
} PSXCommand;

#endif
```

### Paso 3: Implementación Simple del Protocolo

```c
// psx_simple.c - Implementación mínima para entender el protocolo
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "psx_types.h"

// Definición de pines (ajustar según tu hardware)
#define PSX_DAT_PIN  19
#define PSX_CMD_PIN  20
#define PSX_SEL_PIN  21
#define PSX_CLK_PIN  22
#define PSX_ACK_PIN  26

static PSXControllerState controller_state = {
    .buttons1 = 0xFF,  // Todos los botones liberados (lógica negativa)
    .buttons2 = 0xFF,
    .lx = 0x80,        // Sticks en posición central
    .ly = 0x80,
    .rx = 0x80,
    .ry = 0x80,
    .l2 = 0xFF,        // Gatillos sin presión
    .r2 = 0xFF
};

static PSXControllerMode current_mode = PSX_MODE_DIGITAL;
static bool config_mode = false;

// Función para enviar un bit por la línea DAT
void psx_send_bit(uint8_t bit) {
    if (bit) {
        gpio_set_dir(PSX_DAT_PIN, GPIO_IN);  // Hi-Z = lógico '1'
    } else {
        gpio_set_dir(PSX_DAT_PIN, GPIO_OUT);  // Forzar '0'
        gpio_put(PSX_DAT_PIN, 0);
    }
}

// Función para leer un bit de la línea CMD
uint8_t psx_read_bit() {
    return gpio_get(PSX_CMD_PIN);
}

// Enviar pulso ACK
void psx_send_ack() {
    gpio_set_dir(PSX_ACK_PIN, GPIO_OUT);
    gpio_put(PSX_ACK_PIN, 0);
    sleep_us(2);  // Pulso de ~2μs
    gpio_set_dir(PSX_ACK_PIN, GPIO_IN);  // Volver a Hi-Z
}

// Enviar un byte completo
void psx_send_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        // Esperar flanco de bajada del reloj
        while (gpio_get(PSX_CLK_PIN));
        
        // Enviar bit (LSB primero)
        psx_send_bit(data & (1 << i));
        
        // Esperar flanco de subida del reloj
        while (!gpio_get(PSX_CLK_PIN));
    }
    psx_send_ack();
}

// Recibir un byte completo
uint8_t psx_receive_byte() {
    uint8_t data = 0;
    
    for (int i = 0; i < 8; i++) {
        // Esperar flanco de bajada del reloj
        while (gpio_get(PSX_CLK_PIN));
        
        // Esperar flanco de subida del reloj
        while (!gpio_get(PSX_CLK_PIN));
        
        // Leer bit (LSB primero)
        if (psx_read_bit()) {
            data |= (1 << i);
        }
    }
    
    return data;
}

// Procesar comando POLL (0x42)
void psx_process_poll() {
    config_mode = false;
    
    switch (current_mode) {
        case PSX_MODE_DIGITAL:
            psx_send_byte(0x5A);  // Signature
            psx_receive_byte();   // Dummy read
            psx_send_byte(controller_state.buttons1);
            psx_receive_byte();   // Dummy read
            psx_send_byte(controller_state.buttons2);
            break;
            
        case PSX_MODE_ANALOG:
            psx_send_byte(0x5A);
            psx_receive_byte();
            psx_send_byte(controller_state.buttons1);
            psx_receive_byte();
            psx_send_byte(controller_state.buttons2);
            psx_receive_byte();
            psx_send_byte(controller_state.rx);
            psx_receive_byte();
            psx_send_byte(controller_state.ry);
            psx_receive_byte();
            psx_send_byte(controller_state.lx);
            psx_receive_byte();
            psx_send_byte(controller_state.ly);
            break;
    }
}

// Bucle principal de procesamiento PSX (llamar desde Core1)
void psx_main_loop() {
    while (true) {
        // Esperar SEL bajo (inicio de transacción)
        while (gpio_get(PSX_SEL_PIN));
        
        // Primer byte: siempre 0x01
        if (psx_receive_byte() == 0x01) {
            // Enviar modo actual
            psx_send_byte(config_mode ? PSX_MODE_CONFIG : current_mode);
            
            // Segundo byte: comando
            uint8_t cmd = psx_receive_byte();
            
            switch (cmd) {
                case PSX_CMD_POLL:
                    psx_process_poll();
                    break;
                    
                case PSX_CMD_CONFIG:
                    // Implementar modo configuración
                    config_mode = !config_mode;
                    break;
                    
                case PSX_CMD_ANALOG_SWITCH:
                    if (config_mode) {
                        psx_send_byte(0x5A);
                        psx_receive_byte();
                        psx_send_byte(0x00);
                        uint8_t mode_data = psx_receive_byte();
                        current_mode = (mode_data == 0x01) ? PSX_MODE_ANALOG : PSX_MODE_DIGITAL;
                    }
                    break;
            }
        }
        
        // Esperar SEL alto (fin de transacción)
        while (!gpio_get(PSX_SEL_PIN));
    }
}

// Inicialización
void psx_init() {
    // Configurar pines como entrada con Hi-Z
    gpio_init(PSX_DAT_PIN);
    gpio_init(PSX_CMD_PIN);
    gpio_init(PSX_SEL_PIN);
    gpio_init(PSX_CLK_PIN);
    gpio_init(PSX_ACK_PIN);
    
    gpio_set_dir(PSX_DAT_PIN, GPIO_IN);
    gpio_set_dir(PSX_CMD_PIN, GPIO_IN);
    gpio_set_dir(PSX_SEL_PIN, GPIO_IN);
    gpio_set_dir(PSX_CLK_PIN, GPIO_IN);
    gpio_set_dir(PSX_ACK_PIN, GPIO_IN);
    
    // Deshabilitar pull-ups (PSX/PS2 tienen resistencias externas)
    gpio_disable_pulls(PSX_DAT_PIN);
    gpio_disable_pulls(PSX_CMD_PIN);
    gpio_disable_pulls(PSX_SEL_PIN);
    gpio_disable_pulls(PSX_CLK_PIN);
    gpio_disable_pulls(PSX_ACK_PIN);
}

// Función para actualizar el estado del controlador
void psx_update_controller(PSXControllerState* new_state) {
    controller_state = *new_state;
}
```

## Integración con Diferentes Fuentes de Entrada

### Ejemplo 1: Desde USB HID

```c
// usb_to_psx.c
#include "tusb.h"
#include "psx_types.h"

// Convertir reporte HID de gamepad USB a formato PSX
void convert_usb_to_psx(hid_gamepad_report_t* usb_report, PSXControllerState* psx_state) {
    // Inicializar con todos los botones liberados (lógica negativa)
    psx_state->buttons1 = 0xFF;
    psx_state->buttons2 = 0xFF;
    
    // Mapear D-Pad
    if (usb_report->hat == 0 || usb_report->hat == 1 || usb_report->hat == 7) // Norte
        psx_state->buttons1 &= ~PSX_BTN_UP;
    if (usb_report->hat == 1 || usb_report->hat == 2 || usb_report->hat == 3) // Este
        psx_state->buttons1 &= ~PSX_BTN_RIGHT;
    if (usb_report->hat == 3 || usb_report->hat == 4 || usb_report->hat == 5) // Sur
        psx_state->buttons1 &= ~PSX_BTN_DOWN;
    if (usb_report->hat == 5 || usb_report->hat == 6 || usb_report->hat == 7) // Oeste
        psx_state->buttons1 &= ~PSX_BTN_LEFT;
    
    // Mapear botones (ajustar según tu gamepad USB)
    if (usb_report->buttons & 0x01) psx_state->buttons2 &= ~PSX_BTN_X;        // A -> X
    if (usb_report->buttons & 0x02) psx_state->buttons2 &= ~PSX_BTN_CIRCLE;   // B -> O  
    if (usb_report->buttons & 0x04) psx_state->buttons2 &= ~PSX_BTN_SQUARE;   // X -> ▢
    if (usb_report->buttons & 0x08) psx_state->buttons2 &= ~PSX_BTN_TRIANGLE; // Y -> △
    
    if (usb_report->buttons & 0x10) psx_state->buttons2 &= ~PSX_BTN_L1;       // LB
    if (usb_report->buttons & 0x20) psx_state->buttons2 &= ~PSX_BTN_R1;       // RB
    if (usb_report->buttons & 0x40) psx_state->buttons1 &= ~PSX_BTN_SELECT;   // Back
    if (usb_report->buttons & 0x80) psx_state->buttons1 &= ~PSX_BTN_START;    // Start
    
    // Mapear sticks analógicos (convertir de signed 16-bit a unsigned 8-bit)
    psx_state->lx = (usb_report->x >> 8) + 128;  // -32768..32767 -> 0..255
    psx_state->ly = (usb_report->y >> 8) + 128;
    psx_state->rx = (usb_report->z >> 8) + 128;
    psx_state->ry = (usb_report->rz >> 8) + 128;
    
    // Mapear gatillos analógicos
    psx_state->l2 = 255 - usb_report->brake;     // Invertir (PSX usa 0=máximo)
    psx_state->r2 = 255 - usb_report->accelerator;
}

// Callback cuando se recibe un reporte HID
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    if (len >= sizeof(hid_gamepad_report_t)) {
        hid_gamepad_report_t* gamepad = (hid_gamepad_report_t*)report;
        PSXControllerState psx_state;
        
        convert_usb_to_psx(gamepad, &psx_state);
        psx_update_controller(&psx_state);
    }
}
```

### Ejemplo 2: Desde Teclado PS/2

```c
// ps2kbd_to_psx.c
#include "psx_types.h"

// Mapa de teclado a botones PSX (personalizable)
typedef struct {
    uint8_t scancode;
    uint8_t* button_byte;  // Puntero a buttons1 o buttons2
    uint8_t button_mask;
} KeyMapping;

static PSXControllerState kbd_psx_state = {
    .buttons1 = 0xFF, .buttons2 = 0xFF,
    .lx = 0x80, .ly = 0x80, .rx = 0x80, .ry = 0x80,
    .l2 = 0xFF, .r2 = 0xFF
};

// Mapeo configurable de teclas
static const KeyMapping key_mappings[] = {
    // Teclas de movimiento
    {0x1C, &kbd_psx_state.buttons1, PSX_BTN_UP},      // W
    {0x1B, &kbd_psx_state.buttons1, PSX_BTN_LEFT},    // S  
    {0x23, &kbd_psx_state.buttons1, PSX_BTN_DOWN},    // S
    {0x24, &kbd_psx_state.buttons1, PSX_BTN_RIGHT},   // D
    
    // Botones de acción  
    {0x24, &kbd_psx_state.buttons2, PSX_BTN_X},       // J -> X
    {0x25, &kbd_psx_state.buttons2, PSX_BTN_CIRCLE},  // K -> O
    {0x26, &kbd_psx_state.buttons2, PSX_BTN_SQUARE},  // L -> ▢
    {0x27, &kbd_psx_state.buttons2, PSX_BTN_TRIANGLE},// ; -> △
    
    // Botones especiales
    {0x16, &kbd_psx_state.buttons1, PSX_BTN_START},   // ENTER
    {0x76, &kbd_psx_state.buttons1, PSX_BTN_SELECT},  // ESC
};

// Callback desde el driver de teclado PS/2
void ps2_keyboard_callback(uint8_t scancode, bool pressed) {
    // Buscar el mapeo de la tecla
    for (size_t i = 0; i < sizeof(key_mappings) / sizeof(KeyMapping); i++) {
        if (key_mappings[i].scancode == scancode) {
            if (pressed) {
                *(key_mappings[i].button_byte) &= ~key_mappings[i].button_mask;
            } else {
                *(key_mappings[i].button_byte) |= key_mappings[i].button_mask;
            }
            
            psx_update_controller(&kbd_psx_state);
            return;
        }
    }
}
```

### Ejemplo 3: Controlador Virtual con Botones GPIO

```c
// gpio_to_psx.c
#include "psx_types.h"

// Definir pines para botones físicos
#define BTN_UP_PIN      2
#define BTN_DOWN_PIN    3
#define BTN_LEFT_PIN    4
#define BTN_RIGHT_PIN   5
#define BTN_X_PIN       6
#define BTN_CIRCLE_PIN  7
#define BTN_SQUARE_PIN  8
#define BTN_TRIANGLE_PIN 9
#define BTN_START_PIN   10
#define BTN_SELECT_PIN  11

static PSXControllerState gpio_psx_state = {
    .buttons1 = 0xFF, .buttons2 = 0xFF,
    .lx = 0x80, .ly = 0x80, .rx = 0x80, .ry = 0x80,
    .l2 = 0xFF, .r2 = 0xFF
};

void gpio_controller_init() {
    // Configurar pines de entrada con pull-up
    const uint btn_pins[] = {
        BTN_UP_PIN, BTN_DOWN_PIN, BTN_LEFT_PIN, BTN_RIGHT_PIN,
        BTN_X_PIN, BTN_CIRCLE_PIN, BTN_SQUARE_PIN, BTN_TRIANGLE_PIN,
        BTN_START_PIN, BTN_SELECT_PIN
    };
    
    for (size_t i = 0; i < sizeof(btn_pins) / sizeof(uint); i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }
}

void gpio_controller_update() {
    // Leer estado de los botones (lógica negativa: botón presionado = pin bajo)
    gpio_psx_state.buttons1 = 0xFF;
    gpio_psx_state.buttons2 = 0xFF;
    
    if (!gpio_get(BTN_UP_PIN))    gpio_psx_state.buttons1 &= ~PSX_BTN_UP;
    if (!gpio_get(BTN_DOWN_PIN))  gpio_psx_state.buttons1 &= ~PSX_BTN_DOWN;
    if (!gpio_get(BTN_LEFT_PIN))  gpio_psx_state.buttons1 &= ~PSX_BTN_LEFT;
    if (!gpio_get(BTN_RIGHT_PIN)) gpio_psx_state.buttons1 &= ~PSX_BTN_RIGHT;
    
    if (!gpio_get(BTN_X_PIN))        gpio_psx_state.buttons2 &= ~PSX_BTN_X;
    if (!gpio_get(BTN_CIRCLE_PIN))   gpio_psx_state.buttons2 &= ~PSX_BTN_CIRCLE;
    if (!gpio_get(BTN_SQUARE_PIN))   gpio_psx_state.buttons2 &= ~PSX_BTN_SQUARE;
    if (!gpio_get(BTN_TRIANGLE_PIN)) gpio_psx_state.buttons2 &= ~PSX_BTN_TRIANGLE;
    
    if (!gpio_get(BTN_START_PIN))  gpio_psx_state.buttons1 &= ~PSX_BTN_START;
    if (!gpio_get(BTN_SELECT_PIN)) gpio_psx_state.buttons1 &= ~PSX_BTN_SELECT;
    
    psx_update_controller(&gpio_psx_state);
}
```

## Programa Principal de Ejemplo

```c
// main.c - Programa completo de ejemplo
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tusb.h"
#include "psx_types.h"

// Prototipos de funciones (implementadas arriba)
void psx_init();
void psx_main_loop();
void gpio_controller_init();
void gpio_controller_update();

// Función que se ejecuta en Core1
void core1_main() {
    psx_main_loop();  // Bucle principal del protocolo PSX
}

int main() {
    // Inicialización básica
    stdio_init_all();
    
    // Inicializar controlador GPIO
    gpio_controller_init();
    
    // Inicializar protocolo PSX
    psx_init();
    
    // Lanzar Core1 para manejar el protocolo PSX
    multicore_launch_core1(core1_main);
    
    printf("Controlador PSX iniciado\n");
    
    // Core0: actualizar estado del controlador
    while (true) {
        gpio_controller_update();
        sleep_ms(1);  // Actualizar a ~1kHz
    }
    
    return 0;
}
```

## Configuración de Hardware Sugerida

### Lista de Componentes

- **Raspberry Pi Pico** (RP2040)
- **Conector DB9 o DIN** (según tu aplicación)
- **Resistencias pull-up 4.7kΩ** (opcional, PSX/PS2 las incluye)
- **Capacitores de desacoplamiento 0.1μF**
- **LED indicador** (opcional)

### PCB Layout Recomendado

```
Pico Pin    PSX Signal    Notas
--------    ----------    -----
GP19        DATA          Bidireccional, open-drain
GP20        COMMAND       Entrada desde consola  
GP21        SELECT        Entrada, activo bajo
GP22        CLOCK         Entrada desde consola
GP26        ACK           Salida, open-drain, pulso corto
GND         GND           Común
3V3         VCC           Alimentación 3.3V
```

### Consejos de Diseño

1. **Mantener traces cortas** para señales de reloj y datos
2. **Usar plano de tierra** para reducir ruido
3. **Colocar capacitores de desacoplamiento** cerca del Pico
4. **Protección ESD** recomendada para conexiones externas

## Testing y Validación

### Herramientas de Debug

```c
// debug_tools.c
#include <stdio.h>
#include "pico/stdlib.h"

// Monitor serie para debugging del protocolo
void debug_psx_state(PSXControllerState* state) {
    printf("PSX State - B1:0x%02X B2:0x%02X LX:%d LY:%d RX:%d RY:%d\n",
           state->buttons1, state->buttons2,
           state->lx, state->ly, state->rx, state->ry);
}

// Generador de patrones de prueba
void test_pattern_all_buttons() {
    PSXControllerState test_state = {0};
    
    // Probar cada botón individualmente
    for (int i = 0; i < 8; i++) {
        test_state.buttons1 = 0xFF & ~(1 << i);
        test_state.buttons2 = 0xFF;
        psx_update_controller(&test_state);
        debug_psx_state(&test_state);
        sleep_ms(500);
    }
    
    for (int i = 0; i < 8; i++) {
        test_state.buttons1 = 0xFF;
        test_state.buttons2 = 0xFF & ~(1 << i);
        psx_update_controller(&test_state);
        debug_psx_state(&test_state);
        sleep_ms(500);
    }
}
```

Esta guía proporciona una base sólida para implementar compatibilidad PSX/PS2 en cualquier proyecto basado en RP2040, con ejemplos prácticos para diferentes fuentes de entrada y casos de uso.