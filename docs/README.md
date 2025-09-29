# Documentación del PicoGamepadConverter

## Índice de Documentación

### PlayStation 2 / PSX Compatibility Documentation

La documentación sobre compatibilidad con PlayStation 2 está organizada en los siguientes documentos:

#### 📖 [PlayStation2_Compatibility.md](./PlayStation2_Compatibility.md)
**Documentación principal de compatibilidad con PlayStation 2**

Contenido:
- Introducción al protocolo PSX/PS2
- Arquitectura general del sistema
- Protocolo de comunicación SPI modificado
- Implementación del modo dispositivo
- Implementación del modo host
- Detalles específicos de PS2 vs PSX original
- Mapeo de botones y controles
- Implementación de vibración (rumble)

#### 🔧 [PlayStation2_Technical_Reference.md](./PlayStation2_Technical_Reference.md)
**Referencia técnica detallada**

Contenido:
- Análisis profundo del protocolo SPI modificado
- Diagramas de timing del protocolo
- Implementación PIO paso a paso
- Estados del controlador y máquina de estados
- Detalles de comandos PSX/PS2
- Manejo de interrupciones y sincronización
- Arquitectura multicore
- Patrones de datos de ejemplo
- Consideraciones de hardware
- Herramientas de debugging
- Optimizaciones de rendimiento

#### 🚀 [PlayStation2_Implementation_Guide.md](./PlayStation2_Implementation_Guide.md)
**Guía práctica de implementación**

Contenido:
- Implementación básica paso a paso
- Código de ejemplo listo para usar
- Integración con diferentes fuentes de entrada:
  - USB HID gamepad
  - Teclado PS/2
  - Botones GPIO
- Programa principal completo
- Configuración de hardware sugerida
- PCB layout y consejos de diseño
- Herramientas de testing y validación

### Cómo Usar Esta Documentación

1. **Para entender el funcionamiento**: Comienza con `PlayStation2_Compatibility.md`
2. **Para implementación avanzada**: Consulta `PlayStation2_Technical_Reference.md`
3. **Para crear tu propio proyecto**: Usa `PlayStation2_Implementation_Guide.md`

### Estructura del Protocolo PSX/PS2

El protocolo PlayStation utiliza una variante del SPI con las siguientes características:

```
Pines del Protocolo:
- DAT (Data): Línea de datos Controller → Console
- CMD (Command): Línea de comandos Console → Controller  
- SEL (Select): Chip Select (activo bajo)
- CLK (Clock): Señal de reloj generada por la consola
- ACK (Acknowledge): Confirmación Controller → Console
```

### Modos de Operación Soportados

| Modo | Código | Descripción | Bytes de Datos |
|------|--------|-------------|----------------|
| Digital | 0x41 | Solo botones digitales | 3 |
| Analógico | 0x73 | Botones + sticks analógicos | 7 |
| Analógico con Presión | 0x79 | Botones + sticks + presión | 19 |

### Características Implementadas

- ✅ Protocolo completo PSX/PS2
- ✅ Todos los modos de controlador
- ✅ Soporte para vibración (rumble)
- ✅ Sensores de presión
- ✅ Modo configuración
- ✅ Compatibilidad con PS1 y PS2
- ✅ Implementación usando PIO del RP2040
- ✅ Manejo multicore para rendimiento óptimo

### Comandos PSX/PS2 Soportados

| Comando | Código | Función |
|---------|--------|---------|
| POLL | 0x42 | Leer estado del controlador |
| CONFIG | 0x43 | Entrar/salir modo configuración |
| ANALOG_SWITCH | 0x44 | Cambiar modo digital/analógico |
| STATUS | 0x45 | Obtener información del controlador |
| ENABLE_RUMBLE | 0x4D | Configurar vibración |
| POLL_CONFIG | 0x4F | Configurar sensores de presión |

### Recursos Adicionales

- **Código fuente**: Los archivos de implementación se encuentran en:
  - `src/device_files/psx-device/` - Modo dispositivo
  - `src/host_files/psx-lib/` - Modo host
  - `src/psx_definitions.h` - Definiciones del protocolo

- **Esquemas de hardware**: Ver archivos de imagen en el directorio `docs/`
  - `PS1_PS2_pinout.png` - Pinout del conector PSX/PS2
  - `pico_pinout.png` - Pinout del Raspberry Pi Pico

### Notas de Implementación

La implementación utiliza las siguientes características del RP2040:

- **PIO (Programmable I/O)**: Para manejar el protocolo SPI modificado
- **Multicore**: Core0 para lógica principal, Core1 para protocolo PSX
- **Interrupciones GPIO**: Para detectar inicio/fin de transacciones
- **DMA**: Para transferencia eficiente de datos (modo host)
- **Open-drain I/O**: Para compatibilidad eléctrica con PSX/PS2

### Compatibilidad

Esta implementación es compatible con:

- ✅ PlayStation 1 (PSX)
- ✅ PlayStation 2 (PS2)
- ✅ Controladores originales Sony DualShock
- ✅ Controladores de terceros compatibles
- ✅ Adaptadores USB-PSX/PS2

### Licencia

La implementación está bajo licencia GPLv2. Ver archivo `LICENSE` para más detalles.

---

*Documentación creada para el proyecto PicoGamepadConverter*  
*Última actualización: Diciembre 2024*