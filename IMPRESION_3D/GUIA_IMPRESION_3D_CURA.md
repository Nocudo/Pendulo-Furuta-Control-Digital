# Guía de Impresión 3D y Optimización DfAM — Anycubic Kobra 2 (`220 × 220 × 250 mm`)

Todos los archivos de esta carpeta fueron optimizados para impresión FDM **sin soportes (`SUP = 0`)** y empaquetados específicamente para una impresora **Anycubic Kobra 2** (cama de `220 × 220 × 250 mm`, boquilla de `0.4 mm`, extrusor Direct Drive y filamento **PLA 1.75 mm**).

---

## 1. Resumen de Optimización DfAM (Cero Soportes y Cero Colisiones)

En comparación con el diseño base anterior, el rediseño DfAM del péndulo de Furuta y del acoplador del motor elimina el **100 % de las estructuras de soporte** sin modificar las interfaces cinemáticas críticas:

| Métrica | Diseño anterior (Eryone Thinker 300×300) | Diseño actual optimizado (Anycubic Kobra 2) |
| :--- | :--- | :--- |
| Segmentos de soporte en G-Code | `1,365` (`SUPPORT`) + `96` (`SUPPORT-INTERFACE`) | **`0`** (`SUP = 0` en todas las placas) |
| Piezas en `02_Pendulo_Furuta` | 7 piezas (torre monolítica con voladizo a 90°) | **8 piezas** (torre + tapa desmontable con acople coaxial) |
| Colisiones ensamblaje Furuta (`freecad-py`) | `3` interferencias (`0.44` a `1.06 mm³`) | **`0 / 15`** interferencias (`0.0000 mm³`) |
| Colisiones ensamblaje Motor/AS5600 (`freecad-py`) | Verificación individual | **`0 / 6`** interferencias (`0.0000 mm³`) |
| Material / tiempo ahorrado en péndulo | Retiro difícil de soportes internos | **Acabado directo limpio sin posproceso de soportes** |

### Cambios geométricos aplicados por pieza (`02_Pendulo_Furuta`)

1. **[Base_Torre_Furuta](02_Pendulo_Furuta/Base_Torre_Furuta.step) (`120 × 120 × 156 mm`, `maxOv = 40.3°`)** + **[Tapa_Motor_Torre_Furuta](02_Pendulo_Furuta/Tapa_Motor_Torre_Furuta.step) (`52 × 52 × 7.5 mm`, `maxOv = 42.0°`)**:
   - Se dividió la cima de la torre para eliminar el techo plano horizontal de montaje del motor.
   - Las 4 ventanas laterales cierran mediante arcos apuntados a `40.3°` (`Z = 130..146.5 mm`) y el collarín superior ensancha progresivamente a `22.6°` (`42×42 mm → 52×52 mm`).
   - La torre se imprime sola en la **Placa 1** centrada en la cama sin riesgo de colisión.
   - La tapa superior se imprime en la **Placa 2** al lado del brazo y la barra, garantizando `2.5 mm` de luz libre debajo de `Brazo_Horizontal_L88` (`Z = 162.5 mm`).
2. **[Brazo_Horizontal_L88](02_Pendulo_Furuta/Brazo_Horizontal_L88.step) (`87.5 × 28.0 × 39.3 mm`, `maxOv = 44.5°`)**:
   - Base inferior totalmente plana apoyada sobre la cama, chaflanes progresivos y transiciones con pendiente autoportante (`≤ 44.5°`).
3. **[Eje_Portaiman_Pendulo](02_Pendulo_Furuta/Eje_Portaiman_Pendulo.step) (`maxOv = 40.7°`)**:
   - Transición cónica de `32.7°` y rampa en D a `40.6°`; orientado verticalmente (`rot_y +90°`) en la Placa 3 para máxima concentricidad coaxial sin soportes.
4. **[Barra_Pendulo_Furuta](02_Pendulo_Furuta/Barra_Pendulo_Furuta.step) (`157.3 × 9.0 × 7.0 mm`, `maxOv = 0.0°`)**:
   - Cara trasera coplanar para impresión plana (`rot_y +90°`) de la barra en la Placa 2.
5. **[Acoplador_Motor_AS5600](02_Pendulo_Furuta/Acoplador_Motor_AS5600.step) (`maxOv = 44.5°`)** y **[Portaiman_AS5600](02_Pendulo_Furuta/Portaiman_AS5600.step) (`maxOv = 45.0°`)**:
   - Acoplador con loft alineado sin torsión geométrica, pasacables apuntados a `38°`, huecos M3 con techos a `35°` y voladizo máximo de `44.5°`; el portaimán se orienta invertido (`flip_x`) en la Placa 3 para apoyarse plano sobre la cama.

---

## 2. Placas Listas para Anycubic Kobra 2 (6 Lotes de Impresión)

Las 6 placas caben holgadamente dentro del área útil de `220 × 220 × 250 mm` y están ubicadas en [Placas_Cura_Listas](Placas_Cura_Listas).

| Placa | Piezas incluidas y orientación en cama | Altura capa / Relleno | Paredes / Adhesión | Tiempo estimado | Filamento | Capas / Soportes | Archivos directos |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Placa 1**<br>*Torre Motor* | `Base_Torre_Furuta` (centrada, plana) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`10h 12m 18s`** | `128.42 g`<br>(`43.06 m`) | `780` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_1_Torre_Motor.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_1_Torre_Motor.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_1_Torre_Motor.curaprofile) |
| **Placa 2**<br>*Brazo, Barra y Tapa* | `Brazo_Horizontal_L88` (plana)<br>`Barra_Pendulo_Furuta` (`+90°` Y)<br>`Tapa_Motor_Torre_Furuta` (plana) | `0.16 mm` (`0.20 mm` cap. 1)<br>`45% Gyroid` | `5` paredes<br>`Brim 5 mm` | **`2h 45m 34s`** | `27.42 g`<br>(`9.19 m`) | `245` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_2_Brazo_Barra_y_Tapa.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_2_Brazo_Barra_y_Tapa.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_2_Brazo_Barra_y_Tapa.curaprofile) |
| **Placa 3**<br>*Precisión Encoder AS5600* | `Acoplador_Motor_AS5600` (plana)<br>`Eje_Portaiman_Pendulo` (`+90°` Y)<br>`Portaiman_AS5600` (invertida `180°` X) | `0.12 mm` (`0.20 mm` cap. 1)<br>`100% Lines` (sólido) | `6` paredes<br>`Brim 6 mm` | **`1h 14m 08s`** | `8.60 g`<br>(`2.88 m`) | `219` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_3_Precision_Encoder_AS5600.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_3_Precision_Encoder_AS5600.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_3_Precision_Encoder_AS5600.curaprofile) |
| **Placa 4**<br>*Caja Nucleo* | `Caja_Base_Nucleo` (plana)<br>`Tapa_Caja_Nucleo` (invertida `180°` X) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`3h 13m 51s`** | `42.16 g`<br>(`14.13 m`) | `125` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_4_Caja_Nucleo.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_4_Caja_Nucleo.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_4_Caja_Nucleo.curaprofile) |
| **Placa 5**<br>*Caja L298N* | `Caja_Base_L298N` (plana)<br>`Tapa_Caja_L298N` (invertida `180°` X) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`2h 09m 16s`** | `27.68 g`<br>(`9.28 m`) | `130` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_5_Caja_L298N.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_5_Caja_L298N.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_5_Caja_L298N.curaprofile) |
| **Placa 6** *(Opcional)*<br>*Caja ESP32* | `Caja_Base_ESP32` (plana)<br>`Tapa_Caja_ESP32` (invertida `180°` X) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`2h 54m 25s`** | `37.31 g`<br>(`12.51 m`) | `130` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_6_Caja_ESP32.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_6_Caja_ESP32.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_6_Caja_ESP32.curaprofile) |

- **Total Placas 1 a 5 (sistema principal STM32 Nucleo + L298N + Péndulo Furuta):** **`19h 35m 07s`** y **`234.28 g`** (`78.54 m`).
- **Total Placas 1 a 6 (incluyendo carcasa opcional ESP32):** **`22h 29m 32s`** y **`271.59 g`** (`91.05 m`).

---

## 3. Parámetros Configurados para Anycubic Kobra 2 (PLA 1.75 mm)

Los proyectos `.3mf` y perfiles `.curaprofile` utilizan la definición oficial **`Anycubic Kobra 2` (`anycubic_kobra2`)** con estos ajustes comunes:

- **Volumen / Boquilla:** `220 × 220 × 250 mm`, boquilla `0.4 mm`, filamento `1.75 mm`.
- **Temperaturas PLA:** Boquilla **`210 °C`** en capa 1 → **`205 °C`** en impresión (`cool_min_temperature = 200 °C`); cama **`60 °C`**.
- **Retracción Direct Drive Kobra 2:** **`2.0 mm` a `70 mm/s`**.
- **Velocidades según placa:**
  - *Placas 1, 4, 5 y 6 (Estructural / Cajas, `0.20 mm`):* `80 mm/s` relleno/paredes internas (`45 mm/s` pared exterior, `35 mm/s` capas superior/inferior, `125 mm/s` desplazamiento), `cool_min_layer_time = 5.0 s`.
  - *Placa 2 (Cinemática dinámica, `0.16 mm`):* `65 mm/s` (`35 mm/s` pared exterior, `30 mm/s` superior/inferior, `120 mm/s` desplazamiento), `cool_min_layer_time = 7.0 s`.
  - *Placa 3 (Precisión Encoder/Ajustes, `0.12 mm`):* `45 mm/s` (`25 mm/s` pared exterior, `22 mm/s` superior/inferior, `110 mm/s` desplazamiento; `40 mm/s` en eje y portaimán), `cool_min_layer_time = 10.0 s`.

---

## 4. Cómo Imprimir en tu Anycubic Kobra 2

### Opción A — Impresión directa desde microSD (Más rápida)
1. Asegúrate de tener el cabezal y cama PEI limpios (alcohol isopropílico) y el *Auto-Level / Z-Offset* calibrado para PLA a `60 °C`.
2. Copia directamente los archivos de [GCode](Placas_Cura_Listas/GCode) (`Placa_1_Torre_Motor.gcode` a `Placa_5_Caja_L298N.gcode`, más `Placa_6_Caja_ESP32.gcode` si usarás la caja ESP32) a la tarjeta microSD de tu Anycubic Kobra 2.
3. Imprime primero **Placa 3** (`1h 14m`) o **Placa 2** (`2h 45m`) si deseas validar tolerancias de encaje antes de la tirada larga de la **Placa 1** (`10h 12m`).

### Opción B — Ajustar o re-laminar en UltiMaker Cura 5.14+
1. Abre directamente cualquier archivo `.3mf` de [Proyectos_3MF](Placas_Cura_Listas/Proyectos_3MF) en UltiMaker Cura (`Archivo → Abrir archivo(s)...` → elige abrir como proyecto): ya trae cargada la máquina **Anycubic Kobra 2**, la orientación óptima sobre la cama de `220 × 220 mm` y los parámetros por pieza.
2. Si necesitas importar un perfil por separado, usa `Preferencias → Perfiles → Importar` y carga los archivos de [Perfiles_Cura](Placas_Cura_Listas/Perfiles_Cura).

---

## 5. Nota de Ensamblaje (Tornillería para `Tapa_Motor_Torre_Furuta`)

Debido a la división de la cima de la torre para eliminar soportes internos:
- **Fijación tapa ↔ torre:** `4×` tornillos **avellanados `M3 × 10 mm` (`DIN 7991`)** en los alojamientos a `90°` de la tapa (quedan al ras debajo de `Z = 160.0 mm`).
- **Fijación motor Namiki ↔ tapa:** `4×` tornillos **avellanados `M2` (`DIN 965`)** para garantizar que ninguna cabeza sobresalga por encima de `Z = 160.0 mm` y mantener intacta la luz libre de `2.5 mm` respecto a `Brazo_Horizontal_L88`.

---

## 6. Estructura de Archivos Individuales (`.step` y `.stl`)

Si deseas laminar piezas sueltas en el futuro, todos los modelos individuales actualizados se encuentran organizados en:
- [01_Caja_STM32_Nucleo](01_Caja_STM32_Nucleo) (`Caja_Base_Nucleo`, `Tapa_Caja_Nucleo`)
- [02_Pendulo_Furuta](02_Pendulo_Furuta) (`Base_Torre_Furuta`, `Tapa_Motor_Torre_Furuta`, `Brazo_Horizontal_L88`, `Eje_Portaiman_Pendulo`, `Barra_Pendulo_Furuta`, `Masa_Ajustable_Pendulo`, `Acoplador_Motor_AS5600`, `Portaiman_AS5600`)
- [03_Caja_L298N](03_Caja_L298N) (`Caja_Base_L298N`, `Tapa_Caja_L298N`)
- [04_Caja_ESP32](04_Caja_ESP32) (`Caja_Base_ESP32`, `Tapa_Caja_ESP32`)
