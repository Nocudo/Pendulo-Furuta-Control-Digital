# Guía de Impresión 3D y Optimización DfAM — Anycubic Kobra 2 (`220 × 220 × 250 mm`)

Todos los archivos de esta carpeta fueron optimizados para impresión FDM **sin soportes (`SUP = 0`)** y empaquetados específicamente para una impresora **Anycubic Kobra 2** (cama de `220 × 220 × 250 mm`, boquilla de `0.4 mm`, extrusor Direct Drive y filamento **PLA 1.75 mm**).

---

## 1. Resumen de Optimización DfAM (Cero Soportes y Cero Colisiones)

En comparación con el diseño inicial monolítico, el rediseño DfAM del péndulo de Furuta, la partición modular de la torre y el acoplador del motor eliminan el **100 % de las estructuras de soporte** sin modificar las interfaces cinemáticas críticas:

| Métrica | Diseño monolítico previo | Diseño modular actual DfAM (Anycubic Kobra 2) |
| :--- | :--- | :--- |
| Segmentos de soporte en G-Code | `1,365` (`SUPPORT`) + `96` (`SUPPORT-INTERFACE`) | **`0`** (`SUP = 0` en todas las placas) |
| Piezas en `02_Pendulo_Furuta` | 7 piezas (torre monolítica 156 mm, 10h) | **9 piezas** (base inferior + torre superior + tapa desmontable + péndulo) |
| Colisiones ensamblaje Furuta (`freecad-py`) | `3` interferencias (`0.44` a `1.06 mm³`) | **`0 / 23`** interferencias (`0.0000 mm³`) |
| Colisiones ensamblaje Motor/AS5600 (`freecad-py`) | Verificación individual | **`0 / 6`** interferencias (`0.0000 mm³`) |
| Riesgo de fallo en impresión de torre | Alto (tirada continua de 10h 12m) | **Mínimo** (dividida en 2 placas: Base 8h 14m y Torre Superior 2h 40m) |

### Partición mecánica de la torre en dos componentes autoportantes

La torre principal se dividió mecánicamente en el plano **$Z = 70.0\text{ mm}$**, donde la sección del tubo es un cuadrado macizo de $42 \times 42\text{ mm}$ con barreno interno de $\varnothing 32.0\text{ mm}$, ubicado justo entre la terminación de los nervios triangulares ($Z = 68.0\text{ mm}$) y el inicio de las ventanas góticas ($Z = 75.0\text{ mm}$):

1. **[Base_Inferior_Torre_Furuta](02_Pendulo_Furuta/Base_Inferior_Torre_Furuta.step) (`120 × 120 × 82 mm`, Placa 1)**:
   - Pedestal inferior macizo con placa base de $120 \times 120 \times 8\text{ mm}$ y 4 taladros M5 de anclaje a banco.
   - 4 nervios triangulares de refuerzo a $29.5^\circ$ respecto a la vertical ($Z = 8..68\text{ mm}$).
   - **Espiga macho coaxial autoportante**: Cilindro de autocentrado de $\varnothing 31.6\text{ mm}$ de diámetro exterior (holgura diametral de $0.4\text{ mm}$ / $0.2\text{ mm}$ por lado respecto al barreno hembra de $\varnothing 32.0\text{ mm}$ de la torre superior) y $12\text{ mm}$ de altura ($Z = 70..82\text{ mm}$), con chaflán guía cónico de $45^\circ$ en la punta ($Z = 80.5..82.0\text{ mm}$, $\varnothing 28.6\text{ mm}$ en extremo).
   - Barreno pasante central de $\varnothing 24.0\text{ mm}$ ($Z = -1..83\text{ mm}$) que permite descender con holgura los cables del motor Namiki y del encoder AS5600.
   - 4 orificios guía roscables M3 ($\varnothing 2.6\text{ mm} \times 15\text{ mm}$ de profundidad, con avellanado guía) en las esquinas $(\pm 15.0, \pm 15.0\text{ mm})$.

2. **[Torre_Superior_Motor_Furuta](02_Pendulo_Furuta/Torre_Superior_Motor_Furuta.step) (`52 × 52 × 86 mm`, Placa 2)**:
   - Columna superior con barreno coaxial hembra de $\varnothing 32.0\text{ mm}$ que desliza suavemente sobre la espiga de la base inferior.
   - 4 ventanas laterales con arcos apuntados a $40.3^\circ$ ($Z = 130..146.5\text{ mm}$) y ensanchamiento superior a $22.6^\circ$ ($42 \times 42 \to 52 \times 52\text{ mm}$).
   - 4 taladros pasantes de $\varnothing 3.4\text{ mm}$ en las esquinas $(\pm 15.0, \pm 15.0\text{ mm})$ que atraviesan el escalón inferior de las ventanas ($Z = 70..75\text{ mm}$), permitiendo insertar y apretar 4 tornillos M3 $\times$ 12 mm o 16 mm desde el interior de las ventanas con llave Allen.
   - Se imprime invertida a $Z=0$ apoyada sobre su cara plana cuadrada de $42 \times 42\text{ mm}$ sin requerir ningún soporte.

3. **[Tapa_Motor_Torre_Furuta](02_Pendulo_Furuta/Tapa_Motor_Torre_Furuta.step) (`52 × 52 × 7.5 mm`, Placa 3)**:
   - Tapa desmontable de sujeción del motor Namiki 22CL con faldón cilíndrico de centrado y avellanados para 4 tornillos M3 (a torre) y 4 tornillos M2 (a motor). Se imprime en la Placa 3 junto al brazo y la barra.

4. **[Brazo_Horizontal_L88](02_Pendulo_Furuta/Brazo_Horizontal_L88.step) (`87.5 × 28.0 × 39.3 mm`, Placa 3)**:
   - Base inferior totalmente plana apoyada sobre la cama, chaflanes progresivos y transiciones con pendiente autoportante (`≤ 44.5°`).

5. **[Eje_Portaiman_Pendulo](02_Pendulo_Furuta/Eje_Portaiman_Pendulo.step) (Placa 4)** y **[Portaiman_AS5600](02_Pendulo_Furuta/Portaiman_AS5600.step) (Placa 4)**:
   - Transición cónica de `32.7°` y rampa en D a `40.6°`; orientado verticalmente en la Placa 4 para máxima concentricidad coaxial sin soportes.

6. **[Barra_Pendulo_Furuta](02_Pendulo_Furuta/Barra_Pendulo_Furuta.step) (`157.3 × 9.0 × 7.0 mm`, Placa 3)**:
   - Cara trasera coplanar para impresión plana de la barra en la Placa 3.

7. **[Acoplador_Motor_AS5600](02_Pendulo_Furuta/Acoplador_Motor_AS5600.step) (Placa 4)**:
   - Acoplador con loft alineado sin torsión geométrica, pasacables apuntados a `38°`, huecos M3 con techos a `35°` y voladizo máximo de `44.5°`.

---

## 2. Placas Listas para Anycubic Kobra 2 (7 Lotes de Impresión)

Las 7 placas caben holgadamente dentro del área útil de `220 × 220 × 250 mm` y están ubicadas en [Placas_Cura_Listas](Placas_Cura_Listas).

| Placa | Piezas incluidas y orientación en cama | Altura capa / Relleno | Paredes / Adhesión | Tiempo estimado | Filamento | Capas / Soportes | Archivos directos |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Placa 1**<br>*Base Inferior Torre* | `Base_Inferior_Torre_Furuta` (centrada, base plana) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`8h 13m 51s`** | `107.66 g`<br>(`36.10 m`) | `410` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_1_Base_Inferior_Torre.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_1_Base_Inferior_Torre.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_1_Base_Inferior_Torre.curaprofile) |
| **Placa 2**<br>*Torre Superior Motor* | `Torre_Superior_Motor_Furuta` (centrada, cara $42\times42$ en cama) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`2h 39m 49s`** | `30.39 g`<br>(`10.19 m`) | `430` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_2_Torre_Superior_Motor.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_2_Torre_Superior_Motor.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_2_Torre_Superior_Motor.curaprofile) |
| **Placa 3**<br>*Brazo, Barra y Tapa* | `Brazo_Horizontal_L88` (plana)<br>`Barra_Pendulo_Furuta` (`+90°` Y)<br>`Tapa_Motor_Torre_Furuta` (plana) | `0.16 mm` (`0.20 mm` cap. 1)<br>`45% Gyroid` | `5` paredes<br>`Brim 5 mm` | **`2h 45m 34s`** | `27.42 g`<br>(`9.19 m`) | `245` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_3_Brazo_Barra_y_Tapa.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_3_Brazo_Barra_y_Tapa.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_3_Brazo_Barra_y_Tapa.curaprofile) |
| **Placa 4**<br>*Precisión Encoder AS5600* | `Acoplador_Motor_AS5600` (plana)<br>`Eje_Portaiman_Pendulo` (`+90°` Y)<br>`Portaiman_AS5600` (invertida `180°` X) | `0.12 mm` (`0.20 mm` cap. 1)<br>`100% Lines` (sólido) | `6` paredes<br>`Brim 6 mm` | **`1h 14m 09s`** | `8.60 g`<br>(`2.88 m`) | `219` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_4_Precision_Encoder_AS5600.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_4_Precision_Encoder_AS5600.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_4_Precision_Encoder_AS5600.curaprofile) |
| **Placa 5**<br>*Caja Nucleo* | `Caja_Base_Nucleo` (plana)<br>`Tapa_Caja_Nucleo` (invertida `180°` X) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`3h 13m 55s`** | `42.16 g`<br>(`14.13 m`) | `125` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_5_Caja_Nucleo.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_5_Caja_Nucleo.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_5_Caja_Nucleo.curaprofile) |
| **Placa 6**<br>*Caja L298N* | `Caja_Base_L298N` (plana)<br>`Tapa_Caja_L298N` (invertida `180°` X) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`2h 09m 23s`** | `27.68 g`<br>(`9.28 m`) | `130` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_6_Caja_L298N.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_6_Caja_L298N.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_6_Caja_L298N.curaprofile) |
| **Placa 7** *(Opcional)*<br>*Caja ESP32* | `Caja_Base_ESP32` (plana)<br>`Tapa_Caja_ESP32` (invertida `180°` X) | `0.20 mm` (`0.24 mm` cap. 1)<br>`25% Gyroid` | `4` paredes<br>`Skirt` (3 líneas) | **`2h 54m 27s`** | `37.31 g`<br>(`12.51 m`) | `130` capas<br>**`SUP = 0`** | [3MF](Placas_Cura_Listas/Proyectos_3MF/Placa_7_Caja_ESP32.3mf) · [GCode](Placas_Cura_Listas/GCode/Placa_7_Caja_ESP32.gcode) · [Perfil](Placas_Cura_Listas/Perfiles_Cura/Placa_7_Caja_ESP32.curaprofile) |

- **Total Placas 1 a 6 (sistema principal STM32 Nucleo + L298N + Péndulo Furuta modular):** **`20h 16m 41s`** y **`243.91 g`** (`81.77 m`).
- **Total Placas 1 a 7 (incluyendo carcasa opcional ESP32):** **`23h 11m 08s`** y **`281.22 g`** (`94.28 m`).

---

## 3. Parámetros Configurados para Anycubic Kobra 2 (PLA 1.75 mm)

Los proyectos `.3mf` y perfiles `.curaprofile` utilizan la definición oficial **`Anycubic Kobra 2` (`anycubic_kobra2`)** con estos ajustes comunes:

- **Volumen / Boquilla:** `220 × 220 × 250 mm`, boquilla `0.4 mm`, filamento `1.75 mm`.
- **Temperaturas PLA:** Boquilla **`210 °C`** en capa 1 → **`205 °C`** en impresión (`cool_min_temperature = 200 °C`); cama **`60 °C`**.
- **Retracción Direct Drive Kobra 2:** **`2.0 mm` a `70 mm/s`**.
- **Velocidades según placa:**
  - *Placas 1, 2, 5, 6 y 7 (Estructural / Cajas, `0.20 mm`):* `80 mm/s` relleno/paredes internas (`45 mm/s` pared exterior, `35 mm/s` capas superior/inferior, `125 mm/s` desplazamiento), `cool_min_layer_time = 5.0 s`.
  - *Placa 3 (Cinemática dinámica, `0.16 mm`):* `65 mm/s` (`35 mm/s` pared exterior, `30 mm/s` superior/inferior, `120 mm/s` desplazamiento), `cool_min_layer_time = 7.0 s`.
  - *Placa 4 (Precisión Encoder/Ajustes, `0.12 mm`):* `45 mm/s` (`25 mm/s` pared exterior, `22 mm/s` superior/inferior, `110 mm/s` desplazamiento; `40 mm/s` en eje y portaimán), `cool_min_layer_time = 10.0 s`.

---

## 4. Cómo Imprimir en tu Anycubic Kobra 2

### Opción A — Impresión directa desde microSD (Más rápida)
1. Asegúrate de tener el cabezal y cama PEI limpios (alcohol isopropílico) y el *Auto-Level / Z-Offset* calibrado para PLA a `60 °C`.
2. Copia directamente los archivos de [GCode](Placas_Cura_Listas/GCode) (`Placa_1_Base_Inferior_Torre.gcode` a `Placa_6_Caja_L298N.gcode`, más `Placa_7_Caja_ESP32.gcode` si usarás la caja ESP32) a la tarjeta microSD de tu Anycubic Kobra 2.
3. Se recomienda imprimir primero **Placa 4** (`1h 14m`) o **Placa 3** (`2h 45m`) para validar tolerancias de encaje del encoder y motor antes de las piezas estructurales mayores.

### Opción B — Ajustar o re-laminar en UltiMaker Cura 5.14+
1. Abre directamente cualquier archivo `.3mf` de [Proyectos_3MF](Placas_Cura_Listas/Proyectos_3MF) en UltiMaker Cura (`Archivo → Abrir archivo(s)...` → elige abrir como proyecto): ya trae cargada la máquina **Anycubic Kobra 2**, la orientación óptima sobre la cama de `220 × 220 mm` y los parámetros por pieza con **$Z = 0.00\text{ mm}$ (sin elevación ni partes flotantes)**.
2. Si necesitas importar un perfil por separado, usa `Preferencias → Perfiles → Importar` y carga los archivos de [Perfiles_Cura](Placas_Cura_Listas/Perfiles_Cura).

---

## 5. Nota de Ensamblaje y Tornillería

Debido al diseño modular DfAM sin soportes, el ensamble de la torre requiere la siguiente tornillería estándar:

1. **Unión Base Inferior ↔ Torre Superior (Placas 1 y 2):**
   - **`4×` tornillos M3 $\times$ 12 mm o M3 $\times$ 16 mm** (cabeza cilíndrica DIN 912 o botón ISO 7380).
   - Se introducen a través de las 4 ventanas laterales abiertas de la torre superior y se atornillan verticalmente hacia abajo con llave Allen en los orificios guía de $\varnothing 2.6\text{ mm}$ del pedestal base.
2. **Fijación Tapa Motor ↔ Torre Superior (Placas 2 y 3):**
   - **`4×` tornillos avellanados M3 $\times$ 10 mm (DIN 7991)** en los alojamientos cónicos a $90^\circ$ de la tapa (quedan al ras debajo de $Z = 160.0\text{ mm}$).
3. **Fijación Motor Namiki ↔ Tapa Motor:**
   - **`4×` tornillos avellanados M2 (DIN 965)** para garantizar que ninguna cabeza sobresalga por encima de $Z = 160.0\text{ mm}$ y mantener intacta la luz libre de $2.5\text{ mm}$ respecto a `Brazo_Horizontal_L88`.
4. **Fijación Base Pedestal ↔ Banco de Trabajo:**
   - **`4×` tornillos M5** pasantes por los 4 taladros de las esquinas $(\pm 48.0, \pm 48.0\text{ mm})$.

---

## 6. Estructura de Archivos Individuales (`.step` y `.stl`)

Si deseas laminar piezas sueltas en el futuro, todos los modelos individuales actualizados se encuentran organizados en:
- [01_Caja_STM32_Nucleo](01_Caja_STM32_Nucleo) (`Caja_Base_Nucleo`, `Tapa_Caja_Nucleo`)
- [02_Pendulo_Furuta](02_Pendulo_Furuta) (`Base_Inferior_Torre_Furuta`, `Torre_Superior_Motor_Furuta`, `Base_Torre_Furuta`, `Tapa_Motor_Torre_Furuta`, `Brazo_Horizontal_L88`, `Eje_Portaiman_Pendulo`, `Barra_Pendulo_Furuta`, `Masa_Ajustable_Pendulo`, `Acoplador_Motor_AS5600`, `Portaiman_AS5600`)
- [03_Caja_L298N](03_Caja_L298N) (`Caja_Base_L298N`, `Tapa_Caja_L298N`)
- [04_Caja_ESP32](04_Caja_ESP32) (`Caja_Base_ESP32`, `Tapa_Caja_ESP32`)
