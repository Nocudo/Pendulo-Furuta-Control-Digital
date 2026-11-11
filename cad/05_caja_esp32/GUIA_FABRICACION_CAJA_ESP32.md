# Guía Técnica: Caja Protectora de Sobremesa para ESP32 con Placa de Expansión
## Control del Péndulo Invertido Rotacional de Furuta

Este documento detalla el diseño mecánico, tolerancias de fabricación aditiva (FDM/SLA), ensamblaje y conexionado de la **Caja Protectora de Sobremesa** diseñada en **FreeCAD 1.1** para albergar el microcontrolador **ESP32 NodeMCU de 30 pines** montado sobre su **Placa de Expansión con Borneras de Tornillo (Screw Terminal Breakout Shield)**.

---

## 1. Características Técnicas del Diseño

1. **Arquitectura en Dos Piezas con Ajuste Labial (Lip & Groove)**:
   - **Caja Base (`Caja_Base_ESP32`)**: Monolito rígido con espesor de pared de $2.5 - 5.0\text{ mm}$ y esquinas redondeadas exteriores ($R = 4.0\text{ mm}$) para excelente ergonomía y resistencia a caídas.
   - **Tapa Superior Desmontable (`Tapa_Caja_ESP32`)**: Cuenta con un faldón / labio de centraje perimetral de $1.8\text{ mm}$ de profundidad que penetra en la base con $0.3\text{ mm}$ de holgura diametral, eliminando cualquier holgura lateral o juego sin deformarse.
2. **Compatibilidad Universal con Clones de Expansión**:
   - Patas separadoras (standoffs) sobredimensionadas con ranura de ajuste para fijación con tornillos M3 a distancias de entre ejes de **$60.0\text{ mm}$ en X** y rango de **$45.0 - 50.0\text{ mm}$ en Y**, adaptándose sin forzar a cualquier fabricante de placa de expansión de 30 pines.
   - Elevación del PCB a **$5.0\text{ mm}$ del fondo**, protegiendo las soldaduras inferiores contra cortocircuitos.
3. **Acceso Total a Puertos y Cableado sin Abrir la Caja**:
   - **Ventana Frontal USB**: Abertura de $14.0 \times 15.0\text{ mm}$ centrada en la cara frontal, permitiendo conectar cables Micro-USB o USB Tipo C de cabezal grueso para reprogramación, calibración en tiempo real y telemetría serie.
   - **Ranuras Laterales para Borneras**: Dos ventanas corridas de $48.0 \times 7.5\text{ mm}$ a ambos costados, situadas a la altura exacta de las borneras para cablear sensores AS5600, puente H y alimentación hacia la torre del péndulo.
   - **Abertura Trasera Jack DC**: Taladro de $\varnothing 11.0\text{ mm}$ para conectar una fuente externa con jack estándar de $5.5 \times 2.1\text{ mm}$.
4. **Ventilación Térmica Pasiva por Convección Natural**:
   - 6 ranuras inferiores en el suelo de la caja y 8 ranuras superiores en la tapa orientadas longitudinalmente sobre el chip ESP32 y el regulador de tensión, induciendo un efecto chimenea que previene el sobrecalentamiento.
5. **Apoyos de Sobremesa Antideslizantes**:
   - 4 cajeras circulares de $\varnothing 8.5\text{ mm} \times 1.2\text{ mm}$ en la base inferior para alojar topes / patas de silicona o goma 3M estándar.

---

## 2. Archivos del Proyecto

Todos los archivos CAD se encuentran en `cad/05_caja_esp32/` (y la versión optimizada para impresión en `IMPRESION_3D/04_Caja_ESP32/`):

| Archivo | Formato | Descripción |
|---|---|---|
| `Caja_ESP32_Expansion.FCStd` | FreeCAD 1.1 | Archivo nativo paramétrico editable con árbol de operaciones y materiales |
| `Caja_ESP32_Completa_Assembly.step` | STEP AP214 | Ensamble maestro interoperable con tornillería y electrónica de referencia *(el `.stl` completo se regenera con `--with-huge-stl`)* |
| `Caja_Base_ESP32.stl` / `.step` | STL & STEP | **Pieza para impresión 3D**: Cuerpo principal inferior de la caja |
| `Tapa_Caja_ESP32.stl` / `.step` | STL & STEP | **Pieza para impresión 3D**: Tapa superior con rejilla y avellanados M3 |

---

## 3. Parámetros de Impresión 3D Recomendados (Slicer)

| Parámetro | Caja Base (`Caja_Base_ESP32.stl`) | Tapa Superior (`Tapa_Caja_ESP32.stl`) |
|---|---|---|
| **Material sugerido** | PETG / PLA+ / ABS | PETG / PLA+ (color de contraste) |
| **Altura de capa** | $0.20\text{ mm}$ ($0.16\text{ mm}$ en roscas) | $0.16\text{ mm}$ o $0.20\text{ mm}$ |
| **Perímetros / Paredes** | 4 paredes (mínimo) | 4 paredes |
| **Relleno (Infill)** | $30\% - 40\%$ Giroide | $30\%$ Rectilíneo o Giroide |
| **Soportes** | No requeridos (ventanas puentean limpiamente) | No requeridos |
| **Orientación en cama** | Base plana apoyada en la cama ($Z=0$) | Cara exterior plana sobre la cama ($Z$ invertida) |

---

## 4. Lista de Materiales de Ensamble (BOM)

- **1x** Módulo microcontrolador ESP32 NodeMCU (30 pines, ESP-WROOM-32).
- **1x** Placa de expansión para ESP32 30P con borneras de tornillo.
- **4x** Tornillos M3 $\times 6\text{ mm}$ (cabeza abombada ISO 7380 o autorroscantes) para fijar el PCB a los separadores.
- **4x** Tornillos M3 $\times 12\text{ mm}$ (cabeza cilíndrica DIN 912 o avellanada) para cierre de la tapa.
- **4x** Patas de goma de silicona autoadhesivas $\varnothing 8.0\text{ mm}$.
- **1x** Pieza 3D `Caja_Base_ESP32.stl`.
- **1x** Pieza 3D `Tapa_Caja_ESP32.stl`.

---

## 5. Instrucciones de Montaje Paso a Paso

1. **Fijar la placa de expansión en la base**:
   - Insertar la placa de expansión en la cavidad de la caja base orientando las borneras hacia las ranuras laterales y el conector Jack DC hacia la cara trasera.
   - Atornillar los 4 tornillos M3 $\times 6\text{ mm}$ en los 4 separadores cilíndricos interiores.
2. **Insertar el módulo ESP32**:
   - Conectar el módulo ESP32 en los dos zócalos hembra asegurando que el puerto USB quede alineado con la ventana frontal abierta.
3. **Cableado hacia el Péndulo de Furuta**:
   - Pasar los cables I2C del sensor AS5600 motor (VCC, GND, SDA, SCL) y péndulo por las ranuras laterales.
   - Pasar los cables de control PWM / dirección del motor hacia el driver (L298N / TB6612).
   - Ajustar firmemente los tornillos de las borneras verdes con un destornillador plano de precisión.
4. **Cierre de la caja**:
   - Colocar la tapa superior asegurando que el labio interior encastre en el rebaje de la base.
   - Introducir y apretar moderadamente los 4 tornillos M3 $\times 12\text{ mm}$ en las 4 esquinas exteriores.
5. **Colocar patas antideslizantes**:
   - Pegar las 4 gotas de silicona en los alojamientos circulares inferiores para evitar vibraciones en la mesa de laboratorio.
