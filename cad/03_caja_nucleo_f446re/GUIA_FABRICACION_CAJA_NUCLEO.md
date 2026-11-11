# Guía de Fabricación y Ensamble: Caja de Alto Rendimiento Térmico para STM32 Nucleo-64

Esta guía técnica detalla los parámetros de manufactura aditiva FDM, tolerancias de ajuste, disipación térmica y procedimiento de ensamblaje para la caja protectora paramétrica de la tarjeta **STM32 Nucleo-64** (modelo de referencia `NUCLEO-60-F401.STEP`).

---

## 1. Especificaciones de la Caja y Tarjetas Compatibles

* **Tarjetas Compatibles**: Toda la familia oficial **STMicroelectronics Nucleo-64** que comparte el estándar físico de 64 pines:
  * NUCLEO-F401RE, NUCLEO-F411RE, NUCLEO-F446RE, NUCLEO-F410RB
  * NUCLEO-G474RE, NUCLEO-G431RB, NUCLEO-L476RG, NUCLEO-L432KC (con shield)
  * NUCLEO-F103RB, NUCLEO-F303RE, NUCLEO-F072RB
* **Dimensiones Exteriores Ensamblada**:
  * Ancho ($X$): $80.00\text{ mm}$
  * Largo ($Y$): $93.00\text{ mm}$
  * Altura Total ($Z$): $28.00\text{ mm}$ ($25.00\text{ mm}$ base chasis + $3.00\text{ mm}$ tapa superior)
* **Espesor de Paredes**: $2.50\text{ mm}$ estructural continuo con filete exterior de $R=4.0\text{ mm}$ y rebordes interiores de rigidización.
* **Espesor del Suelo**: $2.60\text{ mm}$.

---

## 2. Arquitectura de Flujo de Aire y Gestión Térmica (Airflow)

El microcontrolador STM32F401 (ARM Cortex-M4 a 84 MHz) y el chip ST-LINK V2-1 generan una disipación térmica considerable cuando ejecutan algoritmos de control digital en lazo cerrado a frecuencias elevadas (PWM $\ge 20\text{ kHz}$ y muestreo ADC a $1\text{ kHz}$).

```
                             Aire Caliente (Escape)
                                      ▲▲▲
                     ┌────────────────┴┴┴────────────────┐
                     │   Tapa Superior con Louvers y    │
                     │    Ventana Abierta de Shields     │
                     └────────────────┬┬┬────────────────┘
                                      │││
                   [Regulador LDO ST-LINK y MCU STM32F401]
                                      │││
                     ┌────────────────┴┴┴────────────────┐
                     │  PCB Nucleo-64 Elevado (H=10.0mm) │
                     │   Plenum Inferior Libre (2.0mm)   │
                     └────────────────┬┬┬────────────────┘
                                      ▲▲▲
                             Aire Fresco (Entrada)
                        11 Ranuras Inferiores (32x2.5mm)
```

1. **Plenum Convectivo Sub-Placa ($H=10.0\text{ mm}$)**:
   * La tarjeta no se asienta en el fondo. Se soporta sobre 4 separadores cilíndricos de $10.0\text{ mm}$ de altura ($Z=12.60\text{ mm}$).
   * Los pines machos inferiores de los conectores ST Morpho (que sobresalen $8.04\text{ mm}$ por debajo de la PCB hasta $Z=4.56\text{ mm}$) disponen de un margen libre garantizado de $2.00\text{ mm}$ sobre el suelo, impidiendo doblamiento de pines o cortocircuitos por contacto.
2. **Rejilla Inferior de Admisión (11 Ranuras)**:
   * 11 ranuras de $32.00\text{ mm} \times 2.50\text{ mm}$ distribuidas simétricamente bajo la CPU y el regulador lineal de 3.3V/5V.
3. **Efecto Chimenea (Convective Stack Effect)**:
   * El aire frío ingresa por el fondo a través de las 11 ranuras, baña la cara inferior del PCB y asciende a través de las holguras perimetrales de los conectores Morpho y Arduino.
   * El aire caliente se evacúa continuamente por la ventana superior y las 5 rejillas de deflexión (louvers) ubicadas sobre la zona de alimentación ST-LINK.
4. **Patas Antideslizantes Elevadoras**:
   * En la base inferior se integran 4 cajeras cilíndricas de $\varnothing 8.5\text{ mm} \times 0.6\text{ mm}$ para patas de goma/silicona autoadhesivas de $\varnothing 8.0\text{ mm}$, garantizando que la caja mantenga un despeje de $1.5\text{ mm}$ respecto a la mesa de trabajo para que el flujo de aire inferior nunca se bloquee.

---

## 3. Acceso Irrestricto a Conexiones y Controles

| Puerto / Interfaz | Ubicación en Caja | Dimensiones de Ventana | Función y Uso |
| :--- | :--- | :--- | :--- |
| **ST-LINK USB** | Cara Trasera ($+Y$) | $14.0\text{ mm} \times 8.5\text{ mm}$ | Conexión directa de cable USB Mini-B/Micro-B para programación, depuración SWD y puerto COM virtual sin forzar el cable. |
| **ST Morpho CN7** | Lateral Izquierdo ($-X$) | $54.0\text{ mm} \times 10.5\text{ mm}$ | Acceso completo a los 38 pines machos laterales para conexiones analógicas/digitales directas con jumpers dupont hembra. |
| **ST Morpho CN10** | Lateral Derecho ($+X$) | $54.0\text{ mm} \times 10.5\text{ mm}$ | Acceso a los 38 pines machos laterales restantes para buses de comunicación SPI/I2C/USART y alimentación externa. |
| **Arduino Uno V3 Headers** | Ventana Superior Central | $53.0\text{ mm} \times 44.0\text{ mm}$ | Admite inserción directa de Shields comerciales de Arduino Uno, tarjetas de expansión CNC, controladores de motores o cables jumper. |
| **Pulsadores B1 y B2** | Tapa Superior | $22.0\text{ mm} \times 12.0\text{ mm}$ | Pulsación directa y ergonómica con el dedo de **B1 USER** (Botón azul) y **B2 RESET** (Botón negro). |
| **Jumpers de Configuración** | Tapa Superior | Integrado en ventana superior | Acceso con pinzas a los jumpers JP5 (Power Selection) y CN2 (ST-LINK Discovery/Standalone). |
| **Paso Frontal de Cables** | Cara Frontal ($-Y$) | $42.0\text{ mm} \times 10.0\text{ mm}$ | Salida organizada de arneses de cables hacia actuadores del péndulo de Furuta, motores y encoders. |

---

## 4. Parámetros de Impresión 3D Recomendados (Slicer)

| Parámetro | Valor Recomendado | Justificación Técnica |
| :--- | :--- | :--- |
| **Tecnología** | FDM / FFF | Fácil y accesible en impresoras estándar (Ender 3, Bambu Lab, Prusa, etc.). |
| **Material** | **PETG** (óptimo) o **PLA+** | El PETG ofrece excelente resistencia térmica ($75^\circ\text{C}$ HDT) y alta resiliencia mecánica ante apriete de tornillos. PLA+ es aceptable en laboratorio. |
| **Orientación Base** | Cara inferior en la placa de construcción ($Z=0$) | Evita soportes; las ranuras de ventilación se imprimen limpias mediante bridging. |
| **Orientación Tapa** | Cara exterior superior hacia arriba | Garantiza el mejor acabado estético y biselado de los avellanados de tornillos. |
| **Altura de Capa** | $0.20\text{ mm}$ ($0.16\text{ mm}$ opcional) | Balance óptimo entre tiempo de fabricación y definición de filetes y ranuras. |
| **Perímetros / Paredes** | 4 líneas ($1.6\text{ mm}$ con boquilla de $0.4\text{ mm}$) | Proporciona rigidez estructural en las torretas roscadas y paredes laterales. |
| **Capas Superiores/Inferiores** | 5 capas superiores / 5 capas inferiores | Suelos densos para soportar los postes y evitar pandeo. |
| **Relleno (Infill)** | 25% Giroide o Cuadrícula | Distribución uniforme de esfuerzos mecánicos. |
| **Soportes** | **DESACTIVADOS (0%)** | Todas las ventanas y aberturas están diseñadas con distancias de puenteo auto-portantes ($\le 54\text{ mm}$). |
| **Compensación XY** | $0.00\text{ mm}$ | Las holguras paramétricas ya incluyen $0.50\text{ mm}$ de holgura diametral. |

---

## 5. Lista de Materiales (BOM) y Tornillería

1. **Piezas Impresas 3D**:
   * $1\times$ Chasis Base: `Caja_Base_Nucleo.stl` (volumen de plástico: $\sim 34.0\text{ cm}^3$, aprox. $42\text{ g}$ de filamento).
   * $1\times$ Tapa Superior: `Tapa_Caja_Nucleo.stl` (volumen de plástico: $\sim 12.1\text{ cm}^3$, aprox. $15\text{ g}$ de filamento).
2. **Fijación de la Tarjeta Nucleo-64 a los Separadores**:
   * $4\times$ Tornillos M3 x 6 mm cilíndricos (DIN 912 / ISO 4762) o de cabeza de botón (ISO 7380).
3. **Cierre de la Tapa a las Torretas de la Base**:
   * $4\times$ Tornillos M3 x 12 mm o 14 mm de cabeza avellanada (DIN 7991 / ISO 10642).
   * Roscado directo en los orificios piloto de $\varnothing 2.8\text{ mm}$ de las torretas de la base, o usando insertos roscados M3 de latón térmicos ($\varnothing 4.2\text{ mm} \times 4.0\text{ mm}$).
4. **Patas Antideslizantes**:
   * $4\times$ Gotas / almohadillas de silicona o caucho autoadhesivas $\varnothing 8\text{ mm} \times 1.5\text{ mm}$.

---

## 6. Procedimiento de Ensamble Paso a Paso

1. **Preparación de la Base**:
   * Retirar cualquier rebaba o hilo (stringing) en las ranuras de ventilación y ventanas laterales.
   * Pegar las 4 patas de goma en las cajeras del fondo inferior.
2. **Instalación de la Tarjeta Nucleo-64**:
   * Alinear los 4 orificios de montaje de la placa Nucleo-64 con los 4 separadores cilíndricos de $10.0\text{ mm}$.
   * Verificar que el puerto USB ST-LINK quede centrado en la ventana trasera ($+Y$).
   * Ajustar los 4 tornillos M3 x 6 mm con apriete suave sin sobretensionar el circuito impreso.
3. **Colocación y Cierre de la Tapa**:
   * Encajar la tapa asegurando que el reborde perimetral inferior guíe dentro de la cavidad de la base.
   * Instalar los 4 tornillos avellanados M3 x 12 mm en las 4 esquinas hasta que queden a ras de la superficie biselada.
4. **Operación en Banco**:
   * Conectar el cable USB ST-LINK.
   * Conectar los cables dupont de señales analógicas o digitales a través de las ventanas laterales Morpho o la ventana superior de cabezales Arduino.
