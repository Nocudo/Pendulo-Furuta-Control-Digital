# Guía Técnica de Fabricación y Ensamblaje: Caja Protectora Dual Driver L298N

## 1. Visión General y Especificaciones Técnicas

La caja protectora para el controlador dual de motores DC a puente H **L298N** ha sido diseñada paramétricamente en **FreeCAD 1.1** teniendo como prioridad número uno la **gestión térmica pasiva por convección natural** y la **accesibilidad total a borneras y cabezales de control**.

Debido a que el integrado monolítico L298N opera mediante transistores bipolares de unión (BJT) en lugar de MOSFETs de baja $R_{DS(on)}$, presenta caídas de potencial de saturación colector-emisor considerables:

$$V_{drop} = V_{CE(sat, high)} + V_{CE(sat, low)} \approx 1.8\text{ V} \text{ a } 2.4\text{ V}$$

Bajo corrientes típicas de 1.5 A por motor, la potencia disipada como calor puro asciende a:

$$P_{disp} = I_{motor} \cdot V_{drop} \approx 1.5\text{ A} \times 2.0\text{ V} \approx 3.0\text{ W} \text{ a } 6.0\text{ W}$$

Para evitar el sobrecalentamiento térmico del chip sin requerir forzosamente un ventilador ruidoso, el diseño incorpora un **sistema de chimenea convectiva vertical (efecto tiro natural)**:
1. **6 ranuras inferiores de admisión** ($2.0\text{ mm} \times 12.0\text{ mm}$) bajo la zona caliente.
2. **3 persianas laterales** ($2.0\text{ mm} \times 24.0\text{ mm}$) en la pared adyacente a las aletas de aluminio.
3. **6 persianas superiores tipo louvers** ($2.5\text{ mm} \times 16.0\text{ mm}$) en la tapa, posicionadas exactamente sobre el disipador metálico.

---

## 2. Geometría y Cotas Principales

| Parámetro | Valor Nominal | Tolerancia FDM |
| :--- | :--- | :--- |
| **Dimensiones Exteriores Ensamblado** | $65.00\text{ mm} \times 55.00\text{ mm} \times 29.00\text{ mm}$ | $\pm 0.15\text{ mm}$ |
| **Dimensiones Interiores (Cámara)** | $60.00\text{ mm} \times 50.00\text{ mm} \times 24.00\text{ mm}$ | $\pm 0.15\text{ mm}$ |
| **Espesor de Paredes Laterales** | $2.50\text{ mm}$ | $\pm 0.10\text{ mm}$ |
| **Espesor de Base y Tapa** | $2.00\text{ mm}$ (base) / $3.00\text{ mm}$ (tapa con labio de $1.80\text{ mm}$) | $\pm 0.10\text{ mm}$ |
| **Patrón de Montaje PCB (L298N)** | $43.00\text{ mm} \times 33.00\text{ mm}$ ($\pm 21.50\text{ mm}, \pm 16.50\text{ mm}$) | $\pm 0.10\text{ mm}$ |
| **Postes Elevadores de PCB** | $D_{ext} = 6.00\text{ mm}$, $H = 5.00\text{ mm}$, Barreno $d = 2.40\text{ mm}$ | Para M3 directo |
| **Separación Inferior de Pines Soldados** | $2.39\text{ mm}$ de luz libre sobre el piso | Aislamiento dieléctrico |
| **Ventana Motor A (-Y)** | $15.00\text{ mm} \times 18.00\text{ mm}$ | Acceso total a tornillos |
| **Ventana Motor B (+Y)** | $15.00\text{ mm} \times 18.00\text{ mm}$ | Acceso total a tornillos |
| **Ventana Alimentación (+X)** | $18.00\text{ mm} \times 18.00\text{ mm}$ | 12V, GND, +5V |
| **Ventana Tapa Señales Lógicas** | $12.00\text{ mm} \times 15.00\text{ mm}$ | ENA, IN1, IN2, IN3, IN4, ENB |
| **Postes de Fijación de Tapa** | $R = 4.00\text{ mm}$, Tornillos M3 avellanados en tapa | $57.00\text{ mm} \times 47.00\text{ mm}$ |

---

## 3. Lista de Materiales (BOM)

| Ítem | Componente | Cantidad | Especificación / Referencia |
| :---: | :--- | :---: | :--- |
| **1** | Carcasa Base (`Caja_Base_L298N.stl`) | 1 | Impresa en 3D (PLA/PETG/ABS), gris oscuro |
| **2** | Tapa Superior (`Tapa_Caja_L298N.stl`) | 1 | Impresa en 3D (PLA/PETG/ABS), naranja o rojo |
| **3** | Módulo Driver L298N | 1 | Tarjeta comercial estándar ($43 \times 43\text{ mm}$ o $50 \times 40\text{ mm}$ con disipador) |
| **4** | Tornillos Fijación PCB | 4 | M3 $\times 6\text{ mm}$ o M3 $\times 8\text{ mm}$ (autorroscante para termoplástico) |
| **5** | Tornillos Cierre Caja | 4 | M3 $\times 12\text{ mm}$ o M3 $\times 14\text{ mm}$ DIN 7991 (cabeza avellanada) |
| **6** | Patas de Goma Antivibración (Opcional) | 4 | Diámetro $7.0\text{ mm} - 8.0\text{ mm}$, adhesivas |

---

## 4. Parámetros de Fabricación Aditiva (Slicer 3D)

Para lograr una pieza dimensionalmente perfecta, sin deformaciones por alabeo y sin necesidad de soportes:

- **Orientación de Impresión**:
  - `Caja_Base_L298N`: Posicionar con la base plana apoyada sobre la cama caliente ($Z=0$).
  - `Tapa_Caja_L298N`: Posicionar con la cara exterior plana sobre la cama caliente (labio de encaje hacia arriba) para maximizar la calidad estética del acabado superficial.
- **Estructuras de Soporte**: **NINGUNA** (0% soportes). Todos los voladizos de ventanas y louvers tienen ángulos autoportantes $\le 45^\circ$ o puentes directos $< 18\text{ mm}$.
- **Altura de Capa**: $0.20\text{ mm}$ (óptimo balance resistencia mecánica/tiempo). Primera capa a $0.24\text{ mm}$.
- **Perímetros / Paredes**: 3 a 4 perímetros (espesor de pared $\ge 1.2\text{ mm}$).
- **Capas Superiores / Inferiores**: 4 a 5 capas sólidas.
- **Relleno (Infill)**: $20\% \text{ a } 30\%$ tipo Gyroid o Grid para amortiguación de vibraciones mecánicas de motor.
- **Material Recomendado**:
  - **PETG o ABS / ASA**: Recomendado si el driver L298N trabajará de forma continua a corrientes $> 1.5\text{ A}$ (temperaturas de disipador $\approx 60^\circ\text{C}-70^\circ\text{C}$).
  - **PLA / PLA+**: Apto para usos educativos y control de laboratorio (temperaturas $\le 50^\circ\text{C}$).

---

## 5. Protocolo de Ensamble Paso a Paso

1. **Inspección Previa**:
   - Limpiar rebabas en los 4 postes elevadores interiores y en las ventanas de conexión.
   - Probar el roscado de los tornillos M3 en los agujeros piloto de $\varnothing 2.4\text{ mm}$. Si el filamento se cerró ligeramente, repasar con broca manual de $2.5\text{ mm}$.
2. **Inserción del Módulo L298N**:
   - Alinear los 4 taladros de la placa con los 4 postes elevadores. Las borneras de 2 pines (Motor A y Motor B) deben encarar las ventanas frontal y posterior; la bornera de 3 pines (12V/GND/5V) debe encarar la ventana derecha.
   - Ajustar los 4 tornillos M3 $\times 6\text{ mm}$ con torque moderado ($< 0.5\text{ N}\cdot\text{m}$) para no trasroscar el plástico.
3. **Cableado Eléctrico**:
   - Introducir cables de alimentación (calibre 18-20 AWG) y de motor por las ventanas laterales y apretar con destornillador plano a través del espacio libre superior.
   - Conectar los jumpers hembra Dupont a la regleta de control (ENA, IN1, IN2, IN3, IN4, ENB).
4. **Cierre de la Tapa**:
   - Deslizar el labio perimetral de la tapa dentro de la caja base. El labio de $1.8\text{ mm}$ garantiza un calce hermético sin holguras.
   - Atornillar los 4 tornillos avellanados M3 $\times 12\text{ mm}$ en las 4 esquinas exteriores.

---

## 6. Diagrama de Conectividad con STM32 Nucleo-F446RE

```
   STM32 Nucleo-F446RE                 Driver L298N                     Motor DC Furuta
+-----------------------+           +------------------+             +-------------------+
| PB4 (TIM3_CH1 PWM)    | --------> | ENA (Habilitador)|             |                   |
| PA8 (GPIO Output)     | --------> | IN1 (Sentido +)  |             |                   |
| PA9 (GPIO Output)     | --------> | IN2 (Sentido -)  | ===[OUT1]==>| Borna (+) Motor   |
| GND                   | --------> | GND (Común)      | ===[OUT2]==>| Borna (-) Motor   |
+-----------------------+           |                  |             +-------------------+
                                    | +12V Motor       | <--- Fuente 12V DC Externa (3A)
                                    | GND              | <--- GND Fuente Externa
                                    +------------------+
```
