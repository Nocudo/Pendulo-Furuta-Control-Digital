# Guía Técnica: Péndulo Invertido Rotacional de Furuta Impreso en 3D
## Ensamble Verificado: Brazo Sostenido Solo por Abajo (Entrada Libre del Encoder)

Este documento detalla el diseño mecánico optimizado, manufactura aditiva (impresión 3D), ensamble, tolerancias y formulación matemática para el sistema de **Péndulo Invertido Rotacional de Furuta** desarrollado en **FreeCAD 1.1** para la asignatura de Control Digital (Ingeniería Mecatrónica / Mecánica, Universidad EAFIT).

---

## 🚀 Innovaciones y Correcciones Clave del Rediseño

1. **Brazo Horizontal Monolítico con Caja del Encoder Totalmente Fusionada (`Brazo_Horizontal_L88`)**:
   - **Caja y Brazo en Una Sola Pieza Continua**: La caja distal del rodamiento y encoder se bajó hasta el plano inferior del brazo ($Z = 163.0\text{ mm}$), eliminando cualquier discontinuidad o escalón inferior.
   - **Eliminación Total del Borde Interno**: El fondo de la cajera del PCB AS5600 se sitúa a ras con la superficie superior del brazo ($Z = 171.0\text{ mm}$), haciendo desaparecer por completo el reborde/labio inferior interno.
   - **Deslizamiento Inmediato y Canal de Cables Directo**: El PCB entra libremente deslizándose sobre la cara superior del brazo, y la ranura para cables (`wire_groove`) conecta directamente con los pines de salida sin curvas forzadas.
   - **Acceso 100% Despejado**: Toda la cara posterior se mantiene completamente abierta para atornillar los 4 tornillos M3 de sujeción y conectar el bus I2C.

2. **Elevación Anti-Roce sobre el Armazón del Motor**:
   - La base inferior de todo el brazo se sitúa a **$Z = 163.0\text{ mm}$**, garantizando **$3.0\text{ mm}$ de holgura axial libre** sobre la placa superior de la torre ($Z = 160.0\text{ mm}$).
   - El cubo del motor cuenta con un cuello espaciador inferior a $Z = 162.0\text{ mm}$ ($2.0\text{ mm}$ de luz).
   - **Cero Roce**: El brazo gira libremente $360^\circ$ en el plano horizontal con holgura garantizada respecto a la torre, el reductor y los tornillos M2 de fijación del motor.

3. **Facilidad de Inserción y Servicio del Rodamiento 608**:
   - **Tolerancia FDM de Ajuste Manual**: Diámetro en CAD calibrado a **$\varnothing 22.20\text{ mm}$** ($R = 11.10\text{ mm}$) para compensar la contracción térmica radial del filamento (~$0.12\text{ mm}$), permitiendo deslizar el rodamiento con una suave presión manual del pulgar.
   - **Chaflán Cónico Guía a $45^\circ$**: En la embocadura exterior ($X \in [79.5, 81.5\text{ mm}]$), una transición cónica expansiva ($\varnothing 22.20 \to \varnothing 24.20\text{ mm}$) guía y autocentra el rodamiento automáticamente sin que entre torcido ni muerda las capas.
   - **Hombro de Tope con Cero Rozamiento ($\varnothing 14.0\text{ mm}$)**: Asiento perimetral plano de $3.6\text{ mm}$ que apoya únicamente la pista exterior fija, garantizando cero contacto con la pista interior rotativa ($\varnothing 12.0\text{ mm}$) y los sellos de goma 2RS.
   - **2 Ranuras de Expulsión para Mantenimiento**: Aberturas en la pared trasera del hombro que permiten introducir una punta de llave allen de $2\text{ mm}$ o destornillador plano desde el interior para empujar el rodamiento hacia afuera con total facilidad en caso de sustitución.

4. **Eliminación Total de Interferencias Motor-Torre ($0.0000\text{ mm}^3$) y Acoplador Pegado**:
   - **Plano de Apoyo Inferior ($Z = 156.0\text{ mm}$)**: El motor Namiki 22CL se acopla contra la cara inferior de la placa superior de la torre.
   - **Alojamiento Concéntrico de Cuello**: La cajera de autocentrado de $\varnothing 10.2\text{ mm} \times 1.6\text{ mm}$ se encuentra en el intradós (lado inferior) de la placa, recibiendo el reductor con $0.2\text{ mm}$ de holgura.
   - **Paso Limpio de Eje**: El eje motor ($\varnothing 4.0\text{ mm}$) atraviesa un taladro de $\varnothing 6.0\text{ mm}$ en la placa superior ($Z \in [156, 160\text{ mm}]$) y sobresale $7.5\text{ mm}$ por encima de la torre para acoplar el cubo del brazo.
   - **Acoplador Trasero Pegado con Cianoacrilato (Sin Abrazadera Mecánica)**: El porta encoder del motor `Acoplador_Motor_AS5600` se adhiere directamente al cuerpo del motor mediante pegante instantáneo (holgura capilar $0.08\text{ mm}$), eliminando orejetas, tornillos o tuercas exteriores que sobresalgan dentro de la torre.

5. **Torre Abierta con 4 Columnas y Aletas Completas**:
   - Vanos abiertos de **$28.0\text{ mm}$ de ancho $\times 81.0\text{ mm}$ de alto** en las 4 caras para acceso $360^\circ$ a terminales y cableado I2C.
   - **4 aletas de refuerzo triangulares 100% completas y macizas** (radio $55\text{ mm}$, altura $60\text{ mm}$) anclando rígidamente la torre a la base de $120 \times 120\text{ mm}$.

6. **Despeje Exacto de 3.0 cm al Piso (Operación Sobremesa)**:
   - Pivote situado a **$Z_{\text{pivote}} = 182.8\text{ mm}$**.
   - Con la varilla del péndulo extendiéndose hasta $Z = 34.0\text{ mm}$ y punta redondeada de $R = 4.0\text{ mm}$ ($152.8\text{ mm}$ de longitud total), el punto más bajo en reposo es exactamente **$Z_{\text{mín}} = 30.00\text{ mm}$** ($3.0\text{ cm}$ de despeje respecto a la mesa de trabajo).

7. **Paleta Cromática de Alto Contraste**:
   - Cada componente del sistema cuenta con un color único e inconfundible para evitar confusiones durante el diseño, visualización y montaje:
     - **Torre Principal**: Gris Grafito Oscuro Mate (`#292E36`)
     - **Brazo Horizontal Monolítico**: Azul Eléctrico Cyan (`#00AEE0`)
     - **Barra del Péndulo**: Naranja de Seguridad Fluorescente (`#FF550C`)
     - **Masa Ajustable**: Púrpura Anodizado Imperial (`#A61FC7`)
     - **Eje y Portaimán**: Acero Inoxidable Brillante (`#EBEEF5`)
     - **Rodamiento 608**: Titanio Metálico Pulido (`#808794`)
     - **PCB AS5600 del Péndulo**: Verde Esmeralda FR4 (`#0DC74D`)
     - **Imanes Diametrales**: Rojo Rubí Escarlata (`#F20D26`)
     - **Carcasa Motor Namiki 22CL**: Negro Satinado (`#1A1A1E`)
     - **Reductor Planetario Motor**: Aluminio Champagne (`#D1D1CC`)
     - **Acoplador Trasero Motor AS5600**: Amarillo Lima Ácido (`#E0EB1A`)
     - **Portaimán Trasero Motor**: Rosa Coral Vibrante (`#F2408C`)
     - **PCB AS5600 Trasero Motor**: Azul Real Navy FR4 (`#1440BF`)

---

## 1. Estructura de Archivos del Proyecto

Todos los archivos se encuentran disponibles en `cad/01_pendulo_furuta/` y sincronizados para impresión en `IMPRESION_3D/02_Pendulo_Furuta/`:

| Archivo | Formato | Descripción |
|---|---|---|
| `Pendulo_Furuta_Completo.FCStd` | FreeCAD 1.1 | Ensamble paramétrico completo editable en FreeCAD con árbol de operaciones y materiales |
| `Pendulo_Furuta_Assembly.step` | STEP AP214 | Ensamble master interoperable para SolidWorks, Inventor, Fusion 360, Onshape |
| `Pendulo_Furuta_Assembly.stl` | STL Mesh | Malla 3D del conjunto completo para visualización o simulación física |
| `Base_Torre_Furuta.stl` / `.step` | STL & STEP | **Pieza 3D**: Torre de 4 columnas abiertas ($156\text{ mm}$) con arcos autoportantes y base $120 \times 120\text{ mm}$ |
| `Tapa_Motor_Torre_Furuta.stl` / `.step` | STL & STEP | **Pieza 3D**: Tapa desmontable del motor ($52 \times 52 \times 7.5\text{ mm}$) con avellanados M3/M2 |
| `Brazo_Horizontal_L88.stl` / `.step` | STL & STEP | **Pieza 3D**: Brazo plano DfAM sin soportes, sosteniendo la caja inferior del encoder |
| `Eje_Portaiman_Pendulo.stl` / `.step` | STL & STEP | **Pieza 3D**: Eje para rodamiento 608 ($\varnothing 8\text{ mm}$), copa para imán y espiga D con cono a $32.7°$ |
| `Barra_Pendulo_Furuta.stl` / `.step` | STL & STEP | **Pieza 3D**: Barra oscilante coplanar con cubo hembra D, corredera de $50\text{ mm}$ y despeje de $30\text{ mm}$ |
| `Masa_Ajustable_Pendulo.stl` / `.step` | STL & STEP | **Pieza 3D**: Contrapeso cilíndrico de $\varnothing 18\text{ mm}$ en dos mitades con tornillo M3 |

---

## 2. Parámetros del Modelo Dinámico (EAFIT - Toro Santamaría, 2009)

| Símbolo | Parámetro | Valor Nominal | Unidades | Descripción |
|---|---|---|---|---|
| $L_0$ | Longitud del brazo horizontal | **$88.50$** ($0.0885$) | $\text{mm}$ ($\text{m}$) | Distancia centro a centro entre eje motor y pivote del péndulo |
| $L_1$ | Longitud total del péndulo | **$155.00$** ($0.155$) | $\text{mm}$ ($\text{m}$) | Longitud desde el eje de rotación ($Z=185$) hasta la punta ($Z=30$) |
| $l_1$ | Centro de gravedad del péndulo | **$82.0 - 92.5$** | $\text{mm}$ | Distancia al centro de masa (sintonizable con la masa ajustable) |
| $m_1$ | Masa total del péndulo | **$22.5$** | $\text{g}$ ($0.0225\text{ kg}$) | Masa aligerada por eliminación del tornillo y abrazadera |
| $J_0$ | Inercia rotacional del brazo 1 | **$2.20 \times 10^{-4}$** | $\text{kg}\cdot\text{m}^2$ | Momento de inercia del brazo horizontal sostenido por abajo |
| $J_1$ | Inercia rotacional del péndulo | **$3.10 \times 10^{-4}$** | $\text{kg}\cdot\text{m}^2$ | Momento de inercia respecto a su eje horizontal |

### Ecuaciones de Euler-Lagrange:

$$\tau_0 = \ddot{q}_0 \left[ J_0 + m_1 (L_0^2 + l_1^2 \sin^2 q_1) \right] + \ddot{q}_1 (m_1 L_0 l_1 \cos q_1) + m_1 l_1^2 \dot{q}_0 \dot{q}_1 \sin(2 q_1) - m_1 L_0 l_1 \dot{q}_1^2 \sin q_1$$

$$\tau_1 = \ddot{q}_0 (m_1 L_0 l_1 \cos q_1) + \ddot{q}_1 (J_1 + m_1 l_1^2) - \frac{1}{2} m_1 l_1^2 \dot{q}_0^2 \sin(2 q_1) - m_1 g l_1 \sin q_1$$

---

## 3. Guía de Impresión 3D (Slicer Settings)

| Pieza | Material | Altura de Capa | Perímetros | Relleno (Infill) | Orientación en Cama |
|---|---|---|---|---|---|
| **Base Torre (Columnas Abiertas)** | PLA+ / PETG | 0.20 mm | 4 paredes | 30% Giroide | Base plana sobre la cama (Z=0 hacia abajo). Sin soportes en columnas (puentes de 28mm). |
| **Brazo Horizontal Monolítico** | PETG / ABS | 0.16 mm | 5 paredes | 50% Giroide | Base plana inferior del brazo sobre la cama. Sin soportes. |
| **Barra Péndulo** | PETG / PLA+ | 0.12 - 0.16 mm | 4 paredes | 40% Rectilíneo | Cara lateral plana sobre la cama. |
| **Eje Portaimán** | PETG / Resina | 0.12 mm | 6 paredes o 100% | 100% Sólido | Eje vertical con la brida sobre la cama. |
| **Masa Ajustable** | PLA+ / PETG | 0.16 mm | 4 paredes | 100% Sólido | Cara circular sobre la cama. |
