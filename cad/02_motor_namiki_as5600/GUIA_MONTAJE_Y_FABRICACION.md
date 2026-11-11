# Guía Técnica de Fabricación y Ensamble: Motor Namiki 22CL con Encoder Magnético AS5600

Este proyecto incorpora el sensor rotatorio magnético **AS5600** de 12 bits (4096 pasos/vuelta) a la parte posterior del motor **Namiki 22CL-3501PG**, sustituyendo por completo la tapa y disco del encoder óptico de fábrica.

El sistema fue diseñado en **FreeCAD 1.1** bajo estándares estrictos de ingeniería mecánica y tolerancias para manufactura aditiva (impresión 3D FDM / SLA).

---

## 1. Estructura de Archivos del Proyecto

La carpeta contiene los modelos paramétricos nativos, ensambles completos y piezas listas para fabricar:

| Archivo | Formato | Descripción |
|---|---|---|
| `Motor_22CL_AS5600.FCStd` | FreeCAD 1.1 | Proyecto nativo con árbol de operaciones paramétricas, materiales y ensamblaje completo |
| `Motor_22CL_AS5600_Assembly.step` | STEP AP214 | Ensamble master interoperable para SolidWorks, Inventor, Fusion360, etc. |
| `Motor_22CL_AS5600_Assembly.stl` | STL Mesh | Malla 3D del ensamble completo para visualizadores y pruebas |
| `Acoplador_Motor_AS5600.step` | STEP AP214 | Modelo CAD individual del acoplador/estator |
| `Acoplador_Motor_AS5600.stl` | STL Mesh | **Pieza para imprimir en 3D**: Acoplador/soporte estator (alta resolución) |
| `Portaiman_AS5600.step` | STEP AP214 | Modelo CAD individual del portaimán del rotor |
| `Portaiman_AS5600.stl` | STL Mesh | **Pieza para imprimir en 3D**: Portaimán para eje de 2.0 mm (alta resolución) |

---

## 2. Características del Diseño Mecánico ("Cero Movimiento por Unión Química")

1. **Collarín Cilíndrico de Ajuste Capilar (Capillary Fit Collar)**:
   - Collar concéntrico continuo de $\varnothing 26.5\text{ mm}$ exterior y $\varnothing 22.16\text{ mm}$ interior.
   - Abraza 10.0 mm del cuerpo cilíndrico de acero ($\varnothing 22.0\text{ mm}$) del motor Namiki.
   - Holgura radial optimizada de $0.08\text{ mm}$, calculada para inducir capilaridad uniforme con adhesivo de cianoacrilato (pegante instantáneo / Super Glue).
   - Elimina la necesidad de orejetas de apriete, ranuras de compresión, tornillos o tuercas externas, reduciendo el perfil exterior a un volumen completamente simétrico y compacto que cabe holgadamente dentro de cualquier torre o bastidor.

2. **Tope Axial Interno de Precisión**:
   - Resalto escalonado a $Z = -52.00\text{ mm}$ que hace tope plano contra la tapa trasera del motor.
   - Asegura la repetibilidad absoluta de posición en el montaje axial.

3. **Cámara Libre para el Rotor y Componentes SMD**:
   - Diámetro interno de $15.0\text{ mm}$ para giro sin fricción del portaimán ($7.5\text{ mm}$ de holgura diametral).
   - Diámetro ampliado a $20.0\text{ mm}$ en la base para librar condensadores y resistencias 0603 del PCB.

4. **Alojamiento Antirrotación y Centraje del PCB AS5600**:
   - Cajera perimetral de $23.4 \times 23.4\text{ mm}$ con esquinas redondeadas ($R = 2.0\text{ mm}$) que abraza el contorno exterior del PCB.
   - 4 torretas de esquina con taladros guía de $\varnothing 2.6\text{ mm}$ para tornillos M3 $\times 8\text{ mm}$.
   - Entrehierro (Air Gap) optimizado matemáticamente a **$0.785\text{ mm}$**, dentro del rango nominal de máxima sensibilidad del sensor AS5600 ($0.5 - 1.5\text{ mm}$).

5. **Acceso Eléctrico Posterior y Ventana de Cableado**:
   - El PCB queda orientado con el chip hacia el imán y los pines de cabezal hacia el exterior posterior, permitiendo conectar cables Dupont o soldar directamente sin interferencia.

---

## 3. Parámetros Recomendados para Impresión 3D

| Parámetro | Acoplador (`Acoplador_Motor_AS5600.stl`) | Portaimán (`Portaiman_AS5600.stl`) |
|---|---|---|
| **Material sugerido** | PETG / PLA+ / ABS / Resina Tough | PETG / ABS / Resina Tough |
| **Altura de capa** | 0.16 mm (o 0.12 mm) | 0.12 mm (máxima precisión de eje) |
| **Perímetros / Paredes** | 4 paredes (mínimo) | 5 paredes o 100% relleno |
| **Relleno (Infill)** | 40% - 50% Giroide o Cuadrícula | 100% Sólido |
| **Soportes** | No requeridos (diseño auto-portante) | No requeridos |
| **Orientación en cama** | Base del PCB apoyada en la cama ($Z$ invertida) o boca abajo | Base plana del imán apoyada en cama |

---

## 4. Lista de Materiales de Ensamble (BOM)

- **1x** Motor Namiki 22CL-3501PG (sin tapa de encoder óptico original).
- **1x** Módulo sensor magnético AS5600 (placa breakout $23 \times 23\text{ mm}$).
- **1x** Imán diametral $\varnothing 4.3 - 6.0\text{ mm} \times 2.0 - 2.5\text{ mm}$ (suministrado con el módulo AS5600).
- **Pegante instantáneo** (cianoacrilato / Super Glue / Kola Loka) de viscosidad media para unión motor-acoplador.
- **4x** Tornillos M3 $\times 8\text{ mm}$ (cabeza abombada ISO 7380 o autorroscantes para plástico para fijar el PCB).
- **1x** Pieza 3D `Acoplador_Motor_AS5600.stl`.
- **1x** Pieza 3D `Portaiman_AS5600.stl`.

---

## 5. Instrucciones de Montaje Paso a Paso

1. **Retirar el encoder óptico original**: Desenroscar o retirar la tapa plástica trasera negra (`Encoder_Cap`) y el disco óptico de plástico/vidrio montado en el eje trasero. Limpiar la carcasa metálica con alcohol isopropílico para retirar polvo o grasa.
2. **Instalar el imán en el portaimán**:
   - Presionar el imán diametral en la cajera inferior del `Portaiman_AS5600`.
   - Fijar con una micro-gota de pegante instantáneo si se requiere aseguramiento adicional.
3. **Insertar el portaimán en el eje trasero**:
   - Aplicar una gota minúscula de cianoacrilato en el extremo del eje trasero de $\varnothing 2.0\text{ mm}$ y deslizar el portaimán hasta que quede a $\approx 1.0\text{ mm}$ de la cara trasera del motor (evitando que el pegante penetre al buje interno del motor).
4. **Montar el acoplador 3D en el motor**:
   - Aplicar 2 a 3 gotas de pegante instantáneo sobre la superficie cilíndrica de acero del motor o en el interior del collarín del `Acoplador_Motor_AS5600`.
   - Deslizar inmediatamente con movimiento helicoidal suave hasta que el resalto interno haga tope axial plano a $Z = -52\text{ mm}$.
   - Mantener presionado por 15 a 30 segundos. La unión queda fija, monolítica y sin vibraciones.
5. **Instalar el sensor AS5600**:
   - Colocar el PCB AS5600 dentro de la cajera rectangular del acoplador (con el chip mirando hacia el imán).
   - Atornillar los 4 tornillos M3 $\times 8\text{ mm}$ en las esquinas para fijar el PCB.
6. **Conexión eléctrica**: Conectar los cables I2C (VCC a 3.3V, GND, SCL, SDA, DIR a GND para dirección por defecto).
