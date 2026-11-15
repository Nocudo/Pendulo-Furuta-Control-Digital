# Péndulo Invertido Rotacional de Furuta — Control Digital

Proyecto final de la materia **Control Digital** — Universidad EAFIT, Semestre 2024-2.

Sistema completo de péndulo invertido rotacional (Furuta) con diseño mecánico paramétrico en FreeCAD 1.1, piezas optimizadas para impresión 3D sin soportes (DfAM), firmware de control y documentación de montaje.

---

## 📂 Estructura del Repositorio

```
├── cad/                              # Modelos CAD paramétricos (FreeCAD 1.1)
│   ├── 01_pendulo_furuta/            # Ensamblaje completo del péndulo
│   │   ├── Pendulo_Furuta_Completo.FCStd
│   │   ├── *.step / *.stl           # Piezas exportadas (8 partes imprimibles DfAM)
│   │   ├── scripts/                  # Scripts de construcción y renderizado
│   │   └── renders/                  # Vistas renderizadas
│   ├── 02_motor_namiki_as5600/       # Motor Namiki 22CL + encoder AS5600
│   │   ├── Motor_22CL_AS5600.FCStd
│   │   ├── scripts/
│   │   └── renders/
│   ├── 03_caja_nucleo_f446re/        # Caja para STM32 Nucleo-F446RE
│   │   └── renders/
│   ├── 04_caja_driver_l298n/         # Caja para driver L298N
│   │   ├── Caja_L298N.FCStd
│   │   ├── scripts/
│   │   └── renders/
│   ├── 05_caja_esp32/                # Caja para ESP32 (expansión opcional)
│   │   ├── Caja_ESP32_Expansion.FCStd
│   │   ├── scripts/
│   │   └── renders/
│   └── referencias/                  # Modelos STEP de referencia (motor, rodamiento, tornillos)
│
├── docs/                             # Documentación del proyecto
│   ├── ARQUITECTURA_MONTAJE_PENDULO_FURUTA.md   # Arquitectura mecánica y de control
│   └── Ricardo_ToroSantaMaria_2009.pdf          # Referencia académica
│
├── IMPRESION_3D/                     # Todo lo necesario para imprimir
│   ├── 01_Caja_STM32_Nucleo/        # STL/STEP listos para Caja Nucleo
│   ├── 02_Pendulo_Furuta/            # STL/STEP listos para el péndulo (8 piezas)
│   ├── 03_Caja_L298N/               # STL/STEP listos para Caja L298N
│   ├── 04_Caja_ESP32/               # STL/STEP listos para Caja ESP32
│   ├── Placas_Cura_Listas/          # Placas de impresión preconfiguradas (7 placas)
│   │   ├── Proyectos_3MF/           # Proyectos Cura (.3mf) con piezas posicionadas
│   │   ├── GCode/                   # G-code listo para Anycubic Kobra 2
│   │   └── Perfiles_Cura/           # Perfiles de impresión (.curaprofile)
│   └── GUIA_IMPRESION_3D_CURA.md    # Guía completa de impresión paso a paso
│
├── .gitignore
└── README.md
```

---

## 🔧 Hardware (BOM)

| Componente | Descripción |
|---|---|
| **STM32 Nucleo-F446RE** | Microcontrolador principal — control PID digital |
| **L298N** | Driver de motor DC con puente H |
| **Motor Namiki 22CL-3501PG** | Motor DC con reductor planetario |
| **AS5600** (×2) | Encoders magnéticos de posición absoluta (brazo + péndulo) |
| **Rodamiento 608ZZ** (×2) | Soporte del eje del péndulo |
| **Imanes 6×3 mm** (×2) | Para lectura magnética de los AS5600 |
| **Anycubic Kobra 2** | Impresora 3D FDM para fabricar las piezas |

---

## 🖨️ Impresión 3D

Todas las piezas fueron diseñadas bajo estrictos principios **DfAM** (Design for Additive Manufacturing) para garantizar **cero estructuras de soporte (`SUP = 0`)** y cero interferencias cinemáticas ($0.0000\text{ mm}^3$ en 23 pares de ensamble).

### Resumen de Placas de Impresión (Anycubic Kobra 2 — `220 × 220 × 250 mm`)

| Placa | Componentes Incluidos | Tiempo Estimado | Filamento | Soportes | Orientación Clave |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Placa 1** | `Base_Inferior_Torre_Furuta` | `8h 13m 51s` | `107.66 g` | **`SUP = 0`** | Base $120 \times 120\text{ mm}$ plana en cama, espiga coaxial cónica vertical |
| **Placa 2** | `Torre_Superior_Motor_Furuta` | **`2h 56m 10s`** | **`34.45 g`** | **`SUP = 0`** | **Boca abajo**: brida $52 \times 52\text{ mm}$ en cama, arcos ojivales a $37.8^\circ$ |
| **Placa 3** | `Brazo_Horizontal_L88`, `Barra_Pendulo`, `Tapa_Motor` | `2h 45m 34s` | `27.42 g` | **`SUP = 0`** | Caras planas continuas apoyadas sobre cama ($0.16\text{ mm}$) |
| **Placa 4** | `Acoplador_Motor_AS5600`, `Eje_Portaiman`, `Portaiman` | `1h 14m 09s` | `8.60 g` | **`SUP = 0`** | Orientación vertical concéntrica de alta precisión ($0.12\text{ mm}$, 100% infill) |
| **Placa 5** | `Caja_Base_Nucleo` + `Tapa_Caja_Nucleo` | `3h 13m 55s` | `42.16 g` | **`SUP = 0`** | Tapas invertidas en cama sin soportes |
| **Placa 6** | `Caja_Base_L298N` + `Tapa_Caja_L298N` | `2h 09m 23s` | `27.68 g` | **`SUP = 0`** | Base plana y tapa invertida |
| **Placa 7** *(Opcional)* | `Caja_Base_ESP32` + `Tapa_Caja_ESP32` | `2h 54m 27s` | `37.31 g` | **`SUP = 0`** | Carcasa de expansión Wi-Fi / IoT |

- **Total Placas 1 a 6 (Sistema Principal Furuta + Nucleo + L298N):** **`20h 33m 02s`** · **`247.97 g`** (`83.13 m`).
- **Total Placas 1 a 7 (Incluyendo carcasa opcional ESP32):** **`23h 27m 29s`** · **`285.28 g`** (`95.64 m`).

### Impresión rápida (G-code listo)

Si tienes una **Anycubic Kobra 2**, puedes copiar directamente los archivos `.gcode` a tu tarjeta microSD desde:

```
IMPRESION_3D/Placas_Cura_Listas/GCode/
├── Placa_1_Base_Inferior_Torre.gcode
├── Placa_2_Torre_Superior_Motor.gcode
├── Placa_3_Brazo_Barra_y_Tapa.gcode
├── Placa_4_Precision_Encoder_AS5600.gcode
├── Placa_5_Caja_Nucleo.gcode
├── Placa_6_Caja_L298N.gcode
└── Placa_7_Caja_ESP32.gcode
```

### Impresión personalizada

1. Abre los archivos `.3mf` de `IMPRESION_3D/Placas_Cura_Listas/Proyectos_3MF/` en **UltiMaker Cura 5.x**. Ya vienen pre-centrados con $Z = 0.00\text{ mm}$ (sin elevación ni partes flotantes).
2. Importa el perfil correspondiente de `Perfiles_Cura/`.
3. Consulta la guía completa: [`IMPRESION_3D/GUIA_IMPRESION_3D_CURA.md`](IMPRESION_3D/GUIA_IMPRESION_3D_CURA.md).

---

## 🛠️ Regenerar Modelos CAD

Los modelos se pueden reconstruir desde los archivos `.FCStd` con FreeCAD 1.1 y sus scripts Python.

### Requisitos

- **[FreeCAD 1.1](https://www.freecad.org/)** instalado
- Los scripts detectan automáticamente la ruta de instalación de FreeCAD

### Ejecutar scripts de construcción

```powershell
# Péndulo completo (piezas imprimibles DfAM)
& "C:\...\FreeCAD 1.1\bin\python.exe" cad/01_pendulo_furuta/scripts/build_furuta_complete.py

# Motor + Encoder
& "C:\...\FreeCAD 1.1\bin\python.exe" cad/02_motor_namiki_as5600/scripts/build_full_project.py

# Caja L298N
& "C:\...\FreeCAD 1.1\bin\python.exe" cad/04_caja_driver_l298n/scripts/build_caja_l298n.py

# Caja ESP32
& "C:\...\FreeCAD 1.1\bin\python.exe" cad/05_caja_esp32/scripts/build_caja_esp32.py
```

### Renderizar vistas

```powershell
& "C:\...\FreeCAD 1.1\bin\python.exe" cad/01_pendulo_furuta/scripts/render_furuta_all.py
& "C:\...\FreeCAD 1.1\bin\python.exe" cad/02_motor_namiki_as5600/scripts/render_models.py
```

> [!NOTE]
> Los ensamblajes completos generan STL muy grandes (>250 MB). Para exportarlos, usa el flag `--with-huge-stl` en los scripts de construcción. Estos archivos están excluidos del repositorio por `.gitignore`.

---

## 👥 Flujo de Trabajo Colaborativo

### Clonar el repositorio

```bash
git clone https://github.com/Nocudo/Pendulo-Furuta-Control-Digital.git
cd Pendulo-Furuta-Control-Digital
```

### Crear una rama para tus cambios

```bash
git checkout -b feature/nombre-de-tu-cambio
# Haz tus modificaciones...
git add .
git commit -m "Descripción de los cambios"
git push origin feature/nombre-de-tu-cambio
```

### Crear un Pull Request

1. Ve a la página del repositorio en GitHub.
2. Haz clic en **"Compare & pull request"**.
3. Describe tus cambios y solicita revisión.

### Agregar colaboradores

El dueño del repositorio puede agregar colaboradores en:
**Settings → Collaborators → Add people** → ingresar el usuario de GitHub de cada compañero.

---

## 📖 Documentación Adicional

| Documento | Descripción |
|---|---|
| [`docs/ARQUITECTURA_MONTAJE_PENDULO_FURUTA.md`](docs/ARQUITECTURA_MONTAJE_PENDULO_FURUTA.md) | Arquitectura mecánica, modelo matemático y estrategia de control |
| [`cad/01_pendulo_furuta/GUIA_FABRICACION_Y_CONTROL_FURUTA.md`](cad/01_pendulo_furuta/GUIA_FABRICACION_Y_CONTROL_FURUTA.md) | Guía de fabricación y montaje del péndulo |
| [`cad/02_motor_namiki_as5600/GUIA_MONTAJE_Y_FABRICACION.md`](cad/02_motor_namiki_as5600/GUIA_MONTAJE_Y_FABRICACION.md) | Guía del subsistema motor + encoder |
| [`cad/03_caja_nucleo_f446re/GUIA_FABRICACION_CAJA_NUCLEO.md`](cad/03_caja_nucleo_f446re/GUIA_FABRICACION_CAJA_NUCLEO.md) | Guía de la caja del Nucleo |
| [`cad/04_caja_driver_l298n/GUIA_FABRICACION_CAJA_L298N.md`](cad/04_caja_driver_l298n/GUIA_FABRICACION_CAJA_L298N.md) | Guía de la caja del L298N |
| [`cad/05_caja_esp32/GUIA_FABRICACION_CAJA_ESP32.md`](cad/05_caja_esp32/GUIA_FABRICACION_CAJA_ESP32.md) | Guía de la caja del ESP32 |
| [`IMPRESION_3D/GUIA_IMPRESION_3D_CURA.md`](IMPRESION_3D/GUIA_IMPRESION_3D_CURA.md) | Guía completa de impresión 3D con Cura |

---

## 📄 Licencia

Proyecto académico — Universidad EAFIT, Control Digital 2024-2.
