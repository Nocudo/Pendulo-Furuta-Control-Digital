# Péndulo Invertido Rotacional de Furuta — Control Digital

Proyecto final de la materia **Control Digital** — **Universidad de los Llanos** (Facultad de Ciencias Básicas e Ingeniería, Villavicencio, Colombia).

Sistema completo de péndulo invertido rotacional (Furuta) con diseño mecánico paramétrico en FreeCAD 1.1, piezas optimizadas para impresión 3D sin soportes (DfAM), firmware de control determinista a `1 kHz` en STM32F446RE, gemelo digital 3D en MATLAB, simulador interactivo 3D de cinemática matricial simbólica y artículo científico IEEEtran certificado.

---

## 📂 Estructura del Repositorio

```
├── GUIA_EXPOSICION_PROYECTO.md       # ★ Guía completa de sustentación (filtros, controladores, física y Q&A)
├── README.md                         # Portada ejecutiva e inicio rápido del proyecto
│
├── articulo/                         # ★ Artículo científico IEEEtran IMRyD + Marco Teórico (30 págs., certificado 0/0/0)
│   ├── plantilla.pdf                 # Documento PDF final certificado (0 errores, 0 warnings, 0 sobreposiciones)
│   ├── plantilla.tex                 # Código fuente principal LaTeX (IEEEtran conference)
│   ├── plantilla.bib                 # Base de datos bibliográfica BibTeX (18 referencias verificadas con DOI)
│   ├── secciones/                    # Secciones modulares (01_introduccion a 06_conclusiones, matemática matricial)
│   └── figuras/                      # 11 diagramas vectoriales TikZ + 10 figuras CAD, experimentales y del simulador 3D
│
├── simulacion cinematica/            # ★ Simulador interactivo 3D de cinemática matricial simbólica (HTML5 offline)
│   └── index.html                    # Aplicación web autónoma (sin dependencias): modelo 3D + matrices Ecs. (1)–(19) + RK4
│
├── interfaz_matlab/                  # ★ Interfaz gráfica principal lista para presentación (MATLAB)
│   └── app_telemetria_sistema_furuta_matlab.m   # Dashboard maximizado, gemelo digital 3D y telemetría en vivo
│
├── firmware_stm32/                   # ★ Firmware C embebido (STM32 Nucleo-F446RE @ 180 MHz, 1 kHz)
│   ├── platformio.ini                # Configuración de compilación y flasheo (PlatformIO / HAL)
│   └── Core/
│       ├── Inc/                      # main.h, as5600.h, l298n_motor.h, pid_controller.h, caracterizacion.h, furuta_control.h
│       └── Src/                      # main.c, as5600.c, l298n_motor.c, pid_controller.c, caracterizacion.c, furuta_control.c, stm32f4xx_it.c
│
├── caracterizacion_y_validacion/     # ★ Identificación experimental real en hardware (R² = 98.51%)
│   ├── scripts/                      # Scripts reproducibles de análisis experimental en Python
│   │   ├── analizar_caracterizacion_real_motor.py
│   │   └── analizar_y_optimizar_sistema_furuta.py
│   ├── datos_experimentales/         # Registros CSV medidos en STM32 (CHAR1, CHAR2, CHAR4, Swing-Up y Balance)
│   └── resultados/                   # Gráficas PNG de validación real y reportes JSON finales
│
├── docs/                             # ★ Documentación técnica del proyecto
│   ├── ARQUITECTURA_PINES_Y_CODIGO_STM32_PID.md      # Pinout I2C, código C, controladores y filtros
│   ├── ARQUITECTURA_MONTAJE_PENDULO_FURUTA.md        # Arquitectura mecánica, modelo Euler-Lagrange y BOM
│   └── Ricardo_ToroSantaMaria_2009.pdf               # Referencia académica
│
├── cad/                              # Diseño CAD paramétrico (FreeCAD 1.1), ensamblajes, scripts y renders
│   ├── 01_pendulo_furuta/            # Ensamblaje completo del péndulo
│   ├── 02_motor_namiki_as5600/       # Motor Namiki 22CL + encoder AS5600
│   └── referencias/                  # Modelos STEP de referencia (motor, rodamiento)
│
├── IMPRESION_3D/                     # Piezas imprimibles (.stl/.step) y placas listas para Cura (SUP = 0)
│   ├── 02_Pendulo_Furuta/            # Piezas individuales del péndulo de Furuta (.stl y .step)
│   ├── Placas_Cura_Listas/           # Proyectos .3mf, perfiles .curaprofile y .gcode listos (Placas 1 a 4)
│   └── GUIA_IMPRESION_3D_CURA.md
│
└── .gitignore
```

---

## 🚀 Inicio Rápido para Presentación

> [!TIP]
> Para exponer el proyecto paso a paso (filtros, controladores, el porqué físico de cada decisión y preguntas frecuentes), consulta **[`GUIA_EXPOSICION_PROYECTO.md`](GUIA_EXPOSICION_PROYECTO.md)**.

1. **Conectar la STM32 Nucleo-F446RE** por USB (`COM13`, `115200 bps`) y energizar la fuente de `12 V` del puente H `L298N`.
2. **Abrir la interfaz gráfica en MATLAB** (inicia maximizada y auto-conecta a `COM13`):
   ```matlab
   run('interfaz_matlab/app_telemetria_sistema_furuta_matlab.m')
   ```
3. **Pulsar `CERO (0°)`** con el péndulo colgado en reposo y luego **`▶ INICIAR`** para levantamiento autónomo (`Swing-Up` en `~0.98 s`) y estabilización invertida continua (`M4 Ultra-Silk`) con auto-recuperación.

---

## 🧭 Simulador Interactivo 3D de Cinemática Matricial

Para explorar la cinemática tridimensional y el modelado lagrangiano matricial del artículo sin hardware ni licencias de software, abre directamente en cualquier navegador moderno:

- **[`simulacion cinematica/index.html`](simulacion%20cinematica/index.html)** *(100 % offline, archivo único HTML5 Canvas + CSS + JS, sin dependencias externas)*

**Características principales:**
- **4 modos operativos:** recorrido didáctico `Paso a paso (5)`, manipulación manual de coordenadas y velocidades generalizadas $(\theta, \alpha, \dot{\theta}, \dot{\alpha})$, simulación RK4 en vivo de `SWING_UP → BALANCE_PID` y equilibrio `M4 Ultra-Silk` con inyección de perturbaciones impulsionales.
- **Formulación matricial puramente simbólica (Planteamiento → Matriz → Resultado):** sincronizada directamente con las Ecs. (1)–(19) de [`articulo/plantilla.pdf`](articulo/plantilla.pdf) (`R_θ`, `R_α`, `⁰T₁`, `r_G1 = R_θ · A(α)`, Jacobianos `J_G1` / `¹J_G1`, matriz de inercia `M(q)`, Coriolis `C(q, q̇)` y gravedad `G(q)`).
- **Cámara orbital 3D y capas conmutables:** vista isométrica, superior, plano móvil fijado a $\hat{j}_1\text{--}\hat{k}_0$, frontal, cono de captura, vectores $\mathbf{r}_{G1}$ / $\mathbf{v}_{G1}$, rastro espacial y tema claro/oscuro.

---

## 🎛️ Resumen de Controladores y Filtros (`Ts = 1 ms` · `PWM = 20 kHz`)

| Bloque | Parámetros Activos en Firmware |
| :--- | :--- |
| **PD Péndulo (`pid_pend_alpha`)** | $K_p = 14.50\text{ V/rad}$, $K_i = 0.00$, $K_d = 1.50\text{ V}\cdot\text{s/rad}$ (M4 Ultra-Silk) |
| **PID Brazo (`pid_arm_theta`)** | $K_p = -1.45\text{ V/rad}$, $K_i = -0.75\text{ V/(rad}\cdot\text{s)}$, $K_d = -0.85\text{ V}\cdot\text{s/rad}$ (int. $\pm 1.6\text{ V}$, sat. $\pm 2.2\text{ V}$, sin límite angular) |
| **PID Servo 0° (`pid_motor_pos`)** | $K_p = 13.50$, $K_i = 65.00$, $K_d = 1.45$ + Clegg reset + freno activo `L298N_Brake` |
| **PID Velocidad (`pid_motor_vel`)** | $K_p = 1.42$, $K_i = 27.32$, $K_d = 0.005$, Tustin $N = 40\text{ rad/s}$ |
| **Swing-Up Resonante PD** | $K_p = 13.50$, $K_d = 1.20$, $\omega_n = 8.8484\text{ rad/s}$ (captura en $\|\alpha_{\text{err}}\| < 20^\circ$, $\|\dot{\alpha}\| < 4.8\text{ rad/s}$) |
| **Filtros de Velocidad (×2)** | Doble IIR de 1.er orden en cascada ($\tau = 6\text{ ms}$ c/u, $f_c \approx 15.8\text{ Hz}$) |
| **Filtro Dual-Notch Biquad** | Notch 1 fijo $f_0 = 11.0\text{ Hz}$ ($Q = 1.2$) + Notch 2 adaptativo $f_0 = 6.2\text{ Hz}$ ($Q = 1.4$, $-6\text{ dB}$) |
| **Slew-Rate / Saturación / L298N** | Adaptativo `0.065–0.40 V/ms`, `±6.5–8.5 V`; zona muerta continua `+9.2% / -16.0%` ($V_k = 0.65\text{ V}$) |
| **Planta Identificada ($R^2 = 98.51\%$)** | $K_{m,\text{brazo}} = 3.3720\text{ rad/(s}\cdot\text{V)}$, $\tau_m = 0.1175\text{ s}$, $\omega_n = 8.8484\text{ rad/s}$, $\zeta = 0.0649$, $K_{\text{acople}} = 0.8737$, $J_{1,\text{piv}} = 2.044 \times 10^{-4}\text{ kg}\cdot\text{m}^2$ |

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

- **Total Placas 1 a 4 (Péndulo de Furuta completo):** **`15h 09m 44s`** · **`178.13 g`** (`59.72 m`).

### Impresión rápida (G-code listo)

Si tienes una **Anycubic Kobra 2**, puedes copiar directamente los archivos `.gcode` a tu tarjeta microSD desde:

```
IMPRESION_3D/Placas_Cura_Listas/GCode/
├── Placa_1_Base_Inferior_Torre.gcode
├── Placa_2_Torre_Superior_Motor.gcode
└── Placa_4_Precision_Encoder_AS5600.gcode
```

*(Nota: la **Placa 3** (`Brazo_Horizontal_L88`, `Barra_Pendulo_Furuta`, `Tapa_Motor_Torre_Furuta`) se distribuye como proyecto preconfigurado [`Placa_3_Brazo_Barra_y_Tapa.3mf`](IMPRESION_3D/Placas_Cura_Listas/Proyectos_3MF/Placa_3_Brazo_Barra_y_Tapa.3mf) para laminar directamente en Cura.)*

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
| [`articulo/plantilla.pdf`](articulo/plantilla.pdf) | **Artículo científico IEEEtran certificado (30 págs.):** marco teórico matricial, metodología, validación experimental y discusiones |
| [`simulacion cinematica/index.html`](simulacion%20cinematica/index.html) | **Simulador interactivo 3D offline** de la cinemática y dinámica matricial simbólica acoplada al artículo |
| [`GUIA_EXPOSICION_PROYECTO.md`](GUIA_EXPOSICION_PROYECTO.md) | **Guía completa de sustentación**: funcionamiento, filtros, controladores, física y preguntas frecuentes |
| [`docs/ARQUITECTURA_PINES_Y_CODIGO_STM32_PID.md`](docs/ARQUITECTURA_PINES_Y_CODIGO_STM32_PID.md) | Arquitectura de control STM32F446RE, tabla de pines, explicación del código C/caracterización y diagramas de bloques |
| [`docs/ARQUITECTURA_MONTAJE_PENDULO_FURUTA.md`](docs/ARQUITECTURA_MONTAJE_PENDULO_FURUTA.md) | Arquitectura mecánica, modelo matemático y estrategia de control |
| [`cad/01_pendulo_furuta/GUIA_FABRICACION_Y_CONTROL_FURUTA.md`](cad/01_pendulo_furuta/GUIA_FABRICACION_Y_CONTROL_FURUTA.md) | Guía de fabricación y montaje del péndulo |
| [`cad/02_motor_namiki_as5600/GUIA_MONTAJE_Y_FABRICACION.md`](cad/02_motor_namiki_as5600/GUIA_MONTAJE_Y_FABRICACION.md) | Guía del subsistema motor + encoder |
| [`IMPRESION_3D/GUIA_IMPRESION_3D_CURA.md`](IMPRESION_3D/GUIA_IMPRESION_3D_CURA.md) | Guía completa de impresión 3D con Cura |

---

## 📄 Licencia

Proyecto académico — Universidad de los Llanos, Control Digital (Villavicencio, Colombia).
