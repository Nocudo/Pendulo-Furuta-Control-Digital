# Guía Integral de Ingeniería: Péndulo Invertido Rotacional de Furuta
## Arquitectura Electromecánica, Modelado Cinemático/Dinámico, Caracterización de Planta/Motor y Control PID Digital con STM32 Nucleo-F446RE y Driver L298N

---

**Asignatura:** Control Digital — Proyecto Final de Ingeniería  
**Microcontrolador:** STMicroelectronics STM32 Nucleo-F446RE (ARM Cortex-M4 @ 180 MHz con FPU)  
**Etapa de Potencia:** Driver Puente H L298N (Control PWM + Dirección)  
**Actuador:** Motor DC con reductor planetario Namiki 22CL-3501PG  
**Sensores Angulares:** 2x AS5600 (Encoders magnéticos de 12 bits, 4096 cuentas/rev)  
**Herramientas:** FreeCAD 1.1, STM32CubeIDE, MATLAB / Simulink / Python  

---

## 1. Visión General del Sistema y Grados de Libertad

El **Péndulo Invertido Rotacional de Furuta** es un sistema dinámico mecatrónico subactuado de tipo **SIMO** (*Single-Input Multiple-Output*), altamente no lineal e inestable en lazo abierto.

```
                            Eje Z0 (Vertical)
                                   ▲
                                   │  q0 (Theta): Angulo Brazo Horizontal
                                   │  Impulsado por Motor Namiki 22CL
                                   │  Medido por AS5600 Base (I2C1)
                      [Motor 22CL] │
                         ┌─────────┴─────────┐
                         │   Torre Furuta    │
                         │  (H = 160.0 mm)   │
                         └─────────┬─────────┘
                                   │
                                   ├─── Brazo Horizontal (L0 = 88.5 mm)
                                   │    Giro 360° en plano XY
                                   │    (Despeje axial de 3.0 mm anti-roce)
                                   │
                                   ▼ Extremo Distal (X1)
                         ┌───────────────────┐
                         │   Cabezal Brazo   │───► Rodamiento 608 + AS5600 (I2C2)
                         └─────────┬─────────┘
                                   │
                                   │ q1 (Alfa): Angulo del Pendulo
                                   │ Eje pasivo (sin motor directo)
                                   ▼
                         ┌───────────────────┐
                         │  Barra Pendulo    │ L1 = 155.0 mm
                         │   con Masa M3     │ Masa ajustable l1 = 82-92 mm
                         └─────────┬─────────┘
                                   ▼ Punta Redondeada (R = 4 mm)
                         ─────────────────────  ▲
                         /////////////////////  └── Despeje de 3.0 cm al piso
                         (Mesa de Trabajo)
```

### Grados de Libertad:
1. **Ángulo del Brazo Horizontal ($\theta$ o $q_0$):** Rotación accionada alrededor del eje vertical $Z_0$.
2. **Ángulo del Péndulo ($\alpha$ o $q_1$):** Rotación pasiva alrededor del eje horizontal perpendicular al brazo.
   - $\alpha = 0^\circ$: Posición de reposo inferior (colgado estable).
   - $\alpha = 180^\circ$: Posición vertical invertida (equilibrio inestable a controlar).

---

## 2. Arquitectura Física y Mecánica del Montaje

El montaje mecánico se organiza en 4 módulos integrados desarrollados en FreeCAD 1.1:

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ MODULO 1: BASTIDOR MAESTRO (Base_Torre_Furuta)                               │
│ - Base cuadrada de 120 x 120 x 8 mm con 4 agujeros para pernos M5.           │
│ - 4 Columnas abiertas (ventanas de 28 x 81 mm) para acceso 360° al cableado. │
│ - 4 Aletas triangulares macizas de refuerzo torsional (R = 55 mm, H = 60 mm).│
│ - Placa superior de montaje a Z = 156.0 - 160.0 mm con cajera de cuello D10.2│
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       │
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│ MODULO 2: ACTUADOR Y SENSOR DE BRAZO (Motor 22CL + AS5600)                   │
│ - Motor Namiki 22CL acoplado bajo la placa superior con 2 tornillos M2.      │
│ - Eje motriz de salida D4.0 mm con plano D sobresale 7.5 mm hacia arriba.    │
│ - Acoplador trasero 3D pegado con cianoacrilato capilar (sin abrazaderas).   │
│ - Rotor portaimán a presión en eje trasero D2.0 mm + Imán diametral.         │
│ - PCB AS5600 Base montado con 4 tornillos M3 (lectura de Theta).             │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       │
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│ MODULO 3: BRAZO HORIZONTAL MONOLITICO (Brazo_Horizontal_L88)                 │
│ - Viga horizontal monolítica con longitud entre centros L0 = 88.50 mm.       │
│ - Cubo hembra en D para acople directo al eje de D4.0 mm del motor.          │
│ - Plano inferior continuo a Z = 163.0 mm (3.0 mm de holgura libre anti-roce).│
│ - Cabezal distal integrado para Rodamiento 608 (chaflán cónico a 45°).       │
│ - Hombro de tope perimetral D14.0 mm (apoya sólo la pista exterior fija).    │
│ - 2 Ranuras de expulsión traseras para desmontar el rodamiento con llave.    │
│ - Cajera a ras para PCB AS5600 Distal con canal de cables superior.          │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       │
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│ MODULO 4: ARTICULACION, PENDULO Y MASA AJUSTABLE                             │
│ - Eje Portaimán: cuello D7.96 mm para rodamiento 608, copa para imán y      │
│   espiga exterior en D de D5.0 mm.                                           │
│ - Barra del Péndulo: cubo hembra en D sin tornillos parásitos, L1 = 152.8 mm.│
│ - Masa Ajustable: cilindro D18 mm con corredera de 40 mm y tornillo M3x16.   │
│ - Despeje al piso: exactamente 3.0 cm entre la punta redondeada y la mesa.   │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Arquitectura Electrónica y Conexiones Detalladas

### 3.1. Consideraciones de Compatibilidad: STM32 Nucleo-F446RE y Driver L298N
1. **Niveles Lógicos:**
   - La STM32 opera a $3.3\text{ V}$.
   - El driver L298N utiliza lógica TTL estándar. El umbral de nivel alto es $V_{IH} \ge 2.3\text{ V}$, por lo que las señales PWM y GPIO de $3.3\text{ V}$ de la STM32 son plenamente reconocidas sin necesidad de *level-shifter*.
2. **Caída de Tensión del Puente Bipolar:**
   - El L298N está basado en transistores BJT Darlington, con una caída interna típica de $V_{drop} \approx 1.8\text{ V} - 2.5\text{ V}$.
   - Para entregar $12\text{ V}$ eficaces al motor Namiki en plena saturación, la fuente de alimentación externa debe ser de **$12\text{ V} - 14.5\text{ V}$ DC**.
3. **Masa Común (GND Único):**
   - **Regla Crítica:** La masa de la fuente de $12\text{ V}$, el pin GND del L298N y el pin GND de la STM32 deben estar unidos sólidamente para cerrar el lazo de referencia de las señales lógicas.

---

### 3.2. Diagrama Esquemático de Cableado Pin a Pin

```
                 FUENTE 12V DC EXTERNA (2A - 3A)
                 ┌─────────────────────────────┐
                 │ (+) 12V                 GND │
                 └──────┬───────────────────┬──┘
                        │                   │
                        │                   ├─────────────────────────┐
                        │                   │                         │
                        ▼ (+12V Power)      ▼ (Power GND)             │
     ┌──────────────────────────────────────────────────┐             │
     │            DRIVER PUENTE H L298N                 │             │
     │                                                  │             │
     │ [VMS/12V]  Pin de Alimentación de Potencia Motor │             │
     │ [GND]      Masa de Potencia                      │             │
     │ [5V]       Regulador interno (Jumper 5V puesto)  │             │
     │                                                  │             │
     │ [OUT1] ─────────────► Terminal (+) Motor Namiki  │             │
     │ [OUT2] ─────────────► Terminal (-) Motor Namiki  │             │
     │                                                  │             │
     │ [ENA]  ◄──────────── Señal PWM (Velocidad)       │             │
     │ [IN1]  ◄──────────── Señal Digital (Sentido A)   │             │
     │ [IN2]  ◄──────────── Señal Digital (Sentido B)   │             │
     └──────────────────────────────────────────────────┘             │
              ▲                ▲             ▲                        │
              │                │             │                        │
    TIM1_CH1  │           PB4  │         PB5 │                        │
       (PA8)  │       (GPIO_Out)│   (GPIO_Out)│                       │
              │                │             │                        │
     ┌────────┴────────────────┴─────────────┴──────────┐             │
     │       MICROCONTROLADOR STM32 NUCLEO-F446RE       │             │
     │             (ARM Cortex-M4 @ 180 MHz)            │             │
     │                                                  │             │
     │ [GND] ───────────────────────────────────────────┴─────────────┘
     │ [3.3V] ───┬──────────────────────────────────────┐
     │           │                                      │
     │ [I2C1]    │                                      │
     │  PB8 (SCL1) ──► SCL (Sensor 1 AS5600: Brazo)     │
     │  PB9 (SDA1) ──► SDA (Sensor 1 AS5600: Brazo)     │
     │  VCC ─────┘                                      │
     │  GND ────────────────────────────────────────────┤
     │                                                  │
     │ [I2C2]                                           │
     │  PB10 (SCL2) ─► SCL (Sensor 2 AS5600: Péndulo)   │
     │  PB11 (SDA2) ─► SDA (Sensor 2 AS5600: Péndulo)   │
     │  VCC ────────────────────────────────────────────┘
     │  GND ────────────────────────────────────────────┐
     │                                                  │
     │ [USART2]                                         │
     │  PA2 (TX) ────► ST-LINK USB Virtual COM Port     │
     │  PA3 (RX) ◄──── Telemetría a MATLAB/Python (1kHz)│
     └──────────────────────────────────────────────────┘
```

### Tabla de Pines STM32 Nucleo-F446RE:
| Periférico | Pin F446RE | Conexión L298N / Sensor | Tipo de Señal / Modo |
|---|---|---|---|
| **TIM1_CH1** | **PA8** | **ENA** (L298N) | PWM Salida, $20\text{ kHz}$, resolución 16 bits |
| **GPIO** | **PB4** | **IN1** (L298N) | Salida Digital (Push-Pull, High Speed) |
| **GPIO** | **PB5** | **IN2** (L298N) | Salida Digital (Push-Pull, High Speed) |
| **I2C1_SCL** | **PB8** | **SCL** (AS5600 Motor) | Open-Drain con Pull-Up externo ($4.7\text{ k}\Omega$) |
| **I2C1_SDA** | **PB9** | **SDA** (AS5600 Motor) | Open-Drain con Pull-Up externo ($4.7\text{ k}\Omega$) |
| **I2C2_SCL** | **PB10**| **SCL** (AS5600 Péndulo)| Open-Drain con Pull-Up externo ($4.7\text{ k}\Omega$) |
| **I2C2_SDA** | **PB11**| **SDA** (AS5600 Péndulo)| Open-Drain con Pull-Up externo ($4.7\text{ k}\Omega$) |
| **USART2_TX**| **PA2** | ST-LINK VCP TX | Comunicación UART asíncrona a 115200 o 921600 bps |
| **USART2_RX**| **PA3** | ST-LINK VCP RX | Comunicación UART asíncrona |
| **Alimentación**| **3.3V / GND**| VCC / GND Sensores | Riel regulado interno de la placa Nucleo |

---

## 4. Cinemática y Dinámica Formal del Péndulo de Furuta

### 4.1. Marcos de Referencia y Cinemática Directa

Definimos la convención de sistemas de coordenadas según el estándar de robótica:
* Sistema Inercial Base $\mathcal{F}_0 = \{X_0, Y_0, Z_0\}$ con origen en la intersección del eje motor con el plano de la base.
* Sistema del Brazo $\mathcal{F}_1 = \{X_1, Y_1, Z_1\}$ ubicado en el codo distal del brazo horizontal.
* Coordenadas generalizadas:
  - $\theta(t)$: rotación del brazo horizontal alrededor de $\vec{z}_0$.
  - $\alpha(t)$: rotación del péndulo alrededor del eje horizontal del codo.

```
                    Z0 ▲
                       │      Brazo Horizontal L0
                       │  o=======================o (Pivote Distal F1)
                       │ /                         \
                       │/                           \ Barra del Pendulo
                       └────────► X0                 \  l1 (Centro de Masa)
                      /                               \
                     ▼ Y0                              ▼ m1, J1
```

Vector de posición del centro de masa del péndulo ($m_1$) respecto a la base $\mathcal{F}_0$:

$$\vec{p}_1 = \begin{bmatrix} L_0 \cos\theta - l_1 \sin\alpha \sin\theta \\ L_0 \sin\theta + l_1 \sin\alpha \cos\theta \\ l_1 \cos\alpha \end{bmatrix}$$

Derivando con respecto al tiempo se obtiene el vector de velocidad lineal del centro de masa $\vec{v}_1 = \dot{\vec{p}}_1$:

$$\vec{v}_1 = \begin{bmatrix} 
-\dot{\theta} L_0 \sin\theta - l_1 \dot{\alpha} \cos\alpha \sin\theta - l_1 \dot{\theta} \sin\alpha \cos\theta \\
\dot{\theta} L_0 \cos\theta + l_1 \dot{\alpha} \cos\alpha \cos\theta - l_1 \dot{\theta} \sin\alpha \sin\theta \\
-l_1 \dot{\alpha} \sin\alpha
\end{bmatrix}$$

El cuadrado de la magnitud de la velocidad lineal es:

$$\|\vec{v}_1\|^2 = L_0^2 \dot{\theta}^2 + l_1^2 \dot{\alpha}^2 + l_1^2 \dot{\theta}^2 \sin^2\alpha + 2 L_0 l_1 \dot{\theta} \dot{\alpha} \cos\alpha$$

---

### 4.2. Formulación Dinámica por Euler-Lagrange

La función Lagrangiana es $L = K - V$:

#### Energía Cinética Total ($K$):

$$K = \underbrace{\frac{1}{2} J_0 \dot{\theta}^2}_{\text{Brazo 0}} + \underbrace{\frac{1}{2} m_1 \|\vec{v}_1\|^2 + \frac{1}{2} J_1 \dot{\alpha}^2}_{\text{Péndulo 1}}$$

Sustituyendo $\|\vec{v}_1\|^2$:

$$K = \frac{1}{2} \left[ J_0 + m_1 L_0^2 + m_1 l_1^2 \sin^2\alpha \right] \dot{\theta}^2 + \frac{1}{2} \left[ J_1 + m_1 l_1^2 \right] \dot{\alpha}^2 + m_1 L_0 l_1 \dot{\theta} \dot{\alpha} \cos\alpha$$

#### Energía Potencial Total ($V$):
Tomando como referencia de energía potencial cero el eje horizontal de la articulación:

$$V = m_1 g l_1 \cos\alpha$$

*(donde $\alpha = 0$ corresponde a la posición vertical invertida).*

Aplicando las ecuaciones de Euler-Lagrange con fuerzas disipativas viscosas ($b_0, b_1$):

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}}\right) - \frac{\partial L}{\partial \theta} = \tau_{\text{motor}} - b_0 \dot{\theta}$$

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\alpha}}\right) - \frac{\partial L}{\partial \alpha} = -b_1 \dot{\alpha}$$

#### Ecuaciones No Lineales Acopladas del Movimiento:

$$\left[ J_0 + m_1 (L_0^2 + l_1^2 \sin^2\alpha) \right] \ddot{\theta} + (m_1 L_0 l_1 \cos\alpha) \ddot{\alpha} + m_1 l_1^2 \dot{\theta} \dot{\alpha} \sin(2\alpha) - m_1 L_0 l_1 \dot{\alpha}^2 \sin\alpha + b_0 \dot{\theta} = \tau_{\text{motor}}$$

$$(m_1 L_0 l_1 \cos\alpha) \ddot{\theta} + (J_1 + m_1 l_1^2) \ddot{\alpha} - \frac{1}{2} m_1 l_1^2 \dot{\theta}^2 \sin(2\alpha) - m_1 g l_1 \sin\alpha + b_1 \dot{\alpha} = 0$$

Forma matricial canónica:

$$M(q) \begin{bmatrix} \ddot{\theta} \\ \ddot{\alpha} \end{bmatrix} + C(q, \dot{q}) \begin{bmatrix} \dot{\theta} \\ \dot{\alpha} \end{bmatrix} + G(q) = \begin{bmatrix} \tau_{\text{motor}} \\ 0 \end{bmatrix}$$

---

### 4.3. Linealización en el Punto de Equilibrio Invertido

Alrededor del punto de equilibrio vertical superior inestable ($x_e = [0, 0, 0, 0]^T$ con $\alpha \approx 0$, $\sin\alpha \approx \alpha$, $\cos\alpha \approx 1$, términos de segundo orden $\dot{\theta}^2 \approx 0$, $\dot{\alpha}^2 \approx 0$):

$$M_e = \begin{bmatrix} J_0 + m_1 L_0^2 & m_1 L_0 l_1 \\ m_1 L_0 l_1 & J_1 + m_1 l_1^2 \end{bmatrix} = \begin{bmatrix} \hat{J}_0 & M_{01} \\ M_{01} & \hat{J}_1 \end{bmatrix}$$

$$G_e = \begin{bmatrix} 0 \\ -m_1 g l_1 \alpha \end{bmatrix}, \quad D_e = \begin{bmatrix} b_0 & 0 \\ 0 & b_1 \end{bmatrix}$$

El determinante de la matriz de inercia es $\Delta = \hat{J}_0 \hat{J}_1 - M_{01}^2 > 0$.

Invirtiendo $M_e$, obtenemos el modelo en variables de estado continuas $\dot{x} = A x + B u$:

$$x = \begin{bmatrix} \theta \\ \alpha \\ \dot{\theta} \\ \dot{\alpha} \end{bmatrix}, \quad u = \tau_{\text{motor}}$$

$$A = \begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & \frac{M_{01} m_1 g l_1}{\Delta} & -\frac{\hat{J}_1 b_0}{\Delta} & \frac{M_{01} b_1}{\Delta} \\
0 & \frac{\hat{J}_0 m_1 g l_1}{\Delta} & \frac{M_{01} b_0}{\Delta} & -\frac{\hat{J}_0 b_1}{\Delta}
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 \\ 0 \\ \frac{\hat{J}_1}{\Delta} \\ -\frac{M_{01}}{\Delta}
\end{bmatrix}$$

---

## 5. Metodología de Caracterización Experimental e Identificación de Sistemas

> [!TIP]
> **Enfoque Práctico sin Instrumentos de Laboratorio (Black-Box System Identification):**  
> Si no cuentas con banco dinamométrico, tacómetro óptico, freno de rotor bloqueado u osciloscopio con punta de corriente, **el método estándar y más preciso en ingeniería de control es la Identificación Experimental de Sistemas Entrada-Salida**.  
> Solo necesitas el montaje que ya posees: la **STM32 Nucleo-F446RE**, el driver **L298N**, el motor con su **sensor AS5600** y tu computador con **MATLAB**.

```
    STM32 NUCLEO-F446RE                     PC CON MATLAB
 ┌────────────────────────┐              ┌────────────────────────┐
 │ Inyecta escalones PWM  │              │ Recibe CSV por UART    │
 │ (e.g. 25%, 40%, 60%)   │ ──UART TX──► │ (t, PWM, Theta, Omega) │
 │ Lee encoder AS5600     │  (115200bps) └───────────┬────────────┘
 └────────────────────────┘                          │
                                                     ▼
                                         ┌────────────────────────┐
                                         │ IDENTIFICACION SISTEMA │
                                         │ tfest / fit / Grafico  │
                                         │ G(s) = K / (tau*s + 1) │
                                         └───────────┬────────────┘
                                                     │
                                                     ▼
                                         ┌────────────────────────┐
                                         │ SINTONIA PID EN MATLAB │
                                         │ pidTuner(G) -> Kp,Ki,Kd│
                                         └────────────────────────┘
```

---

### 5.1. Método Principal: Identificación de la Función de Transferencia con MATLAB (Entrada-Salida)

Un motor de corriente continua acoplado a una carga inercial (el brazo) gobernado por tensión/PWM presenta una dinámica dominante de **primer orden en velocidad** y de **segundo orden integrador en posición**:

#### Modelo en Velocidad Angular ($\omega = \dot{\theta}$):
$$G_v(s) = \frac{\Omega(s)}{U_{\text{PWM}}(s)} = \frac{K}{\tau s + 1} e^{-\theta_d s}$$
* $K$: Ganancia estática del sistema $[(\text{rad/s}) / \%\text{PWM}]$ o $[(\text{rad/s}) / \text{V}]$.
* $\tau$: Constante de tiempo electromecánica combinada (típicamente $50\text{ ms} - 150\text{ ms}$).
* $\theta_d$: Retardo de transporte o tiempo muerto debido a la zona muerta del puente H y retraso de filtrado.

#### Modelo en Posición Angular ($\theta$):
$$G_p(s) = \frac{\Theta(s)}{U_{\text{PWM}}(s)} = \frac{1}{s} G_v(s) = \frac{K}{s(\tau s + 1)}$$

---

#### Procedimiento de Adquisición de Datos Paso a Paso:

1. **Determinación de la Zona Muerta (*Dead-Zone*):**
   - El puente H L298N tiene una caída interna de $\approx 2\text{ V}$ y el motor tiene fricción estática (*stiction*).
   - Cargar un programa sencillo en la STM32 que incremente el PWM lentamente de $0\%$ a $100\%$ en pasos de $1\%$ cada $200\text{ ms}$.
   - Detectar a qué valor exacto de PWM el motor comienza a girar (típicamente $PWM_{\text{mín}} \approx 12\% - 18\%$).
   - Este valor $PWM_{\text{deadzone}}$ se utilizará para compensar la zona muerta en la ley de control:
     $$PWM_{\text{efectivo}} = \operatorname{sign}(u) \cdot \left( PWM_{\text{deadzone}} + \frac{100 - PWM_{\text{deadzone}}}{100} |u| \right)$$

2. **Inyección de Escalones de PWM (Lazo Abierto):**
   - Para garantizar linealidad, excitar el motor en una zona operativa media, por ejemplo:
     - Estado inicial: Reposo ($0\%$ PWM).
     - Al tiempo $t = 1.0\text{ s}$: Escalón de entrada $u(t) = +30\%$ PWM constante durante $3.0\text{ s}$.
     - Al tiempo $t = 4.0\text{ s}$: Retorno a $0\%$ PWM.
     - Repetir con escalones de $+45\%$, $+60\%$, $-30\%$ y $-45\%$ para promediar efectos bidireccionales.

3. **Transmisión de Datos por UART a la PC:**
   - Enviar por el puerto serie `USART2` (ST-LINK Virtual COM a 115200 bps) en formato CSV cada período de muestreo fijo ($T_s = 5\text{ ms}$ o $10\text{ ms}$):
     ```
     tiempo_ms, pwm_input, theta_rad, omega_rad_s
     ```

---

#### Script de MATLAB para Identificación Automática y Sintonización PID:

Guarda este script en MATLAB como `identificacion_y_pid_furuta.m`:

```matlab
%% IDENTIFICACION EXPERIMENTAL DEL MOTOR Y SINTONIA PID
clear; clc; close all;

%% 1. Cargar datos experimentales de la STM32
% El archivo 'ensayo_escalon.csv' contiene: [t, u_pwm, theta, omega]
data = readmatrix('ensayo_escalon.csv');

t = data(:, 1) / 1000.0; % Convertir ms a segundos
u = data(:, 2);          % Entrada PWM (%)
theta = data(:, 3);      % Posicion angular (rad)
omega = data(:, 4);      % Velocidad angular (rad/s)

% Periodo de muestreo promedio
Ts = mean(diff(t));

figure('Name', 'Datos Experimentales');
subplot(2,1,1);
plot(t, u, 'b', 'LineWidth', 1.5); grid on;
ylabel('Entrada PWM (%)'); title('Respuesta al Escalón Experimental');
subplot(2,1,2);
plot(t, omega, 'r', 'LineWidth', 1.5); grid on;
ylabel('Velocidad \omega (rad/s)'); xlabel('Tiempo (s)');

%% 2. Identificación de la Función de Transferencia Gv(s) = Omega(s) / U(s)
% Recortar la ventana del escalón (ajustar índices según el ensayo)
idx_inicio = find(u > 5, 1, 'first');
t_step = t(idx_inicio:end) - t(idx_inicio);
u_step = u(idx_inicio:end);
omega_step = omega(idx_inicio:end);

% METODO A: Si tienes System Identification Toolbox:
try
    datos_id = iddata(omega_step, u_step, Ts);
    modelo_tf = tfest(datos_id, 1, 0); % 1 polo, 0 ceros
    [num, den] = tfdata(modelo_tf, 'v');
    G_v = tf(num, den);
    fprintf('Identificación Exitosa con tfest:\n');
    disp(G_v);
catch
    % METODO B: Estimación Analítica por el Método del 63.2% (Sin Toolbox)
    delta_u = max(u_step) - min(u_step);
    omega_ss = mean(omega_step(end-50:end)); % Valor en estado estacionario
    K_est = omega_ss / delta_u;
    
    % Buscar tiempo al 63.2% de omega_ss
    val_63 = 0.632 * omega_ss;
    idx_63 = find(omega_step >= val_63, 1, 'first');
    tau_est = t_step(idx_63);
    
    G_v = tf(K_est, [tau_est, 1]);
    fprintf('Identificación por Método del 63.2%%:\n');
    fprintf('Ganancia K = %.4f (rad/s)/%%\n', K_est);
    fprintf('Constante de tiempo tau = %.4f s\n', tau_est);
end

% Función de transferencia en posición: Gp(s) = Gv(s) / s
G_p = G_v * tf(1, [1 0]);

%% 3. Validación del Modelo vs Datos Reales
figure('Name', 'Validación del Modelo');
lsim(G_v, u_step, t_step);
hold on;
plot(t_step, omega_step, 'r--', 'LineWidth', 1.2);
legend('Modelo Identificado', 'Datos Reales AS5600');
grid on; title('Comparación de Ajuste (Fit)');

%% 4. Sintonía Automática del Controlador PID con pidTuner
% Abrir la interfaz interactiva de sintonía en MATLAB:
fprintf('\nAbriendo pidTuner para sintonizar el lazo...\n');
pidTuner(G_v, 'PID');

% O sintonía automática analítica por colocación de polos óptima:
% pid_disenado = pidtune(G_v, 'PID', pidtuneOptions('PhaseMargin', 60));
% Kp = pid_disenado.Kp;
% Ki = pid_disenado.Ki;
% Kd = pid_disenado.Kd;
% N  = pid_disenado.N;
```

---

### 5.2. Identificación Caja Negra (Black-Box System Identification) de la Barra del Péndulo

> [!NOTE]
> **El Reto de Identificar un Sistema Inestable en Lazo Abierto:**  
> En la posición invertida superior ($\alpha = 180^\circ$), el péndulo no puede excitarse con escalones en lazo abierto porque cae de inmediato.  
> Sin embargo, la física del cuerpo rígido garantiza que la **inercia, masa, fricción y geometría son idénticas en ambas posiciones**. Por lo tanto, la identificación por caja negra se realiza con el péndulo **colgado hacia abajo ($\alpha = 0^\circ$), donde el sistema es 100% estable, seguro y repetible**, y luego se mapea analíticamente al punto de equilibrio invertido con una simple inversión de signo en el término gravitatorio.

```
                 ENSAYO DE CAJA NEGRA 1:                   ENSAYO DE CAJA NEGRA 2:
                 RESPUESTA LIBRE (IMPULSO)                 ACOPLAMIENTO PWM -> ALFA
              ┌─────────────────────────────┐           ┌─────────────────────────────┐
              │ Brazo inmóvil, soltar barra │           │ Inyectar pulso/chirp PWM    │
              │ a alpha = 15°. Capturar     │           │ Brazo acelera -> pendulo se │
              │ oscilaciones libres.        │           │ balancea por inercia base.  │
              └──────────────┬──────────────┘           └──────────────┬──────────────┘
                             │                                         │
                             ▼                                         ▼
              ┌─────────────────────────────┐           ┌─────────────────────────────┐
              │ tfest en MATLAB:            │           │ tfest entrada-salida:       │
              │ G_down(s) = wn^2 /          │           │ G_alpha_u(s) = A(s) / U(s)  │
              │             (s^2 + 2*z*wn*s │           │ (3 polos, 1 cero)           │
              │              + wn^2)        │           └──────────────┬──────────────┘
              └──────────────┬──────────────┘                          │
                             │                                         │
                             └────────────────────┬────────────────────┘
                                                  │
                                                  ▼
                                 ┌─────────────────────────────────┐
                                 │ MAPEO A POSICIÓN INVERTIDA:     │
                                 │ Invertir signo del polo:        │
                                 │ s^2 + 2*z*wn*s - wn^2           │
                                 │ (Polo inestable en +wn)         │
                                 └────────────────┬────────────────┘
                                                  │
                                                  ▼
                                 ┌─────────────────────────────────┐
                                 │ SINTONIA PID EN MATLAB:         │
                                 │ pidTuner(G_up, 'PID')           │
                                 │ Genera Kp, Ki, Kd estabilizador │
                                 └─────────────────────────────────┘
```

---

#### Procedimiento de los Dos Ensayos de Caja Negra para el Péndulo:

#### Ensayo A: Identificación de la Frecuencia Natural ($\omega_n$) y Amortiguamiento ($\zeta$) por Respuesta Libre
1. Mantener el motor apagado y el brazo horizontal quieto (o sujeto fijamente).
2. Con la mano, inclinar el péndulo unos $15^\circ - 20^\circ$ respecto a la vertical inferior y soltarlo libremente.
3. La STM32 captura el vector temporal de oscilaciones libres $\alpha(t)$ a través del sensor AS5600 distal ($T_s = 5\text{ ms}$) y lo envía por UART.
4. En MATLAB, ajustar directamente el modelo de segundo orden subamortiguado:

```matlab
%% ENSAYO A: IDENTIFICACION CAJA NEGRA DEL PENDULO (RESPUESTA LIBRE)
data_pend = readmatrix('ensayo_oscilacion_libre.csv');
t_osc = data_pend(:, 1) / 1000.0; % Tiempo en segundos
alpha_osc = data_pend(:, 2);      % Angulo en radianes
Ts = mean(diff(t_osc));

% Recortar desde el momento en que se suelta (primer pico maximo)
[val_max, idx_pico1] = max(alpha_osc);
t_free = t_osc(idx_pico1:end) - t_osc(idx_pico1);
alpha_free = alpha_osc(idx_pico1:end);

% Identificación automática con System Identification Toolbox:
datos_id_pend = iddata(alpha_free, [], Ts);
% Modelo con 2 polos complejos y 0 ceros:
modelo_pend = tfest(datos_id_pend, 2, 0);
[num_p, den_p] = tfdata(modelo_pend, 'v');

% Extraer frecuencia natural (wn) y amortiguamiento (zeta):
% El denominador es: s^2 + 2*zeta*wn*s + wn^2
wn = sqrt(den_p(3));
zeta = den_p(2) / (2 * wn);

fprintf('=== PARAMETROS IDENTIFICADOS POR CAJA NEGRA ===\n');
fprintf('Frecuencia natural wn: %.3f rad/s (Periodo T = %.3f s)\n', wn, 2*pi/wn);
fprintf('Factor de amortiguamiento zeta: %.5f\n', zeta);

% Función de transferencia colgado (abajo):
G_pend_down = tf(wn^2, [1, 2*zeta*wn, wn^2]);
```

---

#### Ensayo B: Identificación Caja Negra de Acoplamiento Completo (Entrada PWM $\to$ Salida Ángulo $\alpha$)
1. Con el péndulo colgado en reposo estable ($\alpha = 0$), aplicar desde la STM32 un **pulso breve o tren de pulsos de PWM** al motor (por ejemplo, $+35\%$ durante $150\text{ ms}$, luego $-35\%$ durante $150\text{ ms}$ y finalmente $0\%$).
2. El brazo se moverá rápidamente, acelerando el codo horizontal e induciendo una oscilación forzada en el péndulo.
3. Registrar simultáneamente:
   - Entrada: $u(t) = \text{PWM}(\%)$
   - Salida: $\alpha(t) = \text{Ángulo del péndulo (rad)}$
4. En MATLAB, ajustar la función de transferencia de entrada a salida:

```matlab
%% ENSAYO B: IDENTIFICACION DEL ACOPLAMIENTO COMPLETO u_pwm -> alpha
data_cpl = readmatrix('ensayo_acoplamiento_pwm_alfa.csv');
t_cpl = data_cpl(:, 1) / 1000.0;
pwm_in = data_cpl(:, 2);
alpha_out = data_cpl(:, 3);

datos_cpl = iddata(alpha_out, pwm_in, Ts);

% La planta acoplada física tiene 3 polos (1 del motor + 2 del péndulo) y 1 cero derivador:
G_cpl_down = tfest(datos_cpl, 3, 1);
fprintf('\nFunción de transferencia directa PWM -> Alfa (Colgado):\n');
disp(G_cpl_down);
```

---

#### Mapeo a la Posición Invertida ($\alpha = 180^\circ$) y Sintonía PID en MATLAB:

En la posición invertida superior, el par gravitacional deja de ser restaurador y se vuelve desestabilizador. Matemáticamente, el término $+\omega_n^2$ del denominador cambia de signo a **$-\omega_n^2$**:

```matlab
%% 3. Mapeo a la Posición Invertida y Sintonía PID Estabilizador
% Denominador invertido: s^2 + 2*zeta*wn*s - wn^2
% (Genera un polo inestable en +wn en el semiplano derecho)
den_up = [1, 2*zeta*wn, -wn^2];

% Ganancia de acoplamiento estimada del ensayo B:
K_acople = wn^2; % normalizado o extraído de G_cpl_down
G_pend_up = tf([K_acople, 0], den_up); % Incluye cero derivativo por aceleración de base

fprintf('Función de Transferencia del Péndulo Invertido:\n');
disp(G_pend_up);

% 4. Sintonía del Controlador PID con pidTuner:
% pidTuner estabiliza plantas inestables calculando los ceros del PID
% que jalan el polo inestable hacia el semiplano izquierdo:
fprintf('\nAbriendo pidTuner para sintonizar el PID sobre la planta inestable...\n');
pidTuner(G_pend_up, 'PID');
```

Al mover los deslizadores de *pidTuner*, MATLAB te arrojará los valores óptimos de $K_p$, $K_i$, $K_d$ y el filtro derivativo $N$, cerrando el lazo con estabilidad comprobada.

---

## 6. Diseño e Implementación del Controlador

### 6.1. Estrategia de Control Híbrido (Arquitectura de Dos Capas)

Dado que los actuadores lineales no pueden levantar el péndulo desde la posición colgada ($\alpha = 0^\circ$) hasta la invertida ($\alpha = 180^\circ$) debido a la saturación y alineación del par gravitatorio, se implementa una arquitectura híbrida:

```
                            [ ESTADO DEL SISTEMA ]
                                       │
                     ┌─────────────────┴─────────────────┐
                     │ Condicion de conmutacion lineal:  │
                     │  |alpha - 180°| < 15° Y           │
                     │  |d(alpha)/dt|  < 2.5 rad/s       │
                     └─────────────────┬─────────────────┘
                                       │
                      SI ──────────────┴────────────── NO
                      │                                │
                      ▼                                ▼
       ┌───────────────────────────────┐ ┌───────────────────────────────┐
       │ CONTROL EN LINEA: LQR / PID   │ │ CONTROL NO LINEAL: SWING-UP   │
       │ - Estabiliza alpha en 180°    │ │ - Bombeo de energia Lyapunov  │
       │ - Regula posicion brazo theta │ │ - Aumenta amplitud balanceo   │
       │ - Bajo consumo energetico     │ │ - Lleva el pendulo hacia arriba│
       └───────────────────────────────┘ └───────────────────────────────┘
```

#### A. Algoritmo de Levantamiento por Energía (Energy-Based Swing-Up):
La energía mecánica total del péndulo es la suma de cinética y potencial:

$$E(\alpha, \dot{\alpha}) = \frac{1}{2} J_{\text{pivote}} \dot{\alpha}^2 + m_1 g l_1 (\cos\alpha - 1)$$

En el punto invertido deseado ($\alpha = 180^\circ, \dot{\alpha} = 0$), $E_{\text{ref}} = 0$.  
La ley de control no lineal que bombea energía según el método de Lyapunov es:

$$a_{\text{brazo}} = k_E \left( E - E_{\text{ref}} \right) \operatorname{sign}(\dot{\alpha} \cos\alpha)$$

Donde $a_{\text{brazo}} = \ddot{\theta} L_0$ es la aceleración tangencial aplicada por el motor.

---

### 6.2. Controlador PID Digital con Filtro Derivativo y Anti-Windup

Para la zona de estabilización lineal, se implementa un lazo dual o un PID en lazo cerrado para la variable crítica $\alpha(t)$ (ángulo del péndulo):

#### Ecuación Continua del PID con Filtro de Primer Orden:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de_f(t)}{dt}$$

Filtro pasabajos de la derivada con polo en $N$ (típicamente $N \in [8, 20]$):

$$D(s) = \frac{K_d N s}{s + N} E(s)$$

#### Discretización por Tustin / Euler hacia atrás ($T_s = 1.0\text{ ms} \implies 1\text{ kHz}$):

1. **Acción Proporcional:**
   $$P[k] = K_p \cdot e[k]$$

2. **Acción Integral con Anti-Windup por Clamping:**
   $$\tilde{I}[k] = I[k-1] + K_i \cdot T_s \cdot e[k]$$
   Si la salida satura ($u > u_{\max}$ o $u < u_{\min}$) y el signo de $e[k]$ incrementa la saturación, congelar $I[k] = I[k-1]$.

3. **Acción Derivativa Filtrada (Bilineal / Euler):**
   $$D[k] = \frac{K_d \cdot N}{1 + N \cdot T_s} (e[k] - e[k-1]) + \frac{1}{1 + N \cdot T_s} D[k-1]$$

4. **Salida Total:**
   $$u[k] = \operatorname{sat}(P[k] + I[k] + D[k], u_{\min}, u_{\max})$$

---

### 6.3. Código C de Producción para la ISR de Control en STM32 Nucleo-F446RE

Este bloque de código se ejecuta dentro de la rutina de interrupción del temporizador a **$1\text{ kHz}$** (`TIM6_DAC_IRQHandler` o `TIM7`):

```c
/**
 * @file furuta_controller.c
 * @brief Lazo cerrado de control digital a 1 kHz en STM32 Nucleo-F446RE
 */

#include "main.h"
#include <math.h>

/* Definiciones de constantes de hardware */
#define TS_CONTROL        0.001f    // Periodo de muestreo: 1 ms (1000 Hz)
#define PWM_MAX_DUTY      900       // Periodo del Timer PWM (ARR = 900 -> 20 kHz a 180 MHz / 10)
#define MAX_VOLTAGE       12.0f
#define ALPHA_TARGET      3.14159265f // 180 grados en radianes (Invertido)

/* Estructura del controlador PID */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float N_filter;      // Coeficiente del filtro derivativo (e.g. 15.0)
    float u_max;         // Limite superior de saturacion (+12V)
    float u_min;         // Limite inferior de saturacion (-12V)
    float integral;
    float prev_error;
    float prev_derivative;
} PID_Controller;

/* Instancias globales */
PID_Controller pid_alpha = {
    .Kp = 45.0f,
    .Ki = 8.0f,
    .Kd = 3.2f,
    .N_filter = 15.0f,
    .u_max = 12.0f,
    .u_min = -12.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .prev_derivative = 0.0f
};

extern TIM_HandleTypeDef htim1; // Timer de PWM (PA8)

/**
 * @brief Aplica tension y sentido de giro al motor mediante el driver L298N
 * @param voltage Tension deseada entre -12V y +12V
 */
void Motor_SetVoltage(float voltage) {
    // 1. Saturacion de seguridad
    if (voltage > MAX_VOLTAGE)  voltage = MAX_VOLTAGE;
    if (voltage < -MAX_VOLTAGE) voltage = -MAX_VOLTAGE;

    // 2. Calculo de Ciclo de Trabajo PWM (0 a PWM_MAX_DUTY)
    uint32_t duty = (uint32_t)((fabsf(voltage) / MAX_VOLTAGE) * PWM_MAX_DUTY);

    // 3. Control de Direccion (IN1: PB4, IN2: PB5)
    if (voltage > 0.15f) {
        // Giro Horario
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    } else if (voltage < -0.15f) {
        // Giro Antihorario
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        // Zona muerta / Freno activo
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        duty = 0;
    }

    // 4. Actualizar registro CCR1 del Timer 1 (PA8 -> ENA)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

/**
 * @brief Computo del algoritmo PID con Anti-Windup y Derivada Filtrada
 */
float PID_Compute(PID_Controller *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    // Proporcional
    float P = pid->Kp * error;

    // Derivativa filtrada (Tustin / Euler)
    float alpha_filt = 1.0f / (1.0f + pid->N_filter * TS_CONTROL);
    float D = alpha_filt * pid->prev_derivative + 
              (pid->Kd * pid->N_filter * alpha_filt) * (error - pid->prev_error);
    pid->prev_derivative = D;
    pid->prev_error = error;

    // Calculo tentativo sin integrar
    float u_unsat = P + pid->integral + D;

    // Integral con Anti-Windup por Clamping
    if (u_unsat < pid->u_max && u_unsat > pid->u_min) {
        pid->integral += pid->Ki * TS_CONTROL * error;
    }

    // Salida final saturada
    float u_out = P + pid->integral + D;
    if (u_out > pid->u_max) u_out = pid->u_max;
    if (u_out < pid->u_min) u_out = pid->u_min;

    return u_out;
}

/**
 * @brief Rutina de Interrupcion del Timer de Control (Ejecutada a 1000 Hz)
 */
void ControlLoop_ISR(void) {
    // 1. Leer Sensores I2C AS5600
    float theta = AS5600_ReadAngle_I2C1();  // Angulo Brazo en radianes
    float alpha = AS5600_ReadAngle_I2C2();  // Angulo Pendulo en radianes

    // 2. Comprobacion de Seguridad (Watchdog de Angulo)
    if (fabsf(alpha - ALPHA_TARGET) > 0.45f) { // Mas de ~25 grados de error
        Motor_SetVoltage(0.0f);                 // Desconectar motor
        pid_alpha.integral = 0.0f;              // Resetear acumulador
        return;
    }

    // 3. Ejecutar algoritmo PID
    float u_volts = PID_Compute(&pid_alpha, ALPHA_TARGET, alpha);

    // 4. Escribir salida fisica al actuador
    Motor_SetVoltage(u_volts);
}
```

---

## 7. Paso a Paso para Desarrollar el Proyecto

```
FASE 1: FABRICACION     FASE 2: MONTAJE Y      FASE 3: FIRMWARE Y     FASE 4: ENSAYOS
E IMPRESION 3D          ELECTRONICA            PERIFERICOS STM32      Y MODELADO
 ┌──────────────┐        ┌──────────────┐       ┌──────────────┐       ┌──────────────┐
 │ • 9 piezas   │  ───►  │ • L298N+STM32│ ───►  │ • PWM 20kHz  │ ───►  │ • Ra, La, Kt │
 │ • FDM / PETG │        │ • I2C1 + I2C2│       │ • I2C polling│       │ • J1, l1, b1 │
 │ • Rodam. 608 │        │ • Masa comun │       │ • UART DMA   │       │ • Simulink   │
 └──────────────┘        └──────────────┘       └──────────────┘       └──────────────┘
                                                                               │
                                                                               ▼
                                         FASE 6: SINTONIA FINAL       FASE 5: DISENO
                                         Y PRUEBAS EN VIVO            DE CONTROL
                                          ┌──────────────┐             ┌──────────────┐
                                          │ • Watchdog   │  ◄────────  │ • Swing-up   │
                                          │ • 1 kHz loop │             │ • PID Dual   │
                                          │ • Video/Demo │             │ • Espacio est│
                                          └──────────────┘             └──────────────┘
```

### Fase 1: Fabricación Mecánica e Impresión 3D
1. Imprimir las 9 piezas STL del proyecto (empaquetadas en `IMPRESION_3D/`):
   - `Base_Torre_Furuta.stl`: PETG/PLA+, relleno 25% Giroide, 4 perímetros.
   - `Brazo_Horizontal_L88.stl`: PETG/ABS, relleno 50% Giroide, 5 perímetros.
   - `Barra_Pendulo_Furuta.stl`: PETG, 40% Rectilíneo orientado a lo largo del eje.
   - `Eje_Portaiman_Pendulo.stl` y `Portaiman_AS5600.stl`: 100% sólido.
   - `Caja_Base_Nucleo.stl` y `Tapa_Caja_Nucleo.stl`: Chasis ventilado para la STM32.
2. Limpiar rebabas y verificar tolerancias con pie de rey.

### Fase 2: Montaje Físico y Cableado Eléctrico
1. Adherir con cianoacrilato capilar el `Acoplador_Motor_AS5600` al motor Namiki.
2. Insertar a presión el portaimán trasero con su imán diametral en el eje de $2\text{ mm}$ y montar el sensor AS5600 base.
3. Fijar el motor en el intradós de la torre mediante 2 tornillos M2 $\times 5\text{ mm}$.
4. Insertar el rodamiento 608 en el cabezal distal del brazo mediante suave presión en el chaflán cónico a $45^\circ$.
5. Montar el eje portaimán, el imán y el sensor AS5600 distal.
6. Encajar el brazo sobre el eje de salida del motor y conectar la barra del péndulo.
7. Conectar el driver L298N a la fuente externa de $12\text{ V}$, enlazar la masa común con la Nucleo-F446RE y cablear los pines PA8 (PWM), PB4 (IN1), PB5 (IN2) y los dos buses I2C.

### Fase 3: Configuración de Periféricos en STM32CubeIDE
1. Crear proyecto para `NUCLEO-F446RE` en STM32CubeIDE.
2. **RCC:** Configurar reloj a la máxima frecuencia del sistema: **180 MHz** mediante PLL.
3. **TIM1 (PWM Motor):**
   - Canal 1 en modo `PWM Generation CH1` (Pin PA8).
   - Prescaler = $9$ (Reloj de conteo = $180\text{ MHz} / 10 = 18\text{ MHz}$).
   - Counter Period (ARR) = $899$.
   - Frecuencia resultante: $18\text{ MHz} / 900 = \mathbf{20.0\text{ kHz}}$ (infrasónica para oído humano).
4. **GPIO:** Configurar PB4 y PB5 como `GPIO_Output` en modo `Very High Speed`.
5. **I2C1 e I2C2:**
   - I2C1 en PB8 (SCL) y PB9 (SDA) a Fast Mode ($400\text{ kHz}$).
   - I2C2 en PB10 (SCL) y PB11 (SDA) a Fast Mode ($400\text{ kHz}$).
6. **TIM6 / TIM7 (Timer Base de Tiempo):**
   - Configurar interrupción periódica exacta cada $1.0\text{ ms}$ ($1000\text{ Hz}$).
7. **USART2:** Habilitar a 115200 bps o 921600 bps para envío de datos de telemetría por ST-LINK.

### Fase 4: Ensayos de Caracterización Experimental
1. Ejecutar el protocolo de rotor bloqueado para determinar $R_a$ y $L_a$.
2. Realizar ensayo en modo dinamo con tacómetro para extraer $K_e$ y $K_t$.
3. Pesar la barra y medir el centro de gravedad $l_1$ por balance de filo.
4. Grabar oscilaciones libres del péndulo con el AS5600 para calcular $J_1$ por período $T$ y $b_1$ por decremento logarítmico $\delta$.
5. Actualizar los parámetros en el script de simulación de MATLAB/Python.

### Fase 5: Simulación y Sintonía Preliminar
1. Construir el diagrama de bloques en Simulink o el integrador no lineal en Python con `scipy.integrate.solve_ivp`.
2. Incluir la no-linealidad del actuador: saturación de voltaje ($\pm 12\text{ V}$) y zona muerta del puente H ($\approx \pm 0.5\text{ V}$).
3. Sintonizar las ganancias del PID ($K_p, K_i, K_d, N$) buscando:
   - Margen de fase $\text{PM} \ge 45^\circ$.
   - Tiempo de establecimiento $t_s \le 0.8\text{ s}$.
   - Sobreimpulso $M_p \le 10\%$.

### Fase 6: Puesta en Marcha en Hardware Real y Validación
1. **Calibración de Cero (Homing):**
   - Con el péndulo colgado inmóvil, registrar el valor del AS5600 ($\alpha_{\text{down}}$).
   - El ángulo vertical invertido objetivo es exactamente $\alpha_{\text{down}} + 2048$ cuentas ($180^\circ$).
2. **Prueba de Lazo Abierto:**
   - Enviar pulsos breves de PWM ($10\%, 20\%$) y verificar que el sentido de giro físico coincida con la polaridad positiva esperada del encoder $\theta$. Si no coincide, invertir los cables del motor en OUT1/OUT2.
3. **Prueba de Lazo Cerrado:**
   - Posicionar manualmente el péndulo cerca de la vertical ($\pm 10^\circ$).
   - Activar el lazo de control PID a $1\text{ kHz}$.
   - Ajustar finamente $K_d$ para amortiguar oscilaciones rápidas y luego $K_p$ para rigidez posicional.
4. **Verificación de la Parada de Emergencia (Watchdog):**
   - Provocar una perturbación mayor a $25^\circ$; certificar que el motor se desconecta inmediatamente sin giros descontrolados.

---

## 8. Lista Consolidada de Materiales (BOM)

### Componentes Electrónicos:
* **1x** Tarjeta STMicroelectronics STM32 Nucleo-F446RE (ARM Cortex-M4 con FPU).
* **1x** Módulo Driver Puente H Dual L298N con disipador térmico de aluminio.
* **1x** Motor DC con reductor planetario Namiki 22CL-3501PG.
* **2x** Módulos sensores magnéticos absolutos AS5600 (placas breakout de $23 \times 23\text{ mm}$).
* **2x** Imanes cilíndricos de neodimio de magnetización diametral ($\varnothing 4.0\text{ mm} \times 2.0\text{ mm}$).
* **1x** Fuente de poder conmutada regulada 12V DC (3A mínimo).
* **4x** Resistencias de $4.7\text{ k}\Omega$ (Pull-ups externos para buses I2C si no vienen en breakout).
* **Cables de conexión:** Arnés trenzado de jumpers Dupont macho-hembra y hembra-hembra.

### Piezas Mecánicas y Rodamientos:
* **1x** Rodamiento de bolas 608-2RS ($\varnothing 22 \times \varnothing 8 \times 7\text{ mm}$).
* **1x** Kit completo de piezas impresas 3D (`Base_Torre_Furuta`, `Brazo_Horizontal_L88`, `Barra_Pendulo_Furuta`, `Eje_Portaiman_Pendulo`, `Masa_Ajustable_Pendulo`, `Acoplador_Motor_AS5600`, `Portaiman_AS5600`, `Caja_Base_Nucleo`, `Tapa_Caja_Nucleo`).

### Tornillería Estándar (Ferretería M3, M2, M5):
* **8x** Tornillos M3 $\times 6\text{ mm}$ (fijación de los 2 sensores AS5600).
* **4x** Tornillos M3 $\times 6\text{ mm}$ (fijación de la placa Nucleo a los separadores internos).
* **4x** Tornillos M3 $\times 12\text{ mm}$ avellanados (cierre de la tapa de la caja).
* **2x** Tornillos M2 $\times 5\text{ mm}$ (fijación frontal del motor Namiki a la placa superior).
* **1x** Tornillo M3 $\times 16\text{ mm}$ con tuerca autoblocante M3 (fijación de la masa ajustable).
* **4x** Tornillos M5 $\times 25\text{ mm}$ con tuercas mariposa (anclaje de la base a la mesa de trabajo).
* **1x** Adhesivo instantáneo de cianoacrilato líquido (adhesión capilar del acoplador motor).
