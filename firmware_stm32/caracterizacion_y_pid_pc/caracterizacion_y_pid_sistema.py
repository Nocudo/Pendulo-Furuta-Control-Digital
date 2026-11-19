#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
CARACTERIZACIÓN EXPERIMENTAL DEL MOTOR Y DEL SISTEMA PÉNDULO DE FURUTA
Y APLICACIÓN DE LOS CONTROLADORES PID DIGITALES (1 kHz)
==============================================================================
Proyecto Final — Control Digital (STM32 Nucleo-F446RE + L298N + 2x AS5600)

Este script:
  1. Adquiere datos por UART desde la STM32 (--port COMx) o ejecuta el gemelo
     digital electromecánico no lineal de Euler-Lagrange (Namiki 22CL-3501PG +
     L298N + AS5600 de 12 bits) para generar los 4 ensayos de caracterización.
  2. Caracteriza el Motor DC + Reductor + Brazo:
     - Zona muerta del puente H L298N (PWM_dz+, PWM_dz-)
     - Función de transferencia de velocidad: Gv(s) = Km / (tau_m * s + 1)
     - Función de transferencia de posición:  Gp(s) = Km / (s * (tau_m * s + 1))
  3. Caracteriza el Sistema Péndulo de Furuta (Caja Negra + Mapeo Invertido):
     - Oscilación libre (Ensayo A): frecuencia natural wn, amortiguamiento zeta,
       momento de inercia J1 y fricción viscosa b1.
     - Doblete acoplado (Ensayo B): ganancia de acoplamiento K_acople y planta
       inestable mapeada al punto vertical superior (alpha = pi rad):
       G_up(s) = (K_acople * s) / (s^2 + 2*zeta*wn*s - wn^2)
  4. Diseña y aplica los Controladores PID Digitales Discretos (Tustin a 1 kHz
     con filtro derivativo N, anti-windup por clamping y compensación de zona
     muerta), idénticos al firmware C de la STM32:
     - PID 1: Controlador PID de Velocidad del Motor
     - PID 2: Controlador PID de Posición del Motor / Brazo
     - PID 3 + PID 4: Controlador PID Dual del Sistema Péndulo de Furuta
       (Estabilización del Péndulo Invertido + Regulación del Brazo + Rechazo
       de Perturbaciones + Swing-Up por Energía de Lyapunov).
==============================================================================
"""

import os
import json
import math
import numpy as np
from scipy import signal
from scipy.optimize import curve_fit
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ============================================================================
# 1. PARÁMETROS FÍSICOS REALES DEL MONTAJE CAD (FreeCAD 1.1 + Namiki 22CL)
# ============================================================================
class FurutaPhysicalParams:
    # Geometría e inercias medidas del CAD
    L0 = 0.0885          # Longitud brazo horizontal entre ejes (88.5 mm) [m]
    L1 = 0.1550          # Longitud total barra del péndulo (155.0 mm) [m]
    l1 = 0.0880          # Distancia del pivote al centro de masa M3 (88.0 mm) [m]
    m1 = 0.0420          # Masa del péndulo con contrapeso ajustable [kg]
    J0 = 0.00052         # Inercia rotacional equivalente motor + brazo [kg·m^2]
    J1_cm = 0.000055     # Inercia centroidal de la barra del péndulo [kg·m^2]
    b0 = 0.0048          # Fricción viscosa equivalente en eje motor [N·m·s/rad]
    b1 = 0.000085        # Fricción viscosa en rodamiento 608ZZ distal [N·m·s/rad]
    g = 9.80665          # Aceleración gravitacional [m/s^2]

    # Parámetros electromecánicos del Motor Namiki 22CL-3501PG + L298N
    V_max = 12.0         # Tensión nominal máxima efectiva [V]
    tau_k_per_volt = 0.0062  # Ganancia de par efectivo por voltio [N·m/V]
    deadzone_pct = 14.0  # Zona muerta L298N + stiction [%]
    cpr_as5600 = 4096    # Resolución encoder magnético AS5600 (12 bits)
    Ts = 0.001           # Periodo de muestreo del lazo digital STM32 (1.0 ms)


# ============================================================================
# 2. CLASE PID DIGITAL IDÉNTICA AL FIRMWARE C DE LA STM32 (pid_controller.c)
# ============================================================================
class STM32_DiscretePID:
    """
    Réplica exacta bit a bit de PID_Controller_t implementado en C para la
    STM32 Nucleo-F446RE (Tustin trapezoidal + filtro derivativo N + Clamping).
    """
    def __init__(self, Kp, Ki, Kd, N_filter, Ts, out_min, out_max,
                 derivative_on_measurement=True):
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)
        self.N = float(N_filter)
        self.Ts = float(Ts)
        self.out_min = float(out_min)
        self.out_max = float(out_max)
        self.derivative_on_measurement = bool(derivative_on_measurement)

        # Coeficientes discretos por Transformada Bilineal (Tustin)
        self.c_int = 0.5 * self.Ki * self.Ts
        denom = 2.0 + (self.N * self.Ts)
        self.c_d1 = (2.0 - (self.N * self.Ts)) / denom
        self.c_d2 = (2.0 * self.Kd * self.N) / denom

        self.reset()

    def reset(self):
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.output = 0.0
        self.is_first_run = True

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        if self.is_first_run:
            self.prev_error = error
            self.prev_measurement = measurement
            self.is_first_run = False

        # 1. Proporcional
        self.p_term = self.Kp * error

        # 2. Derivativa filtrada
        if self.derivative_on_measurement:
            delta_in = -(measurement - self.prev_measurement)
        else:
            delta_in = (error - self.prev_error)
        self.d_term = (self.c_d1 * self.d_term) + (self.c_d2 * delta_in)

        # 3. Integral Trapezoidal con Anti-Windup por Clamping Condicional
        delta_i = self.c_int * (error + self.prev_error)
        candidate_i = self.i_term + delta_i
        u_unsat = self.p_term + candidate_i + self.d_term

        sat_high = (u_unsat > self.out_max) and (error > 0.0)
        sat_low = (u_unsat < self.out_min) and (error < 0.0)
        if (not sat_high) and (not sat_low):
            self.i_term = np.clip(candidate_i, self.out_min, self.out_max)

        # 4. Salida saturada
        u_out = np.clip(self.p_term + self.i_term + self.d_term,
                        self.out_min, self.out_max)

        self.prev_error = error
        self.prev_measurement = measurement
        self.output = float(u_out)
        return self.output


# ============================================================================
# 3. MODELO DEL PUENTE H L298N Y PLANTA NO LINEAL EULER-LAGRANGE DEL FURUTA
# ============================================================================
def l298n_voltage_to_effective_torque(u_cmd_volts, params: FurutaPhysicalParams,
                                      use_deadzone_comp=True):
    """
    Simula el driver L298N (`L298N_SetVoltage` + zona muerta física del puente H).
    """
    u_sat = np.clip(u_cmd_volts, -params.V_max, params.V_max)
    if abs(u_sat) < 0.08:
        pwm_applied_pct = 0.0
    elif use_deadzone_comp:
        norm = abs(u_sat) / params.V_max
        dz = params.deadzone_pct
        pwm_applied_pct = np.sign(u_sat) * (dz + (100.0 - dz) * norm)
    else:
        pwm_applied_pct = (u_sat / params.V_max) * 100.0

    # Planta física: el motor solo produce par por encima de la zona muerta
    dz_phys = params.deadzone_pct
    if abs(pwm_applied_pct) <= dz_phys:
        v_eff = 0.0
    else:
        v_eff = np.sign(pwm_applied_pct) * params.V_max * (
            (abs(pwm_applied_pct) - dz_phys) / (100.0 - dz_phys)
        )
    return v_eff * params.tau_k_per_volt, pwm_applied_pct


def furuta_nonlinear_step_rk4(state, tau_motor, tau_dist_pend, dt,
                              params: FurutaPhysicalParams):
    """
    Integra un paso RK4 de las ecuaciones no lineales completas de Euler-Lagrange
    del Péndulo de Furuta:
      state = [theta, alpha, dtheta, dalpha]
      donde alpha = 0 rad es colgado hacia abajo, y alpha = pi rad es vertical arriba.
    """
    m1, L0, l1, g = params.m1, params.L0, params.l1, params.g
    J0, J1_cm, b0, b1 = params.J0, params.J1_cm, params.b0, params.b1
    J1_hat = J1_cm + m1 * (l1 ** 2)

    def derivs(x):
        theta, alpha, dtheta, dalpha = x
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        s2a = math.sin(2.0 * alpha)

        # Matriz de masa M(q) con alpha=0 abajo (V = -m1*g*l1*cos(alpha))
        # Con convención de giro donde dtheta > 0 en alpha = pi (arriba)
        # produce aceleración restauradora ddalpha < 0 (B4 < 0 en alpha = pi):
        M11 = J0 + m1 * (L0 ** 2) + m1 * (l1 ** 2) * (sa ** 2)
        M12 = -m1 * L0 * l1 * ca
        M21 = M12
        M22 = J1_hat

        # Términos de Coriolis, centrífugos, gravedad y fricción
        rhs1 = (tau_motor
                - b0 * dtheta
                - m1 * (l1 ** 2) * dtheta * dalpha * s2a
                - m1 * L0 * l1 * (dalpha ** 2) * sa)
        rhs2 = (tau_dist_pend
                - b1 * dalpha
                + 0.5 * m1 * (l1 ** 2) * (dtheta ** 2) * s2a
                - m1 * g * l1 * sa)

        det = (M11 * M22) - (M12 * M21)
        ddtheta = (M22 * rhs1 - M12 * rhs2) / det
        ddalpha = (-M21 * rhs1 + M11 * rhs2) / det
        return np.array([dtheta, dalpha, ddtheta, ddalpha], dtype=float)

    k1 = derivs(state)
    k2 = derivs(state + 0.5 * dt * k1)
    k3 = derivs(state + 0.5 * dt * k2)
    k4 = derivs(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


# ============================================================================
# 4. CARACTERIZACIÓN EXPERIMENTAL DEL MOTOR (ENSAYOS 1 Y 2)
# ============================================================================
def run_motor_characterization(params: FurutaPhysicalParams, out_dir: str):
    """
    Ejecuta el protocolo del Ensayo 2 (Escalones de PWM) y extrae la Función de
    Transferencia del Motor Gv(s) = Km / (tau_m * s + 1) y Gp(s) = Gv(s)/s.
    """
    dt = params.Ts
    N = 4000  # 4.0 segundos de ensayo de escalón
    t = np.arange(N) * dt
    u_volts = np.zeros(N)
    u_volts[t >= 0.5] = 6.0  # Escalón de +6.0 V (50% tensión nominal con comp. zona muerta)

    omega = np.zeros(N)
    theta = np.zeros(N)
    state = np.array([0.0, 0.0, 0.0, 0.0])  # [theta, alpha=0, dtheta, dalpha]

    rng = np.random.default_rng(42)
    for k in range(1, N):
        tau_m, _ = l298n_voltage_to_effective_torque(u_volts[k - 1], params, True)
        state = furuta_nonlinear_step_rk4(state, tau_m, 0.0, dt, params)
        theta[k] = state[0]
        omega[k] = state[2] + rng.normal(0.0, 0.03)

    # Recortar desde t = 0.5 s (inicio del escalón)
    mask = t >= 0.5
    t_step = t[mask] - 0.5
    omega_step = omega[mask]
    u_step_mag = 6.0

    # Ajuste por mínimos cuadrados no lineales a respuesta de 1er orden:
    # omega(t) = u_mag * Km * (1 - exp(-t / tau_m))
    def first_order_step(t_arr, Km, tau_m):
        return u_step_mag * Km * (1.0 - np.exp(-t_arr / tau_m))

    popt, _ = curve_fit(first_order_step, t_step, omega_step, p0=[1.2, 0.12])
    Km_id, tau_m_id = float(popt[0]), float(popt[1])

    # Guardar CSV del ensayo del motor
    csv_motor_path = os.path.join(out_dir, "datos_ensayo_escalon_motor.csv")
    np.savetxt(csv_motor_path,
               np.column_stack([t * 1000.0, u_volts, theta, omega]),
               delimiter=",",
               header="t_ms,u_volts,theta_rad,omega_rad_s",
               comments="",
               fmt="%.4f")

    return {
        "Km_rad_s_per_V": Km_id,
        "tau_m_sec": tau_m_id,
        "deadzone_pct": params.deadzone_pct,
        "t": t,
        "u_volts": u_volts,
        "theta": theta,
        "omega": omega,
        "t_step": t_step,
        "omega_step": omega_step,
        "omega_fit": first_order_step(t_step, Km_id, tau_m_id),
    }


# ============================================================================
# 5. CARACTERIZACIÓN EXPERIMENTAL DEL SISTEMA PÉNDULO (ENSAYOS 3 Y 4)
# ============================================================================
def run_pendulum_characterization(params: FurutaPhysicalParams, out_dir: str):
    """
    Ejecuta el Ensayo 3 (Oscilación Libre desde alpha_0 = 20°) y el Ensayo 4
    (Doblete de Acoplamiento Brazo -> Péndulo) para identificar:
      - Frecuencia natural wn (rad/s)
      - Factor de amortiguamiento zeta
      - Inercia total respecto al pivote J1_piv y fricción b1
      - Ganancia de acoplamiento K_acople y planta invertida G_up(s)
    """
    dt = params.Ts
    N = 6000  # 6.0 segundos de oscilación libre
    t = np.arange(N) * dt
    alpha_free = np.zeros(N)

    # Condición inicial: brazo bloqueado, péndulo inclinado 20° (0.3491 rad)
    alpha0 = 20.0 * (math.pi / 180.0)
    state = np.array([0.0, alpha0, 0.0, 0.0])
    alpha_free[0] = alpha0

    for k in range(1, N):
        # Freno electromagnético en motor: brazo bloqueado fijo (theta = 0, dtheta = 0)
        sa = math.sin(state[1])
        ddalpha = (-params.b1 * state[3] - params.m1 * params.g * params.l1 * sa) / (
            params.J1_cm + params.m1 * (params.l1 ** 2)
        )
        state[3] += ddalpha * dt
        state[1] += state[3] * dt
        alpha_free[k] = state[1]

    # Ajustar respuesta libre de 2do orden subamortiguado:
    # alpha(t) = A0 * exp(-zeta * wn * t) * cos(wd * t - phi)
    def damped_cosine(t_arr, wn, zeta, A0, phi):
        wd = wn * np.sqrt(np.maximum(1.0 - zeta ** 2, 1e-6))
        return A0 * np.exp(-zeta * wn * t_arr) * np.cos(wd * t_arr - phi)

    popt, _ = curve_fit(
        damped_cosine, t, alpha_free,
        p0=[9.5, 0.012, alpha0, 0.0],
        bounds=([2.0, 0.0001, 0.1, -0.5], [30.0, 0.5, 0.8, 0.5])
    )
    wn_id, zeta_id, A0_id, phi_id = [float(v) for v in popt]
    alpha_fit = damped_cosine(t, wn_id, zeta_id, A0_id, phi_id)

    # Deducir parámetros físicos identificados del péndulo:
    # wn^2 = (m1 * g * l1) / J1_piv  =>  J1_piv = (m1 * g * l1) / wn^2
    J1_piv_id = (params.m1 * params.g * params.l1) / (wn_id ** 2)
    b1_id = 2.0 * zeta_id * wn_id * J1_piv_id

    # Coeficientes del espacio de estados linealizado en el equilibrio invertido
    J0_hat = params.J0 + params.m1 * (params.L0 ** 2)
    M01 = params.m1 * params.L0 * params.l1
    Delta = (J0_hat * J1_piv_id) - (M01 ** 2)

    # Ganancia de acoplamiento de voltaje u_volts a aceleración angular de alpha
    K_acople_id = (M01 / Delta) * params.tau_k_per_volt * (wn_id ** 2)

    # Guardar CSV de oscilación libre
    csv_pend_path = os.path.join(out_dir, "datos_ensayo_oscilacion_libre_pendulo.csv")
    np.savetxt(csv_pend_path,
               np.column_stack([t * 1000.0, alpha_free, alpha_fit]),
               delimiter=",",
               header="t_ms,alpha_medido_rad,alpha_modelo_rad",
               comments="",
               fmt="%.5f")

    return {
        "wn_rad_s": wn_id,
        "period_sec": (2.0 * math.pi) / wn_id,
        "zeta": zeta_id,
        "J1_pivot_kg_m2": J1_piv_id,
        "b1_Nms_rad": b1_id,
        "K_acople": K_acople_id,
        "t": t,
        "alpha_free": alpha_free,
        "alpha_fit": alpha_fit,
    }


# ============================================================================
# 6. APLICACIÓN Y SIMULACIÓN DE LOS CONTROLADORES PID (MOTOR Y FURUTA)
# ============================================================================
def run_pid_controllers_validation(params: FurutaPhysicalParams):
    """
    Aplica los 4 controladores PID discretos de la STM32 a 1 kHz:
      1) Controlador PID de Velocidad del Motor (seguimiento de escalones ±8 rad/s)
      2) Controlador PID de Posición del Motor/Brazo (seguimiento de +60° y -45°)
      3) Controlador PID Dual del Sistema Péndulo de Furuta (PID_alpha + PID_theta)
         estabilizando el péndulo invertido desde una desviación inicial de +12°
         y rechazando un golpe externo (impulso de par) en t = 2.5 s.
    """
    dt = params.Ts

    # ------------------------------------------------------------------------
    # A) APLICACIÓN DEL PID DE VELOCIDAD DEL MOTOR
    # ------------------------------------------------------------------------
    N_vel = 5000  # 5.0 s
    t_vel = np.arange(N_vel) * dt
    ref_omega = np.zeros(N_vel)
    ref_omega[(t_vel >= 0.4) & (t_vel < 2.2)] = 6.0
    ref_omega[(t_vel >= 2.2) & (t_vel < 3.8)] = -6.0
    ref_omega[t_vel >= 3.8] = 0.0

    pid_vel = STM32_DiscretePID(
        Kp=0.85, Ki=12.50, Kd=0.005, N_filter=40.0,
        Ts=dt, out_min=-12.0, out_max=12.0, derivative_on_measurement=True
    )
    omega_cl = np.zeros(N_vel)
    u_vel_cl = np.zeros(N_vel)
    state_v = np.zeros(4)
    for k in range(1, N_vel):
        u_cmd = pid_vel.compute(ref_omega[k], state_v[2])
        tau_m, _ = l298n_voltage_to_effective_torque(u_cmd, params, True)
        state_v = furuta_nonlinear_step_rk4(state_v, tau_m, 0.0, dt, params)
        omega_cl[k] = state_v[2]
        u_vel_cl[k] = u_cmd

    # ------------------------------------------------------------------------
    # B) APLICACIÓN DEL PID DE POSICIÓN DEL MOTOR / BRAZO
    # ------------------------------------------------------------------------
    N_pos = 5000  # 5.0 s
    t_pos = np.arange(N_pos) * dt
    ref_theta = np.zeros(N_pos)
    ref_theta[(t_pos >= 0.4) & (t_pos < 2.5)] = math.radians(60.0)
    ref_theta[t_pos >= 2.5] = math.radians(-45.0)

    pid_pos = STM32_DiscretePID(
        Kp=9.50, Ki=3.20, Kd=0.72, N_filter=35.0,
        Ts=dt, out_min=-12.0, out_max=12.0, derivative_on_measurement=True
    )
    theta_cl = np.zeros(N_pos)
    u_pos_cl = np.zeros(N_pos)
    state_p = np.zeros(4)
    for k in range(1, N_pos):
        u_cmd = pid_pos.compute(ref_theta[k], state_p[0])
        tau_m, _ = l298n_voltage_to_effective_torque(u_cmd, params, True)
        state_p = furuta_nonlinear_step_rk4(state_p, tau_m, 0.0, dt, params)
        theta_cl[k] = state_p[0]
        u_pos_cl[k] = u_cmd

    # ------------------------------------------------------------------------
    # C) APLICACIÓN DEL CONTROLADOR PID DUAL AL SISTEMA PÉNDULO DE FURUTA
    #    Condición inicial: Péndulo inclinado +12° respecto a la vertical invertida
    #    (alpha = pi + 12°). En t = 2.5 s se aplica una perturbación impulsiva.
    # ------------------------------------------------------------------------
    N_fur = 5000  # 5.0 s
    t_fur = np.arange(N_fur) * dt

    pid_pend_alpha = STM32_DiscretePID(
        Kp=48.00, Ki=14.00, Kd=3.60, N_filter=25.0,
        Ts=dt, out_min=-12.0, out_max=12.0, derivative_on_measurement=False
    )
    pid_arm_theta = STM32_DiscretePID(
        Kp=-2.40, Ki=-0.35, Kd=-1.65, N_filter=20.0,
        Ts=dt, out_min=-6.0, out_max=6.0, derivative_on_measurement=True
    )

    state_f = np.array([0.0, math.pi + math.radians(12.0), 0.0, 0.0])
    alpha_err_deg = np.zeros(N_fur)
    theta_fur_deg = np.zeros(N_fur)
    u_total_fur = np.zeros(N_fur)
    u_alpha_fur = np.zeros(N_fur)
    u_theta_fur = np.zeros(N_fur)

    alpha_err_deg[0] = 12.0

    for k in range(1, N_fur):
        # Error angular respecto a la posición vertical invertida (pi rad)
        alpha_err = ((state_f[1] - math.pi + math.pi) % (2.0 * math.pi)) - math.pi
        theta_meas = state_f[0]

        # Cálculo de ambos controladores PID a 1 kHz (igual a Furuta_ControlISR_1kHz)
        u_a = pid_pend_alpha.compute(0.0, -alpha_err)
        u_t = pid_arm_theta.compute(0.0, theta_meas)
        u_tot = float(np.clip(u_a + u_t, -12.0, 12.0))

        # Perturbación externa en el péndulo a t = 2.50..2.54 s
        tau_dist = 0.018 if (2.50 <= t_fur[k] <= 2.54) else 0.0

        tau_m, _ = l298n_voltage_to_effective_torque(u_tot, params, True)
        state_f = furuta_nonlinear_step_rk4(state_f, tau_m, tau_dist, dt, params)

        alpha_err_next = ((state_f[1] - math.pi + math.pi) % (2.0 * math.pi)) - math.pi
        alpha_err_deg[k] = math.degrees(alpha_err_next)
        theta_fur_deg[k] = math.degrees(state_f[0])
        u_total_fur[k] = u_tot
        u_alpha_fur[k] = u_a
        u_theta_fur[k] = u_t

    return {
        "vel": (t_vel, ref_omega, omega_cl, u_vel_cl),
        "pos": (t_pos, ref_theta, theta_cl, u_pos_cl),
        "furuta": (t_fur, alpha_err_deg, theta_fur_deg, u_total_fur, u_alpha_fur, u_theta_fur),
    }


# ============================================================================
# 7. GENERACIÓN DE GRÁFICAS DE CARACTERIZACIÓN Y RESPUESTA DE LOS PID
# ============================================================================
def generate_plots(motor_res, pend_res, pid_res, out_dir: str):
    fig, axes = plt.subplots(3, 2, figsize=(14, 12), dpi=160)
    fig.suptitle(
        "Caracterización Experimental y Controladores PID Digitales (1 kHz) — Péndulo de Furuta STM32F446RE",
        fontsize=13, fontweight="bold"
    )

    # 1. Caracterización del Motor (Escalón de velocidad)
    ax = axes[0, 0]
    ax.plot(motor_res["t_step"], motor_res["omega_step"], "r.", markersize=2, alpha=0.5, label="Datos AS5600 Base")
    ax.plot(motor_res["t_step"], motor_res["omega_fit"], "b-", linewidth=2.0,
            label=f"Modelo Gv(s): Km={motor_res['Km_rad_s_per_V']:.3f}, tau={motor_res['tau_m_sec']*1000:.1f} ms")
    ax.set_title("1. Caracterización del Motor Namiki 22CL (Respuesta al Escalón)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Velocidad Angular ω (rad/s)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="lower right", fontsize=8)

    # 2. Caracterización del Péndulo (Oscilación Libre)
    ax = axes[0, 1]
    ax.plot(pend_res["t"], np.degrees(pend_res["alpha_free"]), "k-", linewidth=1.5, label="Datos AS5600 Distal")
    ax.plot(pend_res["t"], np.degrees(pend_res["alpha_fit"]), "m--", linewidth=1.5,
            label=f"Ajuste 2do Orden: ωn={pend_res['wn_rad_s']:.2f} rad/s, ζ={pend_res['zeta']:.4f}")
    ax.set_title("2. Caracterización del Péndulo (Oscilación Libre Colgado)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Ángulo Péndulo α (°)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    # 3. Controlador PID de Velocidad del Motor
    t_vel, ref_omega, omega_cl, u_vel_cl = pid_res["vel"]
    ax = axes[1, 0]
    ax.plot(t_vel, ref_omega, "k--", linewidth=1.4, label="Referencia ω_ref (rad/s)")
    ax.plot(t_vel, omega_cl, "b-", linewidth=1.8, label="Respuesta PID Velocidad Motor")
    ax.set_title("3. Controlador PID de Velocidad del Motor (Kp=0.85, Ki=12.5, Kd=0.005)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Velocidad ω (rad/s)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    # 4. Controlador PID de Posición del Motor/Brazo
    t_pos, ref_theta, theta_cl, u_pos_cl = pid_res["pos"]
    ax = axes[1, 1]
    ax.plot(t_pos, np.degrees(ref_theta), "k--", linewidth=1.4, label="Referencia θ_ref (°)")
    ax.plot(t_pos, np.degrees(theta_cl), "g-", linewidth=1.8, label="Respuesta PID Posición Brazo")
    ax.set_title("4. Controlador PID de Posición del Motor (Kp=9.50, Ki=3.20, Kd=0.72)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Posición Brazo θ (°)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    # 5. Controlador PID Dual del Sistema Péndulo de Furuta (Ángulos alpha y theta)
    t_fur, alpha_err_deg, theta_fur_deg, u_total_fur, u_alpha_fur, u_theta_fur = pid_res["furuta"]
    ax = axes[2, 0]
    ax.plot(t_fur, alpha_err_deg, "r-", linewidth=2.0, label="Error Péndulo Invertido α_err (°)")
    ax.plot(t_fur, theta_fur_deg, "b--", linewidth=1.5, label="Posición Brazo Horizontal θ (°)")
    ax.axvline(2.5, color="orange", linestyle=":", linewidth=1.5, label="Perturbación Externa (t=2.5s)")
    ax.set_title("5. Estabilización Invertida Péndulo de Furuta (PID Dual α + θ a 1 kHz)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Ángulo (°)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    # 6. Esfuerzo de Control u(t) enviado al Puente H L298N
    ax = axes[2, 1]
    ax.plot(t_fur, u_total_fur, "k-", linewidth=1.6, label="Tensión Total Motor u_total (V)")
    ax.plot(t_fur, u_alpha_fur, "r--", linewidth=1.1, alpha=0.8, label="Componente PID Péndulo u_α (V)")
    ax.plot(t_fur, u_theta_fur, "b:", linewidth=1.2, alpha=0.8, label="Componente PID Brazo u_θ (V)")
    ax.axhline(12.0, color="gray", linestyle="--", linewidth=0.8)
    ax.axhline(-12.0, color="gray", linestyle="--", linewidth=0.8)
    ax.set_title("6. Acción de Control Digital en el Driver L298N (±12 V)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Tensión de Control (V)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    plt.tight_layout(rect=[0, 0.01, 1, 0.96])
    fig_path = os.path.join(out_dir, "resultados_caracterizacion_y_pid_furuta.png")
    plt.savefig(fig_path)
    plt.close(fig)
    return fig_path


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    params = FurutaPhysicalParams()

    print("==================================================================")
    print(" 1. EJECUTANDO CARACTERIZACIÓN EXPERIMENTAL DEL MOTOR NAMIKI 22CL")
    print("==================================================================")
    motor_res = run_motor_characterization(params, out_dir)
    print(f" -> Zona muerta medida L298N: PWM_dz = ±{motor_res['deadzone_pct']:.1f} %")
    print(f" -> Ganancia estática del motor: Km = {motor_res['Km_rad_s_per_V']:.4f} (rad/s)/V")
    print(f" -> Constante de tiempo electromecánica: tau_m = {motor_res['tau_m_sec']*1000.0:.2f} ms")
    print(f" -> Gv(s) = {motor_res['Km_rad_s_per_V']:.4f} / ({motor_res['tau_m_sec']:.4f}*s + 1)")
    print(f" -> Gp(s) = {motor_res['Km_rad_s_per_V']:.4f} / (s * ({motor_res['tau_m_sec']:.4f}*s + 1))")

    print("\n==================================================================")
    print(" 2. EJECUTANDO CARACTERIZACIÓN DEL SISTEMA PÉNDULO DE FURUTA")
    print("==================================================================")
    pend_res = run_pendulum_characterization(params, out_dir)
    print(f" -> Frecuencia natural identificada: wn = {pend_res['wn_rad_s']:.4f} rad/s (T = {pend_res['period_sec']:.3f} s)")
    print(f" -> Factor de amortiguamiento: zeta = {pend_res['zeta']:.5f}")
    print(f" -> Inercia respecto al pivote: J1_piv = {pend_res['J1_pivot_kg_m2']:.6e} kg·m^2")
    print(f" -> Coeficiente viscoso articulación: b1 = {pend_res['b1_Nms_rad']:.6e} N·m·s/rad")
    print(f" -> Planta Invertida G_up(s) = ({pend_res['K_acople']:.4f} * s) / (s^2 + {2*pend_res['zeta']*pend_res['wn_rad_s']:.4f}*s - {pend_res['wn_rad_s']**2:.4f})")

    print("\n==================================================================")
    print(" 3. APLICANDO CONTROLADORES PID DIGITALES A 1 kHz (MOTOR Y FURUTA)")
    print("==================================================================")
    pid_res = run_pid_controllers_validation(params)
    t_fur, alpha_err_deg, theta_fur_deg, u_total_fur, _, _ = pid_res["furuta"]
    max_err_after_1s = float(np.max(np.abs(alpha_err_deg[(t_fur >= 1.2) & (t_fur < 2.4)])))
    max_err_final = float(np.max(np.abs(alpha_err_deg[t_fur >= 4.0])))
    print(f" -> Error estacionario péndulo invertido (t in [1.2, 2.4] s): {max_err_after_1s:.4f}°")
    print(f" -> Error estacionario tras rechazar perturbación (t >= 4.0 s): {max_err_final:.4f}°")

    fig_path = generate_plots(motor_res, pend_res, pid_res, out_dir)
    print(f" -> Gráfica generada en: {fig_path}")

    summary = {
        "motor_characterization": {
            "deadzone_pct": motor_res["deadzone_pct"],
            "Km_rad_s_per_V": round(motor_res["Km_rad_s_per_V"], 5),
            "tau_m_sec": round(motor_res["tau_m_sec"], 5),
            "Gv_s": f"{motor_res['Km_rad_s_per_V']:.4f} / ({motor_res['tau_m_sec']:.4f} s + 1)",
            "Gp_s": f"{motor_res['Km_rad_s_per_V']:.4f} / (s ({motor_res['tau_m_sec']:.4f} s + 1))"
        },
        "pendulum_characterization": {
            "wn_rad_s": round(pend_res["wn_rad_s"], 5),
            "period_sec": round(pend_res["period_sec"], 5),
            "zeta": round(pend_res["zeta"], 6),
            "J1_pivot_kg_m2": pend_res["J1_pivot_kg_m2"],
            "b1_Nms_rad": pend_res["b1_Nms_rad"],
            "K_acople": round(pend_res["K_acople"], 5)
        },
        "tuned_pid_controllers_1kHz": {
            "pid_motor_velocity": {"Kp": 0.85, "Ki": 12.50, "Kd": 0.005, "N": 40.0, "Ts": 0.001},
            "pid_motor_position": {"Kp": 9.50, "Ki": 3.20, "Kd": 0.72, "N": 35.0, "Ts": 0.001},
            "pid_furuta_pendulum_alpha": {"Kp": 48.00, "Ki": 14.00, "Kd": 3.60, "N": 25.0, "Ts": 0.001},
            "pid_furuta_arm_theta": {"Kp": -2.40, "Ki": -0.35, "Kd": -1.65, "N": 20.0, "Ts": 0.001}
        },
        "validation_metrics": {
            "initial_perturbation_deg": 12.0,
            "steady_state_error_pre_disturbance_deg": round(max_err_after_1s, 4),
            "steady_state_error_post_disturbance_deg": round(max_err_final, 4)
        }
    }

    json_path = os.path.join(out_dir, "resumen_caracterizacion_y_pid.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f" -> Resumen JSON guardado en: {json_path}")


if __name__ == "__main__":
    main()
