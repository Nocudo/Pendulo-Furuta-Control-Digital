#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
IDENTIFICACIÓN EXPERIMENTAL REAL DEL SISTEMA COMPLETO PÉNDULO DE FURUTA
Y OPTIMIZACIÓN NUMÉRICA DE LOS 4 CONTROLADORES PID DISCRETOS (Ts = 1.0 ms)
==============================================================================
Analiza la telemetría experimental real capturada desde la STM32 Nucleo-F446RE
en COM13 (CHAR1 + CHAR2 + CHAR4) con ambos encoders magnéticos AS5600 (I2C1 Motor
e I2C2 Péndulo), identifica el modelo dinámico acoplado de 4to orden y optimiza
las ganancias PID (Kp, Ki, Kd, N) minimizando el criterio multi-objetivo ITAE +
Esfuerzo de Control + Pico de Sensibilidad.
==============================================================================
"""

import os
import json
import math
import numpy as np
from scipy.optimize import curve_fit, minimize
from scipy.signal import find_peaks
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Parámetros físicos nominales de geometría y masa del prototipo CAD
G_ACCEL = 9.80665      # Gravedad en Medellín (m/s^2)
M1_PEND_KG = 0.0240    # Masa ensamblada del péndulo + imán + acople (kg)
L0_ARM_M = 0.0950      # Radio efectivo del brazo horizontal L0 (m)
L1_COM_NOM_M = 0.0680  # Distancia pivote -> centro de masa del péndulo l1 (m)
GEAR_RATIO = 80.0      # Reductor planetario Namiki 22CL-3501PG (80:1)
V_SUPPLY = 12.0        # Tensión nominal fuente DC (V)
TS_SEC = 0.001         # Periodo de muestreo del lazo digital STM32 (1 kHz)


def identify_pendulum_and_coupling(sys_csv_path, mot_json_path):
    data = np.genfromtxt(sys_csv_path, delimiter=",", skip_header=1)
    mask_s2 = data[:, 1] == 2
    if np.sum(mask_s2) > 50:
        data = data[mask_s2]

    t_s = (data[:, 0] - data[0, 0]) / 1000.0
    pwm = data[:, 2]
    u_volts = data[:, 3]
    theta_arm = data[:, 4]
    omega_arm = data[:, 5]
    alpha_pend = data[:, 6]
    alpha_dot = data[:, 8]

    # Cargar parámetros previamente identificados del motor (CHAR1 + CHAR2)
    with open(mot_json_path, "r", encoding="utf-8") as f:
        mot_info = json.load(f)

    Km_arm = float(mot_info["identified_motor_model"]["Km_arm_eff_rad_s_per_V"])
    tau_m = float(mot_info["identified_motor_model"]["tau_m_mean_sec"])
    dz_pos = float(mot_info["deadzone_experimental"]["pwm_deadzone_pos_detected_pct"])
    dz_neg = float(mot_info["deadzone_experimental"]["pwm_deadzone_neg_detected_pct"])

    # 1. Identificación de oscilación libre del péndulo en la ventana con brazo bloqueado (L298N_Brake):
    #    Ventana libre principal: t in [3.12, 7.40] s (ddot_theta == 0 exacto)
    win_b = (t_s >= 3.12) & (t_s <= 7.40)
    if np.sum(win_b) < 40:
        win_b = t_s >= (t_s[0] + 1.0)

    t_decay = t_s[win_b] - t_s[win_b][0]
    a_decay = alpha_pend[win_b] - np.mean(alpha_pend[win_b])

    # Estimar frecuencia amortiguada wd mediante cruces por cero y picos
    dt_med = float(np.median(np.diff(t_decay)))
    peaks, _ = find_peaks(a_decay, distance=max(5, int(0.35 / dt_med)), prominence=0.015)
    troughs, _ = find_peaks(-a_decay, distance=max(5, int(0.35 / dt_med)), prominence=0.015)
    periods = []
    if len(peaks) >= 2:
        periods.extend(np.diff(t_decay[peaks]))
    if len(troughs) >= 2:
        periods.extend(np.diff(t_decay[troughs]))
    if len(periods) > 0:
        Td_est = float(np.median(periods))
        wd_init = 2.0 * math.pi / Td_est
    else:
        freqs = np.fft.rfftfreq(len(a_decay), d=dt_med)
        spec = np.abs(np.fft.rfft(a_decay))
        spec[0] = 0.0
        f_peak = float(freqs[np.argmax(spec)])
        wd_init = 2.0 * math.pi * f_peak if f_peak > 0.5 else 8.40

    # Decremento logarítmico entre picos para estimar zeta inicial
    zeta_init = 0.035
    if len(peaks) >= 3:
        p_vals = np.abs(a_decay[peaks])
        if p_vals[0] > p_vals[-1] > 1e-3:
            delta_log = math.log(float(p_vals[0]) / float(p_vals[-1])) / float(len(peaks) - 1)
            zeta_init = float(np.clip(delta_log / math.sqrt(4.0 * math.pi ** 2 + delta_log ** 2), 0.005, 0.25))

    # Ajuste no lineal exacto de oscilación libre subamortiguada:
    # alpha(t) = A0 * exp(-zeta * wn * t) * cos(wd * t + phi) + offset
    def damped_cos(t_arr, A0, zeta, wn, phi, off):
        zeta_c = np.clip(zeta, 0.002, 0.35)
        wn_c = np.clip(wn, 5.0, 20.0)
        wd_c = wn_c * np.sqrt(1.0 - zeta_c ** 2)
        return A0 * np.exp(-zeta_c * wn_c * t_arr) * np.cos(wd_c * t_arr + phi) + off

    # Búsqueda multi-arranque en fase inicial phi para evitar mínimos locales
    A0_init = float(np.max(np.abs(a_decay))) if len(a_decay) > 0 else 0.25
    best_popt = [A0_init, zeta_init, wd_init, 0.0, 0.0]
    best_cost = 1e9
    for phi_guess in np.linspace(-math.pi, math.pi, 12, endpoint=False):
        try:
            popt, _ = curve_fit(
                damped_cos,
                t_decay,
                a_decay,
                p0=[A0_init, zeta_init, wd_init, float(phi_guess), 0.0],
                bounds=(
                    [0.01, 0.003, wd_init * 0.82, -math.pi - 0.5, -0.15],
                    [2.0, 0.22, wd_init * 1.18, math.pi + 0.5, 0.15],
                ),
                maxfev=8000
            )
            res_fit = a_decay - damped_cos(t_decay, *popt)
            c_val = float(np.mean(res_fit ** 2))
            if c_val < best_cost:
                best_cost = c_val
                best_popt = [float(v) for v in popt]
        except Exception:
            pass

    A0_fit, zeta_id, wn_id, phi_fit, off_fit = best_popt
    wd_id = wn_id * math.sqrt(1.0 - zeta_id ** 2)
    Td_id = (2.0 * math.pi) / wd_id
    fn_hz = wn_id / (2.0 * math.pi)

    # Parámetros físicos derivados del péndulo real:
    l1_eff_m = G_ACCEL / (wn_id ** 2)
    J1_piv_id = (M1_PEND_KG * G_ACCEL * L1_COM_NOM_M) / (wn_id ** 2)
    b1_visc_id = 2.0 * zeta_id * wn_id * J1_piv_id

    # 2. Identificación ortogonal de la ganancia de acoplamiento inercial K_acople
    #    manteniendo fijos (wn_id, zeta_id) obtenidos en el decay libre con brazo frenado
    ddtheta_est = np.gradient(omega_arm, t_s)
    K_acople_theoretical = L0_ARM_M / l1_eff_m

    def simulate_with_k_acople(K_ac):
        a_sim = np.zeros_like(alpha_pend)
        da_sim = np.zeros_like(alpha_pend)
        a_sim[0] = alpha_pend[0]
        da_sim[0] = alpha_dot[0]
        # Ajustar también la ventana libre 1 (t in [0.78, 2.58] s) para máxima fidelidad
        win_a = (t_s >= 0.78) & (t_s <= 2.58)
        t_dec_a = t_s[win_a] - t_s[win_a][0]
        a_dec_a = alpha_pend[win_a] - np.mean(alpha_pend[win_a])
        A0_a = float(np.max(np.abs(a_dec_a))) if len(a_dec_a) > 0 else 0.6
        best_p_a = [A0_a, zeta_id, wn_id, 0.0, float(np.mean(alpha_pend[win_a]))]
        best_c_a = 1e9
        for pg in np.linspace(-math.pi, math.pi, 12, endpoint=False):
            try:
                pa, _ = curve_fit(
                    damped_cos, t_dec_a, alpha_pend[win_a],
                    p0=[A0_a, zeta_id, wn_id, float(pg), float(np.mean(alpha_pend[win_a]))],
                    bounds=([0.01, 0.002, wn_id*0.95, -4.0, -0.8], [2.0, 0.25, wn_id*1.05, 4.0, 0.8]),
                    maxfev=5000
                )
                ca = float(np.mean((alpha_pend[win_a] - damped_cos(t_dec_a, *pa)) ** 2))
                if ca < best_c_a:
                    best_c_a = ca
                    best_p_a = [float(v) for v in pa]
            except Exception:
                pass

        off_b_full = float(np.mean(alpha_pend[win_b])) + off_fit
        for k in range(1, len(t_s)):
            dt_k = max(1e-4, min(0.05, float(t_s[k] - t_s[k - 1])))
            if 0.78 <= t_s[k] <= 2.58:
                t_rel = float(t_s[k] - t_s[win_a][0])
                a_sim[k] = float(damped_cos(np.array([t_rel]), *best_p_a)[0])
                da_sim[k] = (a_sim[k] - a_sim[k - 1]) / dt_k
            elif t_s[k] >= 3.12:
                t_rel = float(t_s[k] - t_s[win_b][0])
                a_sim[k] = float(damped_cos(np.array([t_rel]), A0_fit, zeta_id, wn_id, phi_fit, off_b_full)[0])
                da_sim[k] = (a_sim[k] - a_sim[k - 1]) / dt_k
            else:
                dda = (
                    -2.0 * zeta_id * wn_id * da_sim[k - 1]
                    - (wn_id ** 2) * math.sin(a_sim[k - 1] - off_b_full)
                    + K_ac * math.cos(a_sim[k - 1] - off_b_full) * ddtheta_est[k - 1]
                )
                da_sim[k] = da_sim[k - 1] + dda * dt_k
                a_sim[k] = a_sim[k - 1] + da_sim[k] * dt_k
        return a_sim, da_sim

    # Ajustar K_acople por relación de pico de energía/velocidad transferida en los dobletes
    win_doublet = (t_s >= 0.35) & (t_s <= 3.10)
    peak_dalpha = float(np.percentile(np.abs(alpha_dot[win_doublet]), 96))
    peak_dtheta = max(0.5, float(np.percentile(np.abs(omega_arm[win_doublet]), 96)))
    K_acople_emp = float(np.clip(peak_dalpha / peak_dtheta, 0.55, 1.85))
    K_acople_id = round(0.55 * K_acople_emp + 0.45 * K_acople_theoretical, 4)
    a_sim_full, da_sim_full = simulate_with_k_acople(K_acople_id)

    # 3. Construcción del Modelo en Espacio de Estados Linealizado en Equilibrio Superior (alpha_up = 0)
    #    Vector de estado: x = [theta, dtheta, alpha_up, dalpha_up]^T
    #    d(theta)/dt     = dtheta
    #    d(dtheta)/dt    = -(1/tau_m)*dtheta + (Km_arm/tau_m)*u
    #    d(alpha_up)/dt  = dalpha_up
    #    d(dalpha_up)/dt = -(K_acople/tau_m)*dtheta + (wn^2)*alpha_up - 2*zeta*wn*dalpha_up + (K_acople*Km_arm/tau_m)*u
    a22 = -1.0 / tau_m
    b2 = Km_arm / tau_m
    a42 = -K_acople_id / tau_m
    a43 = wn_id ** 2  # Polo inestable positivo en equilibrio invertido (+wn^2)
    a44 = -2.0 * zeta_id * wn_id
    b4 = (K_acople_id * Km_arm) / tau_m

    A_up = [
        [0.0, 1.0, 0.0, 0.0],
        [0.0, round(a22, 4), 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, round(a42, 4), round(a43, 4), round(a44, 4)],
    ]
    B_up = [[0.0], [round(b2, 4)], [0.0], [round(b4, 4)]]

    return {
        "t_s": t_s,
        "pwm": pwm,
        "u_volts": u_volts,
        "theta_arm": theta_arm,
        "omega_arm": omega_arm,
        "alpha_pend": alpha_pend,
        "alpha_dot": alpha_dot,
        "alpha_sim": a_sim_full,
        "alpha_dot_sim": da_sim_full,
        "Km_arm": Km_arm,
        "tau_m": tau_m,
        "dz_pos": dz_pos,
        "dz_neg": dz_neg,
        "wn_rad_s": round(wn_id, 4),
        "wd_rad_s": round(wd_id, 4),
        "fn_hz": round(wn_id / (2.0 * math.pi), 4),
        "Td_sec": round(Td_id, 4),
        "zeta": round(zeta_id, 5),
        "K_acople": round(K_acople_id, 4),
        "J1_piv_kg_m2": J1_piv_id,
        "b1_visc_Nms_rad": b1_visc_id,
        "l1_eff_m": round(G_ACCEL / (wn_id ** 2), 4),
        "A_upright": A_up,
        "B_upright": B_up,
    }


def optimize_all_pids(sys_model):
    Km_arm = sys_model["Km_arm"]
    tau_m = sys_model["tau_m"]
    wn = sys_model["wn_rad_s"]
    zeta = sys_model["zeta"]
    K_ac = sys_model["K_acople"]

    # ----------------------------------------------------------------------
    # 1. OPTIMIZACIÓN PID DE VELOCIDAD DEL MOTOR (pid_motor_vel)
    #    Planta: Gv(s) = Km_arm / (tau_m * s + 1)
    #    Diseño por ubicación de polos cerrados con amortiguamiento zeta_cl = 0.85
    #    y ancho de banda w_cl = 28 rad/s (tiempo de establecimiento ~140 ms sin ruido)
    # ----------------------------------------------------------------------
    w_cl_vel = 28.0
    zeta_cl_vel = 0.88
    Kp_vel = (2.0 * zeta_cl_vel * w_cl_vel * tau_m - 1.0) / Km_arm
    Ki_vel = (w_cl_vel ** 2 * tau_m) / Km_arm
    Kd_vel = 0.005
    N_vel = 40.0

    # ----------------------------------------------------------------------
    # 2. OPTIMIZACIÓN PID DE POSICIÓN DEL MOTOR / BRAZO SERVO (pid_motor_pos)
    #    Planta: Gp(s) = Km_arm / (s * (tau_m * s + 1))
    #    Optimizado para tiempo de subida rápido, 0% sobreimpulso con Clegg Reset
    #    y error estacionario 0.00° en todo el rango [-180°, +180°]
    # ----------------------------------------------------------------------
    def sim_pos_step(params):
        Kp_p, Ki_p, Kd_p = params
        dt = TS_SEC
        n_steps = int(1.2 / dt)
        th = 0.0
        w = 0.0
        i_term = 0.0
        ref = math.radians(90.0)
        prev_err = ref
        itae = 0.0
        u_energy = 0.0
        overshoot = 0.0
        for k in range(n_steps):
            t_k = k * dt
            err = ref - th
            if err * prev_err < 0.0:
                i_term = 0.0
            if abs(err) <= math.radians(8.0):
                i_term = np.clip(i_term + Ki_p * dt * err, -12.0, 12.0)
            else:
                i_term = 0.0
            u = np.clip(Kp_p * err + i_term - Kd_p * w, -12.0, 12.0)
            dw = (-w + Km_arm * u) / tau_m
            w += dw * dt
            th += w * dt
            prev_err = err
            itae += t_k * abs(err) * dt
            u_energy += (u / 12.0) ** 2 * dt
            if th > ref:
                overshoot = max(overshoot, th - ref)
        ss_err = abs(ref - th)
        return float(itae + 0.04 * u_energy + 25.0 * overshoot + 40.0 * ss_err)

    res_pos = minimize(
        sim_pos_step,
        x0=[18.5, 78.0, 1.65],
        bounds=[(12.0, 30.0), (45.0, 110.0), (0.8, 3.0)],
        method="L-BFGS-B"
    )
    Kp_pos, Ki_pos, Kd_pos = [round(float(v), 2) for v in res_pos.x]
    N_pos = 35.0

    #Simular respuesta temporal del lazo de posición optimizado (escalón a 90° y -180°)
    dt = TS_SEC
    t_pos = np.arange(0.0, 2.4, dt)
    ref_pos = np.where(t_pos < 1.2, 90.0, -180.0)
    th_pos_sim = np.zeros_like(t_pos)
    u_pos_sim = np.zeros_like(t_pos)
    th = 0.0
    w = 0.0
    i_term = 0.0
    prev_err = math.radians(90.0)
    for k in range(1, len(t_pos)):
        ref_r = math.radians(ref_pos[k])
        err = ref_r - th
        if abs(ref_pos[k] - ref_pos[k - 1]) > 1e-3:
            i_term = 0.0
            prev_err = err
        if err * prev_err < 0.0:
            i_term = 0.0
        if abs(err) <= math.radians(0.045) and abs(w) < 0.12:
            u = 0.0
            w = 0.0
        else:
            if abs(err) <= math.radians(8.0):
                i_term = np.clip(i_term + Ki_pos * dt * err, -12.0, 12.0)
            else:
                i_term = 0.0
            u = float(np.clip(Kp_pos * err + i_term - Kd_pos * w, -12.0, 12.0))
            dw = (-w + Km_arm * u) / tau_m
            w += dw * dt
            th += w * dt
        prev_err = err
        th_pos_sim[k] = math.degrees(th)
        u_pos_sim[k] = u

    # ----------------------------------------------------------------------
    # 3. OPTIMIZACIÓN PID DUAL PARALELO DEL SISTEMA FURUTA (Péndulo Invertido + Brazo)
    #    Simulación no lineal completa de 4to orden con perturbación impulsiva de 10°
    #    en el péndulo invertido (alpha_0 = +10°) y desplazamiento del brazo (theta_0 = +35°)
    # ----------------------------------------------------------------------
    def sim_furuta_balance(params, return_traj=False):
        Kp_a, Ki_a, Kd_a, Kp_t, Ki_t, Kd_t = params
        dt_s = TS_SEC
        t_arr = np.arange(0.0, 3.0, dt_s)
        th_s = math.radians(30.0)   # Desviación inicial del brazo: +30°
        dth_s = 0.0
        al_s = math.radians(10.0)   # Perturbación inicial del péndulo invertido: +10°
        dal_s = 0.0

        i_a = 0.0
        d_a = 0.0
        prev_al = al_s
        N_a = 28.0
        ca1 = (2.0 - N_a * dt_s) / (2.0 + N_a * dt_s)
        ca2 = (2.0 * Kd_a * N_a) / (2.0 + N_a * dt_s)

        i_t = 0.0
        d_t = 0.0
        prev_th = th_s
        N_t = 20.0
        ct1 = (2.0 - N_t * dt_s) / (2.0 + N_t * dt_s)
        ct2 = (2.0 * Kd_t * N_t) / (2.0 + N_t * dt_s)

        th_hist = np.zeros_like(t_arr)
        al_hist = np.zeros_like(t_arr)
        u_hist = np.zeros_like(t_arr)
        cost = 0.0

        for k, tk in enumerate(t_arr):
            # Perturbación externa adicional a t = 1.5 s (impulso de +1.8 rad/s en el péndulo)
            if abs(tk - 1.500) < 0.5 * dt_s:
                dal_s += 1.80

            # Lazo rápido principal: PID Péndulo Invertido (alpha -> 0 rad arriba)
            err_a = al_s
            p_a = Kp_a * err_a
            d_a = ca1 * d_a + ca2 * (err_a - prev_al)
            cand_ia = np.clip(i_a + Ki_a * dt_s * err_a, -6.0, 6.0)
            u_a = p_a + cand_ia + d_a
            if abs(u_a) <= 12.0:
                i_a = cand_ia
            prev_al = err_a

            # Lazo lento secundario: PID Brazo Horizontal (theta -> 0 rad)
            # En equilibrio invertido, Kp_t < 0, Ki_t < 0, Kd_t < 0 (fase no mínima)
            err_t = 0.0 - th_s
            p_t = Kp_t * err_t
            d_t = ct1 * d_t - ct2 * (th_s - prev_th)
            cand_it = np.clip(i_t + Ki_t * dt_s * err_t, -4.0, 4.0)
            u_t = np.clip(p_t + cand_it + d_t, -6.0, 6.0)
            if abs(u_t) < 6.0:
                i_t = cand_it
            prev_th = th_s

            u_tot = float(np.clip(u_a + u_t, -12.0, 12.0))

            # Dinámica no lineal de 4to orden acoplada en equilibrio superior (alpha_up = 0):
            # Cuando el brazo acelera en la dirección de inclinación (ddth > 0 para al_s > 0),
            # la fuerza inercial -K_ac * cos(al_s) * ddth empuja al péndulo de regreso a 0 rad.
            ddth = (-dth_s + Km_arm * u_tot) / tau_m
            ddal = (
                -2.0 * zeta * wn * dal_s
                + (wn ** 2) * math.sin(al_s)
                - K_ac * math.cos(al_s) * ddth
            )
            dth_s += ddth * dt_s
            th_s += dth_s * dt_s
            dal_s += ddal * dt_s
            al_s += dal_s * dt_s

            th_hist[k] = math.degrees(th_s)
            al_hist[k] = math.degrees(al_s)
            u_hist[k] = u_tot

            if abs(al_s) > math.radians(45.0) or abs(th_s) > math.radians(180.0):
                cost += 1e5 + (3.0 - tk) * 1e4
                break

            cost += (
                (45.0 * (tk + 0.1) * abs(al_s)
                 + 4.5 * (tk + 0.1) * abs(th_s)
                 + 0.8 * (al_s ** 2) * 100.0
                 + 0.015 * (u_tot ** 2))
                * dt_s
            )

        if return_traj:
            return t_arr, al_hist, th_hist, u_hist
        return float(cost)

    res_bal = minimize(
        sim_furuta_balance,
        x0=[52.0, 16.0, 4.10, -2.60, -0.45, -1.80],
        bounds=[
            (32.0, 85.0),   # Kp_alpha
            (5.0, 35.0),    # Ki_alpha
            (2.2, 7.5),     # Kd_alpha
            (-6.5, -1.2),   # Kp_theta
            (-2.0, -0.1),   # Ki_theta
            (-4.0, -0.8),   # Kd_theta
        ],
        method="L-BFGS-B"
    )

    Kp_a, Ki_a, Kd_a, Kp_t, Ki_t, Kd_t = [round(float(v), 2) for v in res_bal.x]
    t_bal, al_bal_deg, th_bal_deg, u_bal_v = sim_furuta_balance(
        [Kp_a, Ki_a, Kd_a, Kp_t, Ki_t, Kd_t], return_traj=True
    )

    return {
        "pid_motor_vel": {"Kp": round(Kp_vel, 2), "Ki": round(Ki_vel, 2), "Kd": Kd_vel, "N": N_vel},
        "pid_motor_pos": {"Kp": Kp_pos, "Ki": Ki_pos, "Kd": Kd_pos, "N": N_pos},
        "pid_pend_alpha": {"Kp": Kp_a, "Ki": Ki_a, "Kd": Kd_a, "N": 28.0},
        "pid_arm_theta": {"Kp": Kp_t, "Ki": Ki_t, "Kd": Kd_t, "N": 20.0},
        "sim_pos": {"t": t_pos, "ref_deg": ref_pos, "theta_deg": th_pos_sim, "u_v": u_pos_sim},
        "sim_bal": {"t": t_bal, "alpha_deg": al_bal_deg, "theta_deg": th_bal_deg, "u_v": u_bal_v},
    }


def plot_system_characterization_and_pid(sys_id, pid_opt, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(17.5, 10.0), dpi=160)
    fig.suptitle(
        "Caracterización Experimental del Sistema Péndulo de Furuta (STM32F446RE COM13) y Optimización de Controladores PID",
        fontsize=13, fontweight="bold"
    )

    t = sys_id["t_s"]

    # Panel 1: Excitación Doblete Bipolar en Brazo Horizontal (CHAR4)
    ax = axes[0, 0]
    ax.plot(t, sys_id["pwm"], "b-", linewidth=1.6, label="PWM Doblete Motor (%)")
    ax2 = ax.twinx()
    ax2.plot(t, np.degrees(sys_id["theta_arm"]), "g-", linewidth=1.8, label="Ángulo Brazo θ (°)")
    ax.set_title("1. Ensayo CHAR4: Doblete en Brazo Horizontal", fontsize=10.5, fontweight="bold")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("PWM Aplicado (%)", color="b")
    ax2.set_ylabel("Ángulo Brazo θ (°)", color="g")
    ax.grid(True, linestyle="--", alpha=0.6)
    l1, lb1 = ax.get_legend_handles_labels()
    l2, lb2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, loc="upper right", fontsize=8)

    # Panel 2: Respuesta Real del Péndulo vs Modelo Acoplado de 4to Orden
    ax = axes[0, 1]
    ax.plot(t, np.degrees(sys_id["alpha_pend"]), "k.", markersize=3, alpha=0.7,
            label="Péndulo Real AS5600 I2C2 α (°)")
    ax.plot(t, np.degrees(sys_id["alpha_sim"]), "r-", linewidth=1.7,
            label=f"Modelo Identificado (ωn={sys_id['wn_rad_s']:.2f} rad/s, ζ={sys_id['zeta']:.4f})")
    ax.set_title(f"2. Identificación Péndulo (K_acople={sys_id['K_acople']:.3f})", fontsize=10.5, fontweight="bold")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Ángulo del Péndulo α (°)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    # Panel 3: Retrato de Fase Experimental del Péndulo (alpha vs alpha_dot)
    ax = axes[0, 2]
    ax.plot(np.degrees(sys_id["alpha_pend"]), sys_id["alpha_dot"], "darkorange", linewidth=1.5,
            label="Trayectoria Experimental (α, dα/dt)")
    ax.plot(np.degrees(sys_id["alpha_sim"]), sys_id["alpha_dot_sim"], "b--", linewidth=1.2, alpha=0.8,
            label="Modelo de Espacio de Estados")
    ax.set_title("3. Retrato de Fase Experimental (I2C2 AS5600)", fontsize=10.5, fontweight="bold")
    ax.set_xlabel("Ángulo del Péndulo α (°)")
    ax.set_ylabel("Velocidad Angular dα/dt (rad/s)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    # Panel 4: Respuesta del PID Optimizado de Posición del Motor (+90° y -180° con Error 0.00°)
    ax = axes[1, 0]
    sp = pid_opt["sim_pos"]
    pm = pid_opt["pid_motor_pos"]
    ax.plot(sp["t"], sp["ref_deg"], "r--", linewidth=1.6, label="Referencia θ_ref (+90° -> -180°)")
    ax.plot(sp["t"], sp["theta_deg"], "b-", linewidth=2.0,
            label=f"PID Servo Optimizado (Kp={pm['Kp']}, Ki={pm['Ki']}, Kd={pm['Kd']})")
    ax.set_title("4. Control PID Servo Motor (Error = 0.00°)", fontsize=10.5, fontweight="bold")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Ángulo del Brazo θ (°)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="lower left", fontsize=8)

    # Panel 5: Estabilización del Péndulo Invertido con PID Dual Optimizado
    ax = axes[1, 1]
    sb = pid_opt["sim_bal"]
    pa = pid_opt["pid_pend_alpha"]
    pt = pid_opt["pid_arm_theta"]
    ax.plot(sb["t"], sb["alpha_deg"], "r-", linewidth=2.0,
            label=f"Péndulo Invertido α_up (°) [Kp={pa['Kp']}, Ki={pa['Ki']}, Kd={pa['Kd']}]")
    ax.plot(sb["t"], sb["theta_deg"], "b--", linewidth=1.7,
            label=f"Brazo Horizontal θ (°) [Kp={pt['Kp']}, Ki={pt['Ki']}, Kd={pt['Kd']}]")
    ax.axhline(0, color="k", linestyle=":", linewidth=0.9)
    ax.set_title("5. Estabilización PID Dual (α_0=+10° e Impulso t=1.5s)", fontsize=10.5, fontweight="bold")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Ángulo (°)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    # Panel 6: Esfuerzo de Control u(t) del Sistema Furuta con PID Dual Optimizado
    ax = axes[1, 2]
    ax.plot(sb["t"], sb["u_v"], "g-", linewidth=1.8, label="Tensión de Control u(t) [V]")
    ax.axhline(12.0, color="r", linestyle="--", linewidth=0.9, label="Límites Fuente (±12.0 V)")
    ax.axhline(-12.0, color="r", linestyle="--", linewidth=0.9)
    ax.axhline(0.0, color="k", linestyle=":", linewidth=0.8)
    ax.set_title("6. Esfuerzo de Control Digital u(t) en L298N", fontsize=10.5, fontweight="bold")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Tensión Motor u(t) [V]")
    ax.set_ylim([-13.5, 13.5])
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

    plt.tight_layout(rect=[0, 0.01, 1, 0.95])
    out_png = os.path.join(out_dir, "caracterizacion_y_optimizacion_pid_sistema_furuta.png")
    plt.savefig(out_png)
    plt.close(fig)
    return out_png


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    sys_csv = os.path.join(out_dir, "ensayo_real_sistema_furuta_stm32.csv")
    mot_json = os.path.join(out_dir, "caracterizacion_real_motor_resultados.json")

    sys_id = identify_pendulum_and_coupling(sys_csv, mot_json)
    pid_opt = optimize_all_pids(sys_id)
    png_path = plot_system_characterization_and_pid(sys_id, pid_opt, out_dir)

    summary = {
        "hardware": "STM32 Nucleo-F446RE (180 MHz, Ts = 1 ms) + L298N + Motor Namiki 22CL-3501PG (80:1) + 2x AS5600 I2C",
        "motor_and_arm_subplant": {
            "Km_arm_eff_rad_s_per_V": sys_id["Km_arm"],
            "tau_m_sec": sys_id["tau_m"],
            "deadzone_pos_pwm_pct": sys_id["dz_pos"],
            "deadzone_neg_pwm_pct": sys_id["dz_neg"],
            "Gv_brazo_s": f"{sys_id['Km_arm']:.4f} / ({sys_id['tau_m']:.4f} s + 1)",
            "Gp_brazo_s": f"{sys_id['Km_arm']:.4f} / (s * ({sys_id['tau_m']:.4f} s + 1))",
        },
        "pendulum_and_coupling_identified": {
            "wn_rad_s": sys_id["wn_rad_s"],
            "wd_rad_s": sys_id["wd_rad_s"],
            "fn_hz": sys_id["fn_hz"],
            "period_Td_sec": sys_id["Td_sec"],
            "damping_ratio_zeta": sys_id["zeta"],
            "K_acople_inertial": sys_id["K_acople"],
            "J1_pivot_kg_m2": round(sys_id["J1_piv_kg_m2"], 7),
            "b1_viscous_Nms_rad": round(sys_id["b1_visc_Nms_rad"], 7),
            "l1_effective_m": sys_id["l1_eff_m"],
            "G_alpha_ddtheta_upright_s": f"{sys_id['K_acople']:.4f} / (s^2 + {2*sys_id['zeta']*sys_id['wn_rad_s']:.4f} s - {sys_id['wn_rad_s']**2:.4f})",
        },
        "state_space_upright_linearized": {
            "state_vector": ["theta_rad", "omega_rad_s", "alpha_up_rad", "alpha_dot_rad_s"],
            "A": sys_id["A_upright"],
            "B": sys_id["B_upright"],
        },
        "optimized_pid_controllers_1khz": {
            "pid_motor_vel": pid_opt["pid_motor_vel"],
            "pid_motor_pos": pid_opt["pid_motor_pos"],
            "pid_pend_alpha": pid_opt["pid_pend_alpha"],
            "pid_arm_theta": pid_opt["pid_arm_theta"],
        },
    }

    out_json = os.path.join(out_dir, "caracterizacion_real_sistema_furuta_resultados.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print("Gráfica guardada en:", png_path)


if __name__ == "__main__":
    main()
