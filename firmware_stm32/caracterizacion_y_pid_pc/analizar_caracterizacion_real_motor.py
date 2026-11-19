#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
ANÁLISIS E IDENTIFICACIÓN EXPERIMENTAL REAL DEL MOTOR NAMIKI 22CL-3501PG
A PARTIR DE TELEMETRÍA CAPTURADA EN VIVO DESDE LA STM32 NUCLEO-F446RE (COM13)
==============================================================================
"""

import os
import json
import math
import numpy as np
from scipy.optimize import curve_fit, minimize
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GEAR_RATIO = 80.0  # Reductor planetario Namiki 22CL-3501PG (80:1)
V_SUPPLY = 12.0    # Tensión nominal fuente DC (12.0 V)


def analyze_deadzone_csv(dz_csv_path):
    if not os.path.exists(dz_csv_path):
        return 7.0, 7.0, None
    data = np.genfromtxt(dz_csv_path, delimiter=",", skip_header=1)
    if data.ndim != 2 or len(data) < 10:
        return 7.0, 7.0, None

    t_s = (data[:, 0] - data[0, 0]) / 1000.0
    pwm = data[:, 2]
    if np.max(np.abs(data[:, 5])) < 30.0:
        theta_rot = data[:, 4] * GEAR_RATIO
        omega_rot = data[:, 5] * GEAR_RATIO
    else:
        theta_rot = -data[:, 4]
        omega_rot = -data[:, 5]

    # Buscar el último escalón de PWM positivo y negativo antes de que se corte a 0
    pos_pwms = pwm[pwm > 0.5]
    neg_pwms = np.abs(pwm[pwm < -0.5])
    dz_pos = float(np.max(pos_pwms)) if len(pos_pwms) > 0 else 7.0
    dz_neg = float(np.max(neg_pwms)) if len(neg_pwms) > 0 else 7.0
    return dz_pos, dz_neg, (t_s, pwm, theta_rot, omega_rot)


def analyze_motor_steps_csv(steps_csv_path, dz_pos, dz_neg):
    data = np.genfromtxt(steps_csv_path, delimiter=",", skip_header=1)
    # Filtrar solo filas con state == 2
    mask_s2 = data[:, 1] == 2
    if np.sum(mask_s2) > 50:
        data = data[mask_s2]

    t_s = (data[:, 0] - data[0, 0]) / 1000.0
    dt = float(np.median(np.diff(t_s)))
    pwm = data[:, 2]
    u_volts = data[:, 3]
    if np.max(np.abs(data[:, 5])) < 30.0:
        theta_arm = data[:, 4]
        omega_arm = data[:, 5]
        theta_rotor = theta_arm * GEAR_RATIO
        omega_rotor = omega_arm * GEAR_RATIO
    else:
        theta_rotor = -data[:, 4]
        omega_rotor = -data[:, 5]
        theta_arm = theta_rotor / GEAR_RATIO
        omega_arm = omega_rotor / GEAR_RATIO
    alpha_pend = data[:, 6]
    alpha_dot_pend = data[:, 8]

    # Extraer cada uno de los 8 escalones (±20%, ±30%, ±40%, ±50% PWM)
    step_levels = [20.0, -20.0, 30.0, -30.0, 40.0, -40.0, 50.0, -50.0]
    step_metrics = []

    for lvl in step_levels:
        idx = np.where(np.isclose(pwm, lvl, atol=0.5))[0]
        if len(idx) < 5:
            continue
        i0, i1 = idx[0], idx[-1]
        t_win = t_s[i0:i1 + 1] - t_s[i0]
        w_win = omega_rotor[i0:i1 + 1]
        u_v = (lvl / 100.0) * V_SUPPLY

        # Ajuste de respuesta al escalón de 1er orden: w(t) = w_ss * (1 - exp(-t / tau))
        def step_1st(t_arr, w_ss, tau):
            return w_ss * (1.0 - np.exp(-t_arr / np.maximum(tau, 0.005)))

        p0 = [np.sign(lvl) * 600.0, 0.08]
        try:
            popt, _ = curve_fit(step_1st, t_win, w_win, p0=p0, maxfev=5000)
            w_ss_est, tau_est = float(popt[0]), float(abs(popt[1]))
        except Exception:
            w_ss_est = float(w_win[-1])
            tau_est = 0.085

        step_metrics.append({
            "pwm_pct": lvl,
            "u_volts": round(u_v, 3),
            "duration_ms": round(float(t_win[-1] * 1000.0), 1),
            "omega_rotor_peak_rad_s": round(float(w_win[-1]), 2),
            "omega_rotor_ss_fit_rad_s": round(w_ss_est, 2),
            "omega_arm_ss_fit_rad_s": round(w_ss_est / GEAR_RATIO, 4),
            "tau_m_ms": round(tau_est * 1000.0, 2),
        })

    # Identificación global del modelo dinámico de 1er orden con zona muerta y fricción:
    # d(omega_rotor)/dt = (-omega_rotor + Km_eff * u_eff(pwm)) / tau_m
    def simulate_model(params):
        Km_v, tau_m, tau_coast, dz_v = params
        if tau_m <= 0.005 or tau_coast <= 0.005 or Km_v <= 0 or dz_v < 0:
            return np.full_like(omega_rotor, 1e6)
        w_sim = np.zeros_like(omega_rotor)
        for k in range(1, len(t_s)):
            dt_k = t_s[k] - t_s[k - 1]
            u_k = u_volts[k - 1]
            if abs(u_k) > dz_v:
                u_eff = np.sign(u_k) * (abs(u_k) - dz_v)
                dw = (-w_sim[k - 1] + Km_v * u_eff) / tau_m
            else:
                dw = -w_sim[k - 1] / tau_coast
            w_sim[k] = w_sim[k - 1] + dw * dt_k
        return w_sim

    def loss_fn(params):
        w_sim = simulate_model(params)
        return float(np.mean((omega_rotor - w_sim) ** 2))

    res = minimize(
        loss_fn,
        x0=[195.0, 0.095, 0.115, 0.85],
        bounds=[(50.0, 500.0), (0.02, 0.40), (0.02, 0.40), (0.1, 3.0)],
        method="L-BFGS-B"
    )
    Km_rotor_eff, tau_m_on, tau_m_off, v_dz_fit = [float(v) for v in res.x]
    w_rotor_sim = simulate_model(res.x)

    # Ganancia lineal promedio respecto a tensión total V (con compensación de zona muerta)
    Km_rotor_nominal = float(np.mean([
        abs(m["omega_rotor_ss_fit_rad_s"]) / abs(m["u_volts"]) for m in step_metrics
    ]))
    Km_arm_nominal = Km_rotor_nominal / GEAR_RATIO
    Km_arm_eff = Km_rotor_eff / GEAR_RATIO

    # Calcular métrica de ajuste R^2 (Fit %)
    ss_res = np.sum((omega_rotor - w_rotor_sim) ** 2)
    ss_tot = np.sum((omega_rotor - np.mean(omega_rotor)) ** 2)
    r2_score = float(1.0 - (ss_res / ss_tot))
    nrmse_fit_pct = float((1.0 - np.sqrt(ss_res / ss_tot)) * 100.0)

    return {
        "t_s": t_s,
        "dt": dt,
        "pwm": pwm,
        "u_volts": u_volts,
        "theta_rotor": theta_rotor,
        "omega_rotor": omega_rotor,
        "w_rotor_sim": w_rotor_sim,
        "theta_arm": theta_arm,
        "omega_arm": omega_arm,
        "w_arm_sim": w_rotor_sim / GEAR_RATIO,
        "alpha_pend": alpha_pend,
        "alpha_dot_pend": alpha_dot_pend,
        "step_metrics": step_metrics,
        "Km_rotor_eff_rad_s_per_V": round(Km_rotor_eff, 3),
        "Km_rotor_nominal_rad_s_per_V": round(Km_rotor_nominal, 3),
        "Km_arm_eff_rad_s_per_V": round(Km_arm_eff, 4),
        "Km_arm_nominal_rad_s_per_V": round(Km_arm_nominal, 4),
        "tau_m_accel_ms": round(tau_m_on * 1000.0, 2),
        "tau_m_coast_ms": round(tau_m_off * 1000.0, 2),
        "tau_m_mean_sec": round(0.5 * (tau_m_on + tau_m_off), 4),
        "v_deadzone_fit_V": round(v_dz_fit, 3),
        "pwm_deadzone_fit_pct": round((v_dz_fit / V_SUPPLY) * 100.0, 2),
        "r2_score": round(r2_score, 4),
        "fit_pct": round(nrmse_fit_pct, 2),
    }


def plot_real_characterization(dz_res, mot_res, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9.5), dpi=160)
    fig.suptitle(
        "Caracterización Experimental Real en Hardware — Motor Namiki 22CL-3501PG + L298N + AS5600 (STM32F446RE)",
        fontsize=12.5, fontweight="bold"
    )

    t = mot_res["t_s"]

    # Panel 1: Escalones de Entrada PWM (%) y Tensión (V)
    ax = axes[0, 0]
    ax.plot(t, mot_res["pwm"], "b-", linewidth=1.8, label="PWM Aplicado (%)")
    ax.axhline(mot_res["pwm_deadzone_fit_pct"], color="r", linestyle="--", linewidth=1.0,
               label=f"Zona Muerta Identificada (±{mot_res['pwm_deadzone_fit_pct']:.1f}%)")
    ax.axhline(-mot_res["pwm_deadzone_fit_pct"], color="r", linestyle="--", linewidth=1.0)
    ax.set_title("1. Secuencia Experimental de Escalones Bidireccionales (±20%..±50% PWM)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Ciclo Útil PWM (%)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8.5)

    # Panel 2: Velocidad Angular Real vs Modelo Identificado Gv(s)
    ax = axes[0, 1]
    ax.plot(t, mot_res["omega_rotor"], "k.", markersize=3, alpha=0.65,
            label="Velocidad Medida AS5600 Rotor (rad/s)")
    ax.plot(t, mot_res["w_rotor_sim"], "r-", linewidth=1.8,
            label=f"Modelo Identificado Gv(s) (R² = {mot_res['r2_score']*100:.1f}%)")
    ax.set_title(
        f"2. Ajuste Dinámico Velocidad: τm={mot_res['tau_m_accel_ms']:.1f} ms, "
        f"Km_rot={mot_res['Km_rotor_eff_rad_s_per_V']:.1f} (rad/s)/V",
        fontsize=10.5
    )
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Velocidad Rotor ω_rotor (rad/s)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8.5)

    # Panel 3: Posición Angular y Velocidad en el Eje de Salida del Brazo (Reductor 80:1)
    ax = axes[1, 0]
    ax.plot(t, np.degrees(mot_res["theta_arm"]), "g-", linewidth=2.0,
            label="Ángulo Brazo Horizontal θ_brazo (°)")
    ax2 = ax.twinx()
    ax2.plot(t, mot_res["omega_arm"], "m--", linewidth=1.3, alpha=0.8,
             label="Velocidad Brazo ω_brazo (rad/s)")
    ax.set_title("3. Cinemática en Salida del Reductor Planetario (80:1) — Ida y Vuelta sin Enredo")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Ángulo del Brazo θ (°)", color="g")
    ax2.set_ylabel("Velocidad del Brazo ω (rad/s)", color="m")
    ax.grid(True, linestyle="--", alpha=0.6)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="lower left", fontsize=8.5)

    # Panel 4: Oscilación Inducida en el Péndulo (Sensor 2 AS5600 Distal en I2C2)
    ax = axes[1, 1]
    ax.plot(t, np.degrees(mot_res["alpha_pend"]), "darkorange", linewidth=1.8,
            label="Ángulo Péndulo α medido en I2C2 (°)")
    ax.plot(t, mot_res["alpha_dot_pend"], "c--", linewidth=1.2, alpha=0.8,
            label="Velocidad Péndulo dα/dt (rad/s)")
    ax.set_title("4. Respuesta Acoplada Real del Péndulo (Sensor 2 AS5600 en I2C2)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Ángulo α (°) / Velocidad (rad/s)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8.5)

    plt.tight_layout(rect=[0, 0.01, 1, 0.95])
    out_png = os.path.join(out_dir, "caracterizacion_experimental_real_motor_stm32.png")
    plt.savefig(out_png)
    plt.close(fig)
    return out_png


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    dz_csv = os.path.join(out_dir, "ensayo_real_zona_muerta_stm32.csv")
    steps_csv = os.path.join(out_dir, "ensayo_real_motor_stm32.csv")

    dz_pos, dz_neg, dz_data = analyze_deadzone_csv(dz_csv)
    mot_res = analyze_motor_steps_csv(steps_csv, dz_pos, dz_neg)
    png_path = plot_real_characterization(dz_data, mot_res, out_dir)

    summary = {
        "hardware_measured": "STM32 Nucleo-F446RE (COM13) + Driver L298N + Motor Namiki 22CL-3501PG + 2x AS5600",
        "deadzone_experimental": {
            "pwm_deadzone_pos_detected_pct": dz_pos,
            "pwm_deadzone_neg_detected_pct": dz_neg,
            "pwm_deadzone_dynamic_fit_pct": mot_res["pwm_deadzone_fit_pct"],
            "v_deadzone_dynamic_fit_V": mot_res["v_deadzone_fit_V"],
        },
        "identified_motor_model": {
            "gearbox_ratio": GEAR_RATIO,
            "encoder_mounting": "Eje trasero del rotor (antes del reductor 80:1), polaridad direction = -1",
            "Km_rotor_eff_rad_s_per_V": mot_res["Km_rotor_eff_rad_s_per_V"],
            "Km_rotor_nominal_rad_s_per_V": mot_res["Km_rotor_nominal_rad_s_per_V"],
            "Km_arm_eff_rad_s_per_V": mot_res["Km_arm_eff_rad_s_per_V"],
            "Km_arm_nominal_rad_s_per_V": mot_res["Km_arm_nominal_rad_s_per_V"],
            "tau_m_accel_ms": mot_res["tau_m_accel_ms"],
            "tau_m_coast_ms": mot_res["tau_m_coast_ms"],
            "tau_m_mean_sec": mot_res["tau_m_mean_sec"],
            "Gv_rotor_s": f"{mot_res['Km_rotor_eff_rad_s_per_V']:.2f} / ({mot_res['tau_m_mean_sec']:.4f} s + 1)",
            "Gv_brazo_s": f"{mot_res['Km_arm_eff_rad_s_per_V']:.4f} / ({mot_res['tau_m_mean_sec']:.4f} s + 1)",
            "Gp_brazo_s": f"{mot_res['Km_arm_eff_rad_s_per_V']:.4f} / (s * ({mot_res['tau_m_mean_sec']:.4f} s + 1))",
            "r2_coefficient_of_determination": mot_res["r2_score"],
            "nrmse_fit_pct": mot_res["fit_pct"],
        },
        "step_by_step_measurements": mot_res["step_metrics"],
    }

    json_path = os.path.join(out_dir, "caracterizacion_real_motor_resultados.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print("Gráfica guardada en:", png_path)


if __name__ == "__main__":
    main()
