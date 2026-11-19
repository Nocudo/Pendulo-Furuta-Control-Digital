/**
 ******************************************************************************
 * @file    furuta_control.c
 * @brief   Implementación de la Arquitectura Maestra de Control del Péndulo
 *          Invertido Rotacional de Furuta en STM32 Nucleo-F446RE.
 *
 *          Integra los 4 controladores PID sintonizados tras la caracterización:
 *            1. PID de Velocidad del Motor (pid_motor_vel)
 *            2. PID de Posición del Motor/Brazo solo (pid_motor_pos)
 *            3. PID Estabilizador del Péndulo Invertido (pid_pend_alpha)
 *            4. PID Regulador del Brazo en Equilibrio (pid_arm_theta)
 *          Además del controlador no lineal de bombeo de energía (Swing-Up)
 *          y el protocolo de comandos/telemetría por UART2.
 ******************************************************************************
 */

#include "furuta_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Instancia global del sistema Furuta */
Furuta_System_t g_furuta;

/* Bandera estática de bloqueo electromagnético al alcanzar error 0.00° en modo Servo */
static volatile bool s_pos_locked = false;
/* Contador de tiempo (ms) y setpoint resonante para la secuencia suave de Swing-Up */
static volatile uint32_t s_swing_elapsed_ms = 0U;
static volatile float s_swing_arm_target_rad = 0.0f;
/* Limitador de pendiente (Slew-Rate Limiter) de tensión para movimientos ultra-suaves sin golpes */
static float s_u_slewed_volts = 0.0f;
/* Contador de fallos consecutivos de I2C2 para parada inmediata si se desconecta un cable */
static uint8_t s_i2c_pend_fail_streak = 0U;

static float Furuta_ApplySmoothSlew(float u_target, float max_dv_per_ms, float max_abs_v)
{
    if (u_target > max_abs_v) {
        u_target = max_abs_v;
    } else if (u_target < -max_abs_v) {
        u_target = -max_abs_v;
    }
    float dv = u_target - s_u_slewed_volts;
    if (dv > max_dv_per_ms) {
        s_u_slewed_volts += max_dv_per_ms;
    } else if (dv < -max_dv_per_ms) {
        s_u_slewed_volts -= max_dv_per_ms;
    } else {
        s_u_slewed_volts = u_target;
    }
    return s_u_slewed_volts;
}

/**
 * @brief Calcula la energía mecánica normalizada adimensional del péndulo
 *        respecto al equilibrio superior invertido (donde E_norm = 0):
 *        E_norm = 0.5 * (d_alpha / wn)^2 + (cos(alpha_up) - 1.0)
 */
static float Furuta_ComputePendulumEnergy(const AS5600_Handle_t *enc_pend)
{
    const float wn_id = 9.7778f; /* Frecuencia natural experimental medida sin contrapeso (rad/s) */
    float alpha_up = enc_pend->angle_inverted_err_rad;
    float d_alpha  = enc_pend->velocity_rad_s;

    float e_kin_norm = 0.5f * ((d_alpha / wn_id) * (d_alpha / wn_id));
    float e_pot_norm = cosf(alpha_up) - 1.0f;

    return (e_kin_norm + e_pot_norm);
}

/**
 * @brief Secuencia Suave de Levantamiento Resonante Acotado (Sin Contrapeso, wn = 9.78 rad/s):
 *        - Limita la excursión del brazo a ±50° (reduciéndola a ±22° y ±8° al acercarse
 *          a la cima) y la tensión máxima a ±6.0 V (~53% PWM) con rampa Slew-Rate Limiter.
 */
static float Furuta_ComputeSwingUpVoltage(Furuta_System_t *sys)
{
    float alpha_down = sys->enc_pend.angle_wrapped_rad;      /* 0 rad abajo, ±pi arriba */
    float d_alpha    = sys->enc_pend.velocity_rad_s;
    float theta      = sys->enc_arm.angle_rad;
    float d_theta    = sys->enc_arm.velocity_rad_s;

    sys->pendulum_energy_j = Furuta_ComputePendulumEnergy(&sys->enc_pend);
    s_swing_elapsed_ms++;

    /* Modular carrera y tensión máxima según la energía actual del péndulo ligero: */
    float e_norm = sys->pendulum_energy_j; /* [-2.0 abajo .. 0.0 arriba] */
    float stroke_deg = 50.0f;
    float v_cap      = 6.0f;
    if (e_norm > -0.18f) {
        stroke_deg = 16.0f;  /* Cerca de la cima (155°..165°): impulso suave final para entrar a ±19° */
        v_cap      = 4.6f;
    } else if (e_norm > -0.55f) {
        stroke_deg = 28.0f;  /* Más de 115° de altura: impulso moderado de llegada */
        v_cap      = 5.2f;
    }

    /* Fase 1 (0..260 ms): Primer desplazamiento suave a +38° (semiperiodo Td/2 = 323 ms) */
    if (s_swing_elapsed_ms < 260U) {
        s_swing_arm_target_rad = sys->theta_setpoint_rad + (38.0f * DEG_TO_RAD);
    } else {
        /* Fase 2: Conmutación resonante al cruzar por el fondo (|alpha_down| < 28°) */
        if (fabsf(alpha_down) < (28.0f * DEG_TO_RAD)) {
            if (d_alpha < -0.50f) {
                s_swing_arm_target_rad = sys->theta_setpoint_rad - (stroke_deg * DEG_TO_RAD);
            } else if (d_alpha > 0.50f) {
                s_swing_arm_target_rad = sys->theta_setpoint_rad + (stroke_deg * DEG_TO_RAD);
            }
        }

        if (s_swing_elapsed_ms > 1300U &&
            fabsf(alpha_down) < (5.0f * DEG_TO_RAD) &&
            fabsf(d_alpha) < 0.40f) {
            s_swing_elapsed_ms = 0U;
            s_swing_arm_target_rad = sys->theta_setpoint_rad + (38.0f * DEG_TO_RAD);
        }
    }

    /* Controlador PD suave siguiendo s_swing_arm_target_rad con rampa Slew-Rate */
    float pos_err = s_swing_arm_target_rad - theta;
    float u_raw   = (11.50f * pos_err) - (1.20f * d_theta);

    return Furuta_ApplySmoothSlew(u_raw, 0.045f, v_cap);
}

/**
 * @brief Inicializa todos los drivers, sensores, controladores PID y estados
 *        con las ganancias obtenidas de la caracterización del motor y sistema.
 */
void Furuta_Init(Furuta_System_t *sys)
{
    sys->state               = FURUTA_STATE_IDLE;
    sys->enable_auto_swingup = false;
    sys->tick_counter_1khz   = 0U;

    sys->theta_setpoint_rad   = 0.0f;
    sys->omega_setpoint_rad_s = 0.0f;
    sys->u_alpha_volts        = 0.0f;
    sys->u_theta_volts        = 0.0f;
    sys->u_total_volts        = 0.0f;
    sys->pendulum_energy_j    = 0.0f;

    /* 1. Inicializar Etapa de Potencia L298N (TIM1_CH1 @ 20 kHz, PB4, PB5, PA9, PA10)
     *    Zona muerta con rampa suave pre-umbral: +11.5% PWM (antihorario), -13.5% PWM (horario) */
    L298N_Init(&sys->motor, &htim1, TIM_CHANNEL_1);
    L298N_SetDeadzone(&sys->motor, 11.5f, 13.5f, true);

    /* 2. Inicializar Encoders Magnéticos AS5600 (I2C1 Brazo con direction=-1 y gear=80:1, I2C2 Péndulo) */
    (void)AS5600_Init(&sys->enc_arm,  &hi2c1, TS_CONTROL_SEC, 0.006f, -1);
    (void)AS5600_Init(&sys->enc_pend, &hi2c2, TS_CONTROL_SEC, 0.005f,  1);

    /* 3. Inicializar los 4 Controladores PID Discretos Críticamente Amortiguados (Ts = 1.0 ms)
     *    Ajustados experimentalmente en hardware para CERO oscilaciones parásitas:
     * ----------------------------------------------------------------------
     * A) PID de Velocidad del Motor/Brazo:
     *    Kp = 1.40 V/(rad/s), Ki = 22.00 V/rad, Kd = 0.005 V·s/rad, N = 40 */
    PID_Init(&sys->pid_motor_vel,
             1.40f, 22.00f, 0.005f, 40.0f,
             TS_CONTROL_SEC, -MOTOR_NOMINAL_VOLTAGE, MOTOR_NOMINAL_VOLTAGE,
             true);

    /* B) PID de Posición del Motor/Brazo Solo (Suave, críticamente amortiguado, máx ±5.8 V):
     *    Kp = 13.50 V/rad, Ki = 65.00 V/(rad·s), Kd = 1.45 V·s/rad, N = 35
     *    Velocidad moderada sin sacudir el péndulo y error estacionario 0.00° */
    PID_Init(&sys->pid_motor_pos,
             13.50f, 65.00f, 1.45f, 35.0f,
             TS_CONTROL_SEC, -5.8f, 5.8f,
             true);

    /* C) PID Estabilizador del Péndulo Invertido sin Contrapeso (wn = 9.78 rad/s, K_acople = 1.136):
     *    Kp = 25.50 V/rad, Ki = 8.50 V/(rad·s), Kd = 2.15 V·s/rad, N = 30 */
    PID_Init(&sys->pid_pend_alpha,
             25.50f, 8.50f, 2.15f, 30.0f,
             TS_CONTROL_SEC, -6.2f, 6.2f,
             false);

    /* D) PID Regulador de Posición del Brazo en Equilibrio Furuta (Lazo Lento theta):
     *    Kp = -0.95 V/rad, Ki = -0.22 V/(rad·s), Kd = -0.75 V·s/rad, N = 20 */
    PID_Init(&sys->pid_arm_theta,
             -0.95f, -0.22f, -0.75f, 20.0f,
             TS_CONTROL_SEC, -2.8f, 2.8f,
             true);

    /* 4. Inicializar Gestor de Caracterización Experimental */
    Caracterizacion_Init(&sys->char_mgr);
}

/**
 * @brief Rutina Determinista de Control Digital ejecutada exactamente a 1 kHz
 *        desde la interrupción de hardware TIM6_DAC_IRQHandler.
 */
void Furuta_ControlISR_1kHz(Furuta_System_t *sys)
{
    sys->tick_counter_1khz++;

    /* Si el bucle principal está calibrando los sensores I2C, no colisionar el bus */
    if (sys->state == FURUTA_STATE_CALIBRATING) {
        return;
    }

    /* 1. Adquisición Síncrona de ambos Encoders Magnéticos AS5600 */
    (void)AS5600_Update(&sys->enc_arm);
    if (AS5600_Update(&sys->enc_pend) != HAL_OK) {
        if (s_i2c_pend_fail_streak < 250U) {
            s_i2c_pend_fail_streak++;
        }
    } else {
        s_i2c_pend_fail_streak = 0U;
    }

    float alpha_err = sys->enc_pend.angle_inverted_err_rad;
    float alpha_vel = sys->enc_pend.velocity_rad_s;
    float theta_pos = sys->enc_arm.angle_rad;
    float theta_vel = sys->enc_arm.velocity_rad_s;

    /* 2. Evaluación de la Máquina de Estados Finitos (FSM) */
    switch (sys->state) {

        case FURUTA_STATE_IDLE:
        case FURUTA_STATE_FAULT_STOP: {
            L298N_Coast(&sys->motor);
            s_u_slewed_volts   = 0.0f;
            sys->u_alpha_volts = 0.0f;
            sys->u_theta_volts = 0.0f;
            sys->u_total_volts = 0.0f;

            /* Si el usuario levanta manualmente el péndulo dentro de ±12° desde IDLE,
             * enganchar automáticamente el control PID estabilizador */
            if (sys->state == FURUTA_STATE_IDLE &&
                fabsf(alpha_err) < (12.0f * DEG_TO_RAD) &&
                fabsf(alpha_vel) < 1.5f) {
                PID_Reset(&sys->pid_pend_alpha);
                PID_Reset(&sys->pid_arm_theta);
                sys->theta_setpoint_rad = theta_pos;
                sys->state = FURUTA_STATE_BALANCE_PID;
            }
            break;
        }

        case FURUTA_STATE_CHARACTERIZATION: {
            Caracterizacion_Update_1kHz(&sys->char_mgr,
                                        &sys->enc_arm,
                                        &sys->enc_pend,
                                        &sys->motor,
                                        &sys->pid_motor_vel,
                                        &sys->pid_motor_pos);
            sys->u_total_volts = sys->motor.current_voltage_cmd;
            if (sys->char_mgr.test_finished) {
                sys->state = FURUTA_STATE_IDLE;
            }
            break;
        }

        case FURUTA_STATE_MOTOR_PID_VEL: {
            /* Controlador PID de Velocidad Angular del Motor */
            sys->u_total_volts = PID_Compute(&sys->pid_motor_vel,
                                             sys->omega_setpoint_rad_s,
                                             theta_vel);
            L298N_SetVoltage(&sys->motor, sys->u_total_volts);
            break;
        }

        case FURUTA_STATE_MOTOR_PID_POS: {
            /* Controlador PID de Posición Angular del Motor / Brazo (Modo Servo Error 0.00°)
             * Implementa:
             *   1. Histéresis de llegada con Freno Electromagnético Activo (L298N_Brake)
             *      al entrar en |pos_err| <= 0.018° (0.00° exactos en telemetría), evitando
             *      que la tensión de los cables en ±180° devuelva el brazo.
             *   2. Reinicio de Clegg (Zero-Crossing Integrator Reset): cuando el error
             *      cambia de signo al cruzar la referencia, descarga i_term = 0 de inmediato
             *      para que la integral acumulada jamás cancele a la acción proporcional.
             *   3. Ventana de activación integral en |pos_err| < 8.0° con autoridad hasta
             *      ±12.0 V (±100% PWM) para vencer cualquier fricción seca o tensión de cables. */
            float pos_err = sys->theta_setpoint_rad - theta_pos;
            float abs_err = fabsf(pos_err);

            if (s_pos_locked) {
                if (abs_err > (0.10f * DEG_TO_RAD)) {
                    s_pos_locked = false;
                } else {
                    sys->pid_motor_pos.i_term = 0.0f;
                    sys->pid_motor_pos.d_term = 0.0f;
                    sys->pid_motor_pos.prev_error = pos_err;
                    sys->u_total_volts = 0.0f;
                    L298N_Brake(&sys->motor);
                    break;
                }
            }

            if (abs_err <= (0.045f * DEG_TO_RAD) && fabsf(theta_vel) < 0.12f) {
                s_pos_locked = true;
                s_u_slewed_volts = 0.0f;
                sys->pid_motor_pos.i_term = 0.0f;
                sys->pid_motor_pos.d_term = 0.0f;
                sys->pid_motor_pos.prev_error = pos_err;
                sys->u_total_volts = 0.0f;
                L298N_Brake(&sys->motor);
            } else {
                /* Reinicio de Clegg: si el error cruzó por cero, limpiar integrador */
                if ((pos_err * sys->pid_motor_pos.prev_error) < 0.0f) {
                    sys->pid_motor_pos.i_term = 0.0f;
                }

                /* Fuera de ±5.0° mantener integrador en cero para 0% windup en viajes largos */
                if (abs_err > (5.0f * DEG_TO_RAD)) {
                    sys->pid_motor_pos.i_term = 0.0f;
                } else {
                    sys->pid_motor_pos.i_term += sys->pid_motor_pos.Ki * TS_CONTROL_SEC * pos_err;
                    if (sys->pid_motor_pos.i_term > 6.5f) {
                        sys->pid_motor_pos.i_term = 6.5f;
                    } else if (sys->pid_motor_pos.i_term < -6.5f) {
                        sys->pid_motor_pos.i_term = -6.5f;
                    }
                }

                sys->pid_motor_pos.p_term = sys->pid_motor_pos.Kp * pos_err;
                /* Amortiguamiento derivativo limpio usando velocidad filtrada IIR del brazo */
                sys->pid_motor_pos.d_term = -sys->pid_motor_pos.Kd * theta_vel;
                sys->pid_motor_pos.prev_error = pos_err;
                sys->pid_motor_pos.prev_measurement = theta_pos;

                float u_cmd = sys->pid_motor_pos.p_term
                            + sys->pid_motor_pos.i_term
                            + sys->pid_motor_pos.d_term;

                sys->u_total_volts = Furuta_ApplySmoothSlew(u_cmd, 0.055f, 5.8f);
                L298N_SetVoltage(&sys->motor, sys->u_total_volts);
            }
            break;
        }

        case FURUTA_STATE_SWING_UP: {
            /* Corte inmediato de seguridad si falla I2C2 o si el brazo supera ±85° */
            if (s_i2c_pend_fail_streak > 6U ||
                fabsf(theta_pos - sys->theta_setpoint_rad) > FURUTA_MAX_ARM_ANGLE_RAD ||
                fabsf(theta_pos) > (100.0f * DEG_TO_RAD)) {
                L298N_Coast(&sys->motor);
                s_u_slewed_volts = 0.0f;
                sys->state = FURUTA_STATE_FAULT_STOP;
                break;
            }

            /* Verificar condición de entrada suave al cono de atracción lineal (±16°, |vel| < 3.5 rad/s) */
            if (fabsf(alpha_err) < FURUTA_CAPTURE_ANGLE_RAD &&
                fabsf(alpha_vel) < FURUTA_CAPTURE_VEL_RAD_S) {
                PID_Reset(&sys->pid_pend_alpha);
                PID_Reset(&sys->pid_arm_theta);
                sys->state = FURUTA_STATE_BALANCE_PID;
                break;
            }

            sys->u_total_volts = Furuta_ComputeSwingUpVoltage(sys);
            L298N_SetVoltage(&sys->motor, sys->u_total_volts);
            break;
        }

        case FURUTA_STATE_BALANCE_PID: {
            /* 1. Watchdog de Seguridad Angular:
             *    Corta inmediatamente si el péndulo cae más de ±24°, si el brazo gira más de ±85°,
             *    o si el sensor I2C2 pierde lectura por más de 6 ms */
            if (s_i2c_pend_fail_streak > 6U ||
                fabsf(alpha_err) > FURUTA_WATCHDOG_ANGLE_RAD ||
                fabsf(theta_pos - sys->theta_setpoint_rad) > FURUTA_MAX_ARM_ANGLE_RAD ||
                fabsf(theta_pos) > (100.0f * DEG_TO_RAD)) {
                L298N_Coast(&sys->motor);
                PID_Reset(&sys->pid_pend_alpha);
                PID_Reset(&sys->pid_arm_theta);
                s_u_slewed_volts   = 0.0f;
                sys->u_alpha_volts = 0.0f;
                sys->u_theta_volts = 0.0f;
                sys->u_total_volts = 0.0f;
                sys->state         = FURUTA_STATE_FAULT_STOP;
                break;
            }

            /* 2. Cálculo del Lazo Principal: PID del Ángulo del Péndulo (alpha_err -> 0 rad arriba)
             *    Usando directamente la velocidad angular filtrada IIR (alpha_vel) para evitar
             *    el ruido de cuantización de diferencias finitas a 1 kHz, más Reinicio de Clegg: */
            if ((alpha_err * sys->pid_pend_alpha.prev_error) < 0.0f) {
                sys->pid_pend_alpha.i_term = 0.0f;
            }
            sys->pid_pend_alpha.i_term += sys->pid_pend_alpha.Ki * TS_CONTROL_SEC * alpha_err;
            if (sys->pid_pend_alpha.i_term > 3.5f) {
                sys->pid_pend_alpha.i_term = 3.5f;
            } else if (sys->pid_pend_alpha.i_term < -3.5f) {
                sys->pid_pend_alpha.i_term = -3.5f;
            }
            sys->pid_pend_alpha.p_term = sys->pid_pend_alpha.Kp * alpha_err;
            sys->pid_pend_alpha.d_term = sys->pid_pend_alpha.Kd * alpha_vel;
            sys->pid_pend_alpha.prev_error = alpha_err;
            sys->u_alpha_volts = sys->pid_pend_alpha.p_term
                               + sys->pid_pend_alpha.i_term
                               + sys->pid_pend_alpha.d_term;

            /* 3. Cálculo del Lazo Secundario: PID de Regulación del Brazo (theta -> theta_ref)
             *    Usando directamente la velocidad filtrada IIR del brazo (theta_vel): */
            float arm_bal_err = sys->theta_setpoint_rad - theta_pos;
            sys->pid_arm_theta.i_term += sys->pid_arm_theta.Ki * TS_CONTROL_SEC * arm_bal_err;
            if (sys->pid_arm_theta.i_term > 2.5f) {
                sys->pid_arm_theta.i_term = 2.5f;
            } else if (sys->pid_arm_theta.i_term < -2.5f) {
                sys->pid_arm_theta.i_term = -2.5f;
            }
            sys->pid_arm_theta.p_term = sys->pid_arm_theta.Kp * arm_bal_err;
            sys->pid_arm_theta.d_term = -sys->pid_arm_theta.Kd * theta_vel;
            sys->pid_arm_theta.prev_error = arm_bal_err;
            sys->u_theta_volts = sys->pid_arm_theta.p_term
                               + sys->pid_arm_theta.i_term
                               + sys->pid_arm_theta.d_term;

            /* 4. Superposición de Esfuerzos de Control con Limitador Suave sin retardo de fase (±6.2 V máx) */
            float u_bal_raw = sys->u_alpha_volts + sys->u_theta_volts;
            sys->u_total_volts = Furuta_ApplySmoothSlew(u_bal_raw, 0.22f, 6.2f);

            /* 5. Escritura al puente H L298N con compensación inversa de zona muerta */
            L298N_SetVoltage(&sys->motor, sys->u_total_volts);
            break;
        }

        default:
            L298N_Coast(&sys->motor);
            break;
    }
}

/**
 * @brief Envía una trama CSV de telemetría por USART2 (ST-LINK Virtual COM Port):
 *        Formato: t_ms,state,pwm_pct,u_volts,theta_rad,omega_rad_s,alpha_rad,alpha_err_rad,alpha_dot_rad_s\r\n
 */
void Furuta_SendTelemetry_UART(Furuta_System_t *sys)
{
    char tx_buf[160];
    int len = snprintf(tx_buf, sizeof(tx_buf),
                       "%lu,%d,%.2f,%.3f,%.5f,%.4f,%.4f,%.4f,%.4f\r\n",
                       (unsigned long)sys->tick_counter_1khz,
                       (int)sys->state,
                       sys->motor.current_pwm_pct,
                       sys->u_total_volts,
                       sys->enc_arm.angle_rad,
                       sys->enc_arm.velocity_rad_s,
                       sys->enc_pend.angle_wrapped_rad,
                       sys->enc_pend.angle_inverted_err_rad,
                       sys->enc_pend.velocity_rad_s);

    if (len > 0) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, (uint16_t)len, 25U);
    }
}

/**
 * @brief Procesa comandos ASCII recibidos por UART2 para cambiar de modo,
 *        lanzar ensayos de caracterización o sintonizar ganancias PID en vivo.
 *
 *        Comandos soportados:
 *          STOP         -> Parada inmediata (IDLE)
 *          ZERO         -> Calibrar cero de ambos encoders AS5600 (con péndulo colgado)
 *          BALANCE      -> Activar control PID Dual del Péndulo de Furuta
 *          SWING        -> Activar levantamiento Swing-Up + PID Dual
 *          CHAR1        -> Ensayo 1: Rampa de Zona Muerta del Motor + L298N
 *          CHAR2        -> Ensayo 2: Escalones Multinivel PWM del Motor
 *          CHAR3        -> Ensayo 3: Oscilación Libre del Péndulo (wn, zeta, J1, b1)
 *          CHAR4        -> Ensayo 4: Doblete de Acoplamiento Brazo -> Péndulo
 *          CHAR5A       -> Ensayo 5A: Validación PID de Velocidad del Motor
 *          CHAR5B       -> Ensayo 5B: Validación PID de Posición del Motor
 *          VEL=<val>    -> Modo PID de Velocidad del Motor con setpoint <val> rad/s
 *          POS=<val>    -> Modo PID de Posición del Motor con setpoint <val> grados
 *          PID_A=<kp>,<ki>,<kd>,<n> -> Actualizar ganancias PID del Péndulo
 *          PID_T=<kp>,<ki>,<kd>,<n> -> Actualizar ganancias PID del Brazo
 *          PID_V=<kp>,<ki>,<kd>,<n> -> Actualizar ganancias PID Velocidad Motor
 */
void Furuta_ProcessCommand(Furuta_System_t *sys, const char *cmd_str)
{
    /* Ignorar espacios iniciales */
    while (*cmd_str == ' ' || *cmd_str == '\t') {
        cmd_str++;
    }
    if (*cmd_str == '\0') {
        return;
    }

    if (strncmp(cmd_str, "STOP", 4) == 0 || (cmd_str[0] == 'S' && cmd_str[1] == '\0')) {
        s_pos_locked = false;
        sys->state = FURUTA_STATE_IDLE;
        L298N_Coast(&sys->motor);
    } else if (strncmp(cmd_str, "DIAG", 4) == 0 || (cmd_str[0] == 'D' && cmd_str[1] == '\0')) {
        char dbuf[256];
        int n = snprintf(dbuf, sizeof(dbuf),
            "#DIAG I2C1_Motor: St=0x%02X,AGC=%u,Raw=%u,Zero=%u,Err=%lu | I2C2_Pend: St=0x%02X,AGC=%u,Raw=%u,Zero=%u,Err=%lu\r\n",
            sys->enc_arm.magnet_status, sys->enc_arm.agc_value, sys->enc_arm.raw_counts, sys->enc_arm.zero_offset_counts, (unsigned long)sys->enc_arm.i2c_error_count,
            sys->enc_pend.magnet_status, sys->enc_pend.agc_value, sys->enc_pend.raw_counts, sys->enc_pend.zero_offset_counts, (unsigned long)sys->enc_pend.i2c_error_count);
        if (n > 0) {
            (void)HAL_UART_Transmit(&huart2, (uint8_t *)dbuf, (uint16_t)n, 60U);
        }
    } else if (strncmp(cmd_str, "ZERO", 4) == 0 || (cmd_str[0] == 'Z' && cmd_str[1] == '\0')) {
        s_pos_locked = false;
        sys->state = FURUTA_STATE_CALIBRATING;
        L298N_Coast(&sys->motor);
        (void)AS5600_CalibrateZero(&sys->enc_arm,  64U);
        (void)AS5600_CalibrateZero(&sys->enc_pend, 64U);
        sys->theta_setpoint_rad = 0.0f;
        sys->state = FURUTA_STATE_IDLE;
    } else if (strncmp(cmd_str, "BALANCE", 7) == 0 || (cmd_str[0] == 'B' && cmd_str[1] == '\0')) {
        PID_Reset(&sys->pid_pend_alpha);
        PID_Reset(&sys->pid_arm_theta);
        s_u_slewed_volts          = 0.0f;
        sys->theta_setpoint_rad   = sys->enc_arm.angle_rad;
        sys->enable_auto_swingup  = false;
        sys->state                = FURUTA_STATE_BALANCE_PID;
    } else if (strncmp(cmd_str, "SWING", 5) == 0 || (cmd_str[0] == 'W' && cmd_str[1] == '\0')) {
        PID_Reset(&sys->pid_pend_alpha);
        PID_Reset(&sys->pid_arm_theta);
        s_pos_locked              = false;
        s_u_slewed_volts          = 0.0f;
        s_swing_elapsed_ms        = 0U;
        sys->theta_setpoint_rad   = sys->enc_arm.angle_rad;
        sys->enable_auto_swingup  = true;
        sys->state                = FURUTA_STATE_SWING_UP;
    } else if (strncmp(cmd_str, "CHAR1", 5) == 0 || (cmd_str[0] == '1' && cmd_str[1] == '\0')) {
        Caracterizacion_StartMode(&sys->char_mgr, CHAR_MODE_DEADZONE_RAMP,
                                  &sys->motor, &sys->pid_motor_vel, &sys->pid_motor_pos);
        sys->state = FURUTA_STATE_CHARACTERIZATION;
    } else if (strncmp(cmd_str, "CHAR2", 5) == 0 || (cmd_str[0] == '2' && cmd_str[1] == '\0')) {
        Caracterizacion_StartMode(&sys->char_mgr, CHAR_MODE_MOTOR_STEPS,
                                  &sys->motor, &sys->pid_motor_vel, &sys->pid_motor_pos);
        sys->state = FURUTA_STATE_CHARACTERIZATION;
    } else if (strncmp(cmd_str, "CHAR3", 5) == 0 || (cmd_str[0] == '3' && cmd_str[1] == '\0')) {
        Caracterizacion_StartMode(&sys->char_mgr, CHAR_MODE_PENDULUM_FREE_DECAY,
                                  &sys->motor, &sys->pid_motor_vel, &sys->pid_motor_pos);
        sys->state = FURUTA_STATE_CHARACTERIZATION;
    } else if (strncmp(cmd_str, "CHAR4", 5) == 0 || (cmd_str[0] == '4' && cmd_str[1] == '\0')) {
        Caracterizacion_StartMode(&sys->char_mgr, CHAR_MODE_COUPLED_DOUBLET,
                                  &sys->motor, &sys->pid_motor_vel, &sys->pid_motor_pos);
        sys->state = FURUTA_STATE_CHARACTERIZATION;
    } else if (strncmp(cmd_str, "CHAR5A", 6) == 0 || (cmd_str[0] == '5' && cmd_str[1] == '\0')) {
        Caracterizacion_StartMode(&sys->char_mgr, CHAR_MODE_MOTOR_PID_VELOCITY,
                                  &sys->motor, &sys->pid_motor_vel, &sys->pid_motor_pos);
        sys->state = FURUTA_STATE_CHARACTERIZATION;
    } else if (strncmp(cmd_str, "CHAR5B", 6) == 0 || (cmd_str[0] == '6' && cmd_str[1] == '\0')) {
        Caracterizacion_StartMode(&sys->char_mgr, CHAR_MODE_MOTOR_PID_POSITION,
                                  &sys->motor, &sys->pid_motor_vel, &sys->pid_motor_pos);
        sys->state = FURUTA_STATE_CHARACTERIZATION;
    } else if (strstr(cmd_str, "VEL=") != NULL) {
        const char *p = strstr(cmd_str, "VEL=") + 4;
        sys->omega_setpoint_rad_s = strtof(p, NULL);
        PID_Reset(&sys->pid_motor_vel);
        sys->state = FURUTA_STATE_MOTOR_PID_VEL;
    } else if (strstr(cmd_str, "POS=") != NULL) {
        const char *p = strstr(cmd_str, "POS=") + 4;
        float deg = strtof(p, NULL);
        float new_sp_rad = deg * DEG_TO_RAD;
        if (fabsf(new_sp_rad - sys->theta_setpoint_rad) > 1e-4f || sys->state != FURUTA_STATE_MOTOR_PID_POS) {
            sys->theta_setpoint_rad = new_sp_rad;
            s_pos_locked = false;
            sys->pid_motor_pos.i_term = 0.0f;
            sys->pid_motor_pos.d_term = 0.0f;
            sys->pid_motor_pos.prev_measurement = sys->enc_arm.angle_rad;
            sys->pid_motor_pos.prev_error = sys->theta_setpoint_rad - sys->enc_arm.angle_rad;
            sys->pid_motor_pos.is_first_run = false;
            sys->state = FURUTA_STATE_MOTOR_PID_POS;
        }
    } else if (strncmp(cmd_str, "PID_A=", 6) == 0) {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, n = 25.0f;
        if (sscanf(cmd_str + 6, "%f,%f,%f,%f", &kp, &ki, &kd, &n) >= 3) {
            PID_SetGains(&sys->pid_pend_alpha, kp, ki, kd, n);
        }
    } else if (strncmp(cmd_str, "PID_T=", 6) == 0) {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, n = 20.0f;
        if (sscanf(cmd_str + 6, "%f,%f,%f,%f", &kp, &ki, &kd, &n) >= 3) {
            PID_SetGains(&sys->pid_arm_theta, kp, ki, kd, n);
        }
    } else if (strncmp(cmd_str, "PID_V=", 6) == 0) {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, n = 40.0f;
        if (sscanf(cmd_str + 6, "%f,%f,%f,%f", &kp, &ki, &kd, &n) >= 3) {
            PID_SetGains(&sys->pid_motor_vel, kp, ki, kd, n);
        }
    } else if (strncmp(cmd_str, "PID_P=", 6) == 0) {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, n = 35.0f;
        if (sscanf(cmd_str + 6, "%f,%f,%f,%f", &kp, &ki, &kd, &n) >= 3) {
            PID_SetGains(&sys->pid_motor_pos, kp, ki, kd, n);
        }
    }
}
