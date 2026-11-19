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

/**
 * @brief Calcula la energía mecánica normalizada del péndulo respecto al
 *        punto de equilibrio superior invertido (donde E_ref = 0 J):
 *        E(alpha, d_alpha) = 0.5 * J1_piv * d_alpha^2 + m1 * g * l1 * (cos(alpha_up) - 1)
 *        Siendo alpha_up = angle_inverted_err_rad (0 rad arriba, pi rad abajo).
 */
static float Furuta_ComputePendulumEnergy(const AS5600_Handle_t *enc_pend)
{
    float alpha_up = enc_pend->angle_inverted_err_rad; /* 0 en vertical invertida */
    float d_alpha  = enc_pend->velocity_rad_s;

    float e_kin = 0.5f * FURUTA_J1_PIVOT * (d_alpha * d_alpha);
    float e_pot = FURUTA_M1_KG * FURUTA_GRAVITY * FURUTA_L1_M * (cosf(alpha_up) - 1.0f);

    return (e_kin + e_pot);
}

/**
 * @brief Ley no lineal de Swing-Up basada en Lyapunov (Åström & Furuta, 2000)
 *        Bombea energía al péndulo mientras mantiene el brazo centrado cerca de 0 rad.
 */
static float Furuta_ComputeSwingUpVoltage(Furuta_System_t *sys)
{
    float alpha_up = sys->enc_pend.angle_inverted_err_rad;
    float d_alpha  = sys->enc_pend.velocity_rad_s;
    float theta    = sys->enc_arm.angle_rad;
    float d_theta  = sys->enc_arm.velocity_rad_s;

    sys->pendulum_energy_j = Furuta_ComputePendulumEnergy(&sys->enc_pend);

    /* Signo de bombeo: sign(d_alpha * cos(alpha_up)) */
    float pump_dir = d_alpha * cosf(alpha_up);
    float sign_term = 0.0f;
    if (pump_dir > 0.05f) {
        sign_term = 1.0f;
    } else if (pump_dir < -0.05f) {
        sign_term = -1.0f;
    }

    /* Término de inyección de energía + amortiguamiento suave de centrado del brazo */
    float u_energy = FURUTA_SWINGUP_KE * sys->pendulum_energy_j * sign_term;
    float u_center = -(FURUTA_SWINGUP_KP_ARM * (theta - sys->theta_setpoint_rad))
                     -(FURUTA_SWINGUP_KV_ARM * d_theta);

    float u_cmd = u_energy + u_center;

    /* Limitar esfuerzo de swing-up a ±9.5 V para evitar impactos bruscos */
    if (u_cmd > 9.5f) {
        u_cmd = 9.5f;
    } else if (u_cmd < -9.5f) {
        u_cmd = -9.5f;
    }

    return u_cmd;
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
     *    Zona muerta asimétrica real medida con base fija a la mesa (CHAR1):
     *    +9.0% PWM (antihorario), -11.0% PWM (horario) */
    L298N_Init(&sys->motor, &htim1, TIM_CHANNEL_1);
    L298N_SetDeadzone(&sys->motor, 9.0f, 11.0f, true);

    /* 2. Inicializar Encoders Magnéticos AS5600 (I2C1 Brazo con direction=-1 y gear=80:1, I2C2 Péndulo) */
    (void)AS5600_Init(&sys->enc_arm,  &hi2c1, TS_CONTROL_SEC, 0.006f, -1);
    (void)AS5600_Init(&sys->enc_pend, &hi2c2, TS_CONTROL_SEC, 0.004f,  1);

    /* 3. Inicializar los 4 Controladores PID Discretos Optimizados (Ts = 1.0 ms = 1 kHz)
     *    Sintonizados a partir del Sistema Real Caracterizado con Base Fija:
     *      Gv_brazo(s) = 2.5208 / (0.1135 s + 1) [(rad/s)/V] (R^2 = 0.9832)
     *      G_pend(s)   = 0.8053 / (s^2 + 0.3238 s - 61.3402) (wn = 7.832 rad/s, zeta = 0.0207)
     * ----------------------------------------------------------------------
     * A) PID de Velocidad del Motor/Brazo:
     *    Kp = 1.82 V/(rad/s), Ki = 35.30 V/rad, Kd = 0.005 V·s/rad, N = 40 */
    PID_Init(&sys->pid_motor_vel,
             1.82f, 35.30f, 0.005f, 40.0f,
             TS_CONTROL_SEC, -MOTOR_NOMINAL_VOLTAGE, MOTOR_NOMINAL_VOLTAGE,
             true);

    /* B) PID de Posición del Motor/Brazo Solo (Servo de precisión con error 0.00°):
     *    Kp = 18.13 V/rad, Ki = 77.38 V/(rad·s), Kd = 1.65 V·s/rad, N = 35
     *    Rango completo [-12.0 V, +12.0 V] con Clegg Reset en cruce por cero */
    PID_Init(&sys->pid_motor_pos,
             18.13f, 77.38f, 1.65f, 35.0f,
             TS_CONTROL_SEC, -MOTOR_NOMINAL_VOLTAGE, MOTOR_NOMINAL_VOLTAGE,
             true);

    /* C) PID Estabilizador del Péndulo Invertido (Lazo Rápido Principal alpha):
     *    Kp = 61.81 V/rad, Ki = 28.11 V/(rad·s), Kd = 2.29 V·s/rad, N = 28
     *    Nota: derivative_on_measurement = false (setpoint fijo en 0.0 rad) */
    PID_Init(&sys->pid_pend_alpha,
             61.81f, 28.11f, 2.29f, 28.0f,
             TS_CONTROL_SEC, -MOTOR_NOMINAL_VOLTAGE, MOTOR_NOMINAL_VOLTAGE,
             false);

    /* D) PID Regulador de Posición del Brazo en Equilibrio Furuta (Lazo Lento theta):
     *    Kp = -1.20 V/rad, Ki = -2.00 V/(rad·s), Kd = -0.80 V·s/rad, N = 20 */
    PID_Init(&sys->pid_arm_theta,
             -1.20f, -2.00f, -0.80f, 20.0f,
             TS_CONTROL_SEC, -6.0f, 6.0f,
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
    (void)AS5600_Update(&sys->enc_pend);

    float alpha_err = sys->enc_pend.angle_inverted_err_rad;
    float alpha_vel = sys->enc_pend.velocity_rad_s;
    float theta_pos = sys->enc_arm.angle_rad;
    float theta_vel = sys->enc_arm.velocity_rad_s;

    /* 2. Evaluación de la Máquina de Estados Finitos (FSM) */
    switch (sys->state) {

        case FURUTA_STATE_IDLE:
        case FURUTA_STATE_FAULT_STOP: {
            L298N_Coast(&sys->motor);
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

                /* Fuera de ±8.0° mantener integrador en cero para 0% windup en viajes largos */
                if (abs_err > (8.0f * DEG_TO_RAD)) {
                    sys->pid_motor_pos.i_term = 0.0f;
                } else {
                    sys->pid_motor_pos.i_term += sys->pid_motor_pos.Ki * TS_CONTROL_SEC * pos_err;
                    if (sys->pid_motor_pos.i_term > MOTOR_NOMINAL_VOLTAGE) {
                        sys->pid_motor_pos.i_term = MOTOR_NOMINAL_VOLTAGE;
                    } else if (sys->pid_motor_pos.i_term < -MOTOR_NOMINAL_VOLTAGE) {
                        sys->pid_motor_pos.i_term = -MOTOR_NOMINAL_VOLTAGE;
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
                if (u_cmd > MOTOR_NOMINAL_VOLTAGE) {
                    u_cmd = MOTOR_NOMINAL_VOLTAGE;
                } else if (u_cmd < -MOTOR_NOMINAL_VOLTAGE) {
                    u_cmd = -MOTOR_NOMINAL_VOLTAGE;
                }

                sys->u_total_volts = u_cmd;
                L298N_SetVoltage(&sys->motor, sys->u_total_volts);
            }
            break;
        }

        case FURUTA_STATE_SWING_UP: {
            /* Verificar condición de entrada al cono de atracción lineal (±18°) */
            if (fabsf(alpha_err) < FURUTA_CAPTURE_ANGLE_RAD &&
                fabsf(alpha_vel) < FURUTA_CAPTURE_VEL_RAD_S) {
                PID_Reset(&sys->pid_pend_alpha);
                PID_Reset(&sys->pid_arm_theta);
                sys->theta_setpoint_rad = theta_pos;
                sys->state = FURUTA_STATE_BALANCE_PID;
                break;
            }

            /* Protección de recorrido máximo del brazo durante Swing-Up */
            if (fabsf(theta_pos - sys->theta_setpoint_rad) > FURUTA_MAX_ARM_ANGLE_RAD) {
                L298N_Coast(&sys->motor);
                sys->state = FURUTA_STATE_FAULT_STOP;
                break;
            }

            sys->u_total_volts = Furuta_ComputeSwingUpVoltage(sys);
            L298N_SetVoltage(&sys->motor, sys->u_total_volts);
            break;
        }

        case FURUTA_STATE_BALANCE_PID: {
            /* 1. Watchdog de Seguridad Angular: Si el péndulo cae más de ±25° */
            if (fabsf(alpha_err) > FURUTA_WATCHDOG_ANGLE_RAD) {
                L298N_Coast(&sys->motor);
                PID_Reset(&sys->pid_pend_alpha);
                PID_Reset(&sys->pid_arm_theta);
                sys->u_alpha_volts = 0.0f;
                sys->u_theta_volts = 0.0f;
                sys->u_total_volts = 0.0f;

                if (sys->enable_auto_swingup) {
                    sys->state = FURUTA_STATE_SWING_UP;
                } else {
                    sys->state = FURUTA_STATE_FAULT_STOP;
                }
                break;
            }

            /* 2. Cálculo del Lazo Principal: PID del Ángulo del Péndulo (alpha -> 0 rad arriba)
             *    Nota: Definimos setpoint = 0.0f y measurement = -alpha_err para que una
             *    inclinación positiva del péndulo genere una acción aceleradora del brazo
             *    en la misma dirección que "mete la base debajo del centro de masa". */
            sys->u_alpha_volts = PID_Compute(&sys->pid_pend_alpha,
                                             0.0f,
                                             -alpha_err);

            /* 3. Cálculo del Lazo Secundario: PID de Regulación del Brazo (theta -> theta_ref) */
            sys->u_theta_volts = PID_Compute(&sys->pid_arm_theta,
                                             sys->theta_setpoint_rad,
                                             theta_pos);

            /* 4. Superposición de Esfuerzos de Control (Arquitectura PID Dual Paralela) */
            sys->u_total_volts = sys->u_alpha_volts + sys->u_theta_volts;

            /* Saturación global a [-12.0 V, +12.0 V] */
            if (sys->u_total_volts > MOTOR_NOMINAL_VOLTAGE) {
                sys->u_total_volts = MOTOR_NOMINAL_VOLTAGE;
            } else if (sys->u_total_volts < -MOTOR_NOMINAL_VOLTAGE) {
                sys->u_total_volts = -MOTOR_NOMINAL_VOLTAGE;
            }

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
        sys->theta_setpoint_rad   = sys->enc_arm.angle_rad;
        sys->enable_auto_swingup  = false;
        sys->state                = FURUTA_STATE_BALANCE_PID;
    } else if (strncmp(cmd_str, "SWING", 5) == 0 || (cmd_str[0] == 'W' && cmd_str[1] == '\0')) {
        PID_Reset(&sys->pid_pend_alpha);
        PID_Reset(&sys->pid_arm_theta);
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
