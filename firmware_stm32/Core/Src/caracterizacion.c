/**
 ******************************************************************************
 * @file    caracterizacion.c
 * @brief   Implementación de los protocolos de caracterización experimental
 *          del Motor Namiki 22CL-3501PG y del Péndulo de Furuta, así como la
 *          validación experimental de los controladores PID del motor
 *          (Lazo de Velocidad y Lazo de Posición).
 ******************************************************************************
 */

#include "caracterizacion.h"

/**
 * @brief Inicializa el gestor de ensayos de caracterización
 */
void Caracterizacion_Init(Characterization_Manager_t *mgr)
{
    mgr->active_mode               = CHAR_MODE_NONE;
    mgr->elapsed_ticks_ms          = 0U;
    mgr->test_finished             = false;
    mgr->trigger_detected          = false;

    mgr->detected_deadzone_pos_pct = MOTOR_DEFAULT_DEADZONE_PCT;
    mgr->detected_deadzone_neg_pct = MOTOR_DEFAULT_DEADZONE_PCT;
    mgr->deadzone_pos_found        = false;
    mgr->deadzone_neg_found        = false;

    mgr->target_omega_rad_s        = 0.0f;
    mgr->target_theta_rad          = 0.0f;
}

/**
 * @brief Inicia un protocolo específico de caracterización o validación PID
 */
void Caracterizacion_StartMode(Characterization_Manager_t *mgr,
                               Characterization_Mode_t mode,
                               L298N_Handle_t *motor,
                               PID_Controller_t *pid_vel,
                               PID_Controller_t *pid_pos)
{
    mgr->active_mode        = mode;
    mgr->elapsed_ticks_ms   = 0U;
    mgr->test_finished      = false;
    mgr->trigger_detected   = false;
    mgr->target_omega_rad_s = 0.0f;
    mgr->target_theta_rad   = 0.0f;

    if (mode == CHAR_MODE_DEADZONE_RAMP) {
        mgr->deadzone_pos_found = false;
        mgr->deadzone_neg_found = false;
    }

    PID_Reset(pid_vel);
    PID_Reset(pid_pos);
    L298N_Coast(motor);
}

/**
 * @brief Máquina de estados de caracterización ejecutada cada 1 ms (1 kHz)
 */
void Caracterizacion_Update_1kHz(Characterization_Manager_t *mgr,
                                 AS5600_Handle_t *enc_arm,
                                 AS5600_Handle_t *enc_pend,
                                 L298N_Handle_t *motor,
                                 PID_Controller_t *pid_vel,
                                 PID_Controller_t *pid_pos)
{
    if (mgr->active_mode == CHAR_MODE_NONE || mgr->test_finished) {
        L298N_Coast(motor);
        return;
    }

    uint32_t t_ms = mgr->elapsed_ticks_ms;

    switch (mgr->active_mode) {

        /* ==================================================================
         * ENSAYO 1: CARACTERIZACIÓN DE ZONA MUERTA (DEAD-ZONE RAMP)
         * - 0 a 10 s: Incrementa PWM de 0% a +50% a razón de 1% cada 200 ms.
         * - 10 a 12 s: Reposo en 0% PWM.
         * - 12 a 22 s: Decrementa PWM de 0% a -50% a razón de -1% cada 200 ms.
         * - Detecta automáticamente el umbral donde |omega| > 0.35 rad/s.
         * ================================================================== */
        case CHAR_MODE_DEADZONE_RAMP: {
            float pwm_cmd = 0.0f;

            if (t_ms < 10000U) {
                uint32_t step_idx = t_ms / 200U; /* 0..50 */
                pwm_cmd = (float)step_idx * 1.0f;
                if (!mgr->deadzone_pos_found && fabsf(enc_arm->velocity_rad_s) > 0.35f) {
                    mgr->detected_deadzone_pos_pct = pwm_cmd;
                    mgr->deadzone_pos_found = true;
                }
            } else if (t_ms < 12000U) {
                pwm_cmd = 0.0f;
            } else if (t_ms < 22000U) {
                uint32_t step_idx = (t_ms - 12000U) / 200U; /* 0..50 */
                pwm_cmd = -((float)step_idx * 1.0f);
                if (!mgr->deadzone_neg_found && fabsf(enc_arm->velocity_rad_s) > 0.35f) {
                    mgr->detected_deadzone_neg_pct = fabsf(pwm_cmd);
                    mgr->deadzone_neg_found = true;
                }
            } else {
                pwm_cmd = 0.0f;
                L298N_SetDeadzone(motor,
                                  mgr->detected_deadzone_pos_pct,
                                  mgr->detected_deadzone_neg_pct,
                                  true);
                mgr->test_finished = true;
            }

            L298N_SetRawPWM(motor, pwm_cmd);
            break;
        }

        /* ==================================================================
         * ENSAYO 2: IDENTIFICACIÓN DEL MOTOR POR ESCALONES MULTINIVEL
         * Excita el motor con escalones bidireccionales (+30%, +45%, +60%,
         * -30%, -45%) separados por reposos para identificar:
         *   Gv(s) = Omega(s)/U(s) = Km / (tau_m * s + 1)
         *   Gp(s) = Theta(s)/U(s) = Km / (s * (tau_m * s + 1))
         * ================================================================== */
        case CHAR_MODE_MOTOR_STEPS: {
            float pwm_cmd = 0.0f;

            if (t_ms < 1000U) {
                pwm_cmd = 0.0f;         /* 0.0 - 1.0 s: Reposo inicial */
            } else if (t_ms < 3500U) {
                pwm_cmd = 30.0f;        /* 1.0 - 3.5 s: Escalón +30% PWM */
            } else if (t_ms < 5000U) {
                pwm_cmd = 0.0f;         /* 3.5 - 5.0 s: Reposo */
            } else if (t_ms < 7500U) {
                pwm_cmd = 45.0f;        /* 5.0 - 7.5 s: Escalón +45% PWM */
            } else if (t_ms < 9000U) {
                pwm_cmd = 0.0f;         /* 7.5 - 9.0 s: Reposo */
            } else if (t_ms < 11500U) {
                pwm_cmd = 60.0f;        /* 9.0 - 11.5 s: Escalón +60% PWM */
            } else if (t_ms < 13000U) {
                pwm_cmd = 0.0f;         /* 11.5 - 13.0 s: Reposo */
            } else if (t_ms < 15500U) {
                pwm_cmd = -30.0f;       /* 13.0 - 15.5 s: Escalón -30% PWM */
            } else if (t_ms < 17000U) {
                pwm_cmd = 0.0f;         /* 15.5 - 17.0 s: Reposo */
            } else if (t_ms < 19500U) {
                pwm_cmd = -45.0f;       /* 17.0 - 19.5 s: Escalón -45% PWM */
            } else {
                pwm_cmd = 0.0f;
                mgr->test_finished = true;
            }

            L298N_SetRawPWM(motor, pwm_cmd);
            break;
        }

        /* ==================================================================
         * ENSAYO 3: OSCILACIÓN LIBRE DEL PÉNDULO (CAJA NEGRA ENSAYO A)
         * Bloquea electromagnéticamente el motor (brazo fijo) y captura 10 s
         * de oscilaciones libres subamortiguadas alpha(t) al soltar la barra
         * desde ~15°-20° para identificar wn, zeta, J1 y b1.
         * ================================================================== */
        case CHAR_MODE_PENDULUM_FREE_DECAY: {
            L298N_Brake(motor);
            (void)enc_pend;

            if (!mgr->trigger_detected) {
                /* Esperar a que el usuario incline el péndulo > 12° (0.21 rad) */
                if (fabsf(enc_pend->angle_wrapped_rad) > 0.21f) {
                    mgr->trigger_detected = true;
                    mgr->elapsed_ticks_ms = 0U;
                    return;
                }
            } else {
                if (t_ms >= 10000U) {
                    L298N_Coast(motor);
                    mgr->test_finished = true;
                }
            }
            break;
        }

        /* ==================================================================
         * ENSAYO 4: DOBLETE DE ACOPLAMIENTO BRAZO -> PÉNDULO (ENSAYO B)
         * Aplica un pulso bipolar simétrico de aceleración al motor (+38% por
         * 160 ms, -38% por 160 ms, y luego 0%) con el péndulo colgado para
         * identificar la ganancia cruzada de acoplamiento inercial K_acople.
         * ================================================================== */
        case CHAR_MODE_COUPLED_DOUBLET: {
            float pwm_cmd = 0.0f;

            if (t_ms < 500U) {
                pwm_cmd = 0.0f;
            } else if (t_ms < 660U) {
                pwm_cmd = 38.0f;        /* Pulso positivo 160 ms */
            } else if (t_ms < 820U) {
                pwm_cmd = -38.0f;       /* Pulso negativo 160 ms */
            } else if (t_ms < 2500U) {
                pwm_cmd = 0.0f;
            } else if (t_ms < 2680U) {
                pwm_cmd = -45.0f;       /* Segundo doblete inverso 180 ms */
            } else if (t_ms < 2860U) {
                pwm_cmd = 45.0f;
            } else if (t_ms < 6000U) {
                pwm_cmd = 0.0f;
            } else {
                pwm_cmd = 0.0f;
                mgr->test_finished = true;
            }

            L298N_SetRawPWM(motor, pwm_cmd);
            break;
        }

        /* ==================================================================
         * ENSAYO 5A: APLICACIÓN Y VALIDACIÓN DEL CONTROLADOR PID DE VELOCIDAD
         *            DEL MOTOR (Seguidor de escalones de velocidad rad/s)
         * ================================================================== */
        case CHAR_MODE_MOTOR_PID_VELOCITY: {
            if (t_ms < 1000U) {
                mgr->target_omega_rad_s = 0.0f;
            } else if (t_ms < 4000U) {
                mgr->target_omega_rad_s = 6.0f;     /* +6.0 rad/s */
            } else if (t_ms < 7000U) {
                mgr->target_omega_rad_s = 10.0f;    /* +10.0 rad/s */
            } else if (t_ms < 10000U) {
                mgr->target_omega_rad_s = -8.0f;    /* -8.0 rad/s (Inversión) */
            } else if (t_ms < 12000U) {
                mgr->target_omega_rad_s = 0.0f;     /* Parada controlada a 0 */
            } else {
                mgr->target_omega_rad_s = 0.0f;
                L298N_Coast(motor);
                mgr->test_finished = true;
                break;
            }

            float u_volts = PID_Compute(pid_vel,
                                        mgr->target_omega_rad_s,
                                        enc_arm->velocity_rad_s);
            L298N_SetVoltage(motor, u_volts);
            break;
        }

        /* ==================================================================
         * ENSAYO 5B: APLICACIÓN Y VALIDACIÓN DEL CONTROLADOR PID DE POSICIÓN
         *            DEL MOTOR / BRAZO (Seguidor de escalones angulares rad)
         * ================================================================== */
        case CHAR_MODE_MOTOR_PID_POSITION: {
            if (t_ms < 1000U) {
                mgr->target_theta_rad = 0.0f;
            } else if (t_ms < 4000U) {
                mgr->target_theta_rad = 60.0f * DEG_TO_RAD;     /* +60° (1.047 rad) */
            } else if (t_ms < 7000U) {
                mgr->target_theta_rad = -60.0f * DEG_TO_RAD;    /* -60° (-1.047 rad) */
            } else if (t_ms < 10000U) {
                mgr->target_theta_rad = 90.0f * DEG_TO_RAD;     /* +90° (1.571 rad) */
            } else if (t_ms < 13000U) {
                mgr->target_theta_rad = 0.0f;                   /* Retorno a 0° */
            } else {
                mgr->target_theta_rad = 0.0f;
                L298N_Coast(motor);
                mgr->test_finished = true;
                break;
            }

            float u_volts = PID_Compute(pid_pos,
                                        mgr->target_theta_rad,
                                        enc_arm->angle_rad);
            L298N_SetVoltage(motor, u_volts);
            break;
        }

        default:
            L298N_Coast(motor);
            break;
    }

    mgr->elapsed_ticks_ms++;
}
