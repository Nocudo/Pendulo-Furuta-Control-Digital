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
         * ENSAYO 1: CARACTERIZACIÓN DE ZONA MUERTA (DEAD-ZONE RAMP SEGURA)
         * - 0 a 4.0 s: Rampa positiva 0% a +40% (1% cada 100 ms). Corta en
         *   cuanto detecta movimiento (|theta| > 0.08 rad o |omega| > 0.35 rad/s).
         * - 4.0 a 5.0 s: Reposo en 0% PWM.
         * - 5.0 a 9.0 s: Rampa negativa 0% a -40% (-1% cada 100 ms) que
         *   regresa el brazo hacia 0 rad y detecta la zona muerta negativa.
         * ================================================================== */
        case CHAR_MODE_DEADZONE_RAMP: {
            float pwm_cmd = 0.0f;

            if (t_ms < 4000U) {
                if (!mgr->deadzone_pos_found) {
                    uint32_t step_idx = t_ms / 100U; /* 0..40% */
                    pwm_cmd = (float)step_idx * 1.0f;
                    if (fabsf(enc_arm->velocity_rad_s) > 0.35f || fabsf(enc_arm->angle_rad) > 0.06f) {
                        mgr->detected_deadzone_pos_pct = pwm_cmd;
                        mgr->deadzone_pos_found = true;
                    }
                } else {
                    pwm_cmd = 0.0f; /* Detener apenas se detecta para no enrollar cables */
                }
            } else if (t_ms < 5000U) {
                pwm_cmd = 0.0f;
            } else if (t_ms < 9000U) {
                if (!mgr->deadzone_neg_found) {
                    uint32_t step_idx = (t_ms - 5000U) / 100U; /* 0..40% */
                    pwm_cmd = -((float)step_idx * 1.0f);
                    if (fabsf(enc_arm->velocity_rad_s) > 0.35f) {
                        mgr->detected_deadzone_neg_pct = fabsf(pwm_cmd);
                        mgr->deadzone_neg_found = true;
                    }
                } else {
                    pwm_cmd = 0.0f;
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
         * ENSAYO 2: IDENTIFICACIÓN DEL MOTOR POR ESCALONES BIDIRECCIONALES
         *           SIMÉTRICOS IDA Y VUELTA (CERO ENREDO DE CABLES)
         * Aplica pares simétricos (+PWM para ir y -PWM para volver al centro)
         * en niveles de ±20%, ±30%, ±40% y ±50% separados por pausas de 450 ms.
         * ================================================================== */
        case CHAR_MODE_MOTOR_STEPS: {
            float pwm_cmd = 0.0f;

            if (t_ms < 300U) {
                pwm_cmd = 0.0f;         /* 0.00 - 0.30 s: Reposo inicial */
            } else if (t_ms < 520U) {
                pwm_cmd = 20.0f;        /* 0.30 - 0.52 s: Escalón +20% PWM (220 ms) */
            } else if (t_ms < 970U) {
                pwm_cmd = 0.0f;         /* 0.52 - 0.97 s: Reposo */
            } else if (t_ms < 1190U) {
                pwm_cmd = -20.0f;       /* 0.97 - 1.19 s: Escalón -20% PWM (220 ms - regresa) */
            } else if (t_ms < 1650U) {
                pwm_cmd = 0.0f;         /* 1.19 - 1.65 s: Reposo */
            } else if (t_ms < 1850U) {
                pwm_cmd = 30.0f;        /* 1.65 - 1.85 s: Escalón +30% PWM (200 ms) */
            } else if (t_ms < 2300U) {
                pwm_cmd = 0.0f;         /* 1.85 - 2.30 s: Reposo */
            } else if (t_ms < 2500U) {
                pwm_cmd = -30.0f;       /* 2.30 - 2.50 s: Escalón -30% PWM (200 ms - regresa) */
            } else if (t_ms < 2950U) {
                pwm_cmd = 0.0f;         /* 2.50 - 2.95 s: Reposo */
            } else if (t_ms < 3130U) {
                pwm_cmd = 40.0f;        /* 2.95 - 3.13 s: Escalón +40% PWM (180 ms) */
            } else if (t_ms < 3600U) {
                pwm_cmd = 0.0f;         /* 3.13 - 3.60 s: Reposo */
            } else if (t_ms < 3780U) {
                pwm_cmd = -40.0f;       /* 3.60 - 3.78 s: Escalón -40% PWM (180 ms - regresa) */
            } else if (t_ms < 4250U) {
                pwm_cmd = 0.0f;         /* 3.78 - 4.25 s: Reposo */
            } else if (t_ms < 4410U) {
                pwm_cmd = 50.0f;        /* 4.25 - 4.41 s: Escalón +50% PWM (160 ms) */
            } else if (t_ms < 4900U) {
                pwm_cmd = 0.0f;         /* 4.41 - 4.90 s: Reposo */
            } else if (t_ms < 5060U) {
                pwm_cmd = -50.0f;       /* 4.90 - 5.06 s: Escalón -50% PWM (160 ms - regresa) */
            } else if (t_ms < 5500U) {
                pwm_cmd = 0.0f;         /* 5.06 - 5.50 s: Reposo final */
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
