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
         *           SUAVES Y SIMÉTRICOS IDA Y VUELTA (MÁX ±35% PWM, ±30° GIRO)
         * Aplica pares simétricos (+PWM para ir y -PWM para volver al centro)
         * en niveles de ±20%, ±25%, ±30% y ±35% separados por pausas de 450 ms.
         * ================================================================== */
        case CHAR_MODE_MOTOR_STEPS: {
            float pwm_cmd = 0.0f;

            if (t_ms < 300U) {
                pwm_cmd = 0.0f;         /* 0.00 - 0.30 s: Reposo inicial */
            } else if (t_ms < 480U) {
                pwm_cmd = 20.0f;        /* 0.30 - 0.48 s: Escalón +20% PWM (180 ms) */
            } else if (t_ms < 930U) {
                pwm_cmd = 0.0f;         /* 0.48 - 0.93 s: Reposo */
            } else if (t_ms < 1110U) {
                pwm_cmd = -20.0f;       /* 0.93 - 1.11 s: Escalón -20% PWM (180 ms - regresa) */
            } else if (t_ms < 1560U) {
                pwm_cmd = 0.0f;         /* 1.11 - 1.56 s: Reposo */
            } else if (t_ms < 1730U) {
                pwm_cmd = 25.0f;        /* 1.56 - 1.73 s: Escalón +25% PWM (170 ms) */
            } else if (t_ms < 2180U) {
                pwm_cmd = 0.0f;         /* 1.73 - 2.18 s: Reposo */
            } else if (t_ms < 2350U) {
                pwm_cmd = -25.0f;       /* 2.18 - 2.35 s: Escalón -25% PWM (170 ms - regresa) */
            } else if (t_ms < 2800U) {
                pwm_cmd = 0.0f;         /* 2.35 - 2.80 s: Reposo */
            } else if (t_ms < 2960U) {
                pwm_cmd = 30.0f;        /* 2.80 - 2.96 s: Escalón +30% PWM (160 ms) */
            } else if (t_ms < 3410U) {
                pwm_cmd = 0.0f;         /* 2.96 - 3.41 s: Reposo */
            } else if (t_ms < 3570U) {
                pwm_cmd = -30.0f;       /* 3.41 - 3.57 s: Escalón -30% PWM (160 ms - regresa) */
            } else if (t_ms < 4020U) {
                pwm_cmd = 0.0f;         /* 3.57 - 4.02 s: Reposo */
            } else if (t_ms < 4170U) {
                pwm_cmd = 35.0f;        /* 4.02 - 4.17 s: Escalón +35% PWM (150 ms) */
            } else if (t_ms < 4620U) {
                pwm_cmd = 0.0f;         /* 4.17 - 4.62 s: Reposo */
            } else if (t_ms < 4770U) {
                pwm_cmd = -35.0f;       /* 4.62 - 4.77 s: Escalón -35% PWM (150 ms - regresa) */
            } else if (t_ms < 5200U) {
                pwm_cmd = 0.0f;         /* 4.77 - 5.20 s: Reposo final */
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
         * ENSAYO 4: CARACTERIZACIÓN SUAVE DEL SISTEMA ACOPLADO SIN CONTRAPESO
         *           (DOBLETE SUAVE BRAZO -> PÉNDULO + DECAY LIBRE CON FRENO)
         * Aplica dos dobletes bipolares suaves al motor (+28%/-28% por 150 ms
         * y -34%/+34% por 160 ms, excursión < ±35°) para excitar el péndulo
         * ligero sin golpear la estructura y luego bloquea el brazo con
         * L298N_Brake() para capturar 4.5 s de oscilación libre subamortiguada.
         * ================================================================== */
        case CHAR_MODE_COUPLED_DOUBLET: {
            if (t_ms < 400U) {
                L298N_Brake(motor);
            } else if (t_ms < 550U) {
                L298N_SetRawPWM(motor, 28.0f);      /* Pulso suave +28% (150 ms) */
            } else if (t_ms < 700U) {
                L298N_SetRawPWM(motor, -28.0f);     /* Pulso suave -28% (150 ms - regresa a 0°) */
            } else if (t_ms < 2600U) {
                L298N_Brake(motor);                 /* Brazo bloqueado: oscilación libre 1.90 s */
            } else if (t_ms < 2760U) {
                L298N_SetRawPWM(motor, -34.0f);     /* Pulso suave -34% (160 ms) */
            } else if (t_ms < 2920U) {
                L298N_SetRawPWM(motor, 34.0f);      /* Pulso suave +34% (160 ms - regresa a 0°) */
            } else if (t_ms < 7500U) {
                L298N_Brake(motor);                 /* Brazo bloqueado: decay libre 4.58 s */
            } else {
                L298N_Coast(motor);
                mgr->test_finished = true;
            }
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
