/**
 ******************************************************************************
 * @file    caracterizacion.h
 * @brief   Módulo de Caracterización Experimental del Motor Namiki 22CL y del
 *          Sistema Péndulo de Furuta, incluyendo validación en lazo cerrado
 *          de los controladores PID del motor (velocidad y posición).
 ******************************************************************************
 */

#ifndef __CARACTERIZACION_H
#define __CARACTERIZACION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "as5600.h"
#include "l298n_motor.h"
#include "pid_controller.h"

/**
 * @brief Modos de ensayo de caracterización experimental
 */
typedef enum {
    CHAR_MODE_NONE = 0,             /* Sin ensayo activo (reposo) */
    CHAR_MODE_DEADZONE_RAMP,        /* Ensayo 1: Barrido lento PWM para medir zona muerta */
    CHAR_MODE_MOTOR_STEPS,          /* Ensayo 2: Escalones multinivel PWM -> Gv(s), Gp(s) */
    CHAR_MODE_PENDULUM_FREE_DECAY,  /* Ensayo 3: Oscilación libre péndulo -> wn, zeta, J1, b1 */
    CHAR_MODE_COUPLED_DOUBLET,      /* Ensayo 4: Doblete de aceleración PWM -> G_alpha_u(s) */
    CHAR_MODE_MOTOR_PID_VELOCITY,   /* Ensayo 5A: Validación PID de Velocidad del Motor */
    CHAR_MODE_MOTOR_PID_POSITION    /* Ensayo 5B: Validación PID de Posición del Motor */
} Characterization_Mode_t;

/**
 * @brief Estructura de control de ensayos de caracterización
 */
typedef struct {
    Characterization_Mode_t active_mode;
    uint32_t elapsed_ticks_ms;      /* Tiempo transcurrido del ensayo actual (ms) */
    bool test_finished;             /* Flag de finalización del protocolo */
    bool trigger_detected;          /* Flag de disparo para oscilación libre */

    /* Resultados medidos en vivo en el microcontrolador */
    float detected_deadzone_pos_pct;
    float detected_deadzone_neg_pct;
    bool deadzone_pos_found;
    bool deadzone_neg_found;

    /* Referencias generadas para pruebas de PID del motor */
    float target_omega_rad_s;       /* Setpoint para PID de velocidad del motor */
    float target_theta_rad;         /* Setpoint para PID de posición del motor */
} Characterization_Manager_t;

/* Prototipos de Funciones Públicas ------------------------------------------*/
void Caracterizacion_Init(Characterization_Manager_t *mgr);
void Caracterizacion_StartMode(Characterization_Manager_t *mgr,
                               Characterization_Mode_t mode,
                               L298N_Handle_t *motor,
                               PID_Controller_t *pid_vel,
                               PID_Controller_t *pid_pos);

void Caracterizacion_Update_1kHz(Characterization_Manager_t *mgr,
                                 AS5600_Handle_t *enc_arm,
                                 AS5600_Handle_t *enc_pend,
                                 L298N_Handle_t *motor,
                                 PID_Controller_t *pid_vel,
                                 PID_Controller_t *pid_pos);

#ifdef __cplusplus
}
#endif

#endif /* __CARACTERIZACION_H */
