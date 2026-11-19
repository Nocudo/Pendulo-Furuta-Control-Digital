/**
 ******************************************************************************
 * @file    pid_controller.h
 * @brief   Controlador PID Digital Discreto para sistemas embebidos de tiempo
 *          real (1 kHz). Implementa:
 *            - Integración trapezoidal (Tustin)
 *            - Derivada filtrada pasabajos de primer orden (coeficiente N)
 *            - Opción de derivada sobre el error o sobre la medición
 *            - Anti-Windup por enclavamiento condicional (Clamping)
 *            - Saturación simétrica o asimétrica de salida
 ******************************************************************************
 */

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief Estructura del controlador PID discreto
 */
typedef struct {
    /* Ganancias de sintonía y parámetros temporales */
    float Kp;                       /* Ganancia proporcional */
    float Ki;                       /* Ganancia integral (1/s) */
    float Kd;                       /* Ganancia derivativa (s) */
    float N_filter;                 /* Frecuencia de corte del filtro derivativo (rad/s) */
    float Ts;                       /* Periodo de muestreo (s) */

    /* Límites de saturación del actuador */
    float out_min;                  /* Límite inferior de salida (e.g. -12.0 V) */
    float out_max;                  /* Límite superior de salida (e.g. +12.0 V) */
    bool derivative_on_measurement; /* true: evita derivative kick ante escalones de ref */

    /* Coeficientes discretos precalculados (Tustin Bilineal) */
    float c_int;                    /* 0.5 * Ki * Ts */
    float c_d1;                     /* (2 - N*Ts) / (2 + N*Ts) */
    float c_d2;                     /* (2 * Kd * N) / (2 + N*Ts) */

    /* Estados internos y memoria [k-1] */
    float p_term;                   /* Componente Proporcional actual P[k] */
    float i_term;                   /* Acumulador Integral actual I[k] */
    float d_term;                   /* Componente Derivativa filtrada actual D[k] */
    float prev_error;               /* Error en el instante anterior e[k-1] */
    float prev_measurement;         /* Medición en el instante anterior y[k-1] */
    float output;                   /* Salida de control saturada u[k] */
    bool is_first_run;              /* Evita salto derivativo en la primera iteración */
} PID_Controller_t;

/* Prototipos de Funciones Públicas ------------------------------------------*/
void PID_Init(PID_Controller_t *pid,
              float Kp,
              float Ki,
              float Kd,
              float N_filter,
              float Ts,
              float out_min,
              float out_max,
              bool derivative_on_measurement);

void PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd, float N_filter);
void PID_Reset(PID_Controller_t *pid);
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */
