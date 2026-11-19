/**
 ******************************************************************************
 * @file    pid_controller.c
 * @brief   Implementación del controlador PID digital discreto mediante la
 *          Transformada Bilineal (Tustin):
 *            P[k] = Kp * e[k]
 *            I[k] = I[k-1] + 0.5 * Ki * Ts * (e[k] + e[k-1])  (con Anti-Windup)
 *            D[k] = c_d1 * D[k-1] + c_d2 * (delta_input)      (Filtro polo N)
 *            u[k] = sat(P[k] + I[k] + D[k], out_min, out_max)
 ******************************************************************************
 */

#include "pid_controller.h"

/**
 * @brief Recalcula los coeficientes en diferencias de Tustin cuando cambian
 *        las ganancias o el periodo de muestreo Ts
 */
static void PID_UpdateCoefficients(PID_Controller_t *pid)
{
    /* Coeficiente de integración trapezoidal */
    pid->c_int = 0.5f * pid->Ki * pid->Ts;

    /* Coeficientes del filtro derivativo D(s)/E(s) = (Kd * N * s) / (s + N)
     * Sustituyendo s = (2/Ts) * (1 - z^-1)/(1 + z^-1):
     * D[k] = ((2 - N*Ts)/(2 + N*Ts))*D[k-1] + ((2*Kd*N)/(2 + N*Ts))*(e[k] - e[k-1]) */
    float denom = 2.0f + (pid->N_filter * pid->Ts);
    if (denom > 1e-6f) {
        pid->c_d1 = (2.0f - (pid->N_filter * pid->Ts)) / denom;
        pid->c_d2 = (2.0f * pid->Kd * pid->N_filter) / denom;
    } else {
        pid->c_d1 = 0.0f;
        pid->c_d2 = 0.0f;
    }
}

/**
 * @brief Inicializa una instancia de controlador PID discreto
 */
void PID_Init(PID_Controller_t *pid,
              float Kp,
              float Ki,
              float Kd,
              float N_filter,
              float Ts,
              float out_min,
              float out_max,
              bool derivative_on_measurement)
{
    pid->Kp                        = Kp;
    pid->Ki                        = Ki;
    pid->Kd                        = Kd;
    pid->N_filter                  = (N_filter > 0.0f) ? N_filter : 20.0f;
    pid->Ts                        = (Ts > 0.0f) ? Ts : TS_CONTROL_SEC;
    pid->out_min                   = out_min;
    pid->out_max                   = out_max;
    pid->derivative_on_measurement = derivative_on_measurement;

    PID_UpdateCoefficients(pid);
    PID_Reset(pid);
}

/**
 * @brief Modifica las ganancias del PID en caliente (sin detener el lazo)
 */
void PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd, float N_filter)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    if (N_filter > 0.0f) {
        pid->N_filter = N_filter;
    }
    PID_UpdateCoefficients(pid);
}

/**
 * @brief Reinicia los acumuladores y estados de memoria del controlador PID
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->p_term           = 0.0f;
    pid->i_term           = 0.0f;
    pid->d_term           = 0.0f;
    pid->prev_error       = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output           = 0.0f;
    pid->is_first_run     = true;
}

/**
 * @brief Ejecuta un paso de cálculo del controlador PID a 1 kHz
 * @param pid         Puntero a la instancia del controlador PID
 * @param setpoint    Valor de referencia deseado r[k]
 * @param measurement Valor medido actual de la variable controlada y[k]
 * @return Esfuerzo de control u[k] acotado en [out_min, out_max]
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    if (pid->is_first_run) {
        pid->prev_error       = error;
        pid->prev_measurement = measurement;
        pid->is_first_run     = false;
    }

    /* 1. Término Proporcional P[k] */
    pid->p_term = pid->Kp * error;

    /* 2. Término Derivativo Filtrado D[k] */
    float delta_input;
    if (pid->derivative_on_measurement) {
        /* Derivada sobre la medición: evita 'derivative kick' cuando cambia el setpoint */
        delta_input = -(measurement - pid->prev_measurement);
    } else {
        /* Derivada sobre el error: máxima velocidad de respuesta estabilizadora */
        delta_input = (error - pid->prev_error);
    }
    pid->d_term = (pid->c_d1 * pid->d_term) + (pid->c_d2 * delta_input);

    /* 3. Término Integral Trapezoidal con Anti-Windup por Clamping Condicional */
    float delta_i = pid->c_int * (error + pid->prev_error);
    float candidate_i = pid->i_term + delta_i;
    float u_unsat = pid->p_term + candidate_i + pid->d_term;

    /* Solo integramos si la salida no está saturada, o si el error ayuda a salir de saturación */
    bool saturating_high = (u_unsat > pid->out_max) && (error > 0.0f);
    bool saturating_low  = (u_unsat < pid->out_min) && (error < 0.0f);

    if (!saturating_high && !saturating_low) {
        pid->i_term = candidate_i;
        /* Acotar también el integrador individualmente a [out_min, out_max] */
        if (pid->i_term > pid->out_max) {
            pid->i_term = pid->out_max;
        } else if (pid->i_term < pid->out_min) {
            pid->i_term = pid->out_min;
        }
    }

    /* 4. Suma final y saturación del actuador */
    float u_out = pid->p_term + pid->i_term + pid->d_term;
    if (u_out > pid->out_max) {
        u_out = pid->out_max;
    } else if (u_out < pid->out_min) {
        u_out = pid->out_min;
    }

    /* 5. Actualizar memorias para el instante [k+1] */
    pid->prev_error       = error;
    pid->prev_measurement = measurement;
    pid->output           = u_out;

    return u_out;
}
