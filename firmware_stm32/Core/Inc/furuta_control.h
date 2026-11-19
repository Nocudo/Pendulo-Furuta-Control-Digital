/**
 ******************************************************************************
 * @file    furuta_control.h
 * @brief   Arquitectura Maestra de Control Digital para el Péndulo Invertido
 *          Rotacional de Furuta a 1 kHz:
 *            - Máquina de estados finitos (FSM)
 *            - Control Híbrido: Swing-Up por Energía (Lyapunov) + Captura
 *            - Controlador PID Dual en Paralelo/Cascada (PID_alpha + PID_theta)
 *            - Controladores PID dedicados del motor (Velocidad y Posición)
 *            - Detección de caída y auto-recuperación continua (|alpha_err| > 32°)
 *            - Intérprete de comandos UART en tiempo real y telemetría CSV
 ******************************************************************************
 */

#ifndef __FURUTA_CONTROL_H
#define __FURUTA_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "as5600.h"
#include "l298n_motor.h"
#include "pid_controller.h"
#include "caracterizacion.h"

/* Umbrales Físicos de Conmutación y Recuperación Automática -----------------*/
#define FURUTA_CAPTURE_ANGLE_RAD    (20.0f * DEG_TO_RAD)    /* Conmutación suave a PID: ±20° */
#define FURUTA_CAPTURE_VEL_RAD_S    4.8f                    /* Velocidad máx suave de captura */
#define FURUTA_WATCHDOG_ANGLE_RAD   (32.0f * DEG_TO_RAD)    /* Detección de caída péndulo: ±32° -> auto-recuperar */
#define FURUTA_MAX_ARM_ANGLE_RAD    (135.0f * DEG_TO_RAD)   /* Referencia informativa (sin corte de servo) */

/* Parámetros Físicos Identificados del Péndulo para el Bombeo (Swing-Up) ----*/
#define FURUTA_M1_KG                0.024f                  /* Masa efectiva péndulo (kg) */
#define FURUTA_L1_M                 0.068f                  /* Distancia al CM l1 (m) */
#define FURUTA_J1_PIVOT             0.0002044f              /* Inercia en pivote medida (R²=98.5%, kg·m^2) */
#define FURUTA_GRAVITY              9.80665f                /* Gravedad (m/s^2) */
#define FURUTA_SWINGUP_KE           240.0f                  /* Ganancia bombeo Lyapunov */
#define FURUTA_SWINGUP_KV_ARM       1.10f                   /* Amortiguamiento centrado brazo */
#define FURUTA_SWINGUP_KP_ARM       2.80f                   /* Rigidez centrado brazo */

/**
 * @brief Estados globales de operación del sistema Furuta
 */
typedef enum {
    FURUTA_STATE_IDLE = 0,          /* Motor deshabilitado en espera */
    FURUTA_STATE_CALIBRATING,       /* Homing de encoders AS5600 en posición colgada */
    FURUTA_STATE_CHARACTERIZATION,  /* Ejecutando ensayo de caracterización 1..5 */
    FURUTA_STATE_MOTOR_PID_VEL,     /* Lazo cerrado PID de velocidad del motor solo */
    FURUTA_STATE_MOTOR_PID_POS,     /* Lazo cerrado PID de posición del brazo solo */
    FURUTA_STATE_SWING_UP,          /* Levantamiento no lineal por energía (siempre desde 0°) */
    FURUTA_STATE_BALANCE_PID,       /* Estabilización invertida LQR5 + Notch 11 Hz (sin límite de ángulo) */
    FURUTA_STATE_FAULT_STOP,        /* Parada de seguridad (reservado) */
    FURUTA_STATE_RECOVER_HOME,      /* Auto-recuperación: vuelve a 0°, calma y relanza Swing-Up */
    FURUTA_STATE_STOP_HOME          /* Parada ordenada solicitada por usuario: vuelve a 0° y apaga */
} Furuta_State_t;

/**
 * @brief Contexto global del sistema de control del Péndulo de Furuta
 */
typedef struct {
    Furuta_State_t state;
    bool enable_auto_swingup;       /* true: sube solo; false: espera captura manual */
    uint32_t tick_counter_1khz;     /* Reloj monotónico en milisegundos */

    /* Submódulos de Hardware */
    AS5600_Handle_t enc_arm;        /* AS5600 en I2C1: Brazo theta */
    AS5600_Handle_t enc_pend;       /* AS5600 en I2C2: Péndulo alpha */
    L298N_Handle_t  motor;          /* Driver Puente H L298N en TIM1_CH1 + PB4/PB5 */

    /* Los 4 Controladores PID del Sistema */
    PID_Controller_t pid_motor_vel; /* 1. PID Velocidad del Motor: omega -> V */
    PID_Controller_t pid_motor_pos; /* 2. PID Posición del Motor:  theta -> V */
    PID_Controller_t pid_pend_alpha;/* 3. PID Estabilización Péndulo Invertido: alpha -> V */
    PID_Controller_t pid_arm_theta; /* 4. PID Regulación Brazo en Equilibrio:   theta -> V */

    /* Gestor de Caracterización Experimental */
    Characterization_Manager_t char_mgr;

    /* Setpoints y señales internas de control */
    float theta_setpoint_rad;       /* Referencia deseada para el brazo (rad) */
    float omega_setpoint_rad_s;     /* Referencia deseada de velocidad motor (rad/s) */
    float u_alpha_volts;            /* Esfuerzo de control del PID del péndulo (V) */
    float u_theta_volts;            /* Esfuerzo de control del PID del brazo (V) */
    float u_total_volts;            /* Acción de control combinada enviada al motor (V) */
    float pendulum_energy_j;        /* Energía mecánica instantánea del péndulo (J) */
} Furuta_System_t;

/* Instancia global accesible desde main e ISRs */
extern Furuta_System_t g_furuta;

/* Prototipos de Funciones Públicas ------------------------------------------*/
void Furuta_Init(Furuta_System_t *sys);
void Furuta_ControlISR_1kHz(Furuta_System_t *sys);
void Furuta_ProcessCommand(Furuta_System_t *sys, const char *cmd_str);
void Furuta_SendTelemetry_UART(Furuta_System_t *sys);

#ifdef __cplusplus
}
#endif

#endif /* __FURUTA_CONTROL_H */
