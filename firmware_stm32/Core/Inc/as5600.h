/**
 ******************************************************************************
 * @file    as5600.h
 * @brief   Driver para encoders magnéticos absolutos AS5600 (12-bit, I2C).
 *          Incluye lectura rápida, desenvoltura multi-vuelta (angle unwrapping),
 *          calibración de cero (homing), mapeo al equilibrio invertido y
 *          estimación de velocidad angular con filtro pasabajos IIR.
 ******************************************************************************
 */

#ifndef __AS5600_H
#define __AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Dirección I2C y Registros Internos del AS5600 -----------------------------*/
#define AS5600_I2C_ADDR_7BIT    0x36U
#define AS5600_I2C_ADDR         (AS5600_I2C_ADDR_7BIT << 1) /* 0x6C para HAL */

#define AS5600_REG_ZMCO         0x00U
#define AS5600_REG_ZPOS_H       0x01U
#define AS5600_REG_CONF_H       0x07U
#define AS5600_REG_STATUS       0x0BU   /* Bit 5: MD, Bit 4: ML, Bit 3: MH */
#define AS5600_REG_RAW_ANGLE_H  0x0CU   /* 12-bit Raw Angle [11:8] y [7:0] */
#define AS5600_REG_ANGLE_H      0x0EU   /* 12-bit Scaled Angle */
#define AS5600_REG_AGC          0x1AU   /* Automatic Gain Control (0..255) */
#define AS5600_REG_MAGNITUDE_H  0x1BU   /* Magnitud del campo magnético */

/* Constantes de Conversión de 12 bits (4096 cuentas por revolución) ---------*/
#define AS5600_CPR              4096U
#define AS5600_HALF_CPR         2048
#define AS5600_COUNTS_TO_RAD    (TWO_PI / 4096.0f)      /* ~0.00153398 rad/cuenta */
#define AS5600_COUNTS_TO_DEG    (360.0f / 4096.0f)      /* ~0.08789062 deg/cuenta */
#define AS5600_I2C_TIMEOUT_MS   2U                      /* Timeout estricto < Ts */

/* Flags del Registro STATUS (0x0B) ------------------------------------------*/
#define AS5600_STATUS_MH        (1U << 3)               /* Imán demasiado fuerte */
#define AS5600_STATUS_ML        (1U << 4)               /* Imán demasiado débil */
#define AS5600_STATUS_MD        (1U << 5)               /* Imán detectado OK */

/**
 * @brief Estructura de estado y filtrado de un sensor AS5600
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;        /* Bus I2C asociado (hi2c1 o hi2c2) */
    float Ts;                       /* Periodo de muestreo en segundos (0.001 s) */
    float vel_filter_tau;           /* Constante de tiempo LPF de velocidad (s) */
    int8_t direction;               /* Polaridad de giro (+1 o -1) */

    /* Lecturas crudas y desenvoltura (unwrapping) */
    uint16_t raw_counts;            /* Valor actual de 12 bits [0..4095] */
    uint16_t prev_raw_counts;       /* Valor anterior de 12 bits [0..4095] */
    uint16_t zero_offset_counts;    /* Offset calibrado en reposo */
    int32_t  full_turns;            /* Contador de vueltas completas acumuladas */
    int32_t  cumulative_counts;     /* Cuentas totales multi-vuelta respecto al cero */

    /* Variables cinemáticas en unidades físicas */
    float angle_rad;                /* Ángulo continuo desenvuelto (rad) */
    float prev_angle_rad;           /* Ángulo del instante k-1 (rad) */
    float angle_wrapped_rad;        /* Ángulo normalizado en [-pi, +pi] (rad) */
    float angle_inverted_err_rad;   /* Desviación respecto a vertical superior pi (rad) */
    float velocity_rad_s;           /* Velocidad angular filtrada (rad/s) */

    /* Diagnóstico de comunicación y estado magnético */
    uint8_t magnet_status;          /* Registro 0x0B STATUS */
    uint8_t agc_value;              /* Registro 0x1A AGC */
    uint32_t i2c_error_count;       /* Contador de fallos de bus I2C */
    bool is_initialized;            /* Flag de primera muestra capturada */
} AS5600_Handle_t;

/* Prototipos de Funciones Públicas ------------------------------------------*/
HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *dev,
                              I2C_HandleTypeDef *hi2c,
                              float Ts,
                              float vel_filter_tau,
                              int8_t direction);

HAL_StatusTypeDef AS5600_CalibrateZero(AS5600_Handle_t *dev, uint16_t num_samples);
HAL_StatusTypeDef AS5600_Update(AS5600_Handle_t *dev);
HAL_StatusTypeDef AS5600_CheckMagnet(AS5600_Handle_t *dev, bool *magnet_ok);
float AS5600_WrapToPi(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H */
