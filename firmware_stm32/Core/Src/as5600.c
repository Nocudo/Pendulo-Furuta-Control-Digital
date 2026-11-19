/**
 ******************************************************************************
 * @file    as5600.c
 * @brief   Implementación del driver de encoders magnéticos AS5600 por I2C.
 *          Soporta lectura determinista en < 120 us a 400 kHz Fast Mode,
 *          desenvoltura multi-vuelta (angle unwrapping), calibración de cero,
 *          mapeo de desviación respecto al equilibrio invertido (pi rad) y
 *          derivada numérica con filtro pasabajos IIR de primer orden.
 ******************************************************************************
 */

#include "as5600.h"

/**
 * @brief Normaliza cualquier ángulo en radianes al intervalo [-pi, +pi]
 */
float AS5600_WrapToPi(float angle_rad)
{
    while (angle_rad > M_PI) {
        angle_rad -= TWO_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += TWO_PI;
    }
    return angle_rad;
}

/**
 * @brief Lee el registro RAW_ANGLE (0x0C..0x0D) de 12 bits del AS5600
 */
static HAL_StatusTypeDef AS5600_ReadRaw12Bit(AS5600_Handle_t *dev, uint16_t *out_counts)
{
    uint8_t buf[2] = {0U, 0U};
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c,
                              AS5600_I2C_ADDR,
                              AS5600_REG_RAW_ANGLE_H,
                              I2C_MEMADD_SIZE_8BIT,
                              buf,
                              2U,
                              AS5600_I2C_TIMEOUT_MS);

    if (status == HAL_OK) {
        *out_counts = ((uint16_t)(buf[0] & 0x0FU) << 8) | (uint16_t)buf[1];
    } else {
        dev->i2c_error_count++;
    }

    return status;
}

/**
 * @brief Inicializa la estructura del encoder AS5600 y verifica comunicación I2C
 */
HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *dev,
                              I2C_HandleTypeDef *hi2c,
                              float Ts,
                              float vel_filter_tau,
                              int8_t direction)
{
    dev->hi2c                   = hi2c;
    dev->Ts                     = (Ts > 0.0f) ? Ts : TS_CONTROL_SEC;
    dev->vel_filter_tau         = (vel_filter_tau >= 0.0f) ? vel_filter_tau : 0.005f;
    dev->direction              = (direction >= 0) ? 1 : -1;

    dev->raw_counts             = 0U;
    dev->prev_raw_counts        = 0U;
    dev->zero_offset_counts     = 0U;
    dev->full_turns             = 0;
    dev->cumulative_counts      = 0;

    dev->angle_rad              = 0.0f;
    dev->prev_angle_rad         = 0.0f;
    dev->angle_wrapped_rad      = 0.0f;
    dev->angle_inverted_err_rad = M_PI;
    dev->velocity_rad_s         = 0.0f;

    dev->magnet_status          = 0U;
    dev->agc_value              = 0U;
    dev->i2c_error_count        = 0U;
    dev->is_initialized         = false;

    bool magnet_ok = false;
    (void)AS5600_CheckMagnet(dev, &magnet_ok);

    uint16_t initial_raw = 0U;
    HAL_StatusTypeDef st = AS5600_ReadRaw12Bit(dev, &initial_raw);
    if (st == HAL_OK) {
        dev->raw_counts          = initial_raw;
        dev->prev_raw_counts     = initial_raw;
        dev->zero_offset_counts  = initial_raw;
        dev->is_initialized      = true;
    }

    return st;
}

/**
 * @brief Verifica la presencia e intensidad del imán diametral (registros STATUS y AGC)
 */
HAL_StatusTypeDef AS5600_CheckMagnet(AS5600_Handle_t *dev, bool *magnet_ok)
{
    HAL_StatusTypeDef st;
    st = HAL_I2C_Mem_Read(dev->hi2c,
                          AS5600_I2C_ADDR,
                          AS5600_REG_STATUS,
                          I2C_MEMADD_SIZE_8BIT,
                          &dev->magnet_status,
                          1U,
                          AS5600_I2C_TIMEOUT_MS);

    if (st == HAL_OK) {
        (void)HAL_I2C_Mem_Read(dev->hi2c,
                               AS5600_I2C_ADDR,
                               AS5600_REG_AGC,
                               I2C_MEMADD_SIZE_8BIT,
                               &dev->agc_value,
                               1U,
                               AS5600_I2C_TIMEOUT_MS);

        bool detected = (dev->magnet_status & AS5600_STATUS_MD) != 0U;
        bool too_weak = (dev->magnet_status & AS5600_STATUS_ML) != 0U;
        if (magnet_ok != NULL) {
            *magnet_ok = (detected && !too_weak);
        }
    } else {
        if (magnet_ok != NULL) {
            *magnet_ok = false;
        }
    }
    return st;
}

/**
 * @brief Calibra la posición cero promediando N muestras en reposo (péndulo colgado)
 *        Usa promedio vectorial circular para evitar artefactos si el imán está
 *        cerca de la transición 4095 -> 0.
 */
HAL_StatusTypeDef AS5600_CalibrateZero(AS5600_Handle_t *dev, uint16_t num_samples)
{
    if (num_samples == 0U) {
        num_samples = 64U;
    }

    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    uint16_t valid_samples = 0U;
    uint16_t raw = 0U;

    for (uint16_t i = 0U; i < num_samples; i++) {
        if (AS5600_ReadRaw12Bit(dev, &raw) == HAL_OK) {
            float ang = (float)raw * AS5600_COUNTS_TO_RAD;
            sum_sin += sinf(ang);
            sum_cos += cosf(ang);
            valid_samples++;
        }
        HAL_Delay(1);
    }

    if (valid_samples == 0U) {
        return HAL_ERROR;
    }

    float mean_ang = atan2f(sum_sin / (float)valid_samples, sum_cos / (float)valid_samples);
    if (mean_ang < 0.0f) {
        mean_ang += TWO_PI;
    }

    uint16_t offset = (uint16_t)((mean_ang / TWO_PI) * (float)AS5600_CPR) & 0x0FFFU;

    dev->zero_offset_counts     = offset;
    dev->raw_counts             = offset;
    dev->prev_raw_counts        = offset;
    dev->full_turns             = 0;
    dev->cumulative_counts      = 0;
    dev->angle_rad              = 0.0f;
    dev->prev_angle_rad         = 0.0f;
    dev->angle_wrapped_rad      = 0.0f;
    dev->angle_inverted_err_rad = -M_PI;
    dev->velocity_rad_s         = 0.0f;
    dev->is_initialized         = true;

    return HAL_OK;
}

/**
 * @brief Actualiza lectura del AS5600 a 1 kHz:
 *        1. Lee ángulo crudo de 12 bits.
 *        2. Ejecuta unwrapping multi-vuelta al cruzar 0 <-> 4095.
 *        3. Calcula ángulo continuo, ángulo envuelto [-pi, pi] y error respecto a pi.
 *        4. Estima velocidad angular con filtro pasabajos IIR de primer orden.
 */
HAL_StatusTypeDef AS5600_Update(AS5600_Handle_t *dev)
{
    uint16_t new_raw = 0U;
    HAL_StatusTypeDef st = AS5600_ReadRaw12Bit(dev, &new_raw);
    if (st != HAL_OK) {
        return st;
    }

    if (!dev->is_initialized) {
        dev->raw_counts      = new_raw;
        dev->prev_raw_counts = new_raw;
        dev->is_initialized  = true;
    }

    /* 1. Detección de cruce de vuelta completa (Rollover 0 <-> 4095) */
    int32_t delta_raw = (int32_t)new_raw - (int32_t)dev->prev_raw_counts;
    if (delta_raw < -AS5600_HALF_CPR) {
        dev->full_turns++;
    } else if (delta_raw > AS5600_HALF_CPR) {
        dev->full_turns--;
    }

    dev->prev_raw_counts = new_raw;
    dev->raw_counts      = new_raw;

    /* 2. Cuentas acumuladas respecto al offset de calibración */
    int32_t total_counts = (dev->full_turns * (int32_t)AS5600_CPR)
                         + ((int32_t)new_raw - (int32_t)dev->zero_offset_counts);
    dev->cumulative_counts = (int32_t)dev->direction * total_counts;

    /* 3. Conversión a radianes */
    dev->prev_angle_rad = dev->angle_rad;
    dev->angle_rad      = (float)dev->cumulative_counts * AS5600_COUNTS_TO_RAD;

    /* Ángulo normalizado en [-pi, +pi] (0 rad = péndulo colgado abajo) */
    dev->angle_wrapped_rad = AS5600_WrapToPi(dev->angle_rad);

    /* Error respecto a la posición vertical invertida (alpha_ref = pi rad = 180°)
     * Cuando el péndulo está perfectamente vertical arriba: angle_inverted_err_rad = 0.0 rad */
    dev->angle_inverted_err_rad = AS5600_WrapToPi(dev->angle_wrapped_rad - M_PI);

    /* 4. Derivada numérica filtrada (Filtro IIR de 1er orden: alpha = tau / (tau + Ts)) */
    float raw_vel = (dev->angle_rad - dev->prev_angle_rad) / dev->Ts;
    float alpha_lpf = dev->vel_filter_tau / (dev->vel_filter_tau + dev->Ts);
    dev->velocity_rad_s = (alpha_lpf * dev->velocity_rad_s) + ((1.0f - alpha_lpf) * raw_vel);

    return HAL_OK;
}
