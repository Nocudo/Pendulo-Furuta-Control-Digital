/**
 ******************************************************************************
 * @file    as5600.c
 * @brief   Implementación del driver de encoders magnéticos AS5600 por I2C.
 *          Incluye auto-detección y soporte transparente de inversión SCL/SDA
 *          por Soft-I2C de alta velocidad (~400 kHz) si el usuario conectó
 *          SCL y SDA cruzados en D15/D14 (PB8/PB9) o D6/D3 (PB10/PB3),
 *          además de desenvoltura multi-vuelta y filtro IIR de velocidad.
 ******************************************************************************
 */

#include "as5600.h"

/* Retardo corto (~0.35 us a 180 MHz) para Soft-I2C a ~450 kHz */
static inline void SoftI2C_Delay(void)
{
    for (volatile uint32_t i = 0U; i < 10U; i++) {
        __NOP();
    }
}

static inline void SoftI2C_PinHigh(GPIO_TypeDef *port, uint16_t pin)
{
    port->BSRR = (uint32_t)pin;
}

static inline void SoftI2C_PinLow(GPIO_TypeDef *port, uint16_t pin)
{
    port->BSRR = (uint32_t)pin << 16U;
}

static inline uint8_t SoftI2C_PinRead(GPIO_TypeDef *port, uint16_t pin)
{
    return ((port->IDR & pin) != 0U) ? 1U : 0U;
}

static void SoftI2C_ConfigPinsOD(GPIO_TypeDef *port, uint16_t scl_pin, uint16_t sda_pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    SoftI2C_PinHigh(port, scl_pin | sda_pin);
    GPIO_InitStruct.Pin   = scl_pin | sda_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    SoftI2C_PinHigh(port, scl_pin | sda_pin);
    SoftI2C_Delay();
}

static void SoftI2C_Start(GPIO_TypeDef *port, uint16_t scl, uint16_t sda)
{
    SoftI2C_PinHigh(port, sda);
    SoftI2C_PinHigh(port, scl);
    SoftI2C_Delay();
    SoftI2C_PinLow(port, sda);
    SoftI2C_Delay();
    SoftI2C_PinLow(port, scl);
    SoftI2C_Delay();
}

static void SoftI2C_Stop(GPIO_TypeDef *port, uint16_t scl, uint16_t sda)
{
    SoftI2C_PinLow(port, sda);
    SoftI2C_Delay();
    SoftI2C_PinHigh(port, scl);
    SoftI2C_Delay();
    SoftI2C_PinHigh(port, sda);
    SoftI2C_Delay();
}

static bool SoftI2C_WriteByte(GPIO_TypeDef *port, uint16_t scl, uint16_t sda, uint8_t data)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((data & 0x80U) != 0U) {
            SoftI2C_PinHigh(port, sda);
        } else {
            SoftI2C_PinLow(port, sda);
        }
        data <<= 1U;
        SoftI2C_Delay();
        SoftI2C_PinHigh(port, scl);
        SoftI2C_Delay();
        SoftI2C_PinLow(port, scl);
    }
    /* Leer ACK del esclavo */
    SoftI2C_PinHigh(port, sda);
    SoftI2C_Delay();
    SoftI2C_PinHigh(port, scl);
    SoftI2C_Delay();
    bool ack = (SoftI2C_PinRead(port, sda) == 0U);
    SoftI2C_PinLow(port, scl);
    SoftI2C_Delay();
    return ack;
}

static uint8_t SoftI2C_ReadByte(GPIO_TypeDef *port, uint16_t scl, uint16_t sda, bool send_ack)
{
    uint8_t data = 0U;
    SoftI2C_PinHigh(port, sda);
    for (uint8_t i = 0U; i < 8U; i++) {
        data <<= 1U;
        SoftI2C_Delay();
        SoftI2C_PinHigh(port, scl);
        SoftI2C_Delay();
        if (SoftI2C_PinRead(port, sda) != 0U) {
            data |= 1U;
        }
        SoftI2C_PinLow(port, scl);
    }
    /* Enviar ACK o NACK */
    if (send_ack) {
        SoftI2C_PinLow(port, sda);
    } else {
        SoftI2C_PinHigh(port, sda);
    }
    SoftI2C_Delay();
    SoftI2C_PinHigh(port, scl);
    SoftI2C_Delay();
    SoftI2C_PinLow(port, scl);
    SoftI2C_PinHigh(port, sda);
    SoftI2C_Delay();
    return data;
}

static HAL_StatusTypeDef SoftI2C_MemRead(GPIO_TypeDef *port,
                                         uint16_t scl,
                                         uint16_t sda,
                                         uint8_t reg,
                                         uint8_t *buf,
                                         uint16_t len)
{
    SoftI2C_Start(port, scl, sda);
    if (!SoftI2C_WriteByte(port, scl, sda, AS5600_I2C_ADDR)) {
        SoftI2C_Stop(port, scl, sda);
        return HAL_ERROR;
    }
    if (!SoftI2C_WriteByte(port, scl, sda, reg)) {
        SoftI2C_Stop(port, scl, sda);
        return HAL_ERROR;
    }
    SoftI2C_Start(port, scl, sda);
    if (!SoftI2C_WriteByte(port, scl, sda, AS5600_I2C_ADDR | 0x01U)) {
        SoftI2C_Stop(port, scl, sda);
        return HAL_ERROR;
    }
    for (uint16_t i = 0U; i < len; i++) {
        buf[i] = SoftI2C_ReadByte(port, scl, sda, (i + 1U) < len);
    }
    SoftI2C_Stop(port, scl, sda);
    return HAL_OK;
}

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
 * @brief Lee bytes de registro del AS5600 usando Soft-I2C auto-adaptativo
 *        con caché de polaridad (normal SCL/SDA vs invertida SDA/SCL)
 *        para completar la lectura en < 45 us sin reintentos fallidos.
 */
static HAL_StatusTypeDef AS5600_ReadRegisters(AS5600_Handle_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    static bool s_swapped_i2c1 = false;
    static bool s_swapped_i2c2 = false;

    GPIO_TypeDef *port = GPIOB;
    bool is_i2c1 = (dev->hi2c->Instance == I2C1);
    uint16_t pin_a = is_i2c1 ? ENC_ARM_SCL_PIN  : ENC_PEND_SCL_PIN;
    uint16_t pin_b = is_i2c1 ? ENC_ARM_SDA_PIN  : ENC_PEND_SDA_PIN;
    bool *p_swapped = is_i2c1 ? &s_swapped_i2c1 : &s_swapped_i2c2;

    uint16_t scl_first = (*p_swapped) ? pin_b : pin_a;
    uint16_t sda_first = (*p_swapped) ? pin_a : pin_b;

    if (SoftI2C_MemRead(port, scl_first, sda_first, reg, buf, len) == HAL_OK) {
        return HAL_OK;
    }

    /* Si falló, probar la polaridad opuesta y memorizarla si tiene éxito */
    if (SoftI2C_MemRead(port, sda_first, scl_first, reg, buf, len) == HAL_OK) {
        *p_swapped = !(*p_swapped);
        return HAL_OK;
    }

    dev->i2c_error_count++;
    return HAL_ERROR;
}

/**
 * @brief Lee el registro RAW_ANGLE (0x0C..0x0D) de 12 bits del AS5600
 */
static HAL_StatusTypeDef AS5600_ReadRaw12Bit(AS5600_Handle_t *dev, uint16_t *out_counts)
{
    uint8_t buf[2] = {0U, 0U};
    HAL_StatusTypeDef status = AS5600_ReadRegisters(dev, AS5600_REG_RAW_ANGLE_H, buf, 2U);

    if (status == HAL_OK) {
        *out_counts = ((uint16_t)(buf[0] & 0x0FU) << 8) | (uint16_t)buf[1];
    }
    return status;
}

/**
 * @brief Inicializa la estructura del encoder AS5600 y configura los pines en Open-Drain
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
    dev->gear_ratio             = (hi2c->Instance == I2C1) ? 80.0f : 1.0f;

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

    uint16_t pin_a = (hi2c->Instance == I2C1) ? ENC_ARM_SCL_PIN  : ENC_PEND_SCL_PIN;
    uint16_t pin_b = (hi2c->Instance == I2C1) ? ENC_ARM_SDA_PIN  : ENC_PEND_SDA_PIN;
    SoftI2C_ConfigPinsOD(GPIOB, pin_a, pin_b);

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
    HAL_StatusTypeDef st = AS5600_ReadRegisters(dev, AS5600_REG_STATUS, &dev->magnet_status, 1U);
    if (st == HAL_OK) {
        (void)AS5600_ReadRegisters(dev, AS5600_REG_AGC, &dev->agc_value, 1U);

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
 * @brief Calibra la posición cero promediando N muestras en reposo
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
 * @brief Actualiza lectura del AS5600 a 1 kHz
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

    /* 1. Detección de cruce de vuelta completa (Rollover 0 <-> 4095) y rechazo de glitches I2C */
    int32_t delta_raw = (int32_t)new_raw - (int32_t)dev->prev_raw_counts;
    int32_t circ_delta = delta_raw;
    if (circ_delta < -AS5600_HALF_CPR) {
        circ_delta += (int32_t)AS5600_CPR;
    } else if (circ_delta > AS5600_HALF_CPR) {
        circ_delta -= (int32_t)AS5600_CPR;
    }

    /* En el sensor 2 del péndulo (gear_ratio == 1.0), un salto > 160 cuentas en 1 ms (> 245 rad/s)
     * corresponde a un bit corrupto por ruido capacitivo en el cable largo: descartar muestra */
    if (dev->gear_ratio < 2.0f && (circ_delta > 160 || circ_delta < -160)) {
        dev->i2c_error_count++;
        return HAL_ERROR;
    }

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

    /* 3. Conversión a radianes en el eje de salida (aplicando reductor planetario 80:1 en I2C1) */
    dev->prev_angle_rad = dev->angle_rad;
    dev->angle_rad      = ((float)dev->cumulative_counts * AS5600_COUNTS_TO_RAD) / dev->gear_ratio;

    /* Ángulo normalizado en [-pi, +pi] (0 rad = péndulo colgado abajo) */
    dev->angle_wrapped_rad = AS5600_WrapToPi(dev->angle_rad);

    /* Error respecto a la posición vertical invertida (alpha_ref = pi rad = 180°) */
    dev->angle_inverted_err_rad = AS5600_WrapToPi(dev->angle_wrapped_rad - M_PI);

    /* 4. Derivada numérica filtrada (Filtro IIR de 1er orden: alpha = tau / (tau + Ts)) */
    float raw_vel = (dev->angle_rad - dev->prev_angle_rad) / dev->Ts;
    float alpha_lpf = dev->vel_filter_tau / (dev->vel_filter_tau + dev->Ts);
    dev->velocity_rad_s = (alpha_lpf * dev->velocity_rad_s) + ((1.0f - alpha_lpf) * raw_vel);

    return HAL_OK;
}
