#include "vl53l1x_platform.h"
#include "i2c.h"

uint8_t VL53L1_WaitMs(uint16_t dev, uint32_t wait_ms)
{
    HAL_Delay(wait_ms);
    return 0;
}

uint8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    uint8_t txBuffer[3];
    txBuffer[0] = (index >> 8) & 0xFF;
    txBuffer[1] = index & 0xFF;
    txBuffer[2] = data;

    if(HAL_I2C_Master_Transmit(&hi2c1, (dev << 1), txBuffer, 3, 100) != HAL_OK) return 1;
    return 0;
}

uint8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t txBuffer[4];
    txBuffer[0] = (index >> 8) & 0xFF;
    txBuffer[1] = index & 0xFF;
    txBuffer[2] = (data >> 8) & 0xFF;
    txBuffer[3] = data & 0xFF;

    if(HAL_I2C_Master_Transmit(&hi2c1, (dev << 1), txBuffer, 4, 100) != HAL_OK) return 1;
    return 0;
}

uint8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t txBuffer[6];
    txBuffer[0] = (index >> 8) & 0xFF;
    txBuffer[1] = index & 0xFF;
    txBuffer[2] = (data >> 24) & 0xFF;
    txBuffer[3] = (data >> 16) & 0xFF;
    txBuffer[4] = (data >> 8) & 0xFF;
    txBuffer[5] = data & 0xFF;

    if(HAL_I2C_Master_Transmit(&hi2c1, (dev << 1), txBuffer, 6, 100) != HAL_OK) return 1;
    return 0;
}

uint8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (index >> 8) & 0xFF;
    txBuffer[1] = index & 0xFF;

    if(HAL_I2C_Master_Transmit(&hi2c1, (dev << 1), txBuffer, 2, 100) != HAL_OK) return 1;
    if(HAL_I2C_Master_Receive(&hi2c1, (dev << 1), data, 1, 100) != HAL_OK) return 1;
    return 0;
}

uint8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
    uint8_t rxBuffer[2];
    uint8_t txBuffer[2];
    txBuffer[0] = (index >> 8) & 0xFF;
    txBuffer[1] = index & 0xFF;

    if(HAL_I2C_Master_Transmit(&hi2c1, (dev << 1), txBuffer, 2, 100) != HAL_OK) return 1;
    if(HAL_I2C_Master_Receive(&hi2c1, (dev << 1), rxBuffer, 2, 100) != HAL_OK) return 1;

    *data = (rxBuffer[0] << 8) | rxBuffer[1];
    return 0;
}

uint8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
    uint8_t rxBuffer[4];
    uint8_t txBuffer[2];
    txBuffer[0] = (index >> 8) & 0xFF;
    txBuffer[1] = index & 0xFF;

    if(HAL_I2C_Master_Transmit(&hi2c1, (dev << 1), txBuffer, 2, 100) != HAL_OK) return 1;
    if(HAL_I2C_Master_Receive(&hi2c1, (dev << 1), rxBuffer, 4, 100) != HAL_OK) return 1;

    *data = (rxBuffer[0] << 24) | (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
    return 0;
}

uint8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pData, uint32_t nBytes)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (index >> 8) & 0xFF;
    txBuffer[1] = index & 0xFF;

    if(HAL_I2C_Master_Transmit(&hi2c1, (dev << 1), txBuffer, 2, 100) != HAL_OK) return 1;
    if(HAL_I2C_Master_Receive(&hi2c1, (dev << 1), pData, nBytes, 100) != HAL_OK) return 1;
    return 0;
}
