#include "MLX90640_I2C_Driver.h"
#include "i2c.h"
#include <stdlib.h>

#define MLX90640_I2C_TIMEOUT_MS   100

#define MLX90640_MAX_WORDS        832

static I2C_HandleTypeDef *mlx_hi2c = &hi2c1;

void MLX90640_I2CInit(void)
{
}

int MLX90640_I2CGeneralReset(void)
{
    uint8_t resetCmd = 0x06;
    HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(mlx_hi2c, 0x00, &resetCmd, 1,
                                                      MLX90640_I2C_TIMEOUT_MS);
    HAL_Delay(5);
    return (res == HAL_OK) ? 0 : -1;
}

int MLX90640_I2CRead(uint8_t slaveAddr,
                     uint16_t startAddress,
                     uint16_t nMemAddressRead,
                     uint16_t *data)
{
    uint8_t rxBuffer[32];

    uint16_t wordsRead = 0;

    while (wordsRead < nMemAddressRead)
    {
        uint16_t wordsToRead = nMemAddressRead - wordsRead;

        if (wordsToRead > 16)
        {
            wordsToRead = 16;
        }

        uint16_t address = startAddress + wordsRead;

        volatile uint16_t dbg_address = address;
        volatile uint16_t dbg_wordsRead = wordsRead;
        volatile uint16_t dbg_wordsToRead = wordsToRead;

        HAL_StatusTypeDef status =
            HAL_I2C_Mem_Read(
                &hi2c1,
                (slaveAddr << 1),
                address,
                I2C_MEMADD_SIZE_16BIT,
                rxBuffer,
                wordsToRead * 2,
                1000
            );

        volatile HAL_StatusTypeDef dbg_status = status;

        if (status != HAL_OK)
        {
            return -1;
        }

        volatile uint16_t dbg_block0 =
            ((uint16_t)rxBuffer[0] << 8) | rxBuffer[1];

        volatile uint16_t dbg_block1 =
            ((uint16_t)rxBuffer[2] << 8) | rxBuffer[3];

        volatile uint16_t dbg_block2 =
            ((uint16_t)rxBuffer[4] << 8) | rxBuffer[5];

        volatile uint16_t dbg_block3 =
            ((uint16_t)rxBuffer[6] << 8) | rxBuffer[7];

        for (uint16_t i = 0; i < wordsToRead; i++)
        {
            uint16_t word =
                ((uint16_t)rxBuffer[i * 2] << 8) |
                rxBuffer[i * 2 + 1];

            data[wordsRead + i] = word;
        }

        wordsRead += wordsToRead;
    }

    return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t cmd[4];
    uint16_t check;
    HAL_StatusTypeDef res;

    cmd[0] = (uint8_t)(writeAddress >> 8);
    cmd[1] = (uint8_t)(writeAddress & 0xFF);
    cmd[2] = (uint8_t)(data >> 8);
    cmd[3] = (uint8_t)(data & 0xFF);

    res = HAL_I2C_Master_Transmit(mlx_hi2c, (uint16_t)(slaveAddr << 1), cmd, 4,
                                    MLX90640_I2C_TIMEOUT_MS);
    if (res != HAL_OK) {
        return -1;
    }

    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &check) != 0) {
        return -1;
    }
    if (check != data) {
        return -2;
    }

    return 0;
}

void MLX90640_I2CFreqSet(int freq)
{
    (void)freq;
}
