#ifndef VL53L1X_PLATFORM_H
#define VL53L1X_PLATFORM_H

#include <stdint.h>

uint8_t VL53L1_WaitMs(uint16_t dev, uint32_t wait_ms);
uint8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);
uint8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);
uint8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);
uint8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data);
uint8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data);
uint8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data);
uint8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pData, uint32_t nBytes);

#endif
