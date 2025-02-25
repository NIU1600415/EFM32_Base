#include <stdbool.h>

#ifndef I2C_H
#define I2C_H
void BSP_I2C_Init(uint8_t addr);
bool I2C_WriteRegister(uint8_t reg, uint8_t data);
bool I2C_ReadRegister(uint8_t reg, uint8_t *val);
uint16_t I2C_BlockReadRegister(uint8_t reg, uint8_t *data, uint16_t len);
bool I2C_Test();
#endif
