#include "em_i2c.h"

#ifndef I2C_MULTITASK_H
#define I2C_MULTITASK_H
void I2C_Multitask_Init();
uint8_t I2C_Multitask_Take_Semaphore();
void I2C_Multitask_Give_Semaphore();
I2C_TransferReturn_TypeDef I2C_Multitask_Transfer(I2C_TypeDef *i2c);
#endif
