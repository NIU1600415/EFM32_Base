#include <stdio.h>
#include <stdbool.h>
#include "i2c.h"
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "i2c_multitask.h"

static uint8_t device_addr;

void BSP_I2C_Init(uint8_t addr) {
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	CMU_ClockEnable(cmuClock_I2C1, true);
	GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAnd, 1);
	I2C1->ROUTE = I2C_ROUTE_SDAPEN |
	I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC0;
	I2C_Init(I2C1, &i2cInit);

	device_addr = addr;
}

/**
 * @brief Write register using default I2C bus
 * @param reg register to write
 * @param data data to write
 * @return true on success
 */
bool I2C_WriteRegister(uint8_t reg, uint8_t data) {
	I2C_TransferReturn_TypeDef I2C_Status;
	bool ret_value = false;

	I2C_TransferSeq_TypeDef seq;
	uint8_t dataW[2];

	seq.addr = device_addr;
	seq.flags = I2C_FLAG_WRITE;

	/* Register to write: 0x67 ( INT_FLAT )*/
	dataW[0] = reg;
	dataW[1] = data;

	seq.buf[0].data = dataW;
	seq.buf[0].len = 2;
	I2C_Status = I2C_TransferInit(I2C1, &seq);

	while (I2C_Status == i2cTransferInProgress) {
		I2C_Status = I2C_Transfer(I2C1);
	}

	if (I2C_Status != i2cTransferDone) {
		ret_value = false;
	} else {
		ret_value = true;
	}
	return ret_value;
}

/**
 * @brief Read register from I2C device
 * @param reg Register to read
 * @param val Value read
 * @return true on success
 */
bool I2C_ReadRegister(uint8_t reg, uint8_t *val) {
	I2C_TransferReturn_TypeDef I2C_Status;
	I2C_TransferSeq_TypeDef seq;
	uint8_t data[2];

	seq.addr = device_addr;
	seq.flags = I2C_FLAG_WRITE_READ;

	seq.buf[0].data = &reg;
	seq.buf[0].len = 1;
	seq.buf[1].data = data;
	seq.buf[1].len = 1;

	I2C_Status = I2C_TransferInit(I2C1, &seq);

	while (I2C_Status == i2cTransferInProgress) {
		/* Enable multitask */
		/*uint8_t semaphore_obtained = 0;

		// Request until semaphore obtained
		while (semaphore_obtained == 0) {
			// Request semaphore
			semaphore_obtained = I2C_Multitask_Take_Semaphore();
		}

		// Have semaphore
		I2C_Status = I2C_Multitask_Transfer(I2C1);

		// Release semaphore
		I2C_Multitask_Give_Semaphore();*/

		I2C_Status = I2C_Transfer(I2C1);
	}

	if (I2C_Status != i2cTransferDone) {
		if (I2C_Status == i2cTransferNack) {
			return false;
		} else if (I2C_Status == i2cTransferBusErr) {
			return false;
		} else if (I2C_Status == i2cTransferArbLost) {
			return false;
		} else if (I2C_Status == i2cTransferUsageFault) {
			return false;
		} else if (I2C_Status == i2cTransferSwFault) {
			return false;
		} else {
			return false;
		}
	}

	*val = data[0];

	return true;
}

/**
 * @brief Read multiple values at register from I2C device
 * @param reg Register to read
 * @param data Buffer to read into
 * @param len Buffer length
 * @return amount of elements read on success or -1 on error
 */
uint16_t I2C_BlockReadRegister(uint8_t reg, uint8_t *data, uint16_t len) {
	I2C_TransferReturn_TypeDef I2C_Status;
	I2C_TransferSeq_TypeDef seq;

	seq.addr = device_addr;
	seq.flags = I2C_FLAG_WRITE_READ;

	seq.buf[0].data = &reg;
	seq.buf[0].len = 1;
	seq.buf[1].data = data;
	seq.buf[1].len = len;

	I2C_Status = I2C_TransferInit(I2C1, &seq);

	while (I2C_Status == i2cTransferInProgress)
	{
		// TODO: Semaphores
		I2C_Status = I2C_Transfer(I2C1);
	}

	if (I2C_Status != i2cTransferDone)
	{
		return -1;
	}

	return len;
}

bool I2C_Test() {
	uint8_t id;

	I2C_ReadRegister(0x92, &id);

	printf("APDS-9960 ID: %x\n", id);

	return true;
}
