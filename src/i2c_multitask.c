#include "FreeRTOS.h"
#include "semphr.h"
#include "projdefs.h"
#include "em_i2c.h"
#include "i2c_multitask.h"

#define TICKS_TO_WAIT 0

static SemaphoreHandle_t I2C_Semaphore; // Define our semaphore

void I2C_Multitask_Init() {
	I2C_Semaphore = xSemaphoreCreateBinary();

	if (I2C_Semaphore == NULL){
		// Error
	} else {
		// OK
		xSemaphoreGive(I2C_Semaphore); // Must give semaphore during init because it is empty
	}
}

uint8_t I2C_Multitask_Take_Semaphore() {
	if (xSemaphoreTake(I2C_Semaphore, (TickType_t)TICKS_TO_WAIT) == pdTRUE) {
		// Semaphore obtained
		return 1;
	} else {
		// Semaphore could not be obtained: in use
		return 0;
	}
}

void I2C_Multitask_Give_Semaphore() {
	xSemaphoreGive(I2C_Semaphore);
}
