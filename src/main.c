/***************************************************************************//**
 * @file
 * @brief FreeRTOS Blink Demo for Energy Micro EFM32GG_STK3700 Starter Kit
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "croutine.h"

#include "em_chip.h"
#include "bsp.h"
#include "bsp_trace.h"

#include "apds9960.h"

#define STACK_SIZE_FOR_TASK (configMINIMAL_STACK_SIZE + 10)
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1)

#define LIGHT_SENSOR 1
#define PROXIMITY_SENSOR 0

void _write(const char *ptr) {
	do {
		ITM_SendChar(*ptr++);
	} while (*ptr != '\n');
	ITM_SendChar('\n');
}

int map(uint8_t x, float in_min, float in_max, float out_min, float out_max) {
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min) * 100;
}

float calculateProximity(uint8_t proxVal) {
	if (proxVal >= 254) {
		return 300;
	}

	if (proxVal <= 20) {
		return 1500;
	}

	return map(proxVal, 254, 20, 3, 15);
}

static void I2C_Task() {
	char tmp[32];

	/* Init APDS-9960 I2C module */
	if (APDS9960_init()) {
		_write("APDS-9960 initialization complete\n");
	} else {
		_write("Error during APDS-9960 init\n");
	}

	/*if (APDS9960_enableGestureSensor(false)) {
		_write("APDS-9960 enable gestures successful\n");
	} else {
		_write("Error during APDS-9960 enable gestures\n");
	}*/

#if PROXIMITY_SENSOR
	uint8_t prox;
	int val, units, decimals;

	if (APDS9960_enableProximitySensor(false)) {
		_write("APDS-9960 enable proximity successful\n");
	} else {
		_write("Error during APDS-9960 enable proximity\n");
	}

	if (APDS9960_setProximityGain(PGAIN_2X)) {
		_write("APDS-9960 set prox gain successful\n");
	} else {
		_write("Error during APDS-9960 set prox gain\n");
	}
#endif

#if LIGHT_SENSOR
	uint16_t light_red, light_blue, light_green, light_ambient;

	if (APDS9960_enableLightSensor(false)) {
		_write("APDS-9960 enable light sensor successful\n");
	} else {
		_write("Error during APDS-9960 enable light sensor\n");
	}
#endif

#if PROXIMITY_SENSOR
	while (1) {
		APDS9960_readProximity(&prox);
		val = calculateProximity(prox);
		units = val / 100;
		decimals = (val - (units * 100));
		sprintf(tmp, "%d.%d\n", units, decimals);
		_write(tmp);
	}
#endif

#if LIGHT_SENSOR
	while (1) {
		if (!APDS9960_readRedLight(&light_red) ||
				!APDS9960_readBlueLight(&light_blue) ||
				!APDS9960_readGreenLight(&light_green) ||
				!APDS9960_readAmbientLight(&light_ambient)) {
			_write("Error reading light\n");
		} else {
			sprintf(tmp, "r: %d, g: %d, b: %d, a: %d\n", light_red, light_green, light_blue, light_ambient);
			_write(tmp);
		}
	}
#endif

	/*while (1) {
		if (APDS9960_isGestureAvailable()) {
			_write("Gesture avail\n");
			switch (APDS9960_readGesture()) {
			case DIR_UP:
				_write("UP\n");
				break;
			case DIR_DOWN:
				_write("DOWN\n");
				break;
			case DIR_LEFT:
				_write("LEFT\n");
				break;
			case DIR_RIGHT:
				_write("RIGHT\n");
				break;
			case DIR_NEAR:
				_write("NEAR\n");
				break;
			case DIR_FAR:
				_write("FAR\n");
				break;
			default:
				_write("NONE\n");
			}
		}
	}*/
}

/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();
  /* If first word of user data page is non-zero, enable Energy Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize our I2C Multitask module */
  //I2C_Multitask_Init();
  /* Create main task */
  xTaskCreate(I2C_Task, (const char*) "I2C_Task", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();

  return 0;
}
