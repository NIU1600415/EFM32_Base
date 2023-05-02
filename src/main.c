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

void _write(const char *ptr, int len) {
	int x;
	for (x = 0; x < len; x++) {
		ITM_SendChar(*ptr++);
	}
}

static void I2C_Task() {
	/* Init APDS-9960 I2C module */
	if (APDS9960_Init()) {
		_write("APDS-9960 initialization complete\n");
	} else {
		_write("Error during APDS-9960 init\n");
	}

	if (APDS9960_enableGestureSensor()) {
		_write("APDS-9960 enable gestures successful\n");
	} else {
		_write("Error during APDS-9960 enable gestures\n");
	}

	while (1) {
		if (APDS9960_isGestureAvailable()) {
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
	}
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
