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

QueueHandle_t queue_t1_t2;
QueueHandle_t queue_t2_t3;

struct prox_rgb_raw {
	uint8_t prox;
	uint16_t light_red, light_blue, light_green, light_ambient;
};

struct prox_rgb_parsed {
	char prox[5];
	uint8_t prox_units;
	uint32_t rgba_hex;
};

void _write(const char *ptr) {
	do {
		ITM_SendChar(*ptr++);
	} while (*ptr != '\n');
	ITM_SendChar('\n');
}

int map(uint8_t x, float in_min, float in_max, float out_min, float out_max) {
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min) * 100;
}

float mapTruncate(uint8_t x, uint16_t min, uint16_t max, float in_min, float in_max, float out_min, float out_max) {
	if (x >= max) {
		return map(max, in_min, in_max, out_min, out_max);
	}

	if (x <= min) {
		return map(min, in_min, in_max, out_min, out_max);
	}

	return map(x, in_min, in_max, out_min, out_max);
}

float calculateProximity(uint8_t proxVal) {
	return mapTruncate(proxVal, 20, 254, 254, 20, 3, 15);
}

static bool initAPDS9960Functions() {
	/* Init APDS-9960 I2C module */
	if (APDS9960_init()) {
		_write("APDS-9960 initialization complete\n");
	} else {
		_write("Error during APDS-9960 init\n");
		return false;
	}

	/* Enable APDS-9960 proximity sensor function */
	if (APDS9960_enableProximitySensor(false)) {
		_write("APDS-9960 enable proximity successful\n");
	} else {
		_write("Error during APDS-9960 enable proximity\n");
		return false;
	}

	/* Configure proximity sensor function gain according to library */
	if (APDS9960_setProximityGain(PGAIN_2X)) {
		_write("APDS-9960 set prox gain successful\n");
	} else {
		_write("Error during APDS-9960 set prox gain\n");
		return false;
	}

	/* Eanble APDS-9960 light (rgb) sensor function */
	if (APDS9960_enableLightSensor(false)) {
		_write("APDS-9960 enable light sensor successful\n");
	} else {
		_write("Error during APDS-9960 enable light sensor\n");
		return false;
	}

	return true;
}

/* Read proxmity, rgb from sensor & add raw values to queue */
static void Task_1() {
	struct prox_rgb_raw send_values;
	//char tmp[30];

	while (1) {
		APDS9960_readProximity(&send_values.prox);

		if (!APDS9960_readRedLight(&send_values.light_red) ||
				!APDS9960_readBlueLight(&send_values.light_blue) ||
				!APDS9960_readGreenLight(&send_values.light_green) ||
				!APDS9960_readAmbientLight(&send_values.light_ambient)) {
			_write("Error reading light\n");

			send_values.light_red = send_values.light_blue = send_values.light_green = send_values.light_ambient = 0;
		}

		xQueueSend(queue_t1_t2, &send_values, 0);

		vTaskDelay(300 / portTICK_PERIOD_MS); // 300ms delay
	}
}

/* Read raw proximity & rgb values from queue, convert to readable values, send to queue */
static void Task_2() {
	struct prox_rgb_raw read_values;
	struct prox_rgb_parsed send_values;
	int val, decimals;

	while (1) {
		if (xQueueReceive(queue_t1_t2, &read_values, 0) == pdPASS) {
			val = calculateProximity(read_values.prox);
			send_values.prox_units = val / 100;
			decimals = (val - (send_values.prox_units * 100));
			sprintf(send_values.prox, "%d.%d", send_values.prox_units, decimals);

			send_values.rgba_hex = 0;

			// r
			// INCORRECT VALUES!
			send_values.rgba_hex |= (uint8_t)mapTruncate(read_values.light_red, 0, 37889, 37889, 0, 0, 255) << 24;
			// g
			send_values.rgba_hex |= (uint8_t)mapTruncate(read_values.light_green, 0, 37889, 37889, 0, 0, 255) << 16;
			// b
			send_values.rgba_hex |= (uint8_t)mapTruncate(read_values.light_blue, 0, 37889, 37889, 0, 0, 255) << 8;
			// a
			send_values.rgba_hex |= (uint8_t)mapTruncate(read_values.light_ambient, 0, 37889, 37889, 0, 0, 255);

			xQueueSend(queue_t2_t3, &send_values, 0);
		}

		vTaskDelay(50 / portTICK_PERIOD_MS); // 50ms delay
	}
}

/* Read readable values from queue, react to proximity with LED, write values to console */
static void Task_3() {
	struct prox_rgb_parsed read_values;
	uint8_t led_status = 0;
	char tmp[30];

	while (1) {
		if (xQueueReceive(queue_t2_t3, &read_values, 0) == pdPASS) {
			sprintf(tmp, "Prox: %s, Hex: %x\n", read_values.prox, (unsigned int)read_values.rgba_hex);
			_write(tmp);

			if (led_status == 1 && read_values.prox_units > 8) {
				led_status = 0;

				BSP_LedClear(0);
			} else if (led_status == 0 && read_values.prox_units < 7) {
				led_status = 1;

				BSP_LedSet(0);
			}
		}

		vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms delay
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

  /* Init LED driver */
  BSP_LedsInit();

  /* Init & enable APDS-9960 capabilities */
  if (!initAPDS9960Functions()) {
  	  while (1);
  }

  /* Create queues for tasks */
  queue_t1_t2 = xQueueCreate(5, sizeof(struct prox_rgb_raw));
  queue_t2_t3 = xQueueCreate(5, sizeof(struct prox_rgb_parsed));

  /* Initialize our I2C Multitask module */
  //I2C_Multitask_Init();

  /* Create tasks */
  xTaskCreate(Task_1, (const char*) "Task 1", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);
  xTaskCreate(Task_2, (const char*) "Task 2", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);
  xTaskCreate(Task_3, (const char*) "Task 3", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();

  return 0;
}
