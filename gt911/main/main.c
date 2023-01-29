/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "gt911.h"

#define TAG "Main"

int x,y;

void app_main(void)
{
	ESP_LOGI(TAG, "Ready");

    gt911_init(GT911_I2C_SLAVE_ADDR);

    while(1)
    {
    	if (gt911_read(&x, &y))
    	{
    		ESP_LOGI(TAG, "X: %d, Y: %d", x, y);
    		vTaskDelay(10 / portTICK_PERIOD_MS);
    	}
    }
}
