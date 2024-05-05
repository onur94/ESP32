/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "Main";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (12 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT       2
#define EXAMPLE_PIN_NUM_HSYNC          39
#define EXAMPLE_PIN_NUM_VSYNC          41
#define EXAMPLE_PIN_NUM_DE             40
#define EXAMPLE_PIN_NUM_PCLK           42
#define EXAMPLE_PIN_NUM_DATA0          8 // B0
#define EXAMPLE_PIN_NUM_DATA1          3 // B1
#define EXAMPLE_PIN_NUM_DATA2          46 // B2
#define EXAMPLE_PIN_NUM_DATA3          9 // B3
#define EXAMPLE_PIN_NUM_DATA4          1 // B4
#define EXAMPLE_PIN_NUM_DATA5          5 // G0
#define EXAMPLE_PIN_NUM_DATA6          6 // G1
#define EXAMPLE_PIN_NUM_DATA7          7 // G2
#define EXAMPLE_PIN_NUM_DATA8          15 // G3
#define EXAMPLE_PIN_NUM_DATA9          16 // G4
#define EXAMPLE_PIN_NUM_DATA10         4 // G5
#define EXAMPLE_PIN_NUM_DATA11         45 // R0
#define EXAMPLE_PIN_NUM_DATA12         48 // R1
#define EXAMPLE_PIN_NUM_DATA13         47 // R2
#define EXAMPLE_PIN_NUM_DATA14         21 // R3
#define EXAMPLE_PIN_NUM_DATA15         14 // R4
#define EXAMPLE_PIN_NUM_DISP_EN		   -1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              480
#define EXAMPLE_LCD_V_RES              272

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB             2
#else
#define EXAMPLE_LCD_NUM_FB             1
#endif // CONFIG_EXAMPLE_DOUBLE_FB

// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

uint16_t color[480*272];

void app_main(void)
{
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
	ESP_LOGI(TAG, "Create semaphores");
	sem_vsync_end = xSemaphoreCreateBinary();
	assert(sem_vsync_end);
	sem_gui_ready = xSemaphoreCreateBinary();
	assert(sem_gui_ready);
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
	ESP_LOGI(TAG, "Turn off LCD backlight");
	gpio_config_t bk_gpio_config = {
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
	};
	ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

	ESP_LOGI(TAG, "Install RGB LCD panel driver");
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_rgb_panel_config_t panel_config = {
		.data_width = 16, // RGB565 in parallel mode, thus 16bit in width
		.psram_trans_align = 64,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
		.bounce_buffer_size_px = 100 * EXAMPLE_LCD_H_RES * sizeof(lv_color_t),
#endif
		.clk_src = LCD_CLK_SRC_DEFAULT,
		.disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
		.pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
		.vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
		.hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
		.de_gpio_num = EXAMPLE_PIN_NUM_DE,
		.data_gpio_nums = {
			EXAMPLE_PIN_NUM_DATA0,
			EXAMPLE_PIN_NUM_DATA1,
			EXAMPLE_PIN_NUM_DATA2,
			EXAMPLE_PIN_NUM_DATA3,
			EXAMPLE_PIN_NUM_DATA4,
			EXAMPLE_PIN_NUM_DATA5,
			EXAMPLE_PIN_NUM_DATA6,
			EXAMPLE_PIN_NUM_DATA7,
			EXAMPLE_PIN_NUM_DATA8,
			EXAMPLE_PIN_NUM_DATA9,
			EXAMPLE_PIN_NUM_DATA10,
			EXAMPLE_PIN_NUM_DATA11,
			EXAMPLE_PIN_NUM_DATA12,
			EXAMPLE_PIN_NUM_DATA13,
			EXAMPLE_PIN_NUM_DATA14,
			EXAMPLE_PIN_NUM_DATA15,
		},
		.timings = {
			.pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
			.h_res = EXAMPLE_LCD_H_RES,
			.v_res = EXAMPLE_LCD_V_RES,
			// The following parameters should refer to LCD spec
			.hsync_back_porch = 43,
			.hsync_front_porch = 8,
			.hsync_pulse_width = 4,
			.vsync_back_porch = 12,
			.vsync_front_porch = 8,
			.vsync_pulse_width = 4,
			.flags.pclk_active_neg = true,
		},
		.flags.fb_in_psram = true, // allocate frame buffer in PSRAM
#if CONFIG_EXAMPLE_DOUBLE_FB
		.flags.double_fb = true,   // allocate double frame buffer
#endif // CONFIG_EXAMPLE_DOUBLE_FB
	};
	ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

	ESP_LOGI(TAG, "Register event callbacks");
	esp_lcd_rgb_panel_event_callbacks_t cbs = {
		.on_vsync = example_on_vsync_event,
	};

	ESP_LOGI(TAG, "Initialize RGB LCD panel");
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
	ESP_LOGI(TAG, "Turn on LCD backlight");
	gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    void *buf1 = NULL;
	void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
	ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
	ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
#else
	ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
	buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
	assert(buf1);
	buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
	assert(buf2);
#endif // CONFIG_EXAMPLE_DOUBLE_FB
    
	while (1) {
        ESP_LOGI(TAG, "Red color");
        for (size_t i = 0; i < 480*272; i++)
        {
            color[i] = 0xF800;
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 272, (void *)color);
        vTaskDelay(pdMS_TO_TICKS(1500));

        ESP_LOGI(TAG, "Green color");
        for (size_t i = 0; i < 480*272; i++)
        {
            color[i] = 0x7E0;
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 272, (void *)color);
		vTaskDelay(pdMS_TO_TICKS(1500));

        ESP_LOGI(TAG, "Blue color");
        for (size_t i = 0; i < 480*272; i++)
        {
            color[i] = 0x1F;
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 272, (void *)color);
		vTaskDelay(pdMS_TO_TICKS(1500));
	}
}
