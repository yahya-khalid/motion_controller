/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "mpu6886.h"
#include "sdkconfig.h"

static const char *TAG = "demo";

#define GYROP_FILTER_VAL 5
#define GYROR_FILTER_VAL 5

bool rl_flag = false;
bool init_kb = false;

#define MOUSE_ACTIVE_PIN GPIO_NUM_5
#define MOUSE_LB_PIN GPIO_NUM_41

#define SENSITIVITY_X 0.7 ///< 70% sensitivity
#define SENSITIVITY_Y 0.7 ///< 70% sensitivity
/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD) ),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE) )
};

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};


/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
    //TUD_CDC_DESCRIPTOR(1, 4, 0x81, 8, 0x02, 0x82, 64),
};


/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}


void imu_mouse(int8_t gyro_x, int8_t gyro_y)
{
    //printf("acc: %f, %f, %f, gyro: %f, %f, %f\n", acc_x, acc_y, acc_z, gyro_r, gyro_y, gyro_p);
    if (tud_mounted())
    {
        if (!gpio_get_level(MOUSE_ACTIVE_PIN))
        {
            gyro_x = 0;
            gyro_y = 0;
            //tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
        }
        else
        {
            // send keyboard input for forward
            //uint8_t keycode[6] = {HID_KEY_W};
            //tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
        if (!gpio_get_level(MOUSE_LB_PIN))
        {
            tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, MOUSE_BUTTON_LEFT, gyro_x, gyro_y, 0, 0);
        }
        else
        {
            tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, gyro_x, gyro_y, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void app_main(void)
{
    const gpio_config_t mouse_active_config = {
        .pin_bit_mask = BIT64(MOUSE_ACTIVE_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = true,
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&mouse_active_config));

    const gpio_config_t mouse_lb_config = {
        .pin_bit_mask = BIT64(MOUSE_LB_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = true,
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&mouse_lb_config));

    ESP_LOGI(TAG, "USB initialization");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    mpu6886_init();
    

    float ax, ay, az;
    float gr, gy, gp;

    int8_t mouse_x, mouse_y;

    while (1)
    {
        //mpu6886_accel_data_get(&ax, &ay, &az);
        mpu6886_gyro_data_get(&gr, &gy, &gp);

        gp = gp * SENSITIVITY_X;
        gr = gr * SENSITIVITY_Y;
        if (gp > GYROP_FILTER_VAL || gp < -GYROP_FILTER_VAL)
        {
            if (gp < 0)
            {
                mouse_x = (int8_t)((gp < -127) ? -127 : gp);
            }
            else if (gp > 0)
            {
                mouse_x = (int8_t)((gp > 127) ? 127 : gp);
            }
            else
            {
                mouse_x = 0;
            }
        }
        else
        {
            mouse_x = 0;
        }

        if (gr > GYROR_FILTER_VAL || gr < -GYROR_FILTER_VAL)
        {
            if (gr < 0)
            {
                mouse_y = (int8_t)((gr < -127) ? -127 : gr);
            }
            else if (gr > 0)
            {
                mouse_y = (int8_t)((gr > 127) ? 127 : gr);
            }
            else
            {
                mouse_y = 0;
            }
        }
        else
        {
            mouse_y = 0;
        }

        imu_mouse(-mouse_x, -mouse_y);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}