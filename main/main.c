/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "math.h"
#include "esp_ldo_regulator.h"
#include <stdatomic.h>

#include "codec.h"

static const char* TAG = "MAIN";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_6

atomic_int ext_int = 0;

void audio_task(void* arg){
    uint32_t bytes_read;
    int16_t sample_buffer[256*2];  // 256 stereo frames = 512 int16_t samples
    static float phase = 0.0f;
    const float phase_increment = 2.0f * M_PI * 440.0f / 48000.0f;  // 1kHz at 48kHz sample rate

    while (1) {
        // 256 stereo frames = 512 int16_t samples = 1024 bytes
        size_t buffer_bytes = sizeof(sample_buffer);

        // Read audio data from I2S
        i2s_read(sample_buffer, buffer_bytes, &bytes_read);

        // Generate 1kHz test tone on both channels
        // Each stereo frame has 2 samples (left and right)
        if (!ext_int){
            int num_frames = sizeof(sample_buffer) / sizeof(int16_t) / 2;
            for (int i = 0; i < num_frames; i++) {
                int16_t sample_value = (int16_t)(65535.0f / 2.f * sinf(phase));
                sample_buffer[i * 2] = sample_value;      // Left channel
                sample_buffer[i * 2 + 1] = sample_value;  // Right channel

                // Increment phase and wrap around 2*PI
                phase += phase_increment;
                if (phase >= 2.0f * M_PI) {
                    phase -= 2.0f * M_PI;
                }
            }
        }

        // Write audio data back to I2S
        i2s_write(sample_buffer, buffer_bytes, &bytes_read);
    }
}

void blink_task(void* arg){
    while (1){
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void){
    // config led gpio as output
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BLINK_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_35);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure LDO 4 to 3.3V
    ESP_LOGI(TAG, "Configuring LDO 4 to 3.3V");
    esp_ldo_channel_handle_t ldo4_chan = NULL;
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = 4,           // LDO channel 4
        .voltage_mv = 3300,     // 3.3V in millivolts
        .flags = {
            .adjustable = false,  // Not adjustable after acquisition
        }
    };
    esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &ldo4_chan);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LDO 4 configured to 3.3V successfully");
    } else {
        ESP_LOGE(TAG, "Failed to configure LDO 4: %s", esp_err_to_name(ret));
    }

    InitCodec();
    SetOutputLevels(75, 75);

    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 15, NULL, 1);
    xTaskCreatePinnedToCore(blink_task, "blink_task", 8192, NULL, 5, NULL, 0);

    bool prev_button_state = gpio_get_level(GPIO_NUM_35);
    while (1){
        bool curr_button_state = gpio_get_level(GPIO_NUM_35);
        if (curr_button_state != prev_button_state){
            prev_button_state = curr_button_state;
            if (curr_button_state == 0){
                ext_int = !ext_int;
                ESP_LOGI(TAG, "ext_int set to %d", ext_int);
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
