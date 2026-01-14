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

#include "codec.h"

static const char* TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_6

static uint8_t s_led_state = 0;

static led_strip_handle_t led_strip;

void audio_task(void* arg){
    uint32_t bytes_read;
    int16_t sample_buffer[32*2];
    static float phase = 0.0f;
    const float phase_increment = 2.0f * M_PI * 1000.0f / 48000.0f;  // 1kHz at 48kHz sample rate

    while (1) {
        // Calculate number of stereo samples (each sample is 2 int16_t values = 4 bytes)
        int num_samples = 32*2; // 32 int16_t values = 16 stereo samples

        // Read audio data from I2S
        i2s_read(sample_buffer, 32*2, &bytes_read);

        // create 1kHz test tone on both channels
        for (int i = 0; i < num_samples; i += 2) {
            int16_t sample_value = (int16_t)(10000.0f * sinf(phase));
            sample_buffer[i] = sample_value;      // Left channel
            sample_buffer[i + 1] = sample_value;  // Right channel

            // Increment phase and wrap around 2*PI
            phase += phase_increment;
            if (phase >= 2.0f * M_PI) {
                phase -= 2.0f * M_PI;
            }
        }

        // Write audio data back to I2S
        i2s_write(sample_buffer, 32*2, &bytes_read);
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

    InitCodec();
    SetOutputLevels(58, 58);

    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 5, NULL, 1);

    while (1){
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
