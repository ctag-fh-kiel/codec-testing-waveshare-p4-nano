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

#ifdef CONFIG_CODEC_TLV320AIC3254
    #include "codec.h"
    #define CODEC_INIT() InitCodec()
    #define CODEC_I2S_READ(buf, size, bytes_read) i2s_read(buf, size, bytes_read)
    #define CODEC_I2S_WRITE(buf, size, bytes_written) i2s_write(buf, size, bytes_written)
    #define CODEC_SET_OUTPUT_LEVELS(left, right) SetOutputLevels(left, right)
#elif CONFIG_CODEC_AK4619
    #include "ak4619_tdm.h"
    #define CODEC_INIT() ak4619_tdm_init()
    #define CODEC_I2S_READ(buf, size, bytes_read) ak4619_i2s_read(buf, size, bytes_read)
    #define CODEC_I2S_WRITE(buf, size, bytes_written) ak4619_i2s_write(buf, size, bytes_written)
    #define CODEC_SET_OUTPUT_LEVELS(left, right) // AK4619 output levels set in init
#else
    #error "No codec selected in menuconfig"
#endif

static const char* TAG = "MAIN";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_6

atomic_int ext_int = 0;

void audio_task(void* arg){
    uint32_t bytes_read;

#ifdef CONFIG_CODEC_AK4619
    // AK4619 uses 32-bit samples in TDM mode (4 slots: DAC1_L, DAC1_R, DAC2_L, DAC2_R)
    int32_t sample_buffer[32*4];  // 32 frames × 4 channels = 1024 int32_t samples
#else
    // TLV320AIC3254 uses 16-bit stereo samples
    int16_t sample_buffer[32*2];  // 32 stereo frames = 512 int16_t samples
#endif

    static float phase = 0.0f;
    const float phase_increment = 2.0f * M_PI * 440.0f / 48000.0f;  // 440Hz test tone at 48kHz sample rate

    while (1) {
        // Read audio data from I2S
        size_t buffer_bytes = sizeof(sample_buffer);
        CODEC_I2S_READ(sample_buffer, buffer_bytes, &bytes_read);

        // Generate test tone when ext_int is 0 (or pass through when ext_int is 1)
        if (!ext_int){
#ifdef CONFIG_CODEC_AK4619
            // AK4619: Generate 440Hz sine wave for all 4 TDM channels (32-bit samples)
            int num_frames = 32;
            for (int i = 0; i < num_frames; i++) {
                // 32-bit sample value (use upper bits for better SNR)
                int32_t sample_value = (int32_t)(2147483647.0f * sinf(phase));

                // Fill all 4 TDM slots: DAC1_L, DAC1_R, DAC2_L, DAC2_R
                sample_buffer[i * 4 + 0] = sample_value;  // DAC1 Left
                sample_buffer[i * 4 + 1] = sample_value;  // DAC1 Right
                sample_buffer[i * 4 + 2] = sample_value;  // DAC2 Left
                sample_buffer[i * 4 + 3] = sample_value;  // DAC2 Right

                // Increment phase and wrap around 2*PI
                phase += phase_increment;
                if (phase >= 2.0f * M_PI) {
                    phase -= 2.0f * M_PI;
                }
            }
#else
            // TLV320AIC3254: Generate 440Hz sine wave for stereo (16-bit samples)
            int num_frames = 32;
            for (int i = 0; i < num_frames; i++) {
                int16_t sample_value = (int16_t)(32767.0f * sinf(phase));
                sample_buffer[i * 2] = sample_value;      // Left channel
                sample_buffer[i * 2 + 1] = sample_value;  // Right channel

                // Increment phase and wrap around 2*PI
                phase += phase_increment;
                if (phase >= 2.0f * M_PI) {
                    phase -= 2.0f * M_PI;
                }
            }
#endif
        }

        // Write audio data back to I2S
        CODEC_I2S_WRITE(sample_buffer, buffer_bytes, &bytes_read);
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

    // Initialize selected codec
    CODEC_INIT();

#ifdef CONFIG_CODEC_TLV320AIC3254
    // Set output levels for TLV codec
    CODEC_SET_OUTPUT_LEVELS(75, 75);
#elif CONFIG_CODEC_AK4619
    // Example: Set AK4619 DAC volume
    // Option 1: Set volume using dB values (recommended)
    // ak4619_set_dac1_volume(ak4619_db_to_volume(0.0f), ak4619_db_to_volume(0.0f));   // 0dB (unity gain)
    // ak4619_set_dac2_volume(ak4619_db_to_volume(-6.0f), ak4619_db_to_volume(-6.0f)); // -6dB

    // Option 2: Set volume using register values directly
    // ak4619_set_dac1_volume(0x18, 0x18);  // 0dB (default)
    // ak4619_set_dac2_volume(0x18, 0x18);  // 0dB (default)

    // Option 3: Set all DAC channels to same volume
    // ak4619_set_all_dac_volume(0x18);  // 0dB on all channels
    //ak4619_set_all_dac_volume(ak4619_db_to_volume(-12.0f));  // -3dB on all channels
#endif

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
