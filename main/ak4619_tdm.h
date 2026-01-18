#pragma once
#include "esp_err.h"
#include <stdint.h>

esp_err_t ak4619_tdm_init(void);
esp_err_t ak4619_read_register(uint8_t reg, uint8_t *val);
esp_err_t ak4619_write_register(uint8_t reg, uint8_t val);

// I2S read/write wrappers compatible with TLV codec interface
void ak4619_i2s_read(void *buf, uint32_t size, uint32_t *bytes_read);
void ak4619_i2s_write(void *buf, uint32_t size, uint32_t *bytes_written);

