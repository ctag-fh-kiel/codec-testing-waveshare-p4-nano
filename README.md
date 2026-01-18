# ESP32-P4 Audio Cape

A high-quality audio interface project for the ESP32-P4 microcontroller featuring professional audio codec support with TDM (Time Division Multiplexing) capabilities.

## Overview

This project implements a full-duplex audio interface using the ESP32-P4's I2S peripheral in TDM mode, supporting professional audio codecs for high-quality multi-channel audio processing. The project demonstrates audio passthrough, test tone generation, and provides a foundation for advanced audio applications.

## Supported Hardware

### Audio Codecs

- **AK4619VN** (Primary, TDM Mode)
  - 4-channel ADC (24-bit resolution)
  - 4-channel DAC (32-bit resolution)
  - TDM256 interface with I2S-compatible framing
  - Sample rate: 48 kHz
  - MCLK: 384fs (18.432 MHz)
  
- **TLV320AIC3254** (Legacy Support)
  - 2-channel stereo codec
  - 16-bit samples
  - Standard I2S interface

### Target Board

- **ESP32-P4** waveshare nano p4 development board
- LDO 4 configured to 3.3V for codec power supply

## Features

- **Full-duplex audio processing** with simultaneous ADC/DAC operation
- **TDM mode support** for multi-channel audio (4 slots × 32-bit)
- **Audio passthrough mode** for transparent ADC→DAC routing
- **440Hz test tone generator** for system verification
- **Button-controlled mode switching** (GPIO 35)
- **Status LED** (GPIO 6) for visual feedback
- **I2C codec configuration** with register readback verification
- **Dual-core architecture**: Audio processing on Core 1, UI tasks on Core 0

## Hardware Configuration

### AK4619 Pin Mapping

#### I2S/TDM Interface
- **MCLK**: GPIO 22 (Master Clock, 18.432 MHz)
- **BCLK**: GPIO 47 (Bit Clock)
- **WS/LRCLK**: GPIO 48 (Word Select / Frame Sync)
- **DOUT**: GPIO 46 (Data Out from AK4619 ADC)
- **DIN**: GPIO 23 (Data In to AK4619 DAC)

#### I2C Control Interface
- **SDA**: GPIO 7
- **SCL**: GPIO 8
- **I2C Address**: 0x10 (7-bit)
- **Clock Speed**: 400 kHz

#### Control Pins
- **RESET**: GPIO 53 (optional hardware reset)

### Other GPIO
- **LED**: GPIO 6 (status indicator)
- **Button**: GPIO 35 (mode switch, active low with pull-up)

## Audio Data Format

### TDM Slot Configuration

The AK4619 operates in TDM256 mode with 4 slots:

| Slot | Channel | Bit Width | Description |
|------|---------|-----------|-------------|
| 0 | DAC1/ADC1 Left | 32-bit | Left channel, first stereo pair |
| 1 | DAC1/ADC1 Right | 32-bit | Right channel, first stereo pair |
| 2 | DAC2/ADC2 Left | 32-bit | Left channel, second stereo pair |
| 3 | DAC2/ADC2 Right | 32-bit | Right channel, second stereo pair |

### Bit Width Handling

- **ADC Output**: 24-bit samples from AK4619, automatically padded to 32-bit by ESP32 I2S driver
- **DAC Input**: 32-bit samples, full resolution utilized
- **Memory Format**: 32-bit signed integers (`int32_t`), MSB-first on wire
- **Alignment**: MSB-aligned with automatic zero-padding for ADC data

> **Note**: The AK4619 ADC natively produces 24-bit samples which are left-aligned in the 32-bit TDM slot. The ESP32 I2S driver handles this automatically, padding the lower 8 bits with zeros. This ensures transparent passthrough compatibility with the 32-bit DAC input.

## Software Architecture

### Task Structure

1. **Audio Task** (Core 1, Priority 15)
   - Reads 32 frames (128 samples) from I2S RX
   - Processes audio: passthrough or test tone generation
   - Writes processed audio to I2S TX
   - Buffer size: 32 frames × 4 channels × 4 bytes = 512 bytes

2. **Blink Task** (Core 0, Priority 5)
   - Toggles status LED every second
   - Provides visual system heartbeat

3. **Main Loop** (Core 0)
   - Monitors button state (GPIO 35)
   - Toggles between passthrough and test tone modes
   - Debouncing with 50ms polling

### Operating Modes

- **Mode 0 (Test Tone)**: Generates 440Hz sine wave on all 4 DAC channels
- **Mode 1 (Passthrough)**: Routes ADC input directly to DAC output

## Building and Flashing

### Prerequisites

- ESP-IDF v5.5.1 or later
- ESP32-P4 development board
- AK4619 audio codec cape

### Build Steps

```bash
# Set up ESP-IDF environment
. ~/esp/esp-idf/export.sh

# Configure the project (optional, defaults to AK4619)
idf.py menuconfig
# Navigate to: Example Configuration -> Select Audio Codec

# Build the project
idf.py build

# Flash to ESP32-P4
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration Options

Use `idf.py menuconfig` to configure:

- **Example Configuration → Select Audio Codec**
  - AK4619 (TDM Mode) - Default
  - TLV320AIC3254 (Standard)

## Register Configuration Details

### AK4619 Initialization Sequence

1. **Power Management** (REG 0x00)
   - Powers up ADC1, ADC2, DAC1, DAC2
   - Releases soft reset

2. **Audio Interface Format** (REG 0x01)
   - TDM=1, DCF=010 (I2S compatible)
   - DSL=11 (32-bit slots)
   - BCKP=0, SDOPH=0 (standard polarity)

3. **Audio Interface Format 2** (REG 0x02)
   - DIDL=11 (32-bit DAC input)
   - DODL=00 (24-bit ADC output)
   - SLOT=1 (slot-based framing)

4. **System Clock** (REG 0x03)
   - FS=010 (48kHz with 384fs MCLK)

5. **ADC Input Select** (REG 0x0B)
   - All inputs configured for single-ended operation

## Performance Characteristics

- **Latency**: ~0.67ms (32 samples @ 48kHz)
- **Dynamic Range**: 24-bit ADC, 32-bit DAC
- **Channels**: 4 input, 4 output simultaneous
- **Sample Rate**: 48 kHz
- **Bit Clock**: 6.144 MHz (128fs for TDM256)

## Known Issues & Limitations

- Volume control registers currently commented out (default levels used)
- Hardware reset pin optional (software reset via I2C)
- Single sample rate support (48kHz)

## Future Enhancements

- [ ] Dynamic sample rate switching
- [ ] Volume control via I2C
- [ ] Advanced DSP processing (EQ, compression, effects)
- [ ] USB audio class interface
- [ ] Web-based configuration interface
- [ ] Multi-rate audio mixing

## References

- [AK4619VN Datasheet](https://www.mouser.com/datasheet/2/1431/ak4619vn_en_datasheet-3010069.pdf)
- [ESP32-P4 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-p4_technical_reference_manual_en.pdf)
- [ESP-IDF I2S Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/i2s.html)

## License

This project is in the Public Domain (or CC0 licensed, at your option).

## Authors

- ESP-IDF blink example base
- AK4619 TDM integration and audio processing

