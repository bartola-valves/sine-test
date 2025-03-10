/**
 * @file MCP472lib.cpp
 * @author Alejandro Moglia / @bartola-valves /  valves@bartola.co.uk
 * @brief MCP4728 library example for Raspberry Pi Pico
 * @version 0.1
 * @date 2025-02-28
 *
 * @copyright Copyright (c) 2025 by Bartola Ltd.
 *
 */

#include "MCP4728.h"
#include <stdio.h>
#include <math.h>

MCP4728::MCP4728(i2c_inst_t *i2cPort, uint8_t i2cAddr, uint8_t sdaPin, uint8_t sclPin, int8_t ldacPin, int8_t rdyPin)
{
    _i2c = i2cPort;
    _addr = i2cAddr;
    _sdaPin = sdaPin;
    _sclPin = sclPin;
    _ldacPin = ldacPin;
    _rdyPin = rdyPin;
    _initialized = false;
}

bool MCP4728::begin(uint32_t i2cFreq)
{
    // Initialize I2C
    i2c_init(_i2c, i2cFreq);

    // Set up I2C pins
    gpio_set_function(_sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(_sclPin, GPIO_FUNC_I2C);

    // Enable pull-ups on I2C pins (recommended for I2C)
    gpio_pull_up(_sdaPin);
    gpio_pull_up(_sclPin);

    // Initialize LDAC pin if provided
    if (_ldacPin >= 0)
    {
        gpio_init(_ldacPin);
        gpio_set_dir(_ldacPin, GPIO_OUT);
        gpio_put(_ldacPin, 1); // Set high (inactive)
    }

    // Initialize RDY pin if provided
    if (_rdyPin >= 0)
    {
        gpio_init(_rdyPin);
        gpio_set_dir(_rdyPin, GPIO_IN);
        gpio_pull_up(_rdyPin);
    }

    _initialized = true;

    // Test if device is present by attempting to read a byte (ACK check)
    uint8_t dummy;
    int result = i2c_read_blocking(_i2c, _addr, &dummy, 1, false);
    return (result >= 0);
}

bool MCP4728::setVoltage(Channel channel, float voltage, Vref vref, Gain gain, bool storeEEPROM)
{
    // Calculate appropriate reference voltage
    float refVoltage = (vref == VREF_INT) ? 2.048f : 5.0f; // Assuming VDD = 5V

    // Convert voltage to DAC value
    uint16_t value = voltageToValue(voltage, refVoltage, gain);

    // Set the channel
    return setChannel(channel, value, vref, gain, POWER_DOWN_NORMAL, storeEEPROM);
}

bool MCP4728::setChannel(Channel channel, uint16_t value, Vref vref, Gain gain,
                         PowerMode powerMode, bool storeEEPROM)
{
    if (!_initialized)
        return false;

    // Limit value to 12 bits (0-4095)
    value &= 0x0FFF;

    uint8_t buffer[3];

    // First byte: Command + Channel + Write command
    buffer[0] = (storeEEPROM ? CMD_WRITE_DAC_EEPROM : CMD_WRITE_DAC) | (channel << 1);

    // Second byte: Upper 4 bits of the 12-bit value + VREF + Gain + Power-down mode
    buffer[1] = ((vref & 0x01) << 7) | ((gain & 0x01) << 4) |
                ((powerMode & 0x03) << 5) | ((value >> 8) & 0x0F);

    // Third byte: Lower 8 bits of the 12-bit value
    buffer[2] = value & 0xFF;

    // Send the data to the DAC
    int result = i2c_write_blocking(_i2c, _addr, buffer, 3, false);

    return (result == 3);
}

bool MCP4728::setAllChannels(uint16_t values[4], Vref vref, Gain gain, bool storeEEPROM)
{
    if (!_initialized || storeEEPROM)
    {
        // EEPROM write requires using single channel writes
        bool success = true;
        for (int i = 0; i < 4; i++)
        {
            success &= setChannel((Channel)i, values[i], vref, gain, POWER_DOWN_NORMAL, storeEEPROM);
        }
        return success;
    }

    // Fast write for all channels (only works for non-EEPROM writes)
    uint8_t buffer[9];

    // Sequential write command byte
    buffer[0] = CMD_MULTI_WRITE;

    for (int i = 0; i < 4; i++)
    {
        // Limit value to 12 bits
        values[i] &= 0x0FFF;

        // Upper data bits + VREF + Gain + PD bits
        buffer[i * 2 + 1] = ((vref & 0x01) << 7) | ((gain & 0x01) << 4) | (values[i] >> 8);

        // Lower 8 bits of the 12-bit value
        buffer[i * 2 + 2] = values[i] & 0xFF;
    }

    // Send the data to the DAC
    int result = i2c_write_blocking(_i2c, _addr, buffer, 9, false);

    return (result == 9);
}

bool MCP4728::setAllChannelConfigs(ChannelConfig configs[4], bool storeEEPROM)
{
    bool success = true;

    for (int i = 0; i < 4; i++)
    {
        success &= setChannel((Channel)i, configs[i].value, configs[i].vref,
                              configs[i].gain, configs[i].powerMode, storeEEPROM);
    }

    return success;
}

uint16_t MCP4728::voltageToValue(float voltage, float vref, Gain gain, float opAmpGain)
{
    // Calculate max DAC voltage based on reference and gain
    float max_voltage = vref;
    if (gain == GAIN_2X)
        max_voltage *= 2.0;

    // Apply op-amp gain compensation if needed
    if (opAmpGain != 1.0)
    {
        voltage = voltage / opAmpGain;
    }

    // Calculate the DAC value (0-4095) from the voltage
    float normalized_value = voltage / max_voltage;

    // Use round() instead of truncating for better precision
    uint16_t dac_value = (uint16_t)round(normalized_value * 4095.0);

    // Clamp value to valid range
    if (dac_value > 4095)
        dac_value = 4095;

    return dac_value;
}

float MCP4728::valueToVoltage(uint16_t value, float vref, Gain gain, float opAmpGain)
{
    // Ensure value is in range
    value = (value > 4095) ? 4095 : value;

    // Calculate max DAC voltage
    float max_voltage = vref;
    if (gain == GAIN_2X)
        max_voltage *= 2.0;

    // Calculate output voltage
    float voltage = (value / 4095.0f) * max_voltage;

    // Apply op-amp gain if present
    if (opAmpGain != 1.0)
    {
        voltage *= opAmpGain;
    }

    return voltage;
}

void MCP4728::triggerLDAC()
{
    if (_initialized && _ldacPin >= 0)
    {
        gpio_put(_ldacPin, 0); // Pull LDAC low
        busy_wait_us(1);       // Wait for at least 100ns
        gpio_put(_ldacPin, 1); // Return LDAC high
    }
}

bool MCP4728::isReady()
{
    if (_initialized && _rdyPin >= 0)
    {
        return gpio_get(_rdyPin) == 1; // RDY is high when device is ready
    }
    return true; // Assume ready if no RDY pin connected
}

void MCP4728::scanI2C()
{
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr_prefix = 0; addr_prefix < 8; addr_prefix++)
    {
        printf("%d0:", addr_prefix);
        for (int addr_suffix = 0; addr_suffix < 16; addr_suffix++)
        {
            int addr = (addr_prefix << 4) | addr_suffix;

            // Skip reserved addresses
            if ((addr >= 0x00 && addr <= 0x07) || addr >= 0x78)
            {
                printf("   ");
                continue;
            }

            // Try to detect device
            uint8_t rxdata;
            int ret = i2c_read_blocking(_i2c, addr, &rxdata, 1, false);

            printf(ret >= 0 ? " %02X" : " --", addr);
        }
        printf("\n");
    }
}

bool MCP4728::powerDown(uint8_t channels, PowerMode mode)
{
    bool success = true;

    // For each channel that needs to be powered down
    for (int i = 0; i < 4; i++)
    {
        if (channels & (1 << i))
        {
            // Get current value (we need to read the current value first)
            uint8_t readBuf[3];
            i2c_read_blocking(_i2c, _addr, readBuf, 3, false);

            // Set new power mode but keep value
            uint16_t currentVal = ((readBuf[1] & 0x0F) << 8) | readBuf[2];
            success &= setChannel((Channel)i, currentVal,
                                  (Vref)(readBuf[1] >> 7),
                                  (Gain)((readBuf[1] >> 4) & 0x01),
                                  mode, false);
        }
    }

    return success;
}

bool MCP4728::changeAddress(uint8_t newAddr)
{
    if (!_initialized || _ldacPin < 0)
        return false;

    // Ensure address is in valid range (0-7)
    newAddr &= 0x07;

    printf("Current address: 0x%02X, target address: 0x%02X\n",
           _addr, 0x60 | newAddr);

    // Ensure LDAC is low before attempting address change
    if (_ldacPin >= 0)
    {
        printf("Setting LDAC pin %d low\n", _ldacPin);
        gpio_put(_ldacPin, 0);
        busy_wait_ms(5); // Give it time to settle
    }

    // Prepare just the data portion for General Call
    uint8_t data[2];
    data[0] = 0x06;                  // Address change opcode
    data[1] = 0x60 | (newAddr << 1); // 011xxxxx0 where xxx is the new address

    printf("Sending General Call Address Change to 0x00: 0x%02X 0x%02X\n",
           data[0], data[1]);

    // Send to General Call address (0x00)
    int result = i2c_write_blocking(_i2c, 0x00, data, 2, false);

    // Wait for the change to take effect
    busy_wait_ms(50); // Longer wait for EEPROM operations

    // Return LDAC to high state
    if (_ldacPin >= 0)
    {
        gpio_put(_ldacPin, 1);
    }

    // Update stored address
    uint8_t old_addr = _addr;
    _addr = 0x60 | newAddr;

    // Additional delay before testing
    busy_wait_ms(50);

    // Test communication with new address
    printf("Testing communication with new address 0x%02X...\n", _addr);

    uint8_t dummy;
    int test = i2c_read_blocking(_i2c, _addr, &dummy, 1, false);

    if (test < 0)
    {
        printf("Device not responding at new address, reverting to 0x%02X\n", old_addr);
        _addr = old_addr;
        return false;
    }

    printf("Address change confirmed! Device responding at 0x%02X\n", _addr);
    return true;
}