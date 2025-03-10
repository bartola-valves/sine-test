/**
 * MCP4728 Quad-Channel 12-Bit DAC Driver
 *
 * This library provides an interface to the MCP4728 I2C DAC.
 * Features:
 * - Control of all four DAC channels
 * - Support for internal and external voltage references
 * - Configurable gain settings
 * - Power-down mode control
 * - EEPROM storage capabilities
 */

#ifndef MCP4728_H
#define MCP4728_H

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/**
 * MCP4728 DAC Channel Class
 */
class MCP4728
{
public:
    // DAC channels
    typedef enum
    {
        CHANNEL_A = 0,
        CHANNEL_B = 1,
        CHANNEL_C = 2,
        CHANNEL_D = 3
    } Channel;

    // Reference voltage options
    typedef enum
    {
        VREF_VDD = 0, // Use VDD as reference
        VREF_INT = 1  // Use internal 2.048V reference
    } Vref;

    // Gain options
    typedef enum
    {
        GAIN_1X = 0, // 1x gain
        GAIN_2X = 1  // 2x gain
    } Gain;

    // Power-down modes
    typedef enum
    {
        POWER_DOWN_NORMAL = 0, // Normal operation
        POWER_DOWN_1K = 1,     // 1kΩ pull-down resistor to GND
        POWER_DOWN_100K = 2,   // 100kΩ pull-down resistor to GND
        POWER_DOWN_500K = 3    // 500kΩ pull-down resistor to GND
    } PowerMode;

    // Channel configuration
    typedef struct
    {
        Vref vref;           // Voltage reference selection
        Gain gain;           // Gain setting
        PowerMode powerMode; // Power-down mode
        uint16_t value;      // DAC value (0-4095)
    } ChannelConfig;

    /**
     * Constructor
     *
     * @param i2cPort The I2C instance to use (i2c0 or i2c1)
     * @param i2cAddr The I2C address of the DAC (default 0x60)
     * @param sdaPin The GPIO pin for I2C SDA
     * @param sclPin The GPIO pin for I2C SCL
     * @param ldacPin The GPIO pin for LDAC (optional, -1 if not used)
     * @param rdyPin The GPIO pin for RDY (optional, -1 if not used)
     */
    MCP4728(i2c_inst_t *i2cPort = i2c0, uint8_t i2cAddr = 0x60,
            uint8_t sdaPin = 4, uint8_t sclPin = 5,
            int8_t ldacPin = 6, int8_t rdyPin = 7);

    /**
     * Initialize the MCP4728 DAC
     *
     * @param i2cFreq The I2C frequency in Hz (default 100kHz)
     * @return true if initialization was successful
     */
    bool begin(uint32_t i2cFreq = 100000);

    /**
     * Set a single channel to a specific voltage
     *
     * @param channel The DAC channel (CHANNEL_A - CHANNEL_D)
     * @param voltage The desired output voltage
     * @param vref The voltage reference to use
     * @param gain The gain setting to use
     * @param storeEEPROM Whether to store settings in EEPROM
     * @return true if successful
     */
    bool setVoltage(Channel channel, float voltage, Vref vref, Gain gain, bool storeEEPROM = false);

    /**
     * Set a single channel using a raw 12-bit value
     *
     * @param channel The DAC channel (CHANNEL_A - CHANNEL_D)
     * @param value The 12-bit DAC value (0-4095)
     * @param vref The voltage reference to use
     * @param gain The gain setting to use
     * @param powerMode The power-down mode
     * @param storeEEPROM Whether to store settings in EEPROM
     * @return true if successful
     */
    bool setChannel(Channel channel, uint16_t value, Vref vref, Gain gain,
                    PowerMode powerMode = POWER_DOWN_NORMAL, bool storeEEPROM = false);

    /**
     * Set all four channels at once (faster than setting individually)
     *
     * @param values Array of four 12-bit DAC values
     * @param vref The voltage reference to use for all channels
     * @param gain The gain setting to use for all channels
     * @param storeEEPROM Whether to store settings in EEPROM
     * @return true if successful
     */
    bool setAllChannels(uint16_t values[4], Vref vref, Gain gain, bool storeEEPROM = false);

    /**
     * Set all channels to specific configurations
     *
     * @param configs Array of four channel configurations
     * @param storeEEPROM Whether to store settings in EEPROM
     * @return true if successful
     */
    bool setAllChannelConfigs(ChannelConfig configs[4], bool storeEEPROM = false);

    /**
     * Convert a voltage to a 12-bit DAC value
     *
     * @param voltage The desired output voltage
     * @param vref The reference voltage value (2.048V for internal, typically 5V for VDD)
     * @param gain The gain setting
     * @param opAmpGain Optional op-amp gain to factor in (for output scaling)
     * @return 12-bit DAC value (0-4095)
     */
    static uint16_t voltageToValue(float voltage, float vref, Gain gain, float opAmpGain = 1.0);

    /**
     * Convert a 12-bit DAC value to output voltage
     *
     * @param value The 12-bit DAC value (0-4095)
     * @param vref The reference voltage value
     * @param gain The gain setting
     * @param opAmpGain Optional op-amp gain to factor in
     * @return Output voltage
     */
    static float valueToVoltage(uint16_t value, float vref, Gain gain, float opAmpGain = 1.0);

    /**
     * Trigger LDAC to update all DAC outputs simultaneously
     * (Only works if LDAC pin is connected)
     */
    void triggerLDAC();

    /**
     * Read the RDY/BSY pin to check if DAC is ready
     * (Only works if RDY pin is connected)
     *
     * @return true if DAC is ready
     */
    bool isReady();

    /**
     * Scan the I2C bus for connected devices
     * Useful for debugging
     */
    void scanI2C();

    /**
     * Enter power-down mode for specified channels
     *
     * @param channels Bitfield of channels to power down (bit 0 = CH A, bit 1 = CH B, etc.)
     * @param mode Power-down mode to enter
     * @return true if successful
     */
    bool powerDown(uint8_t channels, PowerMode mode);

    // Add this to the public section of your MCP4728 class
    /**
     * Change the I2C address of the device using LDAC pin selection
     *
     * The device with LDAC pin pulled LOW will accept this command
     * Devices with LDAC pin HIGH will ignore it
     *
     *  We will use multiple GPIO pins to control the address by connecting the LDAC pin to the GPIO pin
     *
     * @param newAddr The new address value (0-7, only 3 LSBs used)
     * @return true if successful
     */
    bool changeAddress(uint8_t newAddr);

    // Add this to the private section
    static const uint8_t CMD_GENERAL_CALL = 0x00;

private:
    i2c_inst_t *_i2c;
    uint8_t _addr;
    uint8_t _sdaPin;
    uint8_t _sclPin;
    int8_t _ldacPin;
    int8_t _rdyPin;
    bool _initialized;

    // MCP4728 commands
    static const uint8_t CMD_WRITE_DAC = 0x40;        // Write to DAC register
    static const uint8_t CMD_WRITE_DAC_EEPROM = 0x58; // Write to DAC and EEPROM
    static const uint8_t CMD_MULTI_WRITE = 0x50;      // Write to multiple DACs
};

#endif // MCP4728_H