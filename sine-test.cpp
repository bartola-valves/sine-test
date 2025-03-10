#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include "hardware/timer.h"
#include "hardware/irq.h"

#include "lib/MCP4728.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 4   // GPIO4 (pin 6)
#define I2C_SCL_PIN 5   // GPIO5 (pin 7)
#define I2C_FREQ 400000 // 400 kHz

// additional connections of the MCP4728
// VDD to 5V
// GND to GND
// SDA to GPIO4
// SCL to GPIO5
// LDAC to GPIO6
// RDY to GPIO7

#define LDAC_PIN 6 // GPIO6 (pin 9)
#define RDY_PIN 7  // GPIO7 (pin 10)

// MCP4728 I2C address (default is 0x60)
#define MCP4728_ADDR 0x60

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// GPIO for timing the ISR
#define ISR_GPIO 20 // GPIO20 (pin 27) we will use this to measure the ISR time spent

// DDS parameters
#define two32 4294967296.0 // 2^32
// Reduce sample rate by 10x for testing
#define Fs 250     // 250Hz sample rate (10x slower)
#define DELAY 4000 // 4000µs between samples (1/250 = 0.004)

// the DDS units:
volatile unsigned int phase_accum_main;
// Adjust phase increment to maintain the same 20Hz output
volatile unsigned int phase_incr_main = (uint32_t)((20.0 * two32) / Fs + 0.5);

uint16_t DAC_data; // output value to the DAC

// Define a static array to hold DAC values (outside the ISR)
static uint16_t dac_values[4];

// DDS sine table
#define sine_table_size 1024 // Increase from 256 to 1024
volatile int sin_table[sine_table_size];

volatile int isr_counter = 0; // to test the ISR is running correctly

// global class object to be accessed by the ISR
// configure the MCP4728 using the class MCP4728
// The LDAC and RDY pins are connected to GPIO6 and GPIO7 respectively as per DEFINES above
// The I2C address is as defined by MCP4728_ADDR
// The I2C port is I2C_PORT
// The I2C frequency is I2C_FREQ
MCP4728 dac(I2C_PORT, MCP4728_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, LDAC_PIN, RDY_PIN);

// Alarm ISR
/*
 * The alarm interrupt service routine (ISR) is called when the alarm goes off every 20 us as defined by the DELAY constant.
 * The ISR will increment the phase accumulator and look up the sine value in the sine table.
 * The sine value is then sent to the DAC.
 * The GPIO 20 (ISR_GPIO) is toggled to measure the ISR time spent.
 */
static void alarm_irq(void)
{

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Check if DAC is ready before processing
    if (!dac.isReady())
    {
        // Skip this update if device is busy
        gpio_put(ISR_GPIO, 0);
        isr_counter++;
        return;
    }
    // INCREMENT THE PHASE ACCUMULATOR - this line was missing!
    phase_accum_main += phase_incr_main;

    uint32_t phase_index = phase_accum_main >> 22;                 // For 1024-point table
    uint32_t fraction = (phase_accum_main >> 14) & 0xFF;           // Fractional part for interpolation
    int v1 = sin_table[phase_index & (sine_table_size - 1)];       // Change to int (signed)
    int v2 = sin_table[(phase_index + 1) & (sine_table_size - 1)]; // Change to int (signed)
    int32_t interpolated = v1 + ((v2 - v1) * fraction) / 256;
    DAC_data = (interpolated + 2048) & 0x0FFF;

    // Set all array values to the same value
    dac_values[0] = dac_values[1] = dac_values[2] = dac_values[3] = DAC_data;

    // Try with a small busy wait before sending data
    busy_wait_us(50); // Small delay before I2C transaction

    // Update all channels
    dac.setAllChannels(dac_values, MCP4728::VREF_INT, MCP4728::GAIN_1X, false);

    dac.triggerLDAC();
    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);

    // In your alarm_irq function, before exiting:
    isr_counter++;
}

/*
 * Main function will initialize the I2C, DAC, and the alarm
 * The purpose is to generate a 100 Hz sine wave using the DDS technique.
 * The sine wave is generated using a lookup table before being sent to the DAC.
 */

int main()
{
    stdio_init_all();
    busy_wait_ms(1000); // wait for UART to start. We will use busy_wait_ms later for delays

    // === build the sine lookup table =======
    // scaled to produce values between -2047 and 2047
    for (int ii = 0; ii < sine_table_size; ii++)
    {
        float angle = ((float)ii * 2.0 * M_PI) / (float)sine_table_size;
        sin_table[ii] = (int)(2047.5 * sin(angle)); // Using 2047.5 gives us the full -2048 to 2047 range
    }
    // Initialize the GPIO for the ISR timing
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    // set initial value to 0
    gpio_put(ISR_GPIO, 0);

    // Initialize I2C hardware first
    i2c_init(I2C_PORT, I2C_FREQ);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Now initialize the DAC
    if (!dac.begin(I2C_FREQ))
    {
        printf("DAC initialization failed!\n");
        // Add error handling here
    }

    printf("Setting initial values for all channels...\n");
    // Initialize all channels to mid-point
    for (int ch = 0; ch < 4; ch++)
    {
        bool success = dac.setChannel(static_cast<MCP4728::Channel>(ch), 2048,
                                      MCP4728::VREF_INT, MCP4728::GAIN_1X);
        if (!success)
        {
            printf("Failed to initialize channel %d\n", ch);
        }
    }
    busy_wait_ms(100); // Give the DAC time to settle

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;
    // Nothing happening here
    while (1)
    {
        // Just monitor or do nothing
        busy_wait_ms(5000);
        printf("ISR still running, counter: %d\n", isr_counter);
    }
}
