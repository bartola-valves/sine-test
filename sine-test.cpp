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
#define Fs 2000            // 2kHz sample rate (1/0.0005 seconds)
#define DELAY 250          // 1/Fs (in microseconds). We will use 250uS instead
// the DDS units:
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (100.0 * two32) / Fs; // 100 Hz sine wave

uint16_t DAC_data; // output value to the DAC

// DDS sine table
#define sine_table_size 256
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

    // DDS phase and sine table lookup
    phase_accum_main += phase_incr_main;
    // The DAC is 12-bit so we need to mask the lower 12 bits and add 2048 to get a positive value.
    // output zero is at 2048.
    DAC_data = (sin_table[phase_accum_main >> 24] + 2048) & 0x0FFF;

    // Send the DAC data to the DAC on the object dac
    dac.setChannel(MCP4728::CHANNEL_A, DAC_data, MCP4728::VREF_INT, MCP4728::GAIN_1X);

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);
}

void scan_i2c()
{
    printf("I2C Bus Scan\n");
    for (int addr = 0; addr < 128; addr++)
    {
        int ret;
        uint8_t rxdata;
        ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
        if (ret >= 0)
        {
            printf("Device found at address 0x%02x\n", addr);
        }
    }
    printf("Scan complete\n");
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
    // scaled to produce values between 0 and 4096
    int ii;
    for (ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
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

    // Add this after i2c_init() in main()
    scan_i2c();

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
    { // Add opening brace
      // for (int i = 0; i < 4096; i += 512)
      // {
      //     printf("Setting DAC to %d\n", i);
      //     // Use setChannel instead of setVoltage
      //     dac.setChannel(MCP4728::CHANNEL_A, i, MCP4728::VREF_INT, MCP4728::GAIN_1X);
      //     busy_wait_ms(1000); // Hold each voltage for 1 second
      // }
    } // Add closing brace
}
