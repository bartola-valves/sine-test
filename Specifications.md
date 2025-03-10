# LFO test application using MCP4728

This application tests the MCP4728 connected via I2C to a Raspberry Pico to generate 4 channels for LFO.

## Hardware overview

1. Raspberry Pico
2. MCP4728 connected to Pico via I2C and level shifter for 3V3 to 5V translation
3. Each output of the MCP4728 (Channels A to D) have an operational amplifier to take the output signal to 5Vpp (0 to 5V).
