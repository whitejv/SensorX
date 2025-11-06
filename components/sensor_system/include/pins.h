/*
 * pins.h - ESP32-C6 Feather GPIO Pin Definitions
 *
 * This file contains all pin definitions for the Adafruit ESP32-C6 Feather board.
 * Pin mappings are based on ESP32-C6 GPIO numbers (not Arduino pin numbers).
 * 
 * Reference: Adafruit ESP32-C6 Feather pinout documentation
 */

#ifndef PINS_H
#define PINS_H

#include <driver/gpio.h>

// ============================================================================
// Analog Pins (ADC channels - ESP32-C6 GPIO numbers)
// Note: Arduino pin numbers differ from GPIO numbers on this board
// Arduino "A0" = GPIO1, Arduino "A1" = GPIO4, etc.
// ============================================================================
#define PIN_A0   GPIO_NUM_1    // GPIO1  - ADC1_CH1 (Arduino "A0"/"1", CircuitPython "A0")
#define PIN_A1   GPIO_NUM_4    // GPIO4  - ADC1_CH4 (Arduino "A1"/"4", CircuitPython "A1")
#define PIN_A2   GPIO_NUM_6    // GPIO6  - ADC1_CH6 (Arduino "A2"/"6", CircuitPython "A2/IO6")
#define PIN_A3   GPIO_NUM_5    // GPIO5  - ADC1_CH5 (Arduino "A3"/"5", CircuitPython "A3/IO5")
#define PIN_A4   GPIO_NUM_3    // GPIO3  - ADC1_CH3 (Arduino "A4"/"3", CircuitPython "A4")
#define PIN_A5   GPIO_NUM_2    // GPIO2  - ADC1_CH2 (Arduino "A5"/"2", CircuitPython "A5")

// Note: A2 (GPIO6) can also function as digital IO6
// Note: A3 (GPIO5) can also function as digital IO5
// Note: GPIO0 is available as digital IO0 (Arduino "0") but NOT as analog A0

// ============================================================================
// Digital I/O Pins (ESP32-C6 GPIO numbers)
// Note: Arduino pin numbers shown in parentheses for reference
// ============================================================================
#define PIN_D0   GPIO_NUM_0    // GPIO0  - Digital IO0 (Arduino "0", CircuitPython "IO0", ADC1_CH0)
#define PIN_D1   GPIO_NUM_1    // GPIO1  - Digital IO1 (Arduino "1", CircuitPython "A0", ADC1_CH1)
#define PIN_D2   GPIO_NUM_2    // GPIO2  - Digital IO2 (Arduino "2", CircuitPython "A5", ADC1_CH2)
#define PIN_D3   GPIO_NUM_3    // GPIO3  - Digital IO3 (Arduino "3", CircuitPython "A4", ADC1_CH3)
#define PIN_D4   GPIO_NUM_4    // GPIO4  - Digital IO4 (Arduino "4", CircuitPython "A1", ADC1_CH4)
#define PIN_D5   GPIO_NUM_5    // GPIO5  - Digital IO5 (Arduino "5", CircuitPython "A3/IO5", ADC1_CH5)
#define PIN_D6   GPIO_NUM_6    // GPIO6  - Digital IO6 (Arduino "6", CircuitPython "A2/IO6", ADC1_CH6)
#define PIN_D7   GPIO_NUM_7    // GPIO7  - Digital IO7 (Arduino "7", CircuitPython "IO7")
#define PIN_D8   GPIO_NUM_8    // GPIO8  - Digital IO8 (Arduino "8", CircuitPython "IO8")
#define PIN_D9   GPIO_NUM_9    // GPIO9  - Digital IO9 (Arduino "9", CircuitPython "IO9", NeoPixel, Boot button)
#define PIN_D10  GPIO_NUM_10   // GPIO10 - Digital IO10 (Arduino "10", SPI CS)
#define PIN_D11  GPIO_NUM_11   // GPIO11 - Digital IO11 (Arduino "11")
#define PIN_D12  GPIO_NUM_12   // GPIO12 - Digital IO12 (Arduino "12", JTAG TDI)
#define PIN_D13  GPIO_NUM_13   // GPIO13 - Digital IO13 (Arduino "13", JTAG TDO)
#define PIN_D14  GPIO_NUM_14   // GPIO14 - Digital IO14 (Arduino "14", CircuitPython "IO14", JTAG TCK)
#define PIN_D15  GPIO_NUM_15   // GPIO15 - Digital IO15 (Arduino "15", CircuitPython "IO15", LED, JTAG TMS)
#define PIN_D16  GPIO_NUM_16   // GPIO16 - Digital IO16 (Arduino "16", CircuitPython "TX", UART TX)
#define PIN_D17  GPIO_NUM_17   // GPIO17 - Digital IO17 (Arduino "17", CircuitPython "RX", UART RX)
#define PIN_D18  GPIO_NUM_18   // GPIO18 - Digital IO18 (Arduino "18", CircuitPython "SCL", I2C SCL)
#define PIN_D19  GPIO_NUM_19   // GPIO19 - Digital IO19 (Arduino "19", CircuitPython "SDA", I2C SDA)
#define PIN_D20  GPIO_NUM_20   // GPIO20 - Digital IO20 (EN pin, NEOPIXEL_I2C_POWER, SDIO_DATA0)
#define PIN_D21  GPIO_NUM_21   // GPIO21 - Digital IO21 (Arduino "21", CircuitPython "SCK", SPI SCK)
#define PIN_D22  GPIO_NUM_22   // GPIO22 - Digital IO22 (Arduino "22", CircuitPython "MOSI", SPI MOSI)
#define PIN_D23  GPIO_NUM_23   // GPIO23 - Digital IO23 (Arduino "23", CircuitPython "MISO", SPI MISO)

// ============================================================================
// Power and Control Pins
// ============================================================================
#define PIN_EN       GPIO_NUM_20  // GPIO20 - 3.3V regulator enable (also I2C power for STEMMA QT)
#define PIN_3V3     0            // Not a GPIO - 3.3V output (hardware pin)
#define PIN_GND     0            // Not a GPIO - Ground (hardware pin)
#define PIN_VIN      0            // Not a GPIO - VIN input (hardware pin)
#define PIN_VBUS     0            // Not a GPIO - USB VBUS (hardware pin)
#define PIN_RESET    0            // Not a GPIO - Reset button (hardware)
#define PIN_BOOT0    GPIO_NUM_9   // GPIO9 - Boot button (shared with NeoPixel/D4)

// ============================================================================
// Onboard LED and NeoPixel
// ============================================================================
#define PIN_ONBOARD_LED   GPIO_NUM_15  // GPIO15 - Onboard red LED
#define PIN_NEOPIXEL      GPIO_NUM_9   // GPIO9 - Onboard NeoPixel (shared with Boot button)

// ============================================================================
// I2C Bus Pins (ESP32-C6 Feather default)
// ============================================================================
#define PIN_I2C_SDA  GPIO_NUM_19  // GPIO19 - I2C SDA (Data line)
#define PIN_I2C_SCL  GPIO_NUM_18  // GPIO18 - I2C SCL (Clock line)
// Note: GPIO20 (EN pin) must be pulled high to power STEMMA QT connector

// ============================================================================
// SPI Bus Pins (ESP32-C6 Feather default)
// ============================================================================
#define PIN_SPI_SCK  GPIO_NUM_21  // GPIO21 - SPI Clock (SCK)
#define PIN_SPI_MOSI GPIO_NUM_22  // GPIO22 - SPI MOSI (COPI - Controller Out/Peripheral In)
#define PIN_SPI_MISO GPIO_NUM_23  // GPIO23 - SPI MISO (CIPO - Controller In/Peripheral Out)
#define PIN_SPI_CS   GPIO_NUM_10  // GPIO10 - SPI Chip Select (CS) - default

// ============================================================================
// UART Pins (ESP32-C6 Feather default)
// ============================================================================
#define PIN_SERIAL_RX GPIO_NUM_17  // GPIO17 - UART RX (Receive)
#define PIN_SERIAL_TX GPIO_NUM_16  // GPIO16 - UART TX (Transmit)

// ============================================================================
// JTAG Debug Pins (for ESP-PROG connection)
// IMPORTANT: These pins must NOT be used for other functions when debugging
// ============================================================================
#define PIN_JTAG_TDI GPIO_NUM_12  // GPIO12 - JTAG Data In (DO NOT USE FOR OTHER FUNCTIONS)
#define PIN_JTAG_TDO GPIO_NUM_13  // GPIO13 - JTAG Data Out (DO NOT USE FOR OTHER FUNCTIONS)
#define PIN_JTAG_TCK GPIO_NUM_14  // GPIO14 - JTAG Clock (DO NOT USE FOR OTHER FUNCTIONS)
#define PIN_JTAG_TMS GPIO_NUM_15  // GPIO15 - JTAG Mode Select (DO NOT USE FOR OTHER FUNCTIONS - shared with LED)


// ============================================================================
// Sensor and I/O Pin Definitions
// ============================================================================

// PCNT Flow Sensor Pins (Hall effect flow sensors - pulse counting)
#define PIN_FLOW_SENSOR_1   GPIO_NUM_7   // GPIO7 - PCNT Flow Sensor 1 (PCNT_UNIT_0)
#define PIN_FLOW_SENSOR_2   GPIO_NUM_8   // GPIO8 - PCNT Flow Sensor 2 (PCNT_UNIT_1)
#define PIN_FLOW_SENSOR_3   GPIO_NUM_11  // GPIO11 - PCNT Flow Sensor 3 (PCNT_UNIT_2)

// One-Wire Temperature Sensor Pin (DS18B20 sensors on shared bus)
#define PIN_ONEWIRE_TEMP    GPIO_NUM_6   // GPIO6 - One-Wire bus for DS18B20 sensors
// Note: GPIO6 is ADC-capable but used as digital One-Wire bus

// Fan Control Pin
#define PIN_FAN_CONTROL     GPIO_NUM_21  // GPIO21 - Fan control output (HIGH = ON, LOW = OFF)
// Note: Changed from GPIO16 (UART TX conflict) to GPIO21 (SPI SCK, available if SPI unused)

// Discrete GPIO Input Pins (config/status inputs)
#define PIN_DISC_INPUT_1    GPIO_NUM_0   // GPIO0 - Discrete input 1 (config/status)
#define PIN_DISC_INPUT_2    GPIO_NUM_1   // GPIO1 - Discrete input 2 (config/status)
#define PIN_DISC_INPUT_3    GPIO_NUM_2   // GPIO2 - Discrete input 3 (config/status)
// Note: GPIO0-2 are ADC-capable but used as digital inputs


// ============================================================================
// Pin Function Validation Macros
// ============================================================================
// ESP32-C6 has GPIO 0-31 available (but not all are usable)
#define IS_ANALOG_PIN(pin)  ((pin) >= GPIO_NUM_0 && (pin) <= GPIO_NUM_5)
#define IS_DIGITAL_PIN(pin) ((pin) >= GPIO_NUM_0 && (pin) <= GPIO_NUM_23 && \
                             (pin) != GPIO_NUM_24 && (pin) != GPIO_NUM_28 && \
                             (pin) != GPIO_NUM_29 && (pin) != GPIO_NUM_30 && \
                             (pin) != GPIO_NUM_31)
#define IS_I2C_PIN(pin)     ((pin) == PIN_I2C_SDA || (pin) == PIN_I2C_SCL)
#define IS_SPI_PIN(pin)     ((pin) == PIN_SPI_MOSI || (pin) == PIN_SPI_MISO || \
                             (pin) == PIN_SPI_SCK || (pin) == PIN_SPI_CS)
#define IS_UART_PIN(pin)    ((pin) == PIN_SERIAL_RX || (pin) == PIN_SERIAL_TX)
#define IS_JTAG_PIN(pin)    ((pin) == PIN_JTAG_TDI || (pin) == PIN_JTAG_TDO || \
                             (pin) == PIN_JTAG_TCK || (pin) == PIN_JTAG_TMS)

// ============================================================================
// Important Pin Sharing Notes
// ============================================================================
// - GPIO0: Assigned as Discrete Input 1, also ADC1_CH0 (but not Arduino "A0")
// - GPIO1: Assigned as Discrete Input 2, Arduino "A0" and "1", can be analog or digital
// - GPIO2: Assigned as Discrete Input 3, Arduino "A5" and "2", can be analog or digital
// - GPIO3: Available as digital IO3, also ADC1_CH3 (Arduino "A4")
// - GPIO4: Available as digital IO4, also ADC1_CH4 (Arduino "A1")
// - GPIO5: Available as digital IO5, also ADC1_CH5 (Arduino "A3")
// - GPIO6: Assigned as One-Wire Temperature bus, Arduino "A2" and "6", can be analog or digital
// - GPIO7: Assigned as PCNT Flow Sensor 1, Digital IO7
// - GPIO8: Assigned as PCNT Flow Sensor 2, Digital IO8
// - GPIO9: Shared between NeoPixel, Boot button, and Digital IO9 (available if unused)
// - GPIO10: Shared between SPI CS and Digital IO10 (available if SPI unused)
// - GPIO11: Assigned as PCNT Flow Sensor 3, Digital IO11
// - GPIO12: RESERVED for JTAG TDI - DO NOT USE for other functions
// - GPIO13: RESERVED for JTAG TDO - DO NOT USE for other functions
// - GPIO14: RESERVED for JTAG TCK - DO NOT USE for other functions
// - GPIO15: RESERVED for JTAG TMS - DO NOT USE for other functions (shared with red LED)
// - GPIO16: Assigned as Fan Control output (shared with UART TX - choose one function)
// - GPIO17: UART RX (reserved if UART needed)
// - GPIO18: RESERVED for I2C SCL - DO NOT USE for other functions
// - GPIO19: RESERVED for I2C SDA - DO NOT USE for other functions
// - GPIO20: RESERVED for STEMMA QT power enable - MUST be HIGH for I2C devices (NEOPIXEL_I2C_POWER)
// - GPIO21: SPI SCK (available if SPI unused)
// - GPIO22: SPI MOSI (available if SPI unused)
// - GPIO23: SPI MISO (available if SPI unused)


// ============================================================================
// GPIO Pin Status Summary Table
// ============================================================================
// | GPIO | Function                  | Status      | Notes                        |
// |------|---------------------------|-------------|------------------------------|
// | 0    | Discrete Input 1          | Assigned    | ADC-capable                  |
// | 1    | Discrete Input 2          | Assigned    | ADC-capable                  |
// | 2    | Discrete Input 3          | Assigned    | ADC-capable                  |
// | 3    | Available                | Free        | ADC-capable                  |
// | 4    | Available                | Free        | ADC-capable                  |
// | 5    | Available                | Free        | ADC-capable                  |
// | 6    | One-Wire Temperature      | Assigned    | ADC-capable                  |
// | 7    | PCNT Flow Sensor 1        | Assigned    | Digital only                 |
// | 8    | PCNT Flow Sensor 2        | Assigned    | Digital only                 |
// | 9    | Boot/NeoPixel             | Shared      | Available if unused          |
// | 10   | SPI CS                   | Shared      | Available if SPI unused       |
// | 11   | PCNT Flow Sensor 3        | Assigned    | Digital only                 |
// | 12   | JTAG TDI                 | Reserved    | DO NOT USE                   |
// | 13   | JTAG TDO                 | Reserved    | DO NOT USE                   |
// | 14   | JTAG TCK                 | Reserved    | DO NOT USE                   |
// | 15   | JTAG TMS / LED           | Reserved    | DO NOT USE                   |
// | 16   | UART TX                  | Reserved    | If UART needed              |
// | 17   | UART RX                  | Reserved    | If UART needed              |
// | 18   | I2C SCL                  | Reserved    | In use                       |
// | 19   | I2C SDA                  | Reserved    | In use                       |
// | 20   | STEMMA QT Power Enable    | Reserved    | In use                       |
// | 21   | Fan Control              | Assigned    | Changed from GPIO16 (UART conflict) |
// | 22   | SPI MOSI                 | Shared      | Available if SPI unused       |
// | 23   | SPI MISO                 | Shared      | Available if SPI unused       |
//
// Summary Statistics:
// - Total GPIO pins (0-23): 24 pins
// - Assigned for sensors/I/O: 9 pins (GPIO0, 1, 2, 6, 7, 8, 11, 20, 21)
// - Reserved for critical functions: 6 pins (GPIO12-15 JTAG, GPIO18-19 I2C)
// - Available for future use: 9 pins (GPIO3, 4, 5, 9, 10, 16, 17, 22, 23)
// - Shared/conflict pins: 5 pins (GPIO9, GPIO16-17 UART, GPIO22-23 SPI)
//
// Notes:
// - GPIO3-5: Fully available ADC-capable pins for future analog sensors or digital I/O
// - GPIO9: Available if NeoPixel not used; boot button conflict only during reset
// - GPIO10, 22-23: Available if SPI not needed; otherwise reserved for SPI
// - GPIO16-17: Reserved for UART TX/RX if UART needed externally
// - GPIO21: Assigned for fan control (changed from GPIO16 to avoid UART conflict)
// - GPIO20: Already configured and used in i2c_manager.c for STEMMA QT power
// - All sensor requirements are covered with 9 pins assigned
// - 9 pins remain available for future expansion
// ============================================================================

#endif /* PINS_H */
