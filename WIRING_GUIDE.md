# ESP32-C6 Sensor Suite Wiring Guide

This document provides the complete wiring connections for deploying the ESP32-C6 Feather to the sensor suite.

## Power Connections

### ESP32-C6 Feather Power Input
- **VIN**: Connect to 5V power supply (or USB power)
- **GND**: Connect to system ground (common ground for all devices)
- **3.3V**: 3.3V output (for powering sensors if needed)

**Note**: Ensure adequate power supply capacity for all sensors and the ESP32.

---

## I2C Bus Connections

The I2C bus connects multiple sensors and devices. All I2C devices share the same SDA, SCL, and power lines.

### I2C Bus Pins (ESP32-C6 Feather)
- **GPIO19 (SDA)**: I2C Data line - connect to all I2C devices
- **GPIO18 (SCL)**: I2C Clock line - connect to all I2C devices
- **GPIO20**: STEMMA QT power enable - MUST be HIGH (handled by firmware)

### I2C Bus Requirements
- **Pull-up resistors**: 4.7kΩ resistors on SDA and SCL lines (to 3.3V)
- **Power**: 3.3V power for all I2C devices
- **Ground**: Common ground connection

### I2C Devices Connected

#### 1. ADS1015/ADS1115 ADC (Address: 0x48)
- **SDA**: Connect to I2C SDA (GPIO19)
- **SCL**: Connect to I2C SCL (GPIO18)
- **VDD**: Connect to 3.3V
- **GND**: Connect to ground
- **Address**: 0x48 (ADDR pin configuration)

**Note**: If using ADS1115 at address 0x49, connect additional device with ADDR pin configured differently.

#### 2. MCP23017 GPIO Expander (Address: 0x20)
- **SDA**: Connect to I2C SDA (GPIO19)
- **SCL**: Connect to I2C SCL (GPIO18)
- **VDD**: Connect to 3.3V
- **GND**: Connect to ground
- **Address**: 0x20 (A0, A1, A2 pins configuration)

#### 3. BME280 Environmental Sensor (Address: 0x77 or 0x76)
- **SDA**: Connect to I2C SDA (GPIO19)
- **SCL**: Connect to I2C SCL (GPIO18)
- **VDD**: Connect to 3.3V
- **GND**: Connect to ground
- **Address**: 0x77 (default) or 0x76 (if CSB pin pulled low)

**Note**: Firmware will automatically detect at both addresses.

---

## PCNT Flow Sensors (Hall Effect Flow Meters)

Three flow sensors connected via PCNT (Pulse Counter) units:

### Flow Sensor 1
- **Signal**: Connect to **GPIO7** (ESP32-C6 Feather pin D7)
- **VCC**: Connect to 5V or 3.3V (check sensor specification)
- **GND**: Connect to ground

### Flow Sensor 2
- **Signal**: Connect to **GPIO8** (ESP32-C6 Feather pin D8)
- **VCC**: Connect to 5V or 3.3V (check sensor specification)
- **GND**: Connect to ground

### Flow Sensor 3
- **Signal**: Connect to **GPIO3** (ESP32-C6 Feather pin D3 / A4)
- **VCC**: Connect to 5V or 3.3V (check sensor specification)
- **GND**: Connect to ground

**Note**: Flow sensors typically output pulses proportional to flow rate. Ensure sensors are powered appropriately and signal levels are compatible with ESP32-C6 (3.3V logic).

---

## One-Wire Temperature Sensors (DS18B20)

Multiple DS18B20 sensors can be connected on a single One-Wire bus:

### One-Wire Bus Connection
- **Data**: Connect to **GPIO6** (ESP32-C6 Feather pin D6 / A2)
- **VCC**: Connect to 3.3V (or 5V if sensors support it)
- **GND**: Connect to ground

### One-Wire Bus Requirements
- **Pull-up resistor**: 4.7kΩ resistor between Data line and VCC
- **Parasitic power**: DS18B20 can be powered from the data line if configured for parasitic power mode

**Note**: Up to multiple DS18B20 sensors can share the same One-Wire bus. Each sensor has a unique 64-bit address.

---

## Fan Control Output

### Fan Control Connection
- **Control Signal**: Connect to **GPIO2** (ESP32-C6 Feather pin D2 / A5)
- **Fan Power**: Connect fan power supply (separate from ESP32 power)
- **Fan Ground**: Connect to common ground

**Note**: 
- GPIO2 outputs 3.3V logic HIGH when fan should be ON
- GPIO2 outputs 0V (LOW) when fan should be OFF
- Fan control circuit should use a transistor/MOSFET to drive the fan if fan requires higher voltage/current than GPIO can provide
- Fan turns ON when temperature >= 70.0°F (BME280 ambient temperature)

---

## Connection Summary Table

| Component | ESP32-C6 Pin | Function | Notes |
|-----------|--------------|----------|-------|
| **Power** | | | |
| Power Supply | VIN | 5V input | USB or external 5V |
| Ground | GND | Common ground | Connect all GND together |
| **I2C Bus** | | | |
| I2C SDA | GPIO19 (D19) | Data line | Shared by all I2C devices |
| I2C SCL | GPIO18 (D18) | Clock line | Shared by all I2C devices |
| I2C Power Enable | GPIO20 (D20) | Power enable | Auto-controlled by firmware |
| **Flow Sensors** | | | |
| Flow Sensor 1 | GPIO7 (D7) | Pulse input | PCNT Unit 0 |
| Flow Sensor 2 | GPIO8 (D8) | Pulse input | PCNT Unit 1 |
| Flow Sensor 3 | GPIO3 (D3/A4) | Pulse input | PCNT Unit 2 |
| **Temperature Sensors** | | | |
| One-Wire Bus | GPIO6 (D6/A2) | Data line | Shared by all DS18B20 |
| **Fan Control** | | | |
| Fan Control | GPIO2 (D2/A5) | Control output | HIGH = ON, LOW = OFF |

---

## Wiring Checklist

### Power Connections
- [ ] ESP32-C6 VIN connected to 5V power supply
- [ ] Common ground established (all GND connections)
- [ ] Adequate power supply capacity verified

### I2C Bus Connections
- [ ] GPIO19 (SDA) connected to all I2C devices
- [ ] GPIO18 (SCL) connected to all I2C devices
- [ ] 4.7kΩ pull-up resistors on SDA and SCL (to 3.3V)
- [ ] ADS1015/ADS1115 connected (address 0x48)
- [ ] MCP23017 GPIO expander connected (address 0x20)
- [ ] BME280 environmental sensor connected (address 0x77 or 0x76)
- [ ] All I2C devices powered (3.3V and GND)

### Flow Sensor Connections
- [ ] Flow Sensor 1 signal → GPIO7
- [ ] Flow Sensor 2 signal → GPIO8
- [ ] Flow Sensor 3 signal → GPIO3
- [ ] All flow sensors powered and grounded

### Temperature Sensor Connections
- [ ] One-Wire data line → GPIO6
- [ ] 4.7kΩ pull-up resistor on One-Wire data line
- [ ] DS18B20 sensors connected (VCC, GND, Data)
- [ ] Sensors powered (3.3V or parasitic power)

### Fan Control Connection
- [ ] Fan control signal → GPIO2
- [ ] Fan power supply connected (if needed)
- [ ] Fan ground connected to common ground
- [ ] Transistor/MOSFET driver circuit (if fan requires >3.3V or >GPIO current)

---

## Important Notes

1. **GPIO20 (I2C Power Enable)**: This pin is automatically controlled by firmware and must be HIGH for I2C devices to work. Do not connect anything else to this pin.

2. **Pull-up Resistors**: 
   - I2C bus requires 4.7kΩ pull-ups on SDA and SCL
   - One-Wire bus requires 4.7kΩ pull-up on data line
   - Some modules have built-in pull-ups - verify before adding external ones

3. **Power Supply**: Ensure adequate current capacity:
   - ESP32-C6: ~200-300mA typical
   - I2C sensors: ~10-50mA each
   - Flow sensors: Check specifications
   - DS18B20: ~1-3mA each
   - Fan: Check specifications (may need separate supply)

4. **Signal Levels**: ESP32-C6 uses 3.3V logic. Ensure all sensors are compatible or use level shifters if needed.

5. **Ground**: All devices must share a common ground connection for proper operation.

6. **Fan Control**: GPIO2 outputs 3.3V logic. If fan requires higher voltage or current, use a transistor/MOSFET driver circuit.

---

## Testing After Wiring

1. **Power On**: Verify ESP32-C6 boots and connects to WiFi
2. **I2C Devices**: Check serial output for I2C device detection
3. **Flow Sensors**: Verify pulse counting in serial output
4. **Temperature Sensors**: Check for DS18B20 detection
5. **Fan Control**: Verify fan turns ON when temperature > 70°F
6. **MQTT**: Verify data publishing to MQTT broker

---

## Troubleshooting

### I2C Devices Not Detected
- Check SDA/SCL connections
- Verify pull-up resistors (4.7kΩ)
- Check power connections (3.3V and GND)
- Verify I2C addresses match firmware expectations

### Flow Sensors Not Counting
- Verify signal connections to correct GPIO pins
- Check sensor power supply
- Verify signal levels (should be 0-3.3V for ESP32-C6)
- Check for loose connections

### Temperature Sensors Not Detected
- Verify One-Wire data line connection to GPIO6
- Check pull-up resistor (4.7kΩ)
- Verify power connections
- Check for bus conflicts or short circuits

### Fan Not Responding
- Verify GPIO2 connection
- Check fan power supply
- Verify fan control circuit (if using transistor/MOSFET)
- Check temperature readings (fan turns ON at >= 70°F)

---

## Revision History

- **2025-11-12**: Initial wiring guide created
- Fan control moved from GPIO0 → GPIO1 → GPIO2 (GPIO0 is strapping pin, GPIO1 is UART TX)

