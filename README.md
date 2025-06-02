# MotoSafe Security System - ESP32 Firmware

A comprehensive motorcycle security system built with ESP32, featuring GPS tracking, GSM communication, movement detection, and remote control capabilities.

## 🔧 Features

- **Movement Detection**: MPU6050 accelerometer for unauthorized movement alerts
- **GPS Tracking**: Real-time location monitoring with NEO-8M GPS module
- **GSM Communication**: SMS commands and alerts via SIM800L module
- **Remote Control**: Complete motorcycle control via SMS commands
- **Voltage Tampering Detection**: Monitors electrical system integrity
- **Kill Switch Control**: Remote motorcycle disable/enable functionality
- **Horn Control**: Automated alarm system with call functionality
- **Sleep Mode**: Power-saving mode for extended battery life
- **Safety Features**: Speed-based disable protection and emergency protocols

## 🛠️ Hardware Components

> **Note**: Please refer to the included schematic diagram for detailed wiring connections.

| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | ESP32 | Main processing unit |
| Accelerometer | MPU6050 | Movement detection |
| GPS Module | NEO-8M | Location tracking |
| GSM Module | SIM800L | SMS/Call communication |
| MOSFET | IRLZ44N | Power control for peripherals |
| Relays | 2-channel relay module | Kill switch and horn control |
| Voltage Divider | 2 Resistors | Tampering detection |

## 📋 Pin Configuration

```cpp
// Power Control
#define MOSFET_PIN 27        // IRLZ44N MOSFET gate pin

// Relay Controls
#define RELAY_KILL_PIN 4     // Motorcycle kill switch relay
#define RELAY_HORN_PIN 19    // Horn control relay

// Sensors
#define VOLTAGE_SENSOR 33    // Voltage divider (tampering detection)
#define MPU_SDA 21          // I2C SDA for MPU6050
#define MPU_SCL 22          // I2C SCL for MPU6050

// Communication Modules
#define SIM800L_RX 14       // SIM800L RX pin
#define SIM800L_TX 13       // SIM800L TX pin
#define GPS_RX 17           // NEO-8M RX pin
#define GPS_TX 16           // NEO-8M TX pin
```

## 🚀 Quick Start

### Prerequisites

- Arduino IDE with ESP32 board support
- Required libraries:
  - `TinyGPSPlus`
  - `Wire` (for I2C communication)
  - `HardwareSerial`
  - `Preferences`

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Skca01/motosafe-esp32-firmware.git
   cd motosafe-esp32-firmware
   ```

2. **Install required libraries:**
   - Open Arduino IDE
   - Go to **Tools → Manage Libraries**
   - Install `TinyGPSPlus` library

3. **Configure hardware connections** (see schematic PDF in repository)

4. **Set up SIM card:**
   - Insert active SIM card into SIM800L
   - Ensure SMS and voice call capabilities are enabled

5. **Upload firmware:**
   - Open `motorcycle_security_system.ino`
   - Select your ESP32 board
   - Upload the code

### Initial Setup

1. **Set owner phone number:**
   ```
   SMS: setnumber +1234567890
   ```
   Replace with your actual phone number (include country code)

2. **Test system status:**
   ```
   SMS: status
   ```
   Verify all components are operational

## 📱 SMS Commands

| Command | Description | Example |
|---------|-------------|---------|
| `status` | Get comprehensive system status | `status` |
| `on` / `off` | Enable/disable motorcycle | `on` |
| `track on` / `track off` | Enable/disable GPS tracking | `track on` |
| `alerts on` / `alerts off` | Enable/disable security alerts | `alerts on` |
| `setnumber <phone>` | Set new owner phone number | `setnumber +1234567890` |
| `sleep` | Enter power-saving sleep mode | `sleep` |
| `wake` | Wake from sleep mode | `wake` |
| `acknowledge` | Stop horn and acknowledge alert | `acknowledge` |
| `help` | List all available commands | `help` |

## ⚡ Power Management

### Normal Operation
- MOSFET enabled for full peripheral power
- All sensors and modules active
- Continuous monitoring and communication

### Sleep Mode
- MOSFET disabled to cut peripheral power
- Minimal power consumption
- Only SMS wake commands processed
- Automatic state restoration on wake

## 🔒 Security Features

### Movement Detection
- **Threshold**: 0.3g acceleration change
- **Response**: Immediate SMS alert + acknowledgment timer
- **Escalation**: Horn activation after 30 seconds without acknowledgment

### Tampering Detection
- **Method**: Voltage monitoring via analog input
- **Threshold**: Configurable voltage level
- **Cooldown**: 30-second interval between alerts

### Kill Switch Safety
- **Speed Check**: Prevents disable above 13 km/h
- **Pending Disable**: Automatic disable when speed reduces
- **Override**: Manual disable with safety warning

## 📞 Alert System

### Alert Types
1. **Movement Alert**: Unauthorized motorcycle movement
2. **Tampering Alert**: Electrical system interference
3. **Ignition Alert**: Unauthorized start attempts

### Response Sequence
1. Immediate SMS notification with GPS coordinates
2. 30-second acknowledgment window
3. Automatic horn activation if not acknowledged
4. Phone call attempts during horn activation
5. Continuous horn pattern until acknowledged

## 🛡️ Safety Protocols

### Speed-Based Protection
- Monitors GPS speed before motorcycle disable
- Prevents dangerous mid-ride shutdowns
- Queues disable request for safe execution

## 📊 System Status Indicators

### Component Status
- **MPU6050**: Movement sensor operational status
- **SIM800L**: GSM connectivity and SMS capability
- **GPS**: Satellite fix and location accuracy
- **MOSFET**: Power distribution status

### Operational Status
- **Motorcycle State**: Enabled/Disabled
- **Alert Mode**: Active/Inactive
- **Tracking Mode**: Enabled/Disabled
- **Sleep State**: Wake/Sleep

## 🔧 Configuration

### Adjustable Parameters

```cpp
// Movement sensitivity (in g-force)
float accelThreshold = 0.3;

// Tracking update interval (milliseconds)
const int TRACKING_INTERVAL = 10000;

// Safety speed threshold (km/h)
const float SAFE_DISABLE_SPEED = 13.0;

// Voltage tampering threshold
const int VOLTAGE_THRESHOLD = 1800;

// Alert acknowledgment timeout (milliseconds)
const int ACKNOWLEDGE_TIMEOUT = 30000;
```

## 📱 Related Projects

- **Android App**: [MotoSafe Mobile App](https://github.com/Skca01/MotoSafeAndroidStudio) - Companion mobile application

## 🐛 Troubleshooting

### Common Issues

**SIM800L not responding:**
- Check power supply (4V recommended)
- Verify UART connections
- Ensure SIM card is active

**GPS not acquiring fix:**
- Check antenna connection
- Ensure clear sky view
- Wait up to 5 minutes for cold start

**MPU6050 not detected:**
- Verify I2C connections (SDA/SCL)
- Check 5V power supply
- Test with I2C scanner

**SMS not sending:**
- Verify phone number format (include country code)
- Check SIM card balance/plan
- Ensure network coverage

### Debug Mode

Enable serial debugging by connecting to USB and monitoring at 115200 baud rate.


## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

## ⚠️ Disclaimer

This security system is designed as a deterrent and monitoring tool. Always comply with local laws and regulations regarding vehicle modifications and tracking devices. The developers are not responsible for any misuse or legal issues arising from the use of this system.

## 📞 Contact

For questions or feedback, contact [Kent Carlo B. Amante](https://github.com/Skca01):
- **GitHub Issues**: [Create an issue](https://github.com/Skca01/motosafe-esp32-firmware/issues)
- **Email**: carloamante125@gmail.com
