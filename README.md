# Mamakhub - PVM

A Portect Vessel Module (PVM) system built on ESP32 that combines GPS tracking with LoRa mesh networking for maritime communication and emergency response.

## Project Overview

This project implements a mesh network of ESP32 devices that can:
- Collect GPS location data from NEO-6M modules
- Transmit location information via LoRa radio
- Forward emergency SOS signals between devices
- Maintain a neighbor list of nearby devices
- Provide emergency button functionality

## Hardware Requirements

### Main Components
- **ESP32 Development Board** (ESP32-WROOM-32)
- **NEO-6M GPS Module**
- **LoRa Radio Module** (SX1276/SX1278 compatible)
- **Push Button** (for SOS functionality)
- **Breadboard and Jumper Wires**
- **External GPS antenna** for better reception
- **Battery pack**

## Hardware Connections

### GPS Module (NEO-6M) to ESP32
```
NEO-6M    ESP32
------    -----
VCC   →   3.3V
GND   →   GND
TX    →   GPIO 16 (RX2)
RX    →   GPIO 17 (TX2)
```

### LoRa Module (SX1276/SX1278) to ESP32
```
LoRa      ESP32
----      -----
3.3V  →   3.3V
GND   →   GND
SCK   →   GPIO 18
MISO  →   GPIO 19
MOSI  →   GPIO 23
NSS   →   GPIO 5
RST   →   GPIO 14
DIO0  →   GPIO 2
```

### Emergency Button to ESP32
```
Button    ESP32
------    -----
One pin → GPIO 15
Other pin → GND
```

## Software Setup

### Prerequisites
1. **PlatformIO IDE** installed in VS Code
2. **Git** (for version control)
3. **USB drivers** for ESP32 (CP2102 or CH340)

### Installation Steps

1. **Clone or Download the Project**
   ```bash
   git clone https://github.com/NsonQ/MarineHack.git
   cd MarineHack
   ```

2. **Open in PlatformIO**
   - Open VS Code
   - Install PlatformIO extension if not already installed
   - Open the project folder in VS Code
   - PlatformIO will automatically detect the `platformio.ini` file

3. **Install Dependencies**
   The project uses the following libraries (automatically installed via `platformio.ini`):
   - `TinyGPSPlus` - For GPS data parsing
   - `LoRa` - For LoRa radio communication

4. **Configure Upload Port**
   - Connect your ESP32 via USB
   - Check Device Manager (Windows) or `ls /dev/tty*` (Linux/Mac) for the correct port
   - Update the `upload_port` in `platformio.ini` if needed:
   ```ini
   upload_port = COM3  ; Change to your port (e.g., COM4, /dev/ttyUSB0)
   ```

## Building and Uploading

### Using PlatformIO

1. **Build the Project**
   - Press `Ctrl+Alt+B` or use PlatformIO: Build from command palette
   - Or click the checkmark (✓) in the PlatformIO toolbar

2. **Upload to ESP32**
   - Press `Ctrl+Alt+U` or use PlatformIO: Upload from command palette
   - Or click the arrow (→) in the PlatformIO toolbar

3. **Monitor Serial Output**
   - Press `Ctrl+Alt+S` or use PlatformIO: Serial Monitor
   - Or click the plug icon in the PlatformIO toolbar
   - Set baud rate to 115200

### Using Command Line
```bash
# Build the project
pio run

# Upload to device
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

## Configuration

### Device ID
Each device automatically generates a unique ID based on its MAC address. You can see this ID in the serial monitor during startup.

### LoRa Settings
- **Frequency**: 433 MHz (configurable in `main.cpp`)
- **Sync Word**: 0xA5 (for network identification)
- **Bandwidth**: Default LoRa settings

### GPS Update Interval
GPS packets are sent every 10 seconds by default. Modify this in the `loop()` function:
```cpp
if (everyNSeconds(gpsTimer, 10))  // Change 10 to desired seconds
```

## Usage

### Normal Operation
1. Power on the device
2. Wait for GPS fix (may take 2-5 minutes outdoors)
3. Device will automatically:
   - Send GPS location every 10 seconds
   - Receive and forward packets from other devices
   - Display neighbor information

### Emergency (SOS) Mode
1. Press the emergency button (GPIO 15)
2. Device will send high-priority SOS packet
3. All nearby devices will forward the SOS signal

### Serial Monitor Output
The device provides colored output for different packet types:
- 🟢 **Green**: GPS packet sent
- 🔴 **Red**: SOS packet sent/received
- 🟡 **Yellow**: Forwarded GPS packet
- 🔵 **Blue**: General packet received

## Troubleshooting

### GPS Issues
- **No GPS data**: Ensure clear sky view, wait 5+ minutes for first fix
- **Incorrect coordinates**: Check wiring, ensure 3.3V power supply
- **Intermittent data**: Consider external GPS antenna

### LoRa Issues
- **No communication**: Check wiring, verify frequency matches between devices
- **Short range**: Check antenna connections, consider external antenna
- **Init failed**: Verify SPI connections, check power supply

### General Issues
- **Upload fails**: Check correct port, ensure ESP32 is in download mode
- **Serial monitor blank**: Verify baud rate (115200), check USB connection
- **Device resets**: Check power supply stability, review error messages

### Common Error Messages
```
"LoRa init failed, restarting..." → Check LoRa module wiring
"No GPS detected" → Check GPS module connections
"Device ID: XXXX" → Shows your device's unique identifier
```

## Project Structure

```
MarineHack/
├── platformio.ini          # PlatformIO configuration
├── include/
│   └── LoRaPacket.h        # LoRa packet structure and functions
├── src/
│   ├── main.cpp            # Main application code
│   └── LoRaPacket.cpp      # LoRa packet implementation
└── README.md               # This file
```

## Features

### Implemented
- ✅ GPS data collection and parsing
- ✅ LoRa mesh networking
- ✅ Packet forwarding system
- ✅ SOS emergency signaling
- ✅ Neighbor device tracking
- ✅ CRC error checking
- ✅ Automatic device ID generation


## License
This project is open source. Please check the repository for license details.