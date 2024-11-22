# ESP32-Based Knee Rehabilitation Data Logger

## Overview
This project implements a real-time data logging system for knee rehabilitation exercises using an ESP32 microcontroller. The system measures knee angles via a potentiometer, logs the data to an SD card, and provides wireless access to the collected data through a web interface.

## Features
- Real-time angle measurement (0-180 degrees)
- Precise timestamping using RTC (PCF8523)
- Local data storage on SD card
- Wi-Fi Access Point for data retrieval
- Web interface for data download
- FreeRTOS task management for concurrent operations
- Error handling and system status monitoring

## Hardware Requirements
- ESP32 Development Board (Feather ESP32 HUZZAH32)
- PCF8523 Real-Time Clock Module
- SD Card Module (SPI interface)
- Potentiometer (10kΩ recommended)
- SD Card (FAT32 formatted)
- USB cable for programming and power

### Pin Connections
| Component | ESP32 Pin |
|-----------|-----------|
| SD Card CS | GPIO 5 |
| Potentiometer | GPIO 34 (ADC) |
| RTC SDA | GPIO 21 (I2C) |
| RTC SCL | GPIO 22 (I2C) |

## Software Architecture

### Core Components
1. **Task Management**
   - `readPotentiometerTask`: Samples potentiometer at 100Hz
   - `sdCardTask`: Writes data to SD card
   - `serverTask`: Handles web server operations

2. **Data Structure**

3. **Resource Management**
- FreeRTOS mutex for SD card access
- Queue for data transfer between tasks
- Error handling enums for system status

### Network Configuration
- **Mode**: Access Point
- **SSID**: "KneeRehab"
- **Password**: "password"
- **mDNS**: "knee-rehab.local"

### Web Interface
- **Root Endpoint** (`/`): System status check
- **Data Endpoint** (`/data`): CSV data download

## Building and Flashing

### Prerequisites
- **PlatformIO IDE**
- **ESP32 Arduino Framework**
- **Required Libraries**:
  - RTClib
  - SD
  - WiFi
  - WebServer
  - ESPmDNS
  - FreeRTOS

### Build Instructions
1. Clone the repository
2. Open in PlatformIO
3. Install dependencies:
  ```bash
  pio pkg install adafruit/RTClib 
  ```
4. Build and upload:
  ```bash
  pio run -t upload
  ```
## Usage Instructions

### Initial Setup
1. Power up the device
2. Connect to "KneeRehab" Wi-Fi network
   1. Password: "password"
3. Access web interface:
   - URL: [http://knee-rehab.local](http://knee-rehab.local) or
   - IP address displayed on Serial monitor

### Data Collection
1. Attach potentiometer to knee joint mechanism
2. Perform rehabilitation exercises
3. Data is automatically logged to SD card

### Data Retrieval
1. Connect to device's Wi-Fi network
2. Visit http://knee-rehab.local/data
3. Download CSV file containing: 
   1. Timestamp
   2. Milliseconds
   3. Angle readings

## Data Format
CSV file format:
```csv
Timestamp,Milliseconds,Angle
1612345678,123,45
1612345678,124,46
1612345678,125,47
...
```

## Error Handling
System monitors and reports errors for:
- SD Card initialization/access
- RTC initialization/timing
- Network connectivity
- Web server operations

## Performance Specifications
- Sampling Rate: 100Hz
- Storage Capacity: Limited by SD card size
- Network Transfer Speed: ~3.3 Kb/s
## Known Limitations
- Some SD cards may require FAT32 formatting
- Web transfer speed limitations
- Single client connection at a time
- Access point range limitations
## Future Improvements
- <input disabled="" type="checkbox"> Increase data transfer speeds
- <input disabled="" type="checkbox"> Multi-client support
- <input disabled="" type="checkbox"> Secure data transmission
- <input disabled="" type="checkbox"> Battery monitoring
- <input disabled="" type="checkbox"> Custom PCB design

## License
This project is licensed under the MIT License - see the LICENSE.md file for details.

## Authors
Cole Rottenberg (cole.rottenberg@gmail.com)
Acknowledgments
FreeRTOS documentation
ESP32 Arduino core developers
Adafruit RTClib contributors