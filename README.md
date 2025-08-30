# STM32F103 OTA Bootloader System

A complete Over-The-Air (OTA) firmware update solution for STM32F103C8T6 microcontroller featuring an ESP32-based web interface and a fail-safe bootloader implementation.

## 🔋 System Overview

This project consists of three main components:
- **ESP32 OTA Bridge** (`ota_bridge.ino`) - Web-based firmware upload interface
- **STM32 Bootloader** (`stm32_bootloader/`) - Bare-metal bootloader with UART protocol
- **STM32 Application** (`stm32_app/`) - FreeRTOS-based application firmware

```
┌─────────────┐    UART     ┌─────────────────┐    Flash    ┌──────────────┐
│   ESP32     │ ◄────────── │ STM32 Bootloader│ ◄────────── │ STM32 App    │
│ Web Server  │             │   (16KB)        │             │ (FreeRTOS)   │
└─────────────┘             └─────────────────┘             └──────────────┘
      ▲                           ▲                               ▲
      │                           │                               │
   Wi-Fi/AP                  UART Protocol                  Application
  Web Interface             STX|TYPE|SEQ|...                    Runtime
```

## ✨ Key Features

### ESP32 OTA Bridge
- **Web-based firmware upload** with drag-and-drop interface
- **Wi-Fi connectivity** with AP fallback mode
- **Real-time logging** and progress monitoring
- **UART framed protocol** for STM32 communication
- **Hardware reset control** (optional NRST pin)

### STM32 Bootloader
- **Bare-metal implementation** (no HAL dependencies)
- **Fail-safe operation** with metadata validation
- **CRC32 verification** (IEEE 802.3 reflected)
- **Secure flash management** with proper locking
- **Dual UART support** (ESP32 + PC logging)
- **Jump to application** with vector table relocation

### STM32 Application
- **FreeRTOS-based** task scheduling
- **UART command interface** for bootloader entry
- **LED status indication**
- **Producer/Consumer queue demo**
- **Proper vector table relocation**

## 🏗️ Hardware Requirements

### STM32F103C8T6 (Blue Pill)
- **MCU**: STM32F103C8T6 (64KB or 128KB Flash variants)
- **Clock**: 8MHz HSE → 72MHz via PLL
- **Flash**: Minimum 64KB for basic operation

### ESP32 DevKit
- **Wi-Fi capability** for web interface
- **UART communication** to STM32

### Connections
```
ESP32          STM32F103C8T6
-----          -------------
GPIO17 (TX2) → PA10 (USART1 RX)
GPIO16 (RX2) ← PA9  (USART1 TX)
GPIO25       → NRST (optional reset)
GND          → GND
```

### Optional PC Connection
```
USB-UART     STM32F103C8T6
--------     -------------
TX         → PB11 (USART3 RX)
RX         ← PB10 (USART3 TX)
GND        → GND
```

## 🗺️ Memory Map

### 128KB Flash Configuration (Default)
```
0x08000000 - 0x08003FFF : Bootloader (16KB)
0x08004000 - 0x0801FBFF : Application Space (111KB)
0x0801FC00 - 0x0801FFFF : Metadata (1KB)
0x20000000 - 0x20004FFF : RAM (20KB)
```

### 64KB Flash Configuration
```
0x08000000 - 0x08003FFF : Bootloader (16KB)
0x08004000 - 0x0800EBFF : Application Space (43KB)
0x0800EC00 - 0x0800FFFF : Metadata (5KB)
```

## 🚀 Getting Started

### Prerequisites
- **STM32CubeIDE** or **ARM GCC toolchain**
- **Arduino IDE** with ESP32 support
- **ST-Link** or compatible programmer
- **STM32CubeProgrammer** (optional)

### Building the Bootloader

1. **Open STM32CubeIDE**
   ```bash
   cd stm32_bootloader
   # Import project into STM32CubeIDE
   ```

2. **Configure memory layout**
   - Verify `STM32F103C8TX_FLASH.ld` matches your flash size
   - Default configuration is for 128KB flash

3. **Build bootloader**
   ```bash
   # In STM32CubeIDE: Project → Build Project
   # Or using command line:
   make -C Debug
   ```

### Building the Application

1. **Open application project**
   ```bash
   cd stm32_app
   # Import into STM32CubeIDE
   ```

2. **Build application**
   ```bash
   # In STM32CubeIDE: Project → Build Project
   make -C Debug
   ```

### Setting up ESP32 Bridge

1. **Install ESP32 support** in Arduino IDE
2. **Configure Wi-Fi credentials** in `ota_bridge.ino`:
   ```cpp
   const char* WIFI_SSID = "YourWiFiName";
   const char* WIFI_PASS = "YourPassword";
   ```
3. **Upload to ESP32**

### Initial Programming

1. **Flash bootloader** to STM32:
   ```bash
   # Using STM32CubeProgrammer
   STM32_Programmer_CLI -c port=SWD -w stm32_bootloader/Debug/stm32_bootloader.elf -v -rst
   ```

2. **Flash initial application**:
   ```bash
   STM32_Programmer_CLI -c port=SWD -w stm32_app/Debug/stm32_app.bin 0x08004000 -v -rst
   ```

## 📡 UART Protocol

### Frame Format
```
STX | TYPE | SEQ | LEN_LO | LEN_HI | PAYLOAD | CRC16_LO | CRC16_HI
 1     1     1      1        1       0-255       1         1
```

### Command Types
- `0x51` (TYPE_Q) - Query commands (INFO, ERASE, VERIFY, RUN)
- `0x48` (TYPE_H) - Firmware header
- `0x44` (TYPE_D) - Data chunk
- `0x45` (TYPE_E) - End of transfer

### Responses
- `0x06` - ACK (Success)
- `0x15` - NACK (Error/Retry)

## 🌐 Web Interface

### Endpoints
- `/` - Main upload interface
- `/upload` - POST firmware upload
- `/info` - System information
- `/enter_boot` - Force bootloader mode
- `/verify` - Verify firmware
- `/run` - Start application
- `/reset` - Hardware reset
- `/log` - Real-time logs

### Usage
1. Connect ESP32 to Wi-Fi
2. Open web browser to ESP32 IP address
3. Select `.bin` file (application binary)
4. Click "Upload Firmware"
5. Monitor progress and logs

## 🔧 Development

### Bootloader Commands
Send commands via UART1 (115200 baud):
- `INFO` - Get system information
- `ERASE` - Erase application flash
- `VERIFY` - Verify current firmware
- `RUN` - Jump to application

### Application Commands
Send commands via UART1 from application:
- `B` - Enter bootloader mode (sets boot request flag)

### Debug Logging
Connect USB-UART to UART3 (PB10/PB11) for debug output at 115200 baud.

## 📁 Project Structure

```
bootloaderF1_OS/
├── README.md                    # This file
├── ota_bridge.ino              # ESP32 web server
├── stm32_bootloader/           # Bootloader project
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── boot.c          # Main bootloader logic
│   │   │   ├── flash.c         # Flash operations
│   │   │   ├── uart.c          # UART communication
│   │   │   └── crc32.c         # CRC32 calculation
│   │   └── Inc/
│   │       └── boot.h          # Bootloader definitions
│   └── Debug/                  # Build output
└── stm32_app/                  # Application project
    ├── Core/
    │   ├── Src/
    │   │   ├── app.c           # Main application
    │   │   ├── bsp_gpio.c      # GPIO functions
    │   │   └── bsp_uart.c      # UART functions
    │   └── Inc/
    └── Debug/                  # Build output
```

## 🛠️ Configuration

### Flash Size Configuration
Edit memory layout in linker scripts:
- `STM32F103C8TX_FLASH.ld` - Adjust `FLASH` origin and length
- Update `APP_BASE` and `META_BASE` addresses accordingly

### UART Configuration
Default settings (configurable in source):
- **Baud rate**: 115200
- **Data bits**: 8
- **Stop bits**: 1
- **Parity**: None

### Wi-Fi Configuration
Update in `ota_bridge.ino`:
```cpp
const char* WIFI_SSID = "YourNetwork";
const char* WIFI_PASS = "YourPassword";
const char* AP_SSID   = "ESP32-OTA";    // Fallback AP
const char* AP_PASS   = "12345678";
```

## 🔍 Troubleshooting

### Common Issues

1. **Application won't start**
   - Check vector table relocation in application
   - Verify application is built for correct base address
   - Check metadata validation

2. **Upload fails**
   - Verify UART connections
   - Check baud rate settings
   - Ensure bootloader is running

3. **Web interface not accessible**
   - Check Wi-Fi credentials
   - Try connecting to AP mode (ESP32-OTA)
   - Verify ESP32 power and connections

### Debug Tips
- Use UART3 for bootloader debug output
- Check LED patterns for system status
- Monitor web interface logs for transfer progress

## 📋 License

This project is provided as-is for educational and development purposes. Individual components may have their own licenses:
- STM32 HAL libraries: STMicroelectronics license
- FreeRTOS: MIT license
- ESP32 Arduino framework: LGPL 2.1