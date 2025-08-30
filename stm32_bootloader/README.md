# STM32F103C8T6 Bootloader

A bare-metal bootloader for STM32F103C8T6 microcontroller running at 72MHz. This bootloader supports firmware updates via UART protocol and implements fail-safe mechanisms for reliable firmware deployment.

## Features

- **Bare-metal implementation** - No HAL dependencies for core bootloader functions
- **UART-based protocol** - Framed communication protocol for firmware updates
- **Fail-safe operation** - Metadata-driven application validation and rollback
- **Flash management** - Secure flash operations with proper locking mechanisms
- **CRC32 verification** - IEEE 802.3 reflected CRC32 for data integrity
- **Dual UART support** - UART1 for ESP32 communication, UART3 for PC logging

## Hardware Requirements

- **MCU**: STM32F103C8T6 (64KB or 128KB Flash variants supported)
- **Clock**: 72MHz system clock (HSE → PLL × 9)
- **Communication**: 
  - UART1 connected to ESP32 for firmware updates
  - UART3 connected to PC for logging (optional)

## Memory Map

### 128KB Flash Configuration (Default)
```
0x08000000 - 0x08003FFF : Bootloader (16KB)
0x08004000 - 0x0801FBFF : Application Space
0x0801FC00 - 0x0801FFFF : Metadata (1KB)
```

### 64KB Flash Configuration (FLASH_64K defined)
```
0x08000000 - 0x08003FFF : Bootloader (16KB)
0x08004000 - 0x0800FBFF : Application Space
0x0800FC00 - 0x0800FFFF : Metadata (1KB)
```

## Protocol Specification

### Frame Format
```
STX | TYPE | SEQ | LEN(LE16) | PAYLOAD | CRC16-CCITT
```

### Message Types
- **TYPE_H (0x01)**: Firmware Header
- **TYPE_D (0x02)**: Data Chunk
- **TYPE_E (0x03)**: End of Transfer
- **TYPE_Q (0x10)**: Command

### Commands (TYPE_Q)
- **CMD_INFO (1)**: Get bootloader information
- **CMD_BOOT (2)**: Request bootloader mode
- **CMD_ERASE (3)**: Erase application flash
- **CMD_BEGIN (4)**: Begin firmware update
- **CMD_VERIFY (5)**: Verify firmware integrity
- **CMD_RUN (6)**: Execute application

### Response Codes
- **ACK (0xAA)**: Command successful
- **NACK (0xEE)**: Command failed

## Firmware Update Process

1. **Header Phase**: Send firmware header with version, size, and CRC32
2. **Data Phase**: Send firmware data in chunks
3. **End Phase**: Signal completion and verify integrity
4. **Validation**: Bootloader validates CRC32 and updates metadata
5. **Execution**: Jump to new application if validation passes

## Metadata Structure

The bootloader maintains metadata in the last flash page:

```c
typedef struct {
    uint32_t magic;    // META_MAGIC (0x4D455441)
    uint32_t version;  // Firmware version
    uint32_t size;     // Application size in bytes
    uint32_t crc32;    // CRC32 of application
    uint32_t flags;    // Status flags (MF_*)
} bl_meta_t;
```

### Status Flags
- **MF_BOOT_REQ**: Application requests bootloader mode
- **MF_VALID**: Application image is validated
- **MF_IN_PROGRESS**: Firmware update in progress
- **MF_ROLLBACK**: Rollback requested

## Build Configuration

### Prerequisites
- STM32CubeIDE or compatible ARM GCC toolchain
- STM32CubeMX (for hardware configuration)

### Build Options
Define `FLASH_64K` in `boot.h` for 64KB flash variants:
```c
#define FLASH_64K
```

### Compilation
```bash
# Using STM32CubeIDE
# Open project and build using IDE

# Using Make (if makefile is configured)
make clean
make all
```

## Project Structure

```
stm32_bootloader/
├── Core/
│   ├── Inc/
│   │   ├── boot.h              # Bootloader public header
│   │   ├── main.h              # Main header
│   │   └── stm32f1xx_*.h       # STM32 HAL headers
│   ├── Src/
│   │   ├── boot.c              # Main bootloader logic
│   │   ├── flash.c             # Flash operations
│   │   ├── uart.c              # UART communication
│   │   ├── crc32.c             # CRC32 calculations
│   │   └── main.c              # System initialization
│   └── Startup/
│       └── startup_stm32f103c8tx.s
├── Drivers/                    # STM32 HAL/CMSIS drivers
├── Debug/                      # Build outputs
├── STM32F103C8TX_FLASH.ld      # Linker script
└── stm32_bootloader.ioc        # STM32CubeMX configuration
```

## Usage

### Entering Bootloader Mode
The bootloader will remain active if:
1. No valid application is found in flash
2. Application sets the `MF_BOOT_REQ` flag in metadata
3. Hardware boot condition is met (implementation dependent)

### Firmware Update via UART
1. Connect to UART1 (typically pins PA9/PA10)
2. Send firmware update commands following the protocol
3. Monitor responses for success/failure
4. Bootloader will validate and program new firmware

### Application Development
Applications should:
1. Set vector table offset to `APP_BASE` (0x08004000)
2. Implement metadata management if bootloader interaction is needed
3. Use proper CRC32 calculation for firmware validation

## Safety Features

- **Flash protection**: All flash operations are properly locked/unlocked
- **CRC validation**: Firmware integrity checked before execution
- **Metadata validation**: Magic number and flag validation
- **Error recovery**: Invalid firmware detection and recovery mechanisms
- **Watchdog compatibility**: Designed to work with watchdog timers

## Debugging

- UART3 can be used for debug logging to PC
- Debug symbols available in Debug/ directory
- Use STM32CubeIDE debugger or compatible tools
- Flash programming via SWD/JTAG supported

## License

This project is provided under STMicroelectronics license terms. See individual source files for specific license information.

## Contributing

1. Follow existing code style and commenting conventions
2. Test on both 64KB and 128KB flash configurations
3. Ensure all safety mechanisms remain functional
4. Update documentation for any protocol changes

## Troubleshooting

### Common Issues

**Bootloader not responding**
- Check UART connections and baud rate
- Verify power and reset conditions
- Ensure proper flash programming of bootloader

**Application not starting**
- Check metadata validity in flash
- Verify application vector table offset
- Confirm CRC32 calculation matches expected format

**Flash programming errors**
- Ensure flash unlock/lock sequence is correct
- Check for flash protection or write protection
- Verify adequate power supply during programming

### Debug Tips

- Use UART3 for logging if available
- Monitor flash content using debugger
- Check metadata area for corruption
- Verify clock configuration matches expectations

---

For technical support or questions, refer to the source code comments or STM32 documentation.
