# seeed-xiao-nrf52840-rs
using Rust on a Seeed Xiao NRF52840 board

Note: I'm using the board without "Sense" in the name. However, it appeared that the same firmware was present on the chip.

# Rust compiler
Install rust e.g. via `rustup`, see [https://rust-lang.org/tools/install/]  
`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
## Target
The compiler must utilize the command set of the specific chip on which the code is intended to run. The nRF52840 is based on the ARM Cortex-M4.
* `rustup target add thumbv7em-none-eabihf` for nrf52840
  * `thumb` compact version of ARM command set, optimized for micro controller
  * `v7em` version of ARM Architecture ARMv7-E with micro controller profile Cortex-M
  * `none` bare metal, no operating system
  * `eabihf` Embedded Application Binary Interface (EABI) with Hardware float

## Hardware Abstraction Layer (HAL)

## Peripheral Access Crates (PACs)

## SVD

## Linker Configuration / Memory Map
The Bootloader and the SoftDevice needs to be concidered by the linker

### Flash
nrf52840: 1 MB FLASH (0x00000000 - 0x00100000)
Bootloader: 16 kB FLASH (0x00000000 - 0x00004000) (overlapping with SoftDevice)
MBR + SoftDevice s140 v7.3.0: 156.0 kB FLASH (0x00000000 - 0x00027000)
User Code: Starting at 0x00027000

### RAM
nrf52840: 256 KB RAM (0x20000000 - 0x20040000)
Bootloader: 0 B RAM (inactive while usercode is running)
SoftDevice s140 v7.3.0: 5.6 kB RAM (0x20000000 - 0x20001678)
User Code: Starting at `0x20001678` (nach SoftDevice)

### Memory Map
```
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* SoftDevice and Bootloader occupy the first 0x27000 bytes */
  FLASH : ORIGIN = 0x00027000, LENGTH = 1024K - 0x27000
  RAM : ORIGIN = 0x20000000, LENGTH = 256K - 0x1678
}

INCLUDE "nrf52840.ld"
```

### Source
* Offcial Documentation of SoftDevice [https://docs.nordicsemi.com/bundle/sds_s140/page/SDS/s1xx/mem_usage/mem_resource_map_usage.html#mem_resource_map_usage__fig_tjt_thp_3r]
* Nordic SoftDevice Downlodes includes Release Notes [https://www.nordicsemi.com/Products/Development-software/S140/]
* Rust Embassy binding to use SoftDevice [https://github.com/embassy-rs/nrf-softdevice/]
* memory.x  [https://github.com/Wumpf/Seeed-nRF52840-Sense-projects/blob/main/memory.x]

# Programming
## Programming Via SWD-Pins (not used here)
The Board provides GND, SWDIO, SWCLK, VCC Pads on the bottom. This can be used with an physical programming device (JTAG).

### probe-rs
`probe-rs` you'll need an external SWD/JTAG Programmer. Not supported by the bootloader.

## Programming Via Bootloader
The board utilizes a modified bootloader [https://github.com/0hotpotman0/Adafruit_nRF52_Bootloader].
It's a Fork of the UF2 Bootloader from Adafruit [https://github.com/adafruit/Adafruit_nRF52_Bootloader].
You can drop UF2 files into the mass storage emulation, there is also an DFU (Direct Firmware Upload) Mode.

### Board Naming Indicates Mode
* List USB devices via console
  * on Mac: `ioreg -p IOUSB`
  * on Linux `lsusb`
* Device naming depends on active mode
  * User Firmware: Board will show up as `XIAO nRF52840 Sense`
  * UF2 Bootloader: Board will show up as `Seeed XIAO nRF52840 Sense`

### Mass Storage Emulation
The mass storage emulation shows 3 files
* `CURRENT.UF2` for downloading the current Firmware 
* `INDEX.HTM` redirects to [https://www.seeedstudio.com]
* `INFO_UF2.TXT` Bootloader / Board / SoftDevice info
```
UF2 Bootloader 0.6.2-12-g459adc9-dirty lib/nrfx (v2.0.0) lib/tinyusb (0.10.1-293-gaf8e5a90) lib/uf2 (remotes/origin/configupdate-9-gadbb8c7)
Model: Seeed XIAO nRF52840
Board-ID: Seeed_XIAO_nRF52840_Sense
Date: Nov 30 2021
SoftDevice: S140 7.3.0
```
You can flash a new firmware by drag and drop (UF2 format required).

### nrfutil
* Nordic replaced `nrfprog` with `nrfutil` some time ago.
* `nrfutil device list` shows connected devices Serialnumber, Product Name, and Mounting Point (/dev/tty.usbmodem11) and traits (capabilities e.g. serialPorts, usb)

User firmware:
```
000683298616
Product         Seeed XIAO nRF52840 Sense
Ports           /dev/tty.usbmodem101
Traits          serialPorts, usb
```
Bootloader:
```
000683298616
Product         XIAO nRF52840 Sense
Ports           /dev/tty.usbmodem101
Traits          serialPorts, usb
```

#### programming via `nrfutil`
`nrfutil device program --serial-number 000683298616 --firmware path_to_some_firmware.hex`

# Serial connection
* Quicktest for Serial output from the device
  * on Mac: `screen /dev/tty.usbmodem11` Connect on serial port /dev/tty.usbmodem11. Hit CTRL+A then D to exit.

# Sources / See also
* https://github.com/Wumpf/Seeed-nRF52840-Sense-projects
