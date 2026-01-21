# seeed-xiao-nrf52840-rs
using Rust on a Seeed Xiao NRF52840 board

> [!NOTE]
> I'm using the board without "Sense" in the name. However, it appeared that the same firmware was present on the chip.

# Prerequisite

## Rust compiler
Install rust e.g. via `rustup`, see [rust documentation](https://rust-lang.org/tools/install/)  
`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`

## Target-Support
The compiler must utilize the command set of the specific chip on which the code is intended to run. The nRF52840 is based on the ARM Cortex-M4.

`rustup target add thumbv7em-none-eabihf` for nrf52840

The target-tripple (in this case `thumbv7em-none-eabihf`) is encoded according to following sceme:
  * `thumb` compact version of ARM command set, optimized for micro controller
  * `v7em` version of ARM Architecture ARMv7-E with micro controller profile Cortex-M
  * `none` bare metal, no operating system
  * `eabihf` Embedded Application Binary Interface (EABI) with Hardware float

Supported (and installed) Targets can be listed by `rustup target list`. The alternative command `rustc --print target-list` shows more targets.

The target needs to be noted in the project configuration file `.cargo/config.toml` (see below).

## Linker
The default rust linker `lld` does support limited systems. However, ARM-based systems are not supported out of the box.  
For ARM developement the detailt toolchain is `arm-none-eabi-gcc`. The containing linker ist named `arm-none-eabi-ld`.

mac: `brew install --cask gcc-arm-embedded`

The linker `arm-none-eabi-ld` needs to be addressed in the `.cargo/config.toml` (see below). As an alternative, it seemed `arm-none-eabi-gcc` could also be specified, and this provides some kind of linker time optimization.

Since we're looking for cross-compiling on an bare metal system, the linker needs detailed specification about adress ranges and data in an memory map (see below).

May the [meld linker](https://github.com/rui314/mold) is an alternative.

* [arm-gnu-toolchain on developer.arm.com](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
* [gcc-arm-embedded on formulae.brew.sh](https://formulae.brew.sh/cask/gcc-arm-embedded)
* [rustc: platform-support/arm-none-eabi](https://doc.rust-lang.org/rustc/platform-support/arm-none-eabi.html#requirements)

## Linker Configuration / Memory Map
The memory organization in your specific controller needs to be specified in a memory map for the linker.  
In case your system is using a bootloader and/or SoftDevice also this needs to be concidered.

The linker-script `memory.x` (sometimes a different name with a `.ld` extension) specifies address regions via MEMORY and program SECTIONS to be placed within regions. Both are optional; however, at least the MEMORY command is important to inform the linker about the accessability, size, and address range of Flash and RAM and the existence of Bootloader and/or Softdevice.

### Memory Map
`memory.x` file, located in the project root
```
MEMORY
{
  /* Flash memory for the application.
   * CHIP nrf52840        0x00000000-0x01000000 (1 MiB = 1024 KiB)

   * MBR                  0x00000000-0x00001000 (4K)
   * SoftDevice           0x00001000-0x00027000 (156K)
   * Application          0x00027000-0x000F4000 (--> 818K)
   * Bootloader + config  0x000F4000-0x000FE000 (38K+2K)
   * MBR Params Page      0x000FE000-0x000FF000 (4K)
   * Bootloader settings  0x000FF000-0x00100000 (4K)
   */
  FLASH (rx) : ORIGIN = 0x27000, LENGTH = 0x000F4000 - 0x00027000 /* 812 KiB */

  /* RAM for the application.
   * CHIP nrf52840        0x20000000-0x20040000 (256 KiB)

   * Bootloader DblReset  0x20007F7C-0x20007F80 (0x04)
   * NOINIT               0x20007F80-0x20008000 (0x80)
   * Bootloader         ( 0x20008000-0x20040000 )
   * Application          0x20008000-0x20040000 (~250K)
   * SoftDevice uses  0x1678 bytes (~5.75KiB) of RAM.
   */

  /* Avoid conflict with NOINIT for OTA bond sharing */
  RAM (rwx) : ORIGIN = 0x20008000, LENGTH = 0x20040000 - 0x20008000 - 0x1678 /* ~252kB */

  /** Location of non initialized RAM. Non initialized RAM is used for exchanging bond information
   *  from application to bootloader when using buttonless DFU OTA. */
  APP_NOINIT (rwx) : ORIGIN = 0x20007F80, LENGTH = 0x80
}
```

### Source
* [Linkerfile of Seeed Bootloader](https://github.com/0hotpotman0/Adafruit_nRF52_Bootloader/blob/master/linker/nrf52840.ld)
* [LD documentation](https://sourceware.org/binutils/docs-2.21/ld/)
* [Offcial Documentation of SoftDevice S140](https://docs.nordicsemi.com/bundle/sds_s140/page/SDS/s1xx/mem_usage/mem_resource_map_usage.html#mem_resource_map_usage__fig_tjt_thp_3r)
* [Nordic SoftDevice Downlodes includes Release Notes](https://www.nordicsemi.com/Products/Development-software/S140/)
* [Rust Embassy binding to use SoftDevice](https://github.com/embassy-rs/nrf-softdevice/)
* memory.x [Wumpf/Seeed-nRF52840-Sense-projects](https://github.com/Wumpf/Seeed-nRF52840-Sense-projects/blob/main/memory.x)
* memory.x [example in embassy-rs/embassy](https://github.com/embassy-rs/embassy/blob/main/examples/nrf52840/memory.x)
* (https://docs.nordicsemi.com/bundle/sds_s140/page/SDS/s1xx/mbr_bootloader/bootloader.html)
* (https://docs.nordicsemi.com/bundle/sds_s140/page/SDS/s1xx/mem_usage/mem_resource_map_usage.html)


# cargo: create an empty rust project

Either
```
cargo new project_name
cd project_name
```
or
```
mkdir project_name
cd project_name
cargo init
```
Options are `--bin` (default) or `--lib`


## Project Configuration

Create the project configuration file `.cargo/config.toml`.  
Define the target and configure the linker and additional linker-flags.

> [!WARNING]
> Watch out, the cargo system also controls some linker flags. Your configuration may collide with the cargo settings.

```toml
[build]
target = "thumbv7em-none-eabihf" # Armv7-EM Architektur, Bare-Metal, Hard Float

[target.thumbv7em-none-eabihf]
linker = "arm-none-eabi-ld" # Linker for ARM
rustflags = [
    "-C", "link-arg=-Tmemory.x",  # Linker script (MEMORY and SECTIONS)
    "-C", "link-arg=--nmagic", # no magic number on bare metal system necessary
#    "-C", "link-arg=--nostartfiles", # no standard start files on bare metal
    "-C", "link-arg=-Map=output.map", # map file for debugging (project root)
#    "-C", "panic=abort", # covered by cargo
#    "-C", "opt-level=z", # covered by cargo
]
```

# Write Your Firmware

## System View Description (SVD)
XML file, describing registers, storage addresses, and peripherals. The file is provided by the semiconductor manufacturer for each specific controller type.

## Peripheral Access Crates (PACs)
Controller Family- or Type-specific.
Rust Crate, based on SVD. Provides safe and typesafe access to hardware registers.

Can be created by using `svd2rust`. However, the community has already done this for you.

* [nrf-rs/nrf-pacs](https://github.com/nrf-rs/nrf-pacs)
* [nrf52840-pac on  crates.io](https://crates.io/crates/nrf52840-pac)

## Hardware Abstraction Layer (HAL)
Independent of platform, makes porting of SW between MCU vendors possible.

Abstraction layer on top of the PAC. Provides a user-friendly API for e.g. UART, SPI, I2C. The developer is not required to work directly with the registers.

* [nrf-rs/nrf-hal](https://github.com/nrf-rs/nrf-hal)
* [nrf52840-hal on crates.io](https://crates.io/crates/nrf52840-hal)

## Board Support Crates (BSCs)
Abstraction Layer on top of HAL, Providing simple access to your external chips on the PCB.

Currently no BSC known for Seeed Xiao nrf52840. It can be created on your own.

* [creates.io](https://crates.io)

# Sourcecode

## Add dependencies

`cargo add <crate>`
* `cortex-m` [crates.io]()
* `cortex-m-rt`
* `embedded-hal`
* `nrf52840-hal`
* `panic-halt` [](https://crates.io/crates/panic-halt)

Put the dependency to your `Cargo.toml`
```toml
[dependencies]
nrf52840-pac = "0.11.0"  # PAC as basis
nrf-hal = "0.16.0"       # HAL for nRF52840
embedded-hal = "0.2.7"   # Trait-definition for HAL (option)

```
Build Script `build.rs`
```rust
fn main () {
    println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rerun-if-changed=memory.x");
}
```

Use this in your rust code `src/main.rs`
```rust
#![no_main]
#![no_std]

use nrf52840_hal as hal;
use nrf52840_pac as pac;
use embedded_hal::serial::Write;

fn main() {
    let peripherals = pac::Peripherals::take().unwrap();
    let gpio = &peripherals.P0; // access GPIO-Port 0

    let port0 = hal::gpio::p0::Parts::new(p.P0);

}
```

## compilen
`cargo build --release`
ELF-File now in `target/thumbv7em-none-eabihf/release`

# Programming

## Programming Via SWD-Pins (not used here)
The Board provides GND, SWDIO, SWCLK, VCC Pads on the bottom. This can be used with an physical programming device (JTAG).

### probe-rs
> [!NOTE]
> Section not verified, just various information puzzling.

> [!NOTE]
> Not supported by the UF2 bootloader on Seeed board.

`probe-rs` you'll need an external SWD/JTAG Programmer.  
probe-rs will support debugging.

`cargo install --locked probe-rs-tools`

Make sure your .cargo/config.toml contains the following: (tbc)
```toml
[target.thumbv6m-none-eabi]
runner = "probe-rs run --chip NRF52840"
```

may:
`cargo flash --chip NRF52840 --release`

* (https://probe.rs)
* [rp-rs/rp-hal-boards](https://github.com/rp-rs/rp-hal-boards/tree/5135e3dafe3e69b112e6d2d72cfb1856a7679b82)


## elf2uf2 converter

`cargo install --locked elf2uf2-rs`

Make sure your .cargo/config.toml contains the following:
```toml
[target.thumbv6m-none-eabi]
runner = "elf2uf2-rs -d"
```

* [rp-rs/rp-hal-boards](https://github.com/rp-rs/rp-hal-boards/tree/5135e3dafe3e69b112e6d2d72cfb1856a7679b82)

## cargo hf2 subcommand
`cargo-hf2` provides a cargo subcommand for requesting USB devices, supporting acceptance of UF2 files, then connecting and uploading.

* [jacobrosenthal/hf2-rs](https://github.com/jacobrosenthal/hf2-rs) [cargo-hf2 on crates.io](https://crates.io/crates/cargo-hf2)

## Programming Via Bootloader
The board utilizes a modified bootloader [0hotpotman0/Adafruit_nRF52_Bootloader](https://github.com/0hotpotman0/Adafruit_nRF52_Bootloader).  
It's a Fork of the UF2 Bootloader from Adafruit [adafruit/Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).  
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
* `INDEX.HTM` redirects to (https://www.seeedstudio.com)
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
* [Wumpf/Seeed-nRF52840-Sense-projects](https://github.com/Wumpf/Seeed-nRF52840-Sense-projects)
