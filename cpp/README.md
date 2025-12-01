# Greaseweazle C++ Library (libusb 0.1)

C++ port of the Greaseweazle host tools using libusb 0.1 as the USB backend.

## Overview

This is a C++ implementation of the Greaseweazle USB communication protocol,
designed to work with libusb 0.1 (or libusb-compat-0.1 on modern systems).
It provides a header-only library for communicating with Greaseweazle floppy
disk controllers.

## Features

- Header-only library (easy integration)
- Full USB protocol implementation
- Flux read/write support
- Drive control (motor, seek, bus type)
- Compatible with libusb 0.1 API

## Requirements

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.14 or later
- libusb 0.1 or libusb-compat-0.1

### Installing libusb 0.1

**Debian/Ubuntu:**
```bash
sudo apt-get install libusb-dev  # or libusb-compat-0.1-dev
```

**Fedora/RHEL:**
```bash
sudo dnf install libusb-compat-0.1-devel
```

**macOS (Homebrew):**
```bash
brew install libusb-compat
```

**Windows:**
Download libusb-win32 from https://libusb.info/

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

### Basic Example

```cpp
#include "greaseweazle.hpp"

using namespace greaseweazle;

int main() {
    // Find and open device
    auto usb = Unit::open();
    if (!usb) {
        std::cerr << "Device not found" << std::endl;
        return 1;
    }

    // Print device info
    std::cout << "Firmware: " << usb->version.first << "." 
              << usb->version.second << std::endl;

    // Set bus type and select drive
    usb->set_bus_type(BusType::Shugart);
    usb->drive_select(0);
    usb->drive_motor(0, true);

    // Seek to track
    usb->seek(0, 0);

    // Read flux data
    Flux flux = usb->read_track(3);  // 3 revolutions
    
    std::cout << "Read " << flux.list.size() << " flux samples" << std::endl;

    // Cleanup
    usb->drive_motor(0, false);
    usb->drive_deselect();

    return 0;
}
```

### CMake Integration

```cmake
find_package(greaseweazle REQUIRED)
target_link_libraries(your_target PRIVATE greaseweazle::greaseweazle)
```

Or include the header-only library directly:

```cmake
add_subdirectory(path/to/greaseweazle/cpp)
target_link_libraries(your_target PRIVATE greaseweazle)
```

## API Reference

### Classes

#### `greaseweazle::Unit`

Main class for USB communication with Greaseweazle.

```cpp
static std::unique_ptr<Unit> open();  // Find and open device

// Device info
uint8_t major, minor;           // Firmware version
uint32_t sample_freq;           // Sample frequency in Hz
uint8_t hw_model, hw_submodel;  // Hardware model
bool update_mode;               // True if in bootloader mode

// Drive control
void set_bus_type(BusType type);
void drive_select(uint8_t unit);
void drive_deselect();
void drive_motor(uint8_t unit, bool state);
void seek(int cyl, int head);
bool get_pin(uint8_t pin);
void set_pin(uint8_t pin, bool level);

// Flux operations
Flux read_track(int revs, uint32_t ticks = 0, int nr_retries = 5);
void write_track(const std::vector<int>& flux_list, 
                 bool terminate_at_index,
                 bool cue_at_index = true,
                 int nr_retries = 5,
                 uint32_t hard_sector_ticks = 0);
void erase_track(uint32_t ticks);
```

#### `greaseweazle::Flux`

Represents raw flux timing data from a floppy disk track.

```cpp
std::vector<double> index_list;  // Index pulse timings
std::vector<double> list;        // Flux transition timings
double sample_freq;              // Sample frequency in Hz
bool index_cued;                 // Whether flux is cued at index

double ticks_per_rev() const;    // Mean time between index pulses
double time_per_rev() const;     // Mean time in seconds
void cue_at_index();             // Align flux data to index
void reverse();                  // Reverse flux data
void scale(double factor);       // Scale timing by factor
```

#### `greaseweazle::BusType`

```cpp
enum class BusType : uint8_t {
    Invalid = 0,
    IBMPC = 1,
    Shugart = 2
};
```

### Exceptions

- `greaseweazle::Fatal` - Unrecoverable errors
- `greaseweazle::CmdError` - Command errors from device

## Examples

The `examples/` directory contains:

- `info.cpp` - Display device information
- `read_track.cpp` - Read flux data from a track

## License

This is free and unencumbered software released into the public domain.
See the file COPYING for more details, or visit http://unlicense.org.

Original Python code written & released by Keir Fraser <keir.xen@gmail.com>
