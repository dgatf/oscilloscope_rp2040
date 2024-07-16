# Oscilloscpe RP2040 - OpenHantek protocol

This is an oscilloscope for the RP2040 that implements [OpenHantek6022](https://github.com/OpenHantek/OpenHantek6022) protocol.

## Specifications

- 2 channels
- 200 kS/s

## Usage

Upload the [binary](https://drive.google.com/file/d/11BkBbbuAzuO7nqTozGVK0Epi27-wBS4m/view?usp=drive_link).

Connect to an OpenHantek6022.

OpenHantek6022 needs to be modified for the RP2040:

- [OpenHantek6022 fork](https://github.com/dgatf/OpenHantek6022)

You can build OpenHantek following the steps.  
Or download the compiled binary for linux: [OpenHantek6022-RP2040.AppImage](https://drive.google.com/file/d/11BkBbbuAzuO7nqTozGVK0Epi27-wBS4m/view?usp=drive_link).

## Pins

- Channel 1 -> GPIO 26
- Channel 2 -> GPIO 27
- Debug enable/disabble -> GPIO 18
- Debug output -> GPIO1 16

If enabled, debug output is on GPIO 16 at 115200bps. To enable debug, connect to ground GPIO 18 at boot.

Led is on during the capture process.

<p align="center"><img src="./images/circuit.png" width="800"><br>  

## Installation

Upload the binary to the RP2040. Drag and drop [oscilloscope.uf2](https://drive.google.com/file/d/11BkBbbuAzuO7nqTozGVK0Epi27-wBS4m/view?usp=sharing).

## Openhantek6022

Device is detected automatically (modifed version).

## Configuration

__Debug mode__  
GPIO 18 to GND: enable debug mode. Debug output is on GPIO 16 at 115200 bps.

If no GPIO is grounded, the default configuration is:

- Debug mode: disabled.

## References

- [OpenHantek6022](https://github.com/OpenHantek/OpenHantek6022)
- [libsigrok](https://github.com/sigrokproject/libsigrok)
- Das Oszi Protocol - eLinux.org
