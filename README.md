# Oscilloscpe RP2040 - OpenHantek protocol

This is an oscilloscope for the RP2040 that implements [OpenHantek6022](https://github.com/OpenHantek/OpenHantek6022) protocol.

## Specifications

- 2 channels
- 200 kS/s

## Usage

Upload the binary [oscilloscope.uf2](https://drive.google.com/file/d/11BkBbbuAzuO7nqTozGVK0Epi27-wBS4m/view?usp=drive_link) to RP2040.

Open OpenHantek6022 (modifed version).

OpenHantek6022 needs to be modified for the RP2040:

- [OpenHantek6022 fork](https://github.com/dgatf/OpenHantek6022)

You can build OpenHantek following the steps.  
Or download the compiled binary for linux: [OpenHantek6022-RP2040.AppImage](https://drive.google.com/file/d/1I9Y5-4aRr0rqPs-FJkVPN9S7YzMd2pfn/view?usp=sharing). For linux you may need to copy OpenHantek [udev rules](https://github.com/dgatf/OpenHantek6022/tree/main/utils).

## Pins

- Channel 1 -> GPIO 26
- Channel 2 -> GPIO 27
- Calibration signal -> GPIO 22
- Debug enable/disabble -> GPIO 18
- Debug output -> GPIO1 16

Voltage at channels (GPIO 26 & 27) must be between 0 and 3.3V.  

If enabled, debug output is on GPIO 16 at 115200bps. To enable debug, connect to ground GPIO 18 at boot.

Led is on during the capture process.

<p align="center"><img src="./images/circuit.png" width="600"><br>  

<p align="center"><img src="./images/openhantek.png" width="600"><br>  

## Configuration

__Debug mode__  
GPIO 18 to GND: enable debug mode. Debug output is on GPIO 16 at 115200 bps.

If no GPIO is grounded, the default configuration is:

- Debug mode: disabled.

## Binaries

- [oscilloscope.uf2](https://drive.google.com/file/d/11BkBbbuAzuO7nqTozGVK0Epi27-wBS4m/view?usp=drive_link)
- Linux: [OpenHantek6022-RP2040.AppImage](https://drive.google.com/file/d/1I9Y5-4aRr0rqPs-FJkVPN9S7YzMd2pfn/view?usp=sharing)
- Windows: you need to compile OpenHantek6022 yourself.

## References

- [OpenHantek6022](https://github.com/OpenHantek/OpenHantek6022)
- [libsigrok](https://github.com/sigrokproject/libsigrok)
- Das Oszi Protocol - eLinux.org
