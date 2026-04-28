# Oscilloscope RP2040 - OpenHantek protocol

An oscilloscope for the RP2040 that implements the [OpenHantek6022](https://github.com/OpenHantek/OpenHantek6022) protocol.

## Specifications

- Up to 1 MS/s
- 2 channels
- OpenHantek6022-compatible protocol

## Usage

1. Upload the [oscilloscope.uf2](#binaries) firmware to the RP2040.

2. Install OpenHantek. OpenHantek6022 needs RP2040 support:
   [OpenHantek6022 fork](https://github.com/dgatf/OpenHantek6022)

   - Build OpenHantek following the project instructions.
   - Or download prebuilt [binaries](https://github.com/dgatf/OpenHantek6022/releases).

3. The external circuit is optional. If you are **not** using the [external circuit](#external-circuit), copy [RP2040_0_calibration.ini](RP2040_0_calibration.ini) to:

   - Linux: `~/.config/OpenHantek/`
   - Windows: `%USERPROFILE%\.config\OpenHantek`

4. Launch the modified version of OpenHantek6022.

## Pins

- Channel 1: GPIO 26
- Channel 2: GPIO 27
- Calibration signal: GPIO 22
- No conversion: GPIO 19
- Debug enable/disable: GPIO 18
- Debug output: GPIO 16

Without the [external circuit](#external-circuit), the input voltage on GPIO 26 and GPIO 27 must stay between 0 V and 3.3 V.

Debug output is available on GPIO 16 at 115200 bps. To enable debug output, connect GPIO 18 to GND at boot.

The LED is on during capture.

<p align="center"><img src="./images/circuit.png" width="600"><br></p>

<p align="center"><img src="./images/openhantek.png" width="600"><br></p>

## External circuit

The external circuit adapts the input signal from ±5 V to the RP2040 ADC range and adds AC/DC coupling.

Signal conversion:

- Step down from 5 V to 3.3 V
- AC/DC coupling
- Optional gain stage
- Center zero voltage at VCC/2

The RP2040 ADC is 12-bit, while OpenHantek expects 8-bit samples. The firmware scales the ADC value from 12 bits to 8 bits. Maximum digital gain is 16. For higher gains, use an additional analog gain stage before centering the signal.

### Materials

- 1 x CD4066 analog switch
- 1 x LM358 op-amp
- 2 x 100 nF ceramic capacitor
- 6 x 10 kΩ resistor
- 2 x 33 kΩ resistor
- 2 x 22 kΩ resistor

If the calibration file was previously copied to the OpenHantek config folder, delete it when using the external circuit.

### Pins

- AC/DC coupling channel 1: GPIO 20
- AC/DC coupling channel 2: GPIO 21

<p align="center"><img src="./images/external_circuit.png" width="600"><br></p>

## Calibration file OpenHantek

If the external circuit is not used, OpenHantek needs a calibration file to map the RP2040 0-3.3 V ADC input to the expected ±5 V range.

Copy [RP2040_0_calibration.ini](RP2040_0_calibration.ini) to:

- Linux: `~/.config/OpenHantek/`
- Windows: `%USERPROFILE%\.config\OpenHantek`

The calibration file sets:

- Offset: `-127`
- Gain: `0.33`

If using the external circuit, set offsets to `0` and gains to `1`, or delete the calibration file.

## Notes

- For rates above ~500 kS/s, the ADC operates at higher clock speeds, which may slightly affect accuracy.
- **Linux:** if the device is not detected, install the appropriate [udev rules](udev/99-openhantek.rules) under `/etc/udev/rules.d/` and reconnect the device.
- **Windows:** you may need to install a WinUSB-compatible driver using Zadig.

Example Linux setup:

```bash
sudo cp udev/99-openhantek.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Binaries

- [RP2040](https://github.com/dgatf/oscilloscope_rp2040/releases)
- [OpenHantek (RP2040)](https://github.com/dgatf/OpenHantek6022/releases)

## References

- [OpenHantek6022](https://github.com/OpenHantek/OpenHantek6022)
- [libsigrok](https://github.com/sigrokproject/libsigrok)
- Das Oszi Protocol - eLinux.org
- [Usb device library](https://github.com/dgatf/usb_library_rp2040)