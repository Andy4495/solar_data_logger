# Solar Data Logger

[![Arduino Compile Sketches](https://github.com/Andy4495/solar_data_logger/actions/workflows/arduino-compile-sketches.yml/badge.svg)](https://github.com/Andy4495/solar_data_logger/actions/workflows/arduino-compile-sketches.yml)

[![Check Markdown Links](https://github.com/Andy4495/solar_data_logger/actions/workflows/CheckMarkdownLinks.yml/badge.svg)](https://github.com/Andy4495/solar_data_logger/actions/workflows/CheckMarkdownLinks.yml)

This sketch takes voltage measurements at regular intervals and stores the data in non-volatile internal [FRAM][4].

The sketch was designed specically for use with an [MSP430FR2433 LaunchPad][5] powered with a solar panel, but could be adapted to other boards (particularly other "FR" variants of the MSP430) and other power sources or data collection needs.

The MspTandV [library][1] is used to get calibrated measurements of $V_{cc}$ and an analog input pin. I use the analog input to measure the raw solar cell voltage output through a resistor divider.

By storing the data in FRAM, the sketch can be run without being connected to a computer and the data can be retrieved at a later time.

The sketch has been tested on the FR2433 variant and should work on other FR variants supported by the [MspTandV][1] library (FR4133, FR6989, FR5969).

## Configuration

The sketch has several `#define` statements that can be updated to fit the project's needs:

| Parameter            | Default Value | Description |
| -------------------- | ------------- | ----------- |
| `LOOP_TIME`          | 60000UL       | Time in ms between voltage readings. |
| `LED_TIME`           | 3000UL        | Time in ms between LED toggling  on and off. |
| `SAMPLE_COUNT`       | 1536          | Number of samples stored in FRAM. Each sample takes 4 bytes. |
| `REBOOT_DETECTED`    | 5555          | Value to store when board is rebooted without clearing previously stored data. |
| `V_DIV_SCALE_FACTOR` | 0.3197        | Scaling factor from the voltage divider. See below. |
| `RAW_ADC_PIN`        | 5             | Analog pin used to measure the raw voltage (typically through a voltage divider). |
| `LED_PWM_LEVEL`      | 32            | Controls LED brightness (0 to 255). Higher is brighter and uses more current. |

The `V_DIV_SCALE_FACTOR` is used to calculate an actual voltage value based on the calibrated ADC reading using the chip's internal voltage reference:

$$
V_{Calculated} =  ADC_{Calibrated} \left(V_{ref} \over ADC\\\_STEPS \right) \div {V\\\_DIV\\\_SCALE\\\_FACTOR}
$$

$ADC\\\_STEPS$ and $V_{ref}$ are configured automatically with the MspTandV library.

`V_DIV_SCALE_FACTOR` needs to be calculated specific to your setup::

![image](https://upload.wikimedia.org/wikipedia/commons/3/31/Impedance_voltage_divider.svg)

$V_{in}$ is the voltage you are trying to measure (i.e., $V_{Calculated}$)  
$V_{out}$ is the analog input pin.  
$Z_1$ and $Z_2$ should be measured with an accurate DMM.

Then calculate and use the result to update `#define V_DIV_SCALE_FACTOR`:

$$
{V\\\_DIV\\\_SCALE\\\_FACTOR} = { Z_2 \over ( Z_1 + Z_2 ) }
$$

For example, if $Z_1$ = 9980 Ohms and $Z_2$ = 4690 Ohms, then `V_DIV_SCALE_FACTOR` should be set to 0.3197.

## Operation

Holding down PUSH1 at reboot prints the stored data over the serial port.

Holding down PUSH2 at reboot clears the data store in FRAM. The LED is briefly flashed when FRAM is cleared.

If a reboot is detected (e.g., due to loss of power from the solar panel), then 5555 (i.e., whatever is defined as REBOOT_DETECTED) is stored in the next data cell.

The LED is flashed every few seconds (per `#define LED_TIME`). Since this takes a little extra power, it may be useful to remove the LED jumper after confirming that the program is running so that the measurements aren't affected by the LED current draw.

## External Libraries

- [MspTandV][1]

## Implementation Details

By default, the FRAM is configured as read-only in the FR2433 variant, so there is a compiler directive to unlock it for write access. This is not needed for other supported FR variants. See [here][47], [here][2], and [here][3].

## References

- MspTandV [library][1]
- MS430FR4xx and MSP430FR2xx Family [User Guide][2]
- FRAM [FAQ][4] from Texas Instruments

## License

The software and other files in this repository are released under what is commonly called the [MIT License][100]. See the file [`LICENSE.txt`][101] in this repository.

[1]: https://github.com/Andy4495/MspTandV
[2]: https://www.ti.com/lit/pdf/slau445
[3]: https://embeddedcomputing.weebly.com/launchpad-msp430fr2433.html
[4]: https://www.ti.com/lit/wp/slat151/slat151.pdf
[5]: https://www.ti.com/tool/MSP-EXP430FR2433
[47]: https://github.com/energia/msp430-lg-core/issues/47
[100]: https://choosealicense.com/licenses/mit/
[101]: ./LICENSE.txt
[//]: # ([200]: https://github.com/Andy4495/Solar-Data-Logger)

[//]: # (This is a way to hack a comment in Markdown. This will not be displayed when rendered.)
