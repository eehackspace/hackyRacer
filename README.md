# Hacky Racer
Information and code for the Hacky Racer project, in particular the ODrive and electrics.

Motor control is done with a Makerbase ODrive v3.6-56V and a Raspberry Pi Pico connected to the UART

## ODrive Cabling

| ODrive                     | Pico  |
| -------------------------- | ----- |
| 5V on M0/M1 encoder header | VBUS  |
| GND                        | GND   |
| GPIO1                      | GPIO1 |
| GPIO2                      | GPIO0 |

At present there is no protection against backflow of power to the USB port from the ODrive, we should fix that.

## Pico Cabling

| Sensor           | Pico      |
| ---------------- | --------- |
| Throttle         | GPIO26_A0 |
| Steering         | GPIO27_A1 |
| Left hand brake  | GPIO2     |
| Right hand brake | GPIO3     |

The throttle and steering pots take 3.3v from the Pico. Brakes use internal pullups.
