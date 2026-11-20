# Rev1 power-on bring-up checklist

For the first rev1 STM32 board, before anything is driven. It checks the two
acceptance criteria of the power-on ticket: every net is wired as the board
config says, and nothing pulses at power-up.

The pins in `firmware/config/board_stm32.h` are UNVERIFIED placeholders until
the rev1 schematic is final. Update them from the schematic first, then set
`MOTOR_DRIVERS_FITTED = true`. Until then the firmware touches none of the
rev1 actuator pins.

## 1. Bare board, unpowered

- [ ] Each pin in `REV1_SAFE_OUTPUTS` (nSLEEP, the four DRV8874 IN pins, the
      emitter gates and the fan gate) has continuity from the STM32 pin to
      its part, and no short to its neighbours, 3V3, VBAT or GND.
- [ ] Each of those nets has its pull to the off level (a pull-down on
      nSLEEP, on the emitter gates and on the fan gate). The pins float from
      reset until the firmware's first instruction, and only the pull keeps
      them off then.
- [ ] Encoder, IMU SPI, console UART, IPROPI and LED nets: continuity per
      the schematic.

## 2. GPIO per net, powered from the bench supply (current limit on, motors and fan unplugged)

- [ ] Flash the STM32 build. At the console, `stats` runs and the clock line
      reads HSE.
- [ ] With a meter or a logic probe on each safe-output net: every one reads
      its off level after boot (nSLEEP, IN1/IN2, emitters and fan all low).
- [ ] `motor 0.2 0.2` before `arm` prints "not armed" and nSLEEP stays low.
- [ ] After `profile <n>` (if the board is new) and `arm`: `motor 0.2 0.2`
      raises nSLEEP and PWMs IN1/IN2 on the right pins; `disarm` drops nSLEEP.

## 3. Scope at power-up (motors, fan and emitters fitted)

- [ ] Scope channels on nSLEEP, one motor IN pin, one emitter gate and the
      fan gate; trigger on the 3V3 rail rising, single shot.
- [ ] Power-cycle from the battery switch 5 times and press reset 5 times:
      no channel leaves its off level, and no wheel twitches, no fan blip,
      no emitter flash (check the emitters with a phone camera).
- [ ] Repeat once while armed and driving, with the reset button: reset
      floats every pin, so each channel must fall to off on its pull at once
      and stay there through the reboot.

Record the scope captures with the board's UID in the ticket.
