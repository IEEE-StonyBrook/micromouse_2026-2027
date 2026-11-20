# Jurababa 2025-2026 Micromouse

Firmware, an `mms`-compatible simulator, and calibration tools for **Jurababa**,
a Raspberry Pi Pico micromouse built for IEEE maze competitions. Treat it as a
real embedded robotics system: noisy sensors, imperfect motors, battery sag,
wheel slip, and a hard real-time budget.

## The chain

Every change keeps this chain truthful:

```text
sensors → estimated state → maze model → planned path → motor command → physical motion
```

A micromouse fails when software lies about the physical world. When a change's
effect on the chain is unclear, say so before editing.

## Prime directive

Make the smallest correct change that improves the system.

Before editing, read the files involved and state the current design and the
assumptions you are relying on. Then patch the smallest region that fixes the
problem, preserving the existing architecture and public interfaces. A refactor
happens when the user asks for one.

Search the repo before assuming how something works. Invent no APIs, files, or
conventions — check first.

Name risky design choices directly, in a sentence or two, then carry on with the
work.

Use precise units in every explanation.

## Ask before changing

Get explicit approval for these. Each one silently breaks the chain everywhere
else in the system:

- pin mappings (`firmware/config/pins.h`)
- motor direction polarity
- maze coordinate conventions
- wall encoding conventions

## Layers

Dependencies point one way. Higher layers depend on lower ones; a lower layer
never reaches up.

```text
app/ → control/, maze/, navigation/
control/ → drivers/, config/, hal/
navigation/ → maze/
maze/ → config/
drivers/ → config/, hal/
hal/pico/ → Pico SDK
hal/stm32/ → libopencm3
```

What each layer is allowed to know:

| Layer | Knows about | Stays out of |
|---|---|---|
| `config/` | constants, compile-time configuration | logic, algorithms, state, functions |
| `drivers/` | GPIO, PWM, ADC, I2C/UART, PIO, raw hardware units | cells, flood fill, strategy |
| `control/` | velocity, acceleration, PID, profiles, pose, kinematics | maze solving |
| `maze/` | cells, directions, walls, coordinates, mouse pose | motors, sensors |
| `navigation/` | flood fill, A*, goals, cell paths, L/F/R conversion | physical sensors |
| `hal/` | platform-neutral interfaces (time, critical section, reset cause, peripheral handles, console, SPI, flash); backends in `hal/pico/`, `hal/stm32/` and `hal/host/` | robot logic |
| `app/` | wiring the above together, CLI, Bluetooth, the control timer; board-specific halves (Bluetooth UART, BOOTSEL, LEDs) in `app/pico/` behind headers in `app/` | — |

Only `hal/pico/`, the driver `.cpp` files, `app/pico/`, `driver_lab/*.cpp` and
`main_pico.cpp` may include the Pico SDK, and only `hal/stm32/` and
`main_stm32.cpp` may include libopencm3. `python3 tools/check_sdk_free.py`
enforces this, and `make -C tests` compiles the platform-neutral code,
`app/*.cpp` included, against `hal/host`. App code reads the clock and sleeps
through `hal::time`, never the SDK.

`make -C tests` also builds and runs `tests/host_tests`, the doctest unit tests
(`tests/*_test.cpp`) for Profile, PID, Drivetrain, Motion, the maze parser,
the console HAL, the LSM6DSR driver (against a fake SPI register map) and the
STM32 bring-up logic. It runs them twice: at the Pico's 500 Hz, then rebuilt at
the STM32's 1 kHz (`-DHOST_LOOP_FREQUENCY_HZ=1000.0f`), where
`rate_trajectory_test.cpp` checks that a profile move and a closed-loop
straight match the 500 Hz run. Each STM32 driver keeps its arithmetic and
state machines in libopencm3-free headers in `hal/stm32/` (`clock_plan.h`,
`clock_startup.h`, `counter_extender.h`, `console_rings.h`, `flash_layout.h`,
`reset_flags.h`, `watchdog_plan.h`) that the host tests include; only the thin register glue is untested off
target. The flash store (`common/flash_store.h`, two A/B slots in sectors 1-2)
and board identity (`common/board_identity.h`) are platform-neutral and run
against `hal/host/memory_flash.h`, which can cut power at any word:
`flash_store_test.cpp` cuts every word of a save and checks the old or the new
record always loads. The next-boot reset verdict (`common/reset_report.h`:
the reset cause plus the breadcrumb, `common/breadcrumb.h`) is platform-neutral
too and tested in `reset_report_test.cpp`.
`tests/fakes/` stands in for the Motor, Encoder and IMU drivers on the host and
adds a first-order two-wheel plant, so control changes can be tested without a
mouse. `tests/sim/` grows that into a physics sim: a DC-motor, two-wheel
plant with knobs for wheel slip, encoder resolution, gyro noise and bias, a
floor seam and a pinned wheel, tested in `physim_test.cpp` and
`crash_physim_test.cpp`. `make -C tests physim` builds a CLI
(`tests/physim --scenario straight10|ip90|ss90 ...`) that prints a CSV trace.
Add a test case to a `*_test.cpp` file; the Makefile picks new files up.

Run `ls firmware/<layer>` for the current file list; it is authoritative and
this document is not.

## Operating modes

Selected at startup: **press `M` within 3 seconds for DriverLab, otherwise
Normal Mode starts automatically.** Keep that behaviour.

**Normal Mode** is the competition mode. It runs on Core 0 only (the UKMARS
single-core shape); Core 1 is unused:

- A repeating hardware timer in `main_pico.cpp` calls `Motion::update()` at
  `LOOP_FREQUENCY_HZ`: encoders, forward and rotation PD, motors. The rate is
  per platform, set in `config/board_pico.h` (500 Hz) or `config/board_stm32.h`
  (1 kHz); express any tick count as a time with `LOOP_TICKS(seconds)`.
- The main loop (`cli.loop()`) handles serial and Bluetooth I/O, polls the
  ToFs at 50 Hz, and runs path planning, maze algorithms and high-level decisions.

**DriverLab Mode** is a single-core calibration and characterisation tool:
open-loop voltage sweeps, step response, PID tuning, and export of
`config.h`/`tuning.h`-compatible constants.

DriverLab is standalone. It owns its own time-based profiles
(`DriverLabProfile`), its own `PID` instances, and its trial state machines
(OL, STEP, MOVE, TURN, TURN-OL, TURN-STEP), driven by one `tick()` per 500 Hz
tick from `main_pico.cpp`. It uses `Motor`, `Encoder`, `IMU`, `ToF` and `Battery`
directly — **the HAL drivers are the only seam** between DriverLab and the
Robot/Motion stack. Keep `Motion` and `Drivetrain` out of DriverLab.

Calibration order: `OL` (open-loop voltage sweep, for the motor constants) →
`STEP` (step response, for the motor time constant) → `EXPORT` (emit the
constants). Running them out of order gives numbers the later steps contradict.

Trials are cooperative state machines: a trial records its parameters, sets
motor voltages or arms a profile, and returns. The next tick advances it one
step. That is what holds the loop period at exactly `1 / LOOP_FREQUENCY_HZ`.

## Interrupt context

No Log or heap in ISR context. Code reachable from the control tick
(`Motion::update()` and everything it calls) or another interrupt handler never
calls `LOG_*`, `printf`, or anything that builds a `std::string`, `new`s, or
grows a container. Raise an atomic flag or counter there and report it from the
main loop, as `Motor::reportBatteryFallback()` does. `python3
tools/check_isr_logging.py` enforces this; add a function to its list when the
control path starts calling it.

**Main loop to tick.** Motion's public mutators (`start_move`, `set_position`,
`reset_drive_system`, ...) never write the tick's state: each queues one
command in Motion's latch under `hal::irq::Lock`, and `Motion::update()` applies
the queue, in issue order, before anything else in the tick. Main-loop reads
(`position()`, `move_finished()`, ...) see the live state with the queued
commands applied, so code that reads straight after a command still sees it.
Add a new mutator as a `Command::Op`; `python3 tools/check_motion_latch.py`
fails if a main-loop function in `motion.cpp` writes tick-owned state.

**Crash watchdog.** `control/crash_detector.*` trips on sustained voltage
saturation (either wheel at ≥0.97·min(`MAX_VOLTAGE`, battery) for 100 ms),
stall current at near-zero wheel speed (only where a board supplies a current
reader; the Pico has none) or forward PD error that keeps growing. Thresholds
are times in `config/motion.h` (`CRASH_*`). `Motion::update()` checks it every
tick and, on a trip, cuts the motors itself (`Drivetrain::stop()`, then the
optional cut hook where the STM32 will sleep its drivers) and holds them cut
with the move frozen; the trip latches until `Motion::clear_crash()`. The main
loop (`CommandLineInterface::reportCrash()`) prints the cause, flashes the
cause code in orange on the LED and halts the run. `CRASH ON|OFF|CLEAR|INHIBIT
ON|OFF|STATUS` drives it; `RESET` and re-arming COMP clear the latch. It
starts **off on the Pico** (`CRASH_WATCHDOG_DEFAULT_ENABLED 0` in
`config/board_pico.h`: the race-proven mouse keeps its behaviour unless armed
with `CRASH ON`) and **on for STM32 builds** (`config/board_stm32.h`).

## Units

Carry units in the name whenever the bare quantity is ambiguous.

| Quantity | Unit | Suffix |
|---|---|---|
| distance | mm | `distance_mm` |
| velocity | mm/s | `velocity_mmps` |
| acceleration | mm/s² | `accel_mmps2` |
| angle | deg or rad — state which | `yaw_deg` |
| angular speed | deg/s or rad/s — state which | `omega_degps` |
| time | s or ms — state which | `dt_s`, `period_ms` |
| PWM | normalised duty or raw — state which | — |

Velocities are **mm/s** throughout: feedforward constants are duty per mm/s,
and DriverLab trials and Motion both speak mm, mm/s, mm/s².

## Naming

Google C++ Style, with these specifics:

| Element | Convention | Example |
|---|---|---|
| files | `snake_case.h/.cpp` | `flood_fill.cpp` |
| classes | `PascalCase` | `Drivetrain` |
| functions | `camelCase` | `updateState()` |
| accessors | no prefix | `velocity()`, `yawDeg()` |
| mutators | `set_` prefix | `set_target()` |
| constants | `SCREAMING_SNAKE` | `MAZE_SIZE` |
| members | `snake_case_` | `omega_degps_` |
| locals | `snake_case` | `dt_s` |

Name boolean queries for the thing itself: `wallFront()`, `finished()`,
`calibrated()`. Keep `is` only where it genuinely reads better (`isValid()`).

Action methods take imperative verbs: `reset()`, `stop()`, `moveDistance()`,
`turnAngle()`, `updateState()`.

Use the standard physics names — `omega` for angular velocity, `yaw` for
heading, `dt` for time delta, `v` for linear velocity in local math — and the
explicit suffixed form for anything persistent.

## Comments

Comment the **why**: non-obvious control logic, hardware quirks, timing
assumptions, unit conversions, maze convention assumptions, safety constraints.

```cpp
// Clamp the integral term to prevent windup while the motor command saturates.
```

Code that speaks for itself needs no comment.

## Formatting

After editing any `.cpp` or `.h`, run clang-format:

```bash
clang-format -i <file_path>
```

`.clang-format` at the repo root defines the style. If you skip formatting, say
so explicitly.

## Building

**The user builds and flashes.** Suggest commands; run them only when asked.

Firmware:

```bash
mkdir -p build && cd build
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j4
```

STM32F405 bring-up firmware for the Adafruit Feather STM32F405 Express
(pins and timers in `config/board_feather.h`): 168 MHz clock, 1 kHz TIM7
tick, USART3 console at 1 Mbaud with `stats`, `flood`, `profile`, `arm`,
`logfill` (fill the run log with a test pattern) and `log` (dump it)
commands, TIM3/TIM4 encoders and an LSM6DSR on SPI2. At boot it reads the
chip UID and loads the board profile it saved in the flash store; an unknown
board lists the presets (`config/board_profiles.h`) and refuses to arm until
`profile <n>` saves one. A damaged store shows as 2 LED flashes every 2 s,
no usable record as 4. The run log (`control/run_log.h`: int16-packed
samples every 8 ms, about 13 s) sits in CCM, which DMA cannot reach, so a
dump formats each row into an SRAM buffer before the console queues it.
Separate build directory; needs an Arm GNU
Toolchain with newlib, found through `ARM_TOOLCHAIN_PATH`,
`PICO_TOOLCHAIN_PATH` or `PATH`:

```bash
cmake -S . -B build-stm32 -G Ninja -DJURABABA_PLATFORM=stm32
ninja -C build-stm32
```

The build runs `tools/check_stm32_layout.py`, which fails if anything lands in
flash sectors 1-2 (the flash store) or code starts before sector 3. Flash
`Jurababa_stm32.hex` or the `.elf`, never a raw `.bin`: a `.bin` zero-fills
the flash store.

At boot the STM32 prints why it last reset, decoded from RCC_CSR (power-up,
brownout, reset button, software, independent or window watchdog, low
power), and what it was doing then: `Breadcrumb` keeps the last boot phase or
console command in `.noinit` SRAM, which survives every reset but a power-off.
An independent watchdog of at least 3 s (sized for the fastest 47 kHz LSI;
about 4.4 s at 32 kHz) resets a hung main loop. Only the main loop feeds it,
never the tick, and a flash-store erase feeds it just before its 500 ms stall.

The brownout reset needs BOR level 3 (reset below about 2.7 V), set once per
board in the option bytes. The firmware never writes option bytes; set it
with STM32CubeProgrammer over SWD (or USB DFU with BOOT0 high), then confirm
with `-ob displ`:

```bash
STM32_Programmer_CLI -c port=SWD -ob BOR_LEV=0   # FLASH_OPTCR BOR_LEV 00 = level 3
```

The option bytes survive reflashing and mass erase. At the factory default
(BOR off) only the 1.8 V power-down reset catches a sag, which reads as a
power-up, or a power fault if the breadcrumb survived.

Simulator: `cd sim && make`, producing `sim/bin/simulator`.

ESP32-C3 BLE bridge (the STM32 CLI over a Nordic UART Service; radio off while
armed via ESP32 EN): see [`esp32/README.md`](esp32/README.md). Tests with
`pio test -d esp32 -e native`, builds with `pio run -d esp32 -e esp32c3`.

## Reporting your work

Close every code change with:

```text
Changed:
- file/path.cpp

Verification:
- Formatting run: yes/no
- Build run: yes/no
- Tests run: yes/no
- Remaining risks:
```

State what you ran and what you did not. A claim of "built and tested" means
the command appears in the transcript.

## Debugging

Restate the symptom, name the subsystem, search for the files, and read the code
before proposing a fix. Then list likely causes most-probable first, identify
the smallest experiment or inspection that separates them, and only then patch.

Per-subsystem pre-flight checks — motors, encoders, sensors, control loop and
`dt`, multicore state ownership, maze conventions, navigation assumptions — are
in **[`docs/subsystem-checklists.md`](docs/subsystem-checklists.md)**. Read the
section for the subsystem you are about to change.

## Commits

Imperative mood, present tense: `Fix encoder velocity sign convention`.

```text
<Verb> <what changed>

[optional body explaining WHY]
```

Verbs: `Add`, `Fix`, `Update`, `Refactor`, `Remove`, `Change`, `Document`,
`Tune`.

## Tooling

`.mcp.json` configures **context7** (library/framework docs — Pico SDK, PyQt6)
and **sequential-thinking** (step-by-step reasoning for hard debugging).

The calibration dashboard is `cd tools && python3 driverlab_dashboard.py`.
