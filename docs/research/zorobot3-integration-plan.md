# ZoroBot3 integration plan

Research and plan for bringing ZoroBot3's ideas (OPRobots, GPL-3) into this
firmware for the 2026–27 season. The spec and its tickets live in GitHub
Issues; this document is the reference behind them.

## 1. Summary

- **Ideas we reimplement from ZoroBot3 (no code copied):**
  - the pulsed-IR on/off front end with wall hysteresis and wall-edge (post) resync;
  - two-front-sensor squaring;
  - the crash/saturation watchdog;
  - the brake-then-sleep driver lifecycle;
  - the fan ramp;
  - calibration and maze stored in flash;
  - an explore stop rule that ends the search once the optimal path is proven.
- **Speed-run ideas for this season:**
  - straight merging that brakes to the turn speed, with a speed pass at plan time (for APEC);
  - the diagonal turn set, as a core feature: re-tuned on Mouse 1, enabled after 5/5 clean runs, and used at APEC if it passes by Feb 15;
  - a planner that picks the route by estimated time instead of path length, built now in the sim behind a switch.
  - Cut for this season: the known-corridor sprint.
- **Loop rate:** 1 kHz on the STM32 from the start; the Pico stays at 500 Hz. `LOOP_FREQUENCY_HZ` becomes a per-platform setting.
- **ZoroBot3 code is a bring-up reference only** for the STM32F405 peripherals: TIM PWM, TIM3/TIM4 encoder mode, ADC injected and DMA, SPI3 and the UID. We write all of it ourselves under MIT, from RM0090 and the datasheets.
- **We keep our own architecture:**
  - the UKMARS-shaped Motion (PD with Kv/Ks/Ka feedforward, steering as a rate setpoint);
  - the tri-state maze, A* and snapshot rollback;
  - the COMP stage flow, DriverLab, trace and breadcrumb, and the console.
- **New work is ours alone:**
  - a thin `firmware/hal/` (pico, stm32 and host backends);
  - a versioned, CRC-checked A/B flash store;
  - LSM6DSR pitch-rate and z-accel seam handling;
  - gyro bias capture at arm time;
  - fan volts feedforward;
  - a pre-arm check screen;
  - ESP32-C3 bridge firmware, in `esp32/` in this repo;
  - a two-layer simulator: a headless runner that speaks the mms protocol (maze regression in CI), and a physics sim that runs our real control code against a motor and wheel model.
- **The Pico Jurababa stays race-ready as the APEC fallback (the Backup mouse).** That is the concrete payoff of keeping the Pico build green. A go/no-go gate on Feb 15 decides which mouse races.
- **Done when:** Mouse 1 passes the Feb 15 gate (full 16×16 search run plus a speed run, 3 of 3), and Mouse 2 passes the seam test (20/20) by Apr 15, both on the ported code.

Reference figures used throughout:
- Our current mouse has 358.32 ticks per wheel turn (geometry.h:18-23), about 0.324 mm per tick.
- The new mouse has 7,273 counts per wheel turn. On a 37 mm wheel that is about 62.6 counts/mm. The new wheel diameter is still to be measured, so every count/mm figure below is per 37 mm wheel.

## 2. Feature matrix

"Reimplement" means the issue body holds a prose spec only. ZoroBot3 line refs stay in this research document and never go into issues, commits or code.

| Feature | ZoroBot3 ref | Ours | Decision | Effort | Phase |
|---|---|---|---|---|---|
| Encoder delta history width | encoders.c (int32) | drivetrain.h:113-114 and driver_lab.h:346-347 `int8_t` | widen to int16 (new-hardware blocker only) | S | P0 |
| 16→32-bit encoder extension | encoders.c:142-151 | encoder.cpp (stateless) | reimplement: in an always-on HAL tick ISR, in every mode | S | P1B |
| TIM3/TIM4 x4 encoder mode + ICF filter | setup.c:308-320 (no filter) | PIO quadrature | reimplement, ICxF=0011 | S | P1B |
| Motor IN/IN slow decay | motors.c, setup.c:277-302 | motor.cpp DIR+PWM | reimplement: `MotorBridge` uses a per-channel {timer, ch, pin, AF} table | M | P1B/P2 |
| DRV8874 brake (both inputs high), then sleep after 3 s | control.c:389-397 | motor.cpp:72-75 (PWM 0, which coasts on a DRV8874) | reimplement in the drive layer | S | P2 |
| Battery feedforward V/Vbat | control.c:68-70 | motor.cpp:49-70 | keep ours | — | — |
| ADC1 scan + DMA (VBAT, IPROPI, buttons) | setup.c:194-240 | battery.cpp | reimplement: ADCPRE /4, sized sample time, IPROPI synced to the PWM or RC-filtered | M | P2 |
| Control tick | main.c | main.cpp:587 timer at 500 Hz; DriverLab polled (main.cpp:446-458) | 1 kHz on STM32, 500 Hz on Pico, `LOOP_FREQUENCY_HZ` per platform; DriverLab also driven from the HAL tick | M | P1/P2 |
| Atomic main→ISR commands | control.c:325-361 | motion.cpp mutators plus about 15 `set_position`/reset call sites | one latched command struct consumed at the top of `update()` | M | P1A |
| No Log or heap in the ISR | — | motor.cpp:59,66 `LOG_DEBUG` (ostringstream) on the ISR path | flag/counter reported from the main loop | S | P0 |
| Crash watchdog (V-sat, IPROPI stall, PD error growth) | motors.c; config.h:42-43 | none | reimplement: the ISR cuts PWM and nSLEEP, the app reports | M | P2 |
| Encoder-vs-gyro slip trip | — | — | cut for APEC, P6 stretch | S | P6 |
| LED fault cause code | control.c:378-388 | mouse.cpp | reimplement | S | P2 |
| Pulsed IR front end (TIM2 OC emitters, TIM2-triggered ADC2 injected, one channel per slot) | sensors.c:240-277, setup.c:176-188 | ToF | reimplement, different shape | M | P3 |
| Emitter isolation self-test | sensors.c:147-191 | — | reimplement as a DriverLab test | S | P2 |
| Median-3 + adaptive EMA | sensors.c | Median3Filter | reuse our median | S | P3 |
| counts→mm IR model | sensors.c, notebook | sensors.h | own `tools/ir_fit.py` from public log-model literature | M | P3 |
| Wall hysteresis | sensors.c:609-640 | `TOF_WALL_HYSTERESIS_MM` unused | reimplement | S | P3 |
| IR invalid state | sensors.c:566-607 (stale) | tof_wall_utils.h | keep ours, add INVALID | S | P3 |
| Side centring | sensors.c:772-788 | tof_wall_utils.h, motion.cpp | keep ours, swap the input | S | P3 |
| Front squaring + distance hold | move.c:1568-1617 | mouse.cpp distance only | reimplement as a Motion mode | M | P3 |
| Wall-edge resync | move.c | conf_edge_detection (diagnostic) | reimplement, seam-gated | L | P4 |
| Last-cell turn-side gating | move.c:1670-1671 | front trigger | reimplement with a hard cap | S | P4 |
| Venue IR calibration menu | calibrations.c | compile-time | reimplement (CLI + buttons) | M | P3 |
| LSM6DSR driver (FIFO, gyro+accel) | lsm6dsr.c | BNO085 RVC | own register layer, or ST BSD-3 upstream | S | P1B/P3 |
| Fixed gyro FS 2000 dps | dynamic FS | — | fixed | — | — |
| Gyro bias at arm (before fan spin-up) + ZUPT + scale cal | lsm6dsr.c | none | reimplement | M | P3 |
| Yaw re-zero on square/wall events; `packet_seq` = sample counter | — | motion.h:83-85 heading hold | ours | S | P3 |
| Gyro inner loop, runtime-selectable | control.c:120-122 | `ROT_FEEDBACK_ENCODER 1` | reimplement | M | P3 |
| Seam detector (pitch rate + z-accel) | absent | decided | ours | M | P3 → P5 |
| Flash store: 2-slot A/B, seq number, CRC32, version | eeprom.c | none | own, in 16 KB sectors 1-2 | M | P2 |
| Maze persistence + resume | floodfill.c | RAM snapshot | reimplement; saved only in the parked window | M | P4 |
| Runtime maze size/goals | maze.c | main.cpp:504 `{3,3}..{4,4}` (8x8, apparently deliberate) | runtime, persisted, shown before arming | S | P0/P4 |
| UID → robot profile | config.c (unknown → A) | none | an unknown UID boots to a profile picker | S | P2 |
| Buttons + menu | buttons.c, menu*.c | BOOTSEL only | non-blocking, table-driven | M | P3 |
| Arm gate + quiet window | control.c, main.c | start_gesture.cpp:251-267 prints | gate in `_write`/hal::uart, ESP32 EN off | S | P3 |
| Pre-arm check screen | — | none | ours | S | P4 |
| Fan TIM1 PA8, ramp, volts FF, interlocks | control.c | none | reimplement (Mouse 1) | S | P4 |
| Separate decel in Profile | control.c:84-100 | profile.cpp:77 | reimplement (no soft accel) | S | P4 |
| Speed tiers per mouse | move.c | motion.h:30-46 | numbers from our own DriverLab only | S | P4 |
| Straight merging + plan-time speed pass | move.c:2105-2253 | mouse.cpp:2553-2563 runs `Fn` one cell at a time | reimplement | M | P4 |
| Re-tune ToF-era turn constants on IR | — | motion.h:44-62 | ours | M | P4 |
| Optimal-path-proven explore stop | floodfill.c:990-1089 | mouse.cpp | reimplement, A* MASK_OPEN vs MASK_CLOSED | S | P4 |
| Curvature (sine-ramp) arc + turn generator | move.c, aux.py | trapezoid, smooth_turn.h | P6 stretch, from public literature | M | P6 |
| Diagonal turn set + compiler | move.h, floodfill.c | diagonalizer.cpp | core: port ours, re-tune on Mouse 1, 5/5 gate | L | P4 |
| Time-based route planner | floodfill.c | A* by path length | ours, in the sim behind a switch; firmware only if it wins | M | Sim |
| Run log (int16-packed, CCM, event bits) | macroarray.c | trace_ in Motion: 1500 × 68 B ≈ 102 KB | move out of Motion, ≤ 56 KB CCM | S | P1B/P2 |
| Static A*/path buffers | — | path_utils.cpp:413-417 vectors + priority_queue | ours | S | P2 |
| Headless mms-protocol runner + maze corpus in CI | utils/simulator | sim/ mms only | own, never copied | M | Sim |
| Physics sim (real Motion/Profile/PID/Drivetrain, slip, noise, seam spikes) | — | none | own, about 1 week | M | Sim |
| Cut this season: corridor sprint, MoveExecutor collapse, explore-exit offset, append-log | — | — | cut | — | — |
| Skip: RC5, time trial, drag race, hardcoded sectors, external SRAM, dynamic gyro FS | — | — | skip | — | — |

## 3. Phased roadmap

Each step below is one issue, written as `deps → step: acceptance`.

Phases:
- **P0:** Pico prep and PCB checklist (now to Nov 30).
- **Sim:** simulator and planner work, in parallel from now.
- **P1A:** HAL extraction on the Pico (Dec).
- **P1B:** standalone STM32 bring-up on the Feather (Dec, in parallel with P1A).
- **P2:** rev1 board bring-up (mid-Dec to Jan).
- **P3:** IR and gyro (Dec to Jan 20).
- **P4:** APEC (Jan 20 to Mar 8), with a go/no-go gate on Feb 15.
- **P5:** seam campaign and Mouse 2 (Mar 9 to Apr).
- **P6:** NYIT polish and stretch goals (to about May 7).

Lisul owns the firmware port. The PCB checklist (P0.1) goes to the board team before the Dec 4 order.

### P0: Pico prep (now to Nov 30), Pico build stays green

1. **PCB checklist before the rev1 order (moved up from P3.14).**
   - Fan flyback diode and gate pull-down.
   - nSLEEP and nFAULT routed.
   - PMODE strapped to PWM.
   - R_IPROPI sized for full scale under 3.3 V at stall.
   - NRST on SWD.
   - BOOT0 reachable for USB DFU at the venue.
   - PC13-15 LEDs sinking.
   - A single IMU on the PA15 CS, with INT1 routed if possible.
   - Ladder buttons and LEDs kept.
   - ESP32 boot strapping, plus an STM32 GPIO driving ESP32 EN (radio off while armed).
   - Emitter gate pull-downs, with a series R or RC so DC-on is survivable.
   - Emitters PA0-PA3 kept on TIM2_CH1-4.

   *Acceptance:* signed-off checklist before the order.
2. **Widen the encoder history** to `int16_t` (drivetrain.h:113-114, driver_lab.h:346-347). Add a `static_assert` sized from the new mouse's counts/mm and its top speed. int8 saturates at about 1.0 m/s at 62.6 counts/mm and 500 Hz, or about 2.0 m/s at the STM32's 1 kHz (half that if the wheel gives about 95 counts/mm). Today's mouse is safe to about 20 m/s. *Acceptance:* the Pico build passes. The ±200 counts/tick unit test is added once P1A.2 gives a host target.
3. **No Log in ISRs.**
   - Replace `LOG_DEBUG` in `Motor::set_motor_volts` (motor.cpp:59,66) with an atomic flag that the main loop reports.
   - Add a "no Log or heap in ISR context" rule to CLAUDE.md.

   *Acceptance:* grep finds no `LOG_` reachable from `Motion::update()`.
4. **Runtime maze size and goals.** The `{3,3}..{4,4}` at main.cpp:504 looks like a deliberate practice setting, with the 16x16 line commented out above it. Add a CLI `MAZE <size> <goals>`, print it at Pico boot and keep the default as it is. *Acceptance:* goals print at Pico boot and `MAZE 16 7,7..8,8` takes effect. The sim already hard-codes 7,7 (sim/main.cpp:93), so it isn't the test.
5. **CI.** GitHub Actions builds the Pico firmware and the host sim. *Acceptance:* green on main.
6. **Doc fixes.**
   - CLAUDE.md:80-81 describes a Core0/Core1 split, but main.cpp:587 runs one timer.
   - `docs/ukmars_parity_matrix.md` says "IMU yaw delta", but `motion.h:74` sets `ROT_FEEDBACK_ENCODER 1`.

   *Acceptance:* the docs match the code.
7. **Tools.** Add 1,000,000 baud to `tools/serial_term.py:31`. *Acceptance:* it connects at 1 Mbaud.

### Sim: simulator and planner (now, in parallel)

Our sim (`sim/main.cpp`) is an mms bot: it drives maze and route logic only, with no physics. Nothing here copies ZoroBot3's GPL simulator.

1. **Headless mms-protocol runner.** Implement mms's command protocol ourselves so `sim/main.cpp` runs without the GUI, against a corpus of public maze files (16×16 contest mazes plus small ones). Keep the mms GUI for watching runs. *Acceptance:* CI runs the corpus on every PR and reports search length and speed-run path per maze.
2. **Time-based planner** behind a switch: route cost is estimated run time (turn and straight costs, straight merging) instead of cell count. *Acceptance:* on the corpus, the planner's estimated speed-run time is never worse than the path-length planner's, and the switch picks either.
3. **Physics sim.** Compile Motion, Profile, PID and Drivetrain for the host (needs P1A.1-2) and run them at loop rate against a simple motor and wheel model with knobs for wheel slip, sensor noise and seam pitch spikes. Time-boxed to about 1 week. *Acceptance:* a 10-cell straight and an SS90 run in the sim, and a seam spike exercises the seam response (P5.2) before Mouse 2 exists.

### P1A: HAL extraction on the Pico (Dec)

1. **Make the driver headers SDK-free.** Use pimpl, or platform members behind `#if`, in `drivers/{motor,encoder,imu,line_sensor}.h` and `driver_lab/driver_lab.h`. Constructors take a board config struct or HAL handles instead of `PIO`, pins or ADC pin numbers. The driver APIs used by Motion stay the same, but construction changes: give each platform its own `main_pico.cpp` / `main_stm32.cpp`. *Acceptance:* no `pico/` or `hardware/` include in `control/`, `maze/`, `navigation/`, `common/` or the driver headers. A Pico mms sim run and a DriverLab OL trace are unchanged.
2. **Host test target** (doctest in CMake) that compiles Motion, Profile, PID and Drivetrain against `hal/host`. Depends on 1. *Acceptance:* the int16 history test (±200 counts/tick) and the Profile tests run in CI.
3. **Console HAL.** `hal::console::getcNonBlocking()` plus a TX path. Switch the CLI (cli.cpp:501, 902, 2019, 2367, 2619) and DriverLab (driver_lab.cpp:124-128, 1658-1663) away from `getchar_timeout_us`/`uart0`. *Acceptance:* the Pico CLI and DriverLab are unchanged over the HC-05.
4. **Latched Motion commands.**
   - Route every mutator through one command struct consumed at the top of `update()`: `start_move/turn`, `stop_at/after`, `reset_drive_system`, `reset_drive_control`, `end_motion_sequence`, `set_position`, and the controller reset in `wait_until_forward_settled`.
   - The lock is `hal::irq::Lock`, which masks with BASEPRI at the control priority and never masks the IR sampler.

   *Acceptance:* a code review or static rule that no ISR-owned member is written outside the latch, plus a Pico search run unchanged.
5. **Board-specific `app/` files** (bluetooth, bootsel_button, leds, start_gesture, firmware_mouse) become per-platform. `cli.cpp` and `mouse.cpp` lose their direct SDK timing calls. *Acceptance:* the Pico binary behaves identically.

### P1B: STM32 bring-up on the Feather F405 (Dec, standalone, not gated on P1A)

This is a small `hal/stm32` bring-up binary with its own main.
- The Feather pinout is unverified here. Check Adafruit's pinout before wiring. If PC8-PC12 go to the SD slot, as one critique recalls, TIM8 4-channel won't fit.
- All timer and pin assignments come from a per-channel `{timer, ch, pin, AF}` table in `board_feather.h`.
- Wire a GPIO abort button and a motor-power kill switch.

1. **CMake STM32 branch.** Choose the platform before `pico_sdk_import.cmake` and `pico_sdk_init()` (CMakeLists.txt:24-29), and keep the VS Code "DO NOT EDIT" block. Also:
   - a separate build directory and toolchain file per platform;
   - libopencm3 pinned, built through ExternalProject plus genlink;
   - our own linker script:
     - sector 0 holds the vectors, sectors 1-2 (16 KB each) hold the flash store, `.text` starts at sector 3;
     - `.data`, `.bss`, `.dma`, heap and stack live in SRAM, and only marked symbols go in `.ccm`;
     - add `.noinit`, and a bounded `_sbrk`;
   - use one vector-table approach (libopencm3 weak aliases plus `extern "C"` handlers);
   - flags `-fno-exceptions -fno-rtti -fno-threadsafe-statics -Wdouble-promotion -fsingle-precision-constant` (STM32 only), and link `-u _printf_float`.

   *Acceptance:* blinky builds in CI and flashes. Heap and stack high-water marks print. The iostream/sstream flash cost is measured.
2. **Clock.** Start HSE ourselves with a timeout, because `rcc_clock_setup_pll()` waits forever for HSERDY. Fall back to HSI with an error blink. Confirm the pinned libopencm3 has the `rcc_hse_12mhz_3v3` 168 MHz entry. *Acceptance:* 168 MHz measured, APB1/APB2 at 42/84 MHz.
3. **Time base and tick.** TIM5 at 1 MHz extended to 64 bits. A TIM7 `PeriodicTimer` that always runs, in every mode, and also does the encoder counter extension. DBGMCU freeze. *Acceptance:* 1 kHz on a scope with under 1 µs jitter; DWT max cycles reported, with the control ISR under 30% of the period.
4. **Console.** USART with a DMA TX ring (in SRAM) that drops and counts when full, never blocks; RX on the IDLE IRQ; retargeted `_write`; a TX gate. 1 Mbaud gives an exact BRR at 42 MHz. *Acceptance:* a CLI echo over USB-UART at 1 Mbaud, and the drop counter works under a flood.
5. **Encoders.** `QuadCounter` on two timers with ICF. Use an AS5047P breakout or any quadrature source. *Acceptance:* 10 hand turns read the expected count, with no aliasing.
6. **LSM6DSR on a breakout** (SPI) to de-risk P3.6 early. *Acceptance:* WHO_AM_I 0x6B and a FIFO read each tick.
7. **Optional:** motor test on DRV8874 breakouts if they are bought. DriverLab characterisation does **not** happen on the Feather, because the motors, wheels and mass differ.

### P2: rev1 board bring-up (mid-Dec to Jan)

1. **`config/board_stm32.h`** from our schematic. *Acceptance:* continuity/GPIO test on every net.
2. **Power-on order.** Emitters off (TIM2 stopped, PA0-PA3 low), nSLEEP low, PWM 0, fan off, ESP32 EN as the profile says, then peripherals. *Acceptance:* the scope shows no motor, fan or emitter pulse at power-up.
3. **STM32 DriverLab port** on the real pins. `DriverLab::tick()` runs from the TIM7 tick, or its polled loop logs its own period overruns. Depends on P1A, P1B. *Acceptance:* DriverLab OL/STEP run on the board.
4. **DriverLab characterisation and averager length.**
   - Measure Kv/Ks/Ka, `MM_PER_TICK` (Q push from `pi*D/7273`) and counts per turn.
   - Choose `ENCODER_AVERAGER_LENGTH` once, at the 1 kHz loop rate. At about 62 counts/mm the 8-tap quantisation reason mostly goes away.

   *Acceptance:* constants committed, and a 500 mm MOVE ends within 2 mm.
5. **Motor lifecycle.**
   - `MotorBridge::brake()` drives both inputs high.
   - The brake-then-sleep timer runs in the drive layer.
   - Wake on arm or countdown, allowing about 1 ms for the DRV8874 wake.

   *Acceptance:* brake holds, the motors sleep when disarmed, and the first command isn't lost.
6. **IPROPI on PA5/PA6.**
   - Check in the DRV8874 datasheet whether IPROPI is chopped by the PWM.
   - If it is, trigger ADC1 from the TIM8 TRGO or CC, or RC-filter it.
   - Calibrate the zero offset with the motors asleep.

   *Acceptance:* a plausible stall current.
7. **Emitter isolation self-test** (4x4 crosstalk matrix). *Acceptance:* diagonal-dominant.
8. **Flash store.**
   - 2-slot A/B in sectors 1-2, each record `{magic, ver, seq, len, uid_hash, payload, crc32}`.
   - Erase with PSIZE x32 explicitly; VDD ≥ 2.7 V (BOR level 3).
   - Save only when disarmed. The precondition forces motors asleep, the fan PWM to 0, the TIM2 IR timer stopped and PA0-PA3 low.

   *Acceptance:* survives 20 power yanks during a save, and a bad CRC falls back to the other slot or to defaults, with an LED code.
9. **UID profile.** Profile = has_fan, divider, IR set, name. An unknown UID boots into a menu that picks and saves a profile, and the board refuses to arm only until a profile is saved. *Acceptance:* both boards identify themselves at boot.
10. **Breadcrumb port.** RCC_CSR flags plus `.noinit`, BOR level 3, and an IWDG timeout of at least 2 s (well above a 16 KB sector erase). *Acceptance:* a brownout is reported on the next boot.
11. **Crash watchdog v1.**
    - Trip on V-sat ≥ 0.97·min(MAX_V, Vbat) for 50 ticks, on an IPROPI stall at near-zero speed, or on runaway forward-PD `m_error_` growth.
    - The ISR cuts PWM and nSLEEP itself. The app loop only reports and sets the LED code.
    - Provide an inhibit flag.

    *Acceptance:* a pinned wheel stops within 150 ms, and a hard accel doesn't trip it.
12. **Run log.**
    - Move the trace out of `Motion` into a buffer placed in CCM, ≤ about 56 KB: int16-packed with event bits, 8 ms decimation, about 1700 samples ≈ 13.6 s.
    - Dump after disarm through an SRAM bounce buffer, since DMA can't reach CCM.
    - Make the A*/path buffers static.

    *Acceptance:* the full STM32 image links with stack/heap margin, and 13 s of history dumps.
13. **ESP32-C3 bridge firmware.**
    - A UART-to-BLE/Wi-Fi bridge at 1 Mbaud; update `tools/hc05_bridge` and the host tooling.
    - While armed the STM32 ignores RX, except a defined abort if the rules allow one.
    - Check for RF and ADC noise on the IR receivers.

    *Acceptance:* the CLI and venue calibration work wirelessly.

### P3: IR and gyro (Dec to Jan 20)

1. **IR front end.**
   - TIM2 output-compare drives the emitter pulses on PA0-PA3 and triggers ADC2 injected on TIM2_CC/TRGO, so the delay from emitter-on to sample is set in hardware.
   - Convert one channel per slot (JL=0, JSQR rewritten each slot). The JEOC ISR only reads JDR1 and preloads the next slot.
   - The IR timer sits at top priority and is never masked.
   - Set the ADC2 SMPR from our receiver's R_load (ZoroBot3's 15 cycles is too short for a high-ohm load), or add a buffer.

   *Acceptance:* 1 kHz per sensor; the scope shows settling before the sample; full 12-bit settling at the chosen SMPR; ISR under 5 µs.
2. **IR filtering.** Median-3, a validity flag (off level near full scale, or on−off at the noise floor), and EMA on the steering signal only. *Acceptance:* the sensor reads INVALID under a torch, never a stale value.
3. **IR model.** `IR SAMPLE` command, an encoder-driven sweep, and `tools/ir_fit.py` written from public log-model fitting references, cited in the commit. *Acceptance:* RMSE under 5 mm, including the 80–90 mm band.
4. **Wall hysteresis**, and rename `tof_wall` to `wall_sensing`. *Acceptance:* no flicker at posts.
5. **Steering on IR.** *Acceptance:* a 5-cell straight at 500 mm/s stays within ±3 mm of centre.
6. **LSM6DSR driver.**
   - Settings: SPI3, bounded reset, FS 2000 dps fixed, accel enabled.
   - Each tick, read the FIFO, or a 12-byte gyro+accel burst on DRDY.
   - Integrate yaw over every sample, or set LPF1 under 250 Hz if only one sample is used.
   - `packet_seq()` becomes the sample counter (DriverLab TURN-STEP, driver_lab.cpp:1478).

   *Acceptance:* CCW+ sign, and boot reports gyro health.
7. **Bias and scale.** Capture bias at arm, before any fan spin-up. Add ZUPT and a 5×360° CW+CCW scale trial, stored in flash. *Acceptance:* under 2° drift over 60 s still (and with the fan on, for Mouse 1), and under 1° error on 10 turns.
8. **Yaw semantics.** The LSM6DSR yaw is integrated rate, unlike the BNO085's fused yaw. Re-zero heading on squaring and wall events before heading hold (motion.h:83-85) uses it. *Acceptance:* heading hold doesn't steer on drift over a 5-minute search.
9. **Rotation source switch** (runtime gyro or encoder), then re-fit ROT_KM/TM. *Acceptance:* gyro mode is at least as accurate in A/B spins.
10. **Seam detector v0.** Pitch rate and z-accel spikes from the FIFO burst, logged per cell, with no reaction yet. Buy a taped-plywood test floor now. *Acceptance:* the noise floor (fan on and fan off) and the spike amplitudes are recorded.
11. **Buttons and menu.** Non-blocking and table-driven; `abortRequested()` ORs in a button. *Acceptance:* arm, abort and resume work with the ESP32 off.
12. **Venue calibration.** CAL SIDE/FRONT/MID/GYRO/AMBIENT with old→new deltas, CAL SAVE/DISCARD and bounds. Depends on 2-4, 7, 11 and P2.8. *Acceptance:* a full calibration in under 2 minutes survives a power cycle.
13. **Quiet window.**
    - The gated window runs from the start of the gesture wait (`comp_armed_` = true, cli.cpp:923) to the end of the stage run. The run itself prints, and `comp_armed_` is false during it (cli.cpp:935).
    - The gate lives in `_write`/hal::uart, so printf, `printFormat`, `writeDiagnostic` and Log are all covered. It drops output, never blocks.
    - Remove the 500 ms report at start_gesture.cpp:251-267, and add about 300 ms dwell and a 1 s hold to the gesture.
    - ESP32 EN goes low for the window.

    *Acceptance:* zero UART bytes in the window on a logic analyser, and RF silence on a sniffer.
14. **Front squaring Motion mode** with an honest settle test and a timeout. *Acceptance:* yaw within ±1° after a stop.

### P4: APEC, Mouse 1 with fan (Jan 20 to Mar 8)

1. **Fan.** TIM1 PA8, a slew ramp, `duty = V_fan/Vbat` every tick, and forced off on fault, HALT, disarm or flash save. Spin-up happens after bias capture. *Acceptance:* the fan is 0 after every abort path, and suction is steady as the battery sags.
2. **DriverLab with the fan on**, with a gain struct per mouse and fan state. *Acceptance:* constants committed per profile.
3. **Separate decel in Profile**; `brakingDistance()` uses it. *Acceptance:* host tests cover the Braking transition.
4. **Speed tiers.** Per-mouse array, numbers from our own DriverLab only, persisted. *Acceptance:* the tier survives a reboot, and "armed" is never persisted.
5. **Re-tune ToF-era constants on IR**: `SMOOTH_TURN_FRONT_TRIGGER_MM` 125, entry 76, exit 61, angles 95/95.5, and the encoder holdover (motion.h:44-62). Use PATH ruler runs. *Acceptance:* SS90 lands within ±1° and ±3 mm at search speed.
6. **Wall-edge resync.** Motion emits edge events, and the run code applies each once per cell. Per-side offsets come from an extended `conf_edge_detection`, stored in flash. Events are suppressed in the seam window. *Acceptance:* error after a 10-cell straight is under 5 mm with resync, against more than 15 mm without.
7. **Last-cell turn-side gating**, capped at nominal + 20 mm, with an LED/log when the cap fires. *Acceptance:* no wall hits with the edge masked.
8. **Optimal-path-proven explore stop** (A* MASK_OPEN vs MASK_CLOSED, with leg and time caps). Leave `navigation/flood_fill.cpp` alone: it is built in CMakeLists.txt:58 and sim/Makefile. *Acceptance:* on the sim corpus the speed path equals the true optimum on at least 90% of mazes.
9. **Diagonals.** Port our diagonalizer to the STM32 and re-tune the diagonal turns on Mouse 1 with our own DriverLab numbers. The compiler rejects unknown patterns loudly; add diagonal post steering. *Acceptance:* 5/5 clean speed runs with diagonals on a test maze, then diagonals are enabled; used at APEC only if this passes by Feb 15.
10. **Maze persistence.** Save the trusted snapshot only in the parked window after stage 2 (motors asleep, fan and emitters off, cli.cpp:965-983), or after a gesture-wait disarm. Never between stages 1 and 2, which run straight through. Boot offers resume, tagged with size and goals. *Acceptance:* a power cycle after stage 2 still completes stage 3.
11. **Straight merging and speed pass.** One Profile move per straight with `final_speed` = the next turn's entry speed. A forward/backward speed pass at plan time. *Acceptance:* a cardinal smooth run beats the per-cell `Fn` executor.
12. **Pre-arm screen and run book.**
    - Before arming, show maze size and goals, profile, tier, battery (voltage and mAh used against a per-mouse budget), calibration age and gyro health, and refuse to arm on any failure.
    - The run book:
      - stage 1 at the validated search tier;
      - save after each permitted window;
      - a faster tier only after a clean speed run;
      - a fixed rule for when to bank a time.

    *Acceptance:* shown on the CLI and LEDs, and a refusal is tested.
13. **Feb 15 go/no-go gate.** STM32 Mouse 1 must finish a full 16x16 search plus a speed run 3 times out of 3, or the Pico Jurababa races APEC. Keep the Pico mouse race-ready on a dated checklist. Race on rev1 if it works; rev2 is for blockers only. *Acceptance:* gate decision recorded.
14. **APEC dress rehearsal.** Venue calibration, three COMP stages, a forced brownout and a forced crash. *Acceptance:* passes twice in a row.

### P5: Seam campaign and Mouse 2 (Mar 9 to Apr)

1. **Mouse 2 build.** Second board, UID profile, DriverLab without the fan (start in Feb if rev2 boards arrive). *Acceptance:* Mouse 2 passes P2.4 and P3.5 acceptance.
2. **Seam response.** Timed inhibits:
   - wall_inhibit (steering 0, derivative reset);
   - encoder_distrust (forward measured = commanded, rotation gyro-only);
   - an IR filter freeze with the median re-primed afterwards.

   Depends on P3.9-10. *Acceptance:* no steering kick and no false wall crossing a taped seam.
3. **Map gating.** `update_map()` skips or marks the cell untrusted inside the window. *Acceptance:* no wrong wall over 50 crossings.
4. **Re-centre after a seam.** Use front squaring if a wall is ahead, else the next post edge. In search, a dead-end back-to-wall re-zero with the crash inhibit. *Acceptance:* heading error is back under 1° within one cell.
5. **Thresholds on NYIT-style plywood**, per profile. *Acceptance:* no false triggers on a flat floor over 10 minutes.
6. **Seam test.** 20 crossings on Mouse 2: 10 at search speed and 10 at speed-run speed, straight and mid-turn, plus a lifted-tape strip. Each must end centred in the next cell. *Acceptance:* 20/20 by Apr 15.

### P6: NYIT polish and stretch (to about May 7)

1. **Diagonal tuning on Mouse 2.** Re-tune the P4.9 diagonals without the fan, with turn tables regenerated by our own tool from public sine-ramp slalom sources if needed. *Acceptance:* 5/5 on a test maze, otherwise stage 5 stays off.
2. **Curvature profile (stretch).** ω = v_cmd·κ(s_cmd), with a host test for the integrated angle. *Acceptance:* ±1°/±3 mm at three entry speeds.
3. **Risk score per path** to choose stage 3 or 5 at the venue. *Acceptance:* shown on the pre-arm screen.
4. **Encoder-vs-gyro slip trip (stretch).** *Acceptance:* staged slips are classified.

## 4. HAL design

```
firmware/
  hal/            cross-platform interfaces: time.h irq.h gpio.h motor_bridge.h quad_counter.h
                  adc.h uart.h console.h system.h leds.h
  hal/pico/       wraps today's SDK code (PIO quad, DIR+PWM, adc_read, repeating_timer, WS2812, BOOTSEL)
  hal/stm32/      libopencm3: TIM2 (IR), TIM3/4, TIM8, TIM1, ADC1-DMA, ADC2-inj, SPI3, USART3-DMA, TIM5/TIM7, UID
                  STM32-only modules (no Pico implementation): flash_store, spi, buttons, fan, ir_frontend
  hal/host/       fake time, in-memory flash, motor and wheel model (physics sim)
  main_pico.cpp | main_stm32.cpp
  config/board_pico.h (pins.h unchanged) | board_stm32.h | board_feather.h   (pin changes need approval)
```

- **Layering.**
  - app → control/navigation → drivers (`Motor`, `Encoder`, `Battery`, `IMU`, `WallSensors`) → hal → SDK.
  - The driver APIs used by Motion keep their shape, but the constructors change to take board config or HAL handles. DriverLab's console use changes (P1A.3), and each platform has its own main.
  - Navigation and maze stay free of hal and control.
- **Interfaces.**
  - `hal::time`: `nowUs()` (64-bit), `nowMs()`, `sleepMs()`, `PeriodicTimer(hz, prio, fn)`.
  - `hal::irq::Lock`: RAII that raises BASEPRI to the control priority, never PRIMASK, so the IR sampler is never delayed.
  - `MotorBridge`: `drive(duty)`, `brake()`, `sleep(bool)`, `fault()`, configured from a per-channel {timer, ch, pin, AF} table.
  - `QuadCounter::raw16()`, extended in the always-on tick.
  - `console`: `getcNonBlocking()`, `write()`, `setTxGate()`. When full it drops and counts, never blocks.
  - `system`: `uid()`, `resetCause()`, `noinit`, watchdog.
- **Memory (STM32).**
  - DMA buffers (UART rings, ADC1 array, SPI buffers), the stack and the heap stay in SRAM.
  - Only explicitly marked symbols (the run log) go in `.ccm`.
  - The heap is bounded, and high-water marks are printed.
- **Wall sensing.** `WallSensorFrontEnd` produces mm plus validity: IR on the STM32, ToF on the Pico.
- **Pico build stays green.**
  - `JURABABA_PLATFORM=pico` is the default.
  - The STM32 build excludes ToF, line sensor, PIO and BOOTSEL.
  - CI builds pico, stm32 and host, and runs host tests and the maze regression.
  - Every HAL refactor commit must leave a Pico mms run and a DriverLab OL unchanged.
  - `SIMULATOR_BUILD`/`run_on_simulator` (147 uses) stay as they are this season.
- **Loop rate.** 1 kHz on the STM32 and 500 Hz on the Pico (no FPU). `LOOP_FREQUENCY_HZ` (config/motion.h:14) moves to the per-platform board config, and anything pre-multiplied by it (PID Kd) is re-derived per platform.

## 5. Licence hygiene (GPL-3 → MIT)

1. **No copying.** No ZoroBot3 source, tables, notebooks, datasets or fitted constants (IR a/b/c, turn_params, weights, .map mazes) enter our repo, not even as comments. Every number comes from our own DriverLab runs or measurements: speed tiers, crash thresholds and turn parameters included.
2. **Write from primary sources:** RM0090, the datasheets and public literature. Each issue body is a prose spec with no ZoroBot3 file:line refs, and the reference is closed before coding. The decision column says "reimplement", never "adopt".
3. **What may be restated:** pin facts from our schematic, register addresses, the UID base 0x1FFF7A10, textbook geometry (84 mm, 174 mm), and generic algorithms written in our own structure.
4. **Deliberate re-shapes:**
   - the IR sampler (TIM2-driven, one channel per slot);
   - the path compiler (a derived transition table with unit tests);
   - the RC5 decoder is skipped.
5. **The PCB is a GPL-3 derivative.** `zb3/LICENSE` covers `pcb_files/` (kicad_project, gerbers, bom) and `3d_model/`. Our modified KiCad files are GPL-3 if distributed. Keep them out of the MIT firmware repo, in a separate GPL-3 repo, or redraw from datasheets.
6. **Third-party code.**
   - ST lsm6dsr_reg is BSD-3; fetch it from ST, never from ZoroBot3's lib/.
   - libopencm3 is LGPL-3, so static linking carries relink obligations if binaries are ever distributed. Flag this to the team; internal use is fine.
   - ZoroBot3's docs have no separate licence, so treat them as GPL-3. Paraphrasing ideas is fine; translating paragraphs is not.
7. **Commit trail:** cite "idea from ZoroBot3 (GPL-3), reimplemented" plus the public source used, and never paste snippets.
8. **Hard to reimplement cleanly:** the turn generator (`aux.py`) and the IR fit (notebook). Derive both from public sine-ramp slalom and log-model fitting sources, and cite them. Their outputs are wrong for our robots anyway.

## 6. Top risks and ZoroBot3 gotchas

**Top risks**

1. **The PCB order is now.** Anything missed on the P0.1 checklist becomes permanent for rev1: ESP32 EN, BOOT0, emitter pull-downs, fan flyback, nSLEEP/nFAULT, R_IPROPI.
2. **Schedule, with one owner.** The STM32 port and the IR work compete with APEC tuning. Mitigations: the Feb 15 gate, the Pico fallback, P5 after Mar 9, and the cuts in §2.
3. **RAM.** The current trace (about 102 KB) plus vector heap use breaks the first full STM32 link. P2.12 fixes it, and P1B.1 reports the margins.
4. **Flash erase stalls every ISR.** Mitigations: 16 KB sectors, saves only with motors, fan and emitters forced off, and IWDG ≥ 2 s.
5. **Encoder overflow on the new hardware** above about 0.67–1.0 m/s with int8 history. Fixed in P0.2.
6. **Seam false positives and negatives** on Mouse 2. The thresholds are unmeasured, so P3.10 collects data early.
7. **Venue IR ambient** (#42). The validity flag and the ambient calibration page are mandatory.
8. **Loop-rate or averager changes invalidate the PD tuning.** The STM32 starts at 1 kHz, and the averager is chosen once, in P2.4.
9. **Radio activity while armed.** Gating TX isn't enough, so ESP32 EN must be routed.

**ZoroBot3 gotchas not to copy**

- `rcc_hse_8mhz_3v3` (setup.c:20) is wrong for a 12 MHz crystal.
- ADCPRE is left at /2, giving 42 MHz (setup.c:199), over the 36 MHz maximum.
- ADC2: `JSWSTART` is passed as the trigger, so JEXTSEL=0 makes TIM1_CC4 the trigger (setup.c:181). All 4 injected ranks are scanned per emitter edge, so each sits at a different delay. 15-cycle sample time.
- NVIC: IRQs are enabled without handlers (setup.c:64-68). This is latent as configured: the USART3 RXNEIE line is commented out, and ADC1 DMA uses stream 4 while NVIC enables stream 0. A configuration change would hang in `blocking_handler`. EXTI at priority 0 erases flash inside the ISR (rc5.c).
- `abs(dist <= 2)` settle test (move.c:1600-1601). Unit mixing in the front PID. Explore diagonal kp/ki swapped.
- Disabled correction terms keep stale outputs. No zero guard on V/Vbat. The fan battery normalisation is computed once, and 3S/2S can be misclassified.
- Arc ω indexed by measured distance with a fixed ω_max. The docs say "ms" where they mean mm.
- Uncapped last-cell loop. One-directional speed downgrade. The same accel for speed-up and braking.
- `side_sensors_calibration(true)` overwrites offsets at every start. Raw thresholds are disabled with `&& false`. When on ≤ off, the old distance is kept.
- Gyro bias stored as int16×1000. No WHO_AM_I check. Unbounded reset poll. SPI race on FS reload.
- EEPROM: a single slot in sector 11, erased with X16; data applied even with a bad checksum; no version.
- Queue drops pushes past 2048. Sign bug in the weight table. Cell 0 used as a sentinel. macroarray wrap bug.
- Unknown UID becomes robot A. `reset_maze_on_start_explore=true` by default.
- The docs' pin tables are wrong in many places. Trust our schematic.

## 7. What we deliberately do NOT take

| Item | Why |
|---|---|
| Flat bitfield maze, add-only walls | Our tri-state map and snapshot rollback are more expressive, and wall-encoding changes need approval. |
| Insertion-sort flood queue, greedy descent | Lossy. We keep A* and make its buffers static (P2.12). |
| Parallel sensor PIDs summed into volts | Our cascade and Kv/Ks/Ka feedforward are better. |
| Dynamic gyro full scale | Extra states and a bus race. Fixed 2000 dps is enough. |
| 16 kHz SysTick doing everything | Couples every rate. We use separate timers: TIM2 IR at top priority, then TIM7 control. |
| Blocking menus and printf, the failsafe-bypass debug mode | They would starve our CLI. DriverLab is the safer bench mode. |
| Auto-run two-hand chord | Easy to trigger by accident. |
| RC5, time trial, drag race, hardcoded sectors, RC turns | Not needed for APEC or NYIT. |
| Encoder EMA speed | We keep the moving average and choose its length once (P2.4). |
| ln lookup table | The FPU makes `logf` cheap. |
| External SPI SRAM logger | CCM is enough at 8 ms decimation. |
| ZoroBot3 turn tables, IR constants, weights, .map mazes | GPL data, and wrong for our robots. Mazes come from public collections. |
| Corridor sprint, append-log, MoveExecutor collapse | Cut for one owner this season. None has a proven score gain before NYIT. |

## Review notes

**Accepted after checking the code**
- **int8 history.** Confirmed at drivetrain.h:113-114 and driver_lab.h:346-347. The current mouse has 358.32 ticks per turn (geometry.h:18-23), so the fix is a new-hardware blocker and not a bug today.
- **Goals and sim.** main.cpp:504 uses `{3,3}..{4,4}`, with the 7,7 line commented out above it. sim/main.cpp:93 hard-codes 7,7.
- **flood_fill.cpp is live.** It is listed in CMakeLists.txt:58 and sim/Makefile:12.
- **PID has no integral term.** pid.cpp: `m_error_` is the accumulated position error, so the P0.3 anti-windup is dropped.
- **SDK leaks.** Driver and DriverLab headers include SDK headers. motion.cpp:10 includes `pico/stdlib.h`. sim/Makefile doesn't build control/. Seven app/ files include the SDK.
- **DriverLab is polled.** It runs from `sleep_until` in main.cpp:446-458.
- **Log on the ISR path.** `LOG_DEBUG` is in `set_motor_volts`, and log.h builds an `ostringstream`.
- **Motor::stop writes PWM 0** (motor.cpp:72-75).
- **Trace size.** TRACE_CAPACITY 1500 of a 16-float struct (motion.h:196-216, 290).
- **Vector heap use** in path_utils.cpp:413-417.
- **CLI reads USB stdio.** `getchar_timeout_us` at cli.cpp:501, 902, 2019, 2367, 2619.
- **`comp_armed_` covers only the gesture wait** (cli.cpp:923/935), and `sleep_ms(1000)` sits between stages 1 and 2.
- **No 1 Mbaud** in serial_term.py:31.
- **Parity matrix vs code.** The matrix says "IMU yaw delta"; `ROT_FEEDBACK_ENCODER 1` is set.
- **Turn constants are ToF-tuned** (motion.h:44-62).
- **Smooth `Fn` runs one cell at a time** (mouse.cpp:2553-2563).
- **ZoroBot3 ADC2.** `JSWSTART` trigger, 15 cycles, and a scan of all ranks (setup.c:176-188). ADCPRE is commented out (setup.c:199).
- **ZoroBot3 EEPROM** is erased with X16. LICENSE is GPL-3, and `pcb_files/` exists.

**Accepted on datasheet or reference-manual grounds, verify during bring-up**
- BASEPRI instead of PRIMASK.
- TIM2 for IR, with PA0-PA3 as TIM2_CH1-4.
- libopencm3's HSERDY busy-wait.
- CCM is not reachable by DMA.
- DRV8874 coasts at 00 and needs about 1 ms to wake; whether IPROPI is chopped is a datasheet check (P2.6).
- LSM6DSR ODR vs tick aliasing.
- Emitter DC hazard during a halt.
- ESP32 radio activity regardless of UART.

**Merged or partially rejected**
- **Overflow speed.** The critiques give 666 mm/s (assuming about 95 counts/mm) and 1.01 m/s (a 37 mm wheel). Both are kept as a range, and the `static_assert` uses the measured wheel.
- **Clamping `m_error_` as a fix (critique 2, A4).** Rejected. Clamping silently shortens moves. Runaway `m_error_` becomes a watchdog trip input instead.
- **HAL scope.** Critique 1 asks for a wider P0.5 and critique 2 for a smaller P0. Merged: P0 keeps only the cheap items, the HAL extraction moves to P1A with the wider SDK-free acceptance, and STM32 bring-up (P1B) isn't gated on it.
- **Feather.** Critique 1 asks to add an IMU and DriverLab to the Feather; critique 2 asks to drop DriverLab there. Merged: no DriverLab characterisation on the Feather, but an LSM6DSR breakout to de-risk the driver.
- **Feather pinout.** The claim that PC8-PC12 go to the SD slot isn't verified here. The per-channel pin table makes the plan work either way, and the pinout must be checked before wiring.
- **Flash layout.** The append-log in sectors 10-11 is replaced by 2-slot A/B in 16 KB sectors 1-2. This answers both the IWDG concern and the request to cut the append-log.