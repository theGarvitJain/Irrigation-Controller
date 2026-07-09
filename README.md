# Irrigation Controller

Bare-metal STM32 firmware for an automated, multi-zone irrigation controller. Built for ECE-298
(Embedded Systems, F2024) as a "Water Reservoir System" project: a single pump and reservoir feed three
irrigation zones at different elevations, on an operator-configured schedule, while minimizing electrical
running cost.

## Requirements

- One reservoir, one pump, one servo-driven valve/distributor routing pump output to either the INLET
  (refill) line or Zone 1/2/3. Each zone sits at a different elevation, so each needs a different pump
  speed (head/pressure) to reach it.
- The reservoir must be filled before any zone irrigation is allowed to start.
- Each pipeline (INLET + 3 zones) runs for an operator-configured block of hours out of a 24-hour cycle,
  with no overlapping schedules.
- Reservoir depth, motor RPM, and the active pipeline are tracked and reported at least once per
  wall-clock hour.
- The pump must shut down immediately and raise an alarm if the reservoir runs dry.
- Since nobody has 24 real hours to demo this, the system runs on **scaled time**: a full 24-hour cycle is
  compressed to 4.8 minutes (~300x), so one "scaled hour" takes 12 seconds of real time.

## Hardware

| Role | Device |
|---|---|
| MCU | STM32F401RE (Nucleo-64) |
| Pump motor | Brushed DC motor, PWM speed control via a motor driver |
| Valve routing | Servo motor, positions the distributor between INLET/Zone1/Zone2/Zone3 |
| Motor speed feedback | Optical slotted encoder → RPM via EXTI edge counting |
| Reservoir depth | HC-SR04-style ultrasonic sensor (trigger pulse + timer input-capture echo) |
| Manual override | Potentiometer on an ADC channel |
| Status indicators | RGB LED (pipeline colour), dual seven-segment display (depth %), Nucleo LD2 |
| Operator I/O | USART2 to a terminal (setup entry + live status log), B1 button (mode switch) |

## Firmware architecture

The whole application lives in `main.c`, generated from STM32CubeMX with the application logic added in
the `USER CODE` regions. It's a two-mode state machine:

### Setup Mode (`user_input()`)
Runs immediately after reset, before any peripherals that touch the motor/servo are armed. The pump motor
is guaranteed off. Over UART, the operator enters, for each of the 4 pipelines (INLET, Zone 1, 2, 3):

- **Pipeline choice** — which physical connection (0 = INLET, 1–3 = zones).
- **Pump PWM setting** — 0 = manual (potentiometer-controlled), or a fixed 70% / 85% / 99% duty cycle.
- **Start/stop hour** (00–23, scaled wall-clock) — when that pipeline is active.

Input is collected byte-by-byte via UART RX interrupts (`HAL_UART_RxCpltCallback`) with local echo, then
parsed with `atoi`. Once all entries are in, the config is echoed back over UART and the system idles,
blinking LD2, until the B1 button is pressed to enter Run Mode.

### Run Mode (main loop)
Driven by TIM5, which fires a periodic interrupt every ~198.5 ms; 60 of those ticks are counted as one
scaled hour (`HAL_TIM_PeriodElapsedCallback`), giving the 12-seconds-per-hour scaling described above.
Each iteration of the loop:

1. `get_active_pipeline()` checks the schedule table for a pipeline whose start/stop window contains the
   current scaled hour.
2. If one is active: `servo_num()` points the distributor at it, `set_led()` drives the RGB LED to the
   pipeline's colour (PURPLE for INLET, RED/GREEN/BLUE for zones 1–3), and `DC_motor_speed()` drives the
   motor PWM — either a fixed duty cycle or the potentiometer reading via `get_potentio_num()`.
3. If no pipeline is scheduled for the current hour, the motor and LEDs are held off (`DC_motor_reset()`,
   `reset_led()`) — an "empty zone" hour.
4. `water_level()` updates the simulated/measured reservoir depth (replenishing for INLET, depleting for
   zones, at a rate tied to the active PWM setting), and `get_rpm()` converts encoder tick counts into
   real-time RPM.
5. `DIGITS_Display()` and `run_output()` push the current hour, pipeline, PWM setting, RPM, and depth to
   the seven-segment display and the UART terminal once per scaled hour.
6. If depth ever reaches 0%, the controller immediately kills the motor and servo PWM, turns LD2 off,
   reports `RESERVOIR IS EMPTY` over UART, flashes the RGB LED white (`flash_white_led()`), and halts —
   requiring a manual reset. If the schedule reaches hour 24 first, it reports completion and stops
   cleanly instead.

### Peripheral configuration

| Peripheral | Use |
|---|---|
| TIM2 | 50 Hz PWM (channel 1) driving the servo — four fixed `CCR1` pulse widths select INLET/Zone1/2/3 |
| TIM3 | PWM (channels 1 & 3) driving the DC pump motor's speed controller |
| TIM4 | Input capture (channel 2, both edges) on the ultrasonic sensor's echo line, with 16-bit overflow handling for the pulse-width measurement |
| TIM5 | Base timer, periodic interrupt driving the scaled wall clock |
| ADC1 | 8-bit conversion on the potentiometer channel for manual PWM override |
| USART2 | 115200 baud, interrupt-driven RX, for setup entry and status logging |
| EXTI | B1 button (mode switch, falling edge) and the RPM encoder pulse (rising edge, tick counting) |
| GPIO | RGB LED, seven-segment digit lines (BCD-decoded in `DIGITS_Display()`), HC-SR04 trigger pulse |

## Why it's built this way

- **Two-mode split (Setup/Run)** keeps all UART parsing and blocking `while(rcv_intpt_flag == 0)` waits
  out of the real-time control loop, so Run Mode only ever deals with scheduling and PWM/servo updates.
- **Interrupt-driven everything that matters for timing** — RPM ticks, ultrasonic echo edges, and the
  scaled clock — because polling any of these on a fast, jittery, or event-driven signal in a `while(1)`
  loop would either miss events or block the motor/servo updates.
- **Scaled time** exists purely so a 24-hour irrigation cycle can be demoed in under 5 minutes without
  changing any of the scheduling logic — the controller doesn't know it isn't running in real time.
- **Fixed PWM presets per pipeline** reflect the real system: each zone sits at a different elevation
  (head), so it needs a different minimum pump RPM to actually reach it, while the potentiometer gives a
  manual override for tuning/demo purposes.
- **Fail-safe on empty reservoir** is deliberately the highest-priority check in the loop — the code
  guarantees the pump physically cannot keep running once the reservoir hits 0%, rather than relying on
  correct scheduling to prevent it.

## Repo layout

Currently a single-file firmware source:

- `main.c` — CubeMX-generated STM32F401RE init code plus the application logic described above, in the
  `USER CODE` regions.

## Possible next steps

- **DMA for ADC/UART** — the potentiometer read and UART transmits are all blocking HAL calls; moving
  them to circular DMA would free the CPU during Run Mode instead of stalling on every conversion/report.
- **RTOS (FreeRTOS)** — the single `while(1)` loop currently interleaves scheduling, motor control, and
  reporting; splitting these into prioritized tasks would let a reservoir-empty fault preempt routine
  telemetry logging instead of racing it in a flat loop.
