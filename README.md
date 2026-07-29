# powtran-controller

Command-line controller and cycle scripting for **Powtran PI150 / PI500 /
PI550** frequency inverters (VFDs) over Modbus RTU (RS485), written in C++
with libmodbus.

Built to drive a motor test bench for sensor characterization: start, stop
and speed-change a motor programmatically, or script repeatable test
sequences in a small YAML file while logging telemetry (frequency,
slip-adjusted rpm, voltage, current, power, torque) once per second.

```
[Linux PC (USB-RS485) or ARM board] --RS485--> [Powtran VFD] --> [motor]
```

## Verified register map

The protocol is shared across the PI150/PI500/PI550 family; everything below
was verified live against a PI150 (including two deviations from the PI500
manual, marked **quirk**).

| Addr | R/W | Meaning |
|------|-----|---------|
| `0x2000` | W | control: 1 fwd, 2 rev, 3 fwd jog, 4 rev jog, 5 coast, 6 decel stop, 7 fault reset |
| `0x1000` | W | frequency setpoint, 0–10000 = 0–100 % of max frequency (F0.19) |
| `0x1000`–`0x1007` | R | monitor block, see below |
| `0x3000` | R | status: 1 fwd run, 2 rev run, 3 stop |
| `0x8000` | R | fault code (0 = none) |
| `0xFgii` | RW | parameter Fg.ii, persisted to EEPROM (e.g. F0.19 → `0xF013`, index in hex) |
| `0x0gii` | W | same parameter, RAM only (no EEPROM wear). **Quirk**: write-only on the PI150 — read back via `0xFgii` |

Monitor block (`0x1000`+): setpoint echo (0.01 %) — **quirk**: not the running
frequency the manual suggests — then running freq (0.01 Hz), bus V (0.1 V),
output V (1 V), output I (0.01 A), power (0.1 kW), torque (0.1 %), and rotor
speed estimate (0.01 Hz electrical, slip-adjusted: reads below the field
frequency while motoring, above it while regenerating).

Useful parameters: F0.13/F0.14 accel/decel time (RAM `0x000D`/`0x000E`),
F0.15 ramp time unit, F0.19 max frequency, F0.18 carrier frequency
(`0xF012`), F9.00–F9.02 baud/parity/address.

Vendor manuals are linked under [References](#references).

## Building

```sh
make            # -> build/x86/powtran-ctl (needs libmodbus-dev)
make check      # self-test of the cycle parser, no hardware needed
make cross CROSS_CXX="arm-linux-gnueabihf-g++ --sysroot=..."   # ARM target
```

## VFD setup

One-time wiring/config: RS485 A/B to the VFD's 485+/485−, and switch the VFD
to serial control (writes F0.11=2, F0.03=9 to EEPROM):

```sh
powtran-ctl setup       # enable communication control
powtran-ctl setup off   # revert to keypad control
```

## Usage

```
powtran-ctl [-p port] [-b baud] [-a slave] [--rts down|up|none]
            [--rpm-per-hz N] <command>
```

Defaults: `/dev/ttyS1`, 9600 8N2, slave 1, RTS down, 30 rpm/Hz (4-pole
motor). With a USB-RS485 converter, the converter usually handles bus
direction itself:

```sh
powtran-ctl -p /dev/ttyUSB0 --rts none status
```

### One-shot commands

```sh
powtran-ctl status              # run state, freq, rpm, V, A, kW, torque, fault
powtran-ctl run 1500 3          # 1500 rpm, 3 s ramp; negative rpm = reverse
powtran-ctl hz 25 3             # same but by output frequency
powtran-ctl stop 2              # decelerating stop, 2 s ramp
powtran-ctl coast               # free-wheel stop (no braking)
powtran-ctl reset               # fault reset
powtran-ctl read 0xF013         # raw register read (also: read <addr> <count>)
powtran-ctl write 0x000D 10     # raw register write
```

Ramp arguments are optional — omit them to keep the VFD's current accel/decel
times. `ramp_s` uses the VFD's native semantics: seconds for a full
0 → max-frequency sweep, so partial speed changes complete proportionally
faster. Ramp times go to the RAM parameter copies, so no EEPROM wear.

### Cycle files

```sh
powtran-ctl cycle cycles/example.yaml
```

```yaml
rpm_per_hz: 30     # motor speed per Hz of drive frequency
max_rpm: 1500      # safety clamp on run targets
loop: 1            # cycle repetitions, 0 = forever

cycle:
  - run 1500 1     # run at 1500 rpm, 1 s ramp
  - wait 2         # hold 2 s, printing one telemetry line per second
  - run 750 0.5    # speed change, 0.5 s ramp
  - wait 2
  - run -750 1     # negative rpm = reverse
  - wait 2
  - stop 1         # decelerating stop, 1 s ramp
  - wait 1
```

Steps: `run <rpm> [ramp_s]`, `hz <hz> [ramp_s]`, `stop [ramp_s]`, `coast`,
`wait <s>`, `reset`, `write <addr> <value>`.
Config keys: `port`, `baud`, `slave`, `rts`, `rpm_per_hz`, `max_rpm`, `loop`
(CLI flags take precedence).

Behavior:

- Motion commands return immediately; the VFD ramps in the background. Follow
  `run` with a `wait` to hold a speed.
- The whole file is validated before the first Modbus write — a typo aborts
  before the motor starts.
- **Ctrl-C** anywhere triggers a decelerating stop (exit code 130). An
  unrecoverable Modbus error mid-cycle also stops the motor (exit 1).
- Telemetry goes to stdout, one line per second during `wait`:

```
t=   2.0  st=fwd  f= 48.90Hz set=100.0% bus=318.7V out=212V I= 1.73A P=0.2kW T=18.6% rpm=1450
```

Real capture — accelerating a 4-pole 0.75 kW motor through 26 → 49 Hz
(`powtran-ctl status` during a 10 s ramp):

```
freq:        26.06 Hz    speed:  778 rpm    output: 116 V, 1.70 A, torque 18.3 %
freq:        32.67 Hz    speed:  983 rpm    output: 145 V, 1.73 A, torque 19.2 %
freq:        40.54 Hz    speed: 1219 rpm    output: 178 V, 1.74 A, torque 19.3 %
freq:        48.90 Hz    speed: 1450 rpm    output: 212 V, 1.73 A, torque 18.6 %
```

## Troubleshooting

- **`Invalid CRC` / `Response not from requested slave` while running** — EMI
  from the VFD output couples into the RS485 pair; worst during regenerative
  deceleration. The tool flushes the line and retries (6×), and a cycle
  tolerates dropped telemetry samples (aborts only after 3 consecutive
  failures). To lower the error rate: 120 Ω termination at the VFD end,
  shielded twisted pair routed away from motor cables, or a lower carrier
  frequency (`powtran-ctl write 0xF012 20` = 2 kHz).
- **Motor doesn't start on `run`** — check `setup` was applied (read
  `0xF00B` = 2, `0xF003` = 9) and no fault is latched (`status`).
- **No response at all** — verify wiring with
  `mbpoll -F -m rtu -b 9600 -P none -d 8 -s 2 -a 1 -0 -r 63746 -c 1 -1 <port>`
  (reads parameter F9.02, the slave address — expect 1).

## References

- [PI150 English Manual V4.1](https://cnc-tehnologi.ru/files/Pi150.pdf) — parameter list
- [PI500 English User Guide](https://static1.squarespace.com/static/5b60a97de7494070b92f2702/t/615b2cd8f5a6c623bde1b493/1633366698283/powtran-PI-500+ENGLISH+USER+GUIDE.pdf) — Appendix I,
  RS485 communication protocol

## License

[MIT](LICENSE)
