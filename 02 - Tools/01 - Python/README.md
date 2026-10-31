<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/ACwing-UAV/ACwing/main/brand/ACwing_logo_white.png">
    <img alt="ACwing" src="https://raw.githubusercontent.com/ACwing-UAV/ACwing/main/brand/ACwing_logo_black.png" width="200">
  </picture>
</p>

# ACwing FC – PC tools in Python (`acwing_tools`)

**Project:** ACwing · **Author:** Andrea Cirillo · **Overview:** [repository README](../../README.md) · [documentation](../../03%20-%20Docs/index.md)

Python equivalents of the MATLAB tools in [`02 - Tools/00 - Matlab/`](../00%20-%20Matlab), for users
without MATLAB. They cover:
- telemetry decoding;
- the live 3D viewer;
- raw capture and the byte-level sniffer;
- magnetometer calibration;
- the comparison of the embedded filter with the original and extended IEKF.

The two filters are ported too.

The Python tools produce the **same results** as the MATLAB ones:
- the filters agree with the MATLAB code to ~1e-14;
- the comparison reports the same figures on the same captures.

Logs are interchangeable: a `.mat` written by one viewer is read by the
other language's tools (checked both ways, with identical calibration results). The test suite checks all of this against reference
outputs computed by the MATLAB code itself (see [Tests](#tests)).

## Contents

- [Installation](#installation)
- [MATLAB ↔ Python map](#matlab--python-map)
- [Command line](#command-line)
- [Typical workflows](#typical-workflows)
- [Python API](#python-api)
- [Log format](#log-format)
- [Working without the board](#working-without-the-board)
- [Tests](#tests)
- [Package layout](#package-layout)

## Installation

Python ≥ 3.9. From this folder (`02 - Tools/01 - Python`; quote the path, it contains spaces):

```bash
python -m venv .venv
.venv\Scripts\activate             # Windows   (Linux/macOS: source .venv/bin/activate)
pip install -e .                   # numpy, scipy, matplotlib, pyserial + the "acwing-tools" command
pip install -e ".[ble]"            # optional: BLE link (bleak)
```

Without installing, run from this folder with `pip install -r requirements.txt`
and `python -m acwing_tools ...`.

Before the ACwing rename the package was `iekf_tools` and the command
`iekf-tools`. The old command still works, as an alias of `acwing-tools`. If
you installed the package under its old name, uninstall it first:
`pip uninstall iekf-tools`, then run `pip install -e .`.

The board appears as **"STMicroelectronics STLink Virtual COM Port"**:
- **Windows**: `COMx`;
- **Linux**: `/dev/ttyACMx`. Add your user to the `dialout` group if the port
  cannot be opened.
- **macOS**: `/dev/cu.usbmodem*`.

## MATLAB ↔ Python map

| MATLAB (`00 - Matlab/`) | Python (`acwing_tools`) | Command line |
|---|---|---|
| `Viewer/acwingLiveViewer.m` | `viewer.live_viewer`, `viewer.LiveViewer` | `acwing-tools viewer COM6` |
| `acwingLiveViewer("ble", ...)` | `ble_io.BleSource` + `viewer.LiveViewer` | `acwing-tools viewer --ble [NAME]` |
| `blelist` | `ble_io.scan` | `acwing-tools scan` |
| `FiltersComparison/tlmCapture.m` | `capture.tlm_capture` | `acwing-tools capture COM6 --duration 60` |
| `FiltersComparison/tlmSniffer.m` | `capture.tlm_sniffer` | `acwing-tools sniff COM6` |
| `FiltersComparison/compareEmbeddedVsMatlab.m` | `compare.compare_embedded` | `acwing-tools compare acwing_cap_*.bin` |
| `MagCalib/calibMagAxes.m` | `magcalib.calib_mag_axes` | `acwing-tools calib-mag acwing_log_*.mat` |
| `MagCalib/fitHardIron.m` | `magcalib.fit_hard_iron` | `acwing-tools fit-hard-iron acwing_log_*.mat` |
| `MagCalib/magCheck.m` | `magcalib.mag_check` | `acwing-tools mag-check acwing_log_*.mat --hard-iron X,Y,Z` |
| `Telemetry/parseTelemetry.m` | `telemetry.parse_telemetry` | |
| `Telemetry/decodeState/Init/Status/Replay.m` | `telemetry.decode_state/init/status/replay` | |
| `Telemetry/crc16ccitt.m`, `statusDiagnosis.m` | `telemetry.crc16ccitt`, `telemetry.status_diagnosis` | |
| `IEKF_originalVersion/iekfAttitude.m` (+ `expSO3`, `logSO3`, `skewSym`) | `filters.iekf_attitude` (+ `exp_so3`, `log_so3`, `skew_sym`) | |
| `IEKF_embeddedVersion/iekfAttitudeEmb.m` + `iekfCore*.m` | `filters.iekf_attitude_emb` + `filters.embedded.core_*` | |
| `Viewer/quat2rotmLocal.m`, `so3ToQuat.m` | `common.quat2rotm`, `common.rotm2quat` | |
| `initPath.m` | not needed (`pip install -e .`) | |
| — | `logio.load_log`, `logio.save_log` (MATLAB-compatible `.mat`) | `acwing-tools info file` |
| — | `simulate.synthetic_stream` (test stream, no board) | `acwing-tools viewer --replay file.bin` |

Function and option names follow MATLAB. Arrays keep the MATLAB layout:
- inputs `3 × N`;
- attitudes `3 × 3 × N`;
- covariances `n × n × N`.

So porting a MATLAB script is mechanical.

## Command line

`acwing-tools <command>` (or `python -m acwing_tools <command>`); `-h` on any
command lists its options.
Lists of numbers (`--map`, `--hard-iron`) take commas or spaces. When the
first value is negative, write `--hard-iron=-5,0,0`, otherwise the shell
argument is taken as an option.

| Command | What it does |
|---|---|
| `viewer PORT [--baud 921600] [--log-dir DIR]` | Live 3D board + triad in the NWU frame. Text panel: roll/pitch/yaw, packet rate, lost packets, display lag, bias, flags, calibration. Writes `acwing_log_*.mat` (every 10 s and on exit) and the raw `acwing_log_*.bin`. Stop by closing the window or with Ctrl+C: the log is saved in both cases. |
| `viewer --ble [NAME_OR_ADDRESS] [--decim 4]` | Same viewer over BLE (needs `bleak`). Without a name, the strongest board advertising `ACwFC` is used (`IEKF…` and `STM32WB`, the names of older firmware, are recognised too). `--decim N`: STATE every N acc updates (4 = 60 Hz, 8 = 30 Hz), written to the CMD characteristic. The panel adds a `BLE:` line (state, MTU, drops, notifications). |
| `scan [--timeout 5]` | Lists the BLE devices advertising, ACwing boards marked. |
| `capture --ble [NAME] [--duration 30] [--decim 4]` | Records the BLE stream to `acwing_ble_*.bin` without graphics and prints every 2 s the notifications/s, kB/s, STATE/s, lag and the firmware BLE fields. Use it to tell a slow link from a slow viewer. |
| `viewer --replay FILE.bin [--rate B/s]` | Same viewer fed by a recorded capture (default 19 kB/s ≈ real time; `--rate 0` = as fast as possible). |
| `capture PORT [--duration 60] [--out-dir DIR]` | Raw bytes to `acwing_cap_*.bin`, no decoding: for `APP_TLM_REPLAY = 1` (~44 kB/s). Reports the rate over the second half (~44 kB/s expected). |
| `sniff PORT [--duration 3]` / `sniff --file FILE.bin` | Bytes → frames → STATUS → diagnosis of the first broken link (no bytes, stuck DMA, wrong baud rate, IMU, interrupt, FIFO, sensor hub, calibration). |
| `compare FILE.bin [--no-plot]` | Embedded vs reference filters ([validation](../../03%20-%20Docs/tools/validation.md)). Exit code 0 = MATCH, 1 = DIVERGE, 2 = unusable capture. |
| `calib-mag LOG.mat [--gap 0.2] [--map M] [--hard-iron H]` | Magnetometer axis map + hard-iron ([magnetometer calibration](../../03%20-%20Docs/tools/mag-calibration.md)); prints the two `#define` lines for `app_config.h`. `--map`/`--hard-iron`: values of the firmware that recorded the log (default identity / 0); the result is always absolute. |
| `mag-check LOG.mat [--hard-iron H]` | Is the current calibration still valid? Sphere fit of the logged field and rotation consistency with the gyro; prints the corrected `MAG_HARD_IRON` if needed. |
| `fit-hard-iron LOG.mat [--no-plot]` | Sphere fit of the hard-iron offset, with a 3D plot. |
| `info FILE` | Summary of a `.bin` (frames per type, last STATUS diagnosis) or a `.mat` log. |

## Typical workflows

These follow the [PC tools](../../03%20-%20Docs/tools/pc-tools.md), [magnetometer calibration](../../03%20-%20Docs/tools/mag-calibration.md) and [validation](../../03%20-%20Docs/tools/validation.md) pages step by step; only the commands change.

**Live view** (firmware with `APP_TLM_REPLAY = 0`):

```bash
acwing-tools viewer COM6              # start, THEN reset the board and keep it still 2-4 s
```

**Live view over BLE** (firmware with `APP_TLM_BLE 1`, see [BLE telemetry](../../03%20-%20Docs/firmware/ble.md)):

```bash
acwing-tools scan                     # the board must be listed (disconnect any phone first)
acwing-tools viewer --ble             # or: --ble ACwFC --decim 8
```

The BLE reception runs in a separate process. Bluetooth notifications arrive
through Python callbacks, and in the viewer's own process the 3D redraw delayed
them: on Windows the viewer showed 23 packets/s instead of 60 and a growing
lag, with nothing lost. The panel's `BLE:` line shows the notifications/s
actually received.

The BLE link carries the same frames as USB, with STATE decimated and no
REPLAY. Log and `.bin` have the same format as over USB, so every other tool
works on them. Use USB for `capture` and `compare`.

**Magnetometer calibration** ([page](../../03%20-%20Docs/tools/mag-calibration.md)):
1. Record ~40 s of rotations about all axes with the viewer.
2. Run, passing the values of the firmware that recorded the log:
   ```bash
   acwing-tools calib-mag acwing_log_YYYYMMDD_HHMMSS.mat --map 1,0,0,0,-1,0,0,0,-1 --hard-iron 100.31,-0.56,-166.07
   ```
   If that firmware had the identity map and zero offset, omit both options.
   Do not omit them on a calibrated firmware: the result would be the residual
   correction (≈ identity, ≈ 0), not the calibration.
3. To check an existing calibration without re-flashing:
   ```bash
   acwing-tools mag-check acwing_log_YYYYMMDD_HHMMSS.mat --hard-iron 100.31,-0.56,-166.07
   ```

**Embedded-vs-reference validation** ([page](../../03%20-%20Docs/tools/validation.md)):
1. Flash with `APP_TLM_REPLAY 1`, and `APP_COMPARE_ORIGINAL` 1 or 0.
2. Record and compare:
   ```bash
   acwing-tools capture COM6 --duration 60      # then reset the board immediately
   acwing-tools compare acwing_cap_YYYYMMDD_HHMMSS.bin
   ```

The comparison prints the same report as `compareEmbeddedVsMatlab.m`:
- capture integrity;
- absolute and sigma-normalised differences;
- MATCH/DIVERGE;
- the coverage line in mode 0.

It then shows three figures: Euler angles, continuous quaternion, and
difference/bias.

## Python API

```python
from acwing_tools.telemetry import read_capture, parse_telemetry, decode_state
from acwing_tools.filters import iekf_attitude, iekf_attitude_emb
from acwing_tools.compare import compare_embedded
from acwing_tools.logio import load_log
from acwing_tools.magcalib import calib_mag_axes

frames, rest = parse_telemetry(read_capture("acwing_cap.bin"))
states = [decode_state(f.payload) for f in frames if f.type == 1]

out = compare_embedded("acwing_cap.bin", do_plot=False)   # out.ok, out.dAng, out.extended, ...

L = load_log("acwing_log.mat")          # works for MATLAB and Python logs
cal = calib_mag_axes(L)               # cal.M, cal.hardIronBody, cal.ratio

# the filters, as in MATLAB: inputs 3 x N, options with the MATLAB names
R, b, P = iekf_attitude(gyro, acc, mag, dt, gRef, mRef, {"sigmaAcc": 0.03})
R, b, P, info = iekf_attitude_emb(gyro, acc, mag, dt, gRef, mRef,
                                  {"accNisGate": 4, "inflTau": 0.5, "lockoutTime": 0.5})
```

Decoded packets and results are `Struct` objects: a `dict` with attribute access
(`s.q`, `I.sigmaAcc`), with the field names of the MATLAB structs.

## Log format

Both viewers save the variable `L` in `acwing_log_YYYYMMDD_HHMMSS.mat` with the
same layout (see `logio.py`):

| Field | Shape | Content |
|---|---|---|
| `seq` | 1×N | filter sample index (wrap-around removed) |
| `t_us` | 1×N | filter time [µs] |
| `q` | 4×N | attitude quaternion `[w x y z]`, body → world |
| `bias` | 3×N | gyro bias [rad/s] |
| `acc` | 3×N | accelerometer window mean [g] |
| `gyro` | 3×N | gyro window mean [rad/s] |
| `mag` | 3×N | magnetometer, body frame, hard-iron removed [µT] |
| `flags` | 1×N | STATE flags: 1 running, 2 new mag, 4 acc de-weighted, 8 mag de-weighted, 16 overrun, 32 still |
| `t_pc` | 1×N | PC time since the viewer started [s] |
| `init` | struct | last INIT packet (empty if none) |
| `status` | struct array | STATUS packets (10 Hz) |
| `resetAt` | 1×K | sample indices (1-based) after a board reboot |

Next to every log, the viewer writes the raw byte stream (`.bin`), which
`compare` and `info` accept.

## Working without the board

- `acwing-tools viewer --replay acwing_log_*.bin` replays any recorded session in
  real time.
- `simulate.synthetic_stream()` generates a telemetry stream with a known
  motion, magnetometer axis map and hard-iron:
  ```bash
  python -c "from acwing_tools.simulate import write_stream; write_stream('sim.bin')"
  acwing-tools viewer --replay sim.bin
  ```
  The stream contains INIT, STATUS and STATE frames, with the raw sensor field
  as the firmware logs it during the calibration acquisition. `calib-mag` on
  the resulting log must recover the given map and offset.

## Tests

```bash
pip install -e ".[test]"
pytest                    # 26 tests, ~25 s
```

| Test file | Checks |
|---|---|
| `test_telemetry.py` | CRC check value; parser on random chunks with garbage and false syncs; skip of REPLAY; decoding of every packet type; STATUS diagnosis |
| `test_filters.py` | `iekf_attitude` and `iekf_attitude_emb` (three configurations, all extensions) vs the MATLAB outputs: attitude < 1e-11, bias < 1e-12, covariance < 1e-14; SO(3) maps |
| `test_compare.py` | `compare_embedded` on a mode-1 and a mode-0 capture (stillness, NIS inflation, mag age, 1 lockout) vs the MATLAB `compareEmbeddedVsMatlab` figures; **negative control**: a 20 % change of the dip tolerance in INIT must give DIVERGE |
| `test_ble.py` | STATUS of 54 and 62 bytes; board discovery by name/address/RSSI; a stream cut into 20-byte notifications reassembles exactly; viewer fed by notifications from another thread (4× decimated stream: 0 lost, BLE line in the panel, `.bin` identical); a real gap on a decimated stream counted once; reception through the worker process (radio replaced by a recorded stream); headless `capture --ble` |
| `test_tools.py` | viewer replay (log, `.bin` identical to the stream), `.mat` round trip, `calib_mag_axes` recovers a known axis map and hard-iron, `fit_hard_iron`, `mag_check` finds a stale hard-iron and its absolute correction, sniffer diagnosis, CLI |

Fixtures in `tests/data`:
- `capture_mode1.bin` and `capture_mode0.bin` (160 kB each): frames produced by
  the firmware C code compiled on a PC with a simulated IMU. They have the same
  format as the board's.
- `filter_inputs.mat`: synthetic inputs for the filter parity test.
- `filter_reference.mat` and `compare_reference.mat`: outputs of the MATLAB
  code on those inputs.

To regenerate the references after a change in the MATLAB filters:

```bash
python tests/reference/make_inputs.py                  # from 02 - Tools/01 - Python
cd tests/reference && matlab -batch make_reference     # or: octave-cli --eval make_reference
```

## Package layout

```
02 - Tools/01 - Python/
├── README.md, pyproject.toml, requirements.txt
├── acwing_tools/
│   ├── __main__.py      command line (acwing-tools)
│   ├── common.py        Struct, quaternion/rotation helpers
│   ├── telemetry.py     CRC, parser, decoders, STATUS diagnosis
│   ├── filters/
│   │   ├── original.py  iekf_attitude + exp/log/skew (IEKF_originalVersion)
│   │   └── embedded.py  iekf_attitude_emb + core_* (IEKF_embeddedVersion, mirror of iekf.c)
│   ├── compare.py       embedded vs original/extended filter
│   ├── viewer.py        live 3D viewer + logging
│   ├── capture.py       raw capture, sniffer
│   ├── magcalib.py      axis map + hard-iron, sphere fit, calibration check
│   ├── logio.py         MATLAB-compatible .mat logs
│   ├── serial_io.py     serial port / file replay sources
│   ├── ble_io.py        BLE source (bleak, in a worker process): scan, discovery, TLM notifications, CMD
│   └── simulate.py      synthetic telemetry stream (USB or BLE: decimated STATE,
│                        62-byte STATUS, notification cutting)
└── tests/               pytest suite, fixtures, MATLAB reference generators
```

**BLE requirements:** a Bluetooth 4.0+ adapter, with Windows 10+, macOS, or
Linux with BlueZ (`bluetoothd` running). No pairing is needed: the service is
open. A board connected to a phone is not visible to the PC: disconnect the
phone first.
