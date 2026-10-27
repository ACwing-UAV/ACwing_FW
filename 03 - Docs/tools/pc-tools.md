# PC tools and live viewer


Every tool exists in MATLAB and in Python, with the same options and outputs.
MATLAB setup:

```matlab
run('<repo>/02 - Tools/00 - Matlab/initPath.m')   % once per session
```

Python setup (details in [`02 - Tools/01 - Python/README.md`](https://github.com/andrea-cirillo/ACwing_FlightController_FW/blob/main/02%20-%20Tools/01%20-%20Python/README.md)):

```bash
cd "02 - Tools/01 - Python"
python -m pip install -e .        # installs the acwing-tools command
```

`initPath.m` adds all folders (tools, original filter, extended filter) to the
path and prints which `iekfAttitude.m` is in use (it must be the one in
`FiltersComparison/IEKF_originalVersion`; `compareEmbeddedVsMatlab` warns if
another copy shadows it).

**Live viewer** (firmware with `APP_TLM_REPLAY = 0`):

```matlab
L = acwingLiveViewer("serial", "COM6");                 % 921600 baud by default
L = acwingLiveViewer("serial", "COM6", 921600, "C:\logs") % choose the log folder
L = acwingLiveViewer("ble");                            % BLE, board found automatically
```

1. Start the viewer, then reset the board and keep it **still for 2–4 s**
   (static calibration; it restarts if the board moves).
2. The 3D triad follows the board in the levelled NWU frame, seen with x (N)
   to the right, y (W) into the screen and z up, slightly from above. The text panel shows
   roll/pitch/yaw, packet rate (~240 Hz), lost packets, **display lag** (warning
   above 0.5 s), gyro bias, flags (`ACC de-weighted`, `MAG de-weighted`, `STILL`)
   and the calibration result. Before the filter starts it shows the STATUS
   diagnosis instead.
3. Close the figure to stop. The log (`acwing_log_*.mat`, autosaved every 10 s) is
   also returned and copied into the workspace as `acwingLog`; a raw `.bin` capture
   is written next to it.

**Byte-level link check:**

```matlab
tlmSniffer("COM6")      % bytes -> frames -> STATUS -> diagnosis
```

**Python equivalents** (installation in [`02 - Tools/01 - Python/README.md`](https://github.com/andrea-cirillo/ACwing_FlightController_FW/blob/main/02%20-%20Tools/01%20-%20Python/README.md)):

```bash
acwing-tools viewer COM6                  # live viewer; --log-dir DIR, --baud B
acwing-tools viewer --replay file.bin     # replay a recorded session (no board needed)
acwing-tools viewer --ble                 # BLE (pip install bleak), board found automatically
acwing-tools sniff COM6                   # byte-level link check
```

The Python viewer shows the same panel and writes the same `.mat` log (variable
`L`) and raw `.bin`. Logs are interchangeable between MATLAB and Python.
