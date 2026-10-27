"""Command-line interface: ``python -m acwing_tools <command> ...`` (or ``acwing-tools`` once installed).

Commands (MATLAB equivalent in brackets):

  viewer         live 3D attitude viewer + log, USB or BLE [acwingLiveViewer]
  scan           list the boards advertising over BLE      [blelist]
  capture        raw telemetry to .bin, no decoding       [tlmCapture]
  sniff          byte-level link diagnosis                [tlmSniffer]
  compare        embedded filter vs reference filters     [compareEmbeddedVsMatlab]
  calib-mag      magnetometer axis map + hard-iron        [calibMagAxes]
  fit-hard-iron  hard-iron sphere fit                     [fitHardIron]
  mag-check      is the firmware mag calibration still valid?  [magCheck]
  info           summary of a capture (.bin) or log (.mat)
"""
from __future__ import annotations

import argparse
import sys


def _cmd_viewer(a):
    from .viewer import live_viewer
    try:
        live_viewer(port=a.port, baud=a.baud, log_dir=a.log_dir, replay=a.replay,
                    replay_rate=None if a.rate <= 0 else a.rate,
                    ble=a.ble, ble_decim=a.decim)
    except (RuntimeError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


def _cmd_scan(a):
    from .ble_io import is_board_name, scan
    try:
        devs = scan(a.timeout)
    except (ImportError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if not devs:
        print("No BLE device found.")
        return 1
    print(f"{'name':24s} {'address':40s} RSSI")
    for name, addr, rssi in devs:
        mark = "  <-- ACwing board" if is_board_name(name) else ""
        print(f"{name or '-':24s} {addr:40s} {rssi:4d}{mark}")
    return 0 if any(is_board_name(n) for n, _, _ in devs) else 1


def _cmd_capture(a):
    from .capture import ble_capture, tlm_capture
    if a.ble is not None:
        try:
            ble_capture(a.ble, a.duration, a.decim, a.out_dir)
        except (RuntimeError, ImportError) as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2
        return 0
    if not a.port:
        print("capture: give a serial port or --ble [NAME]", file=sys.stderr)
        return 2
    tlm_capture(a.port, a.duration, a.baud, a.out_dir)


def _cmd_sniff(a):
    from .capture import tlm_sniffer
    if a.file:
        with open(a.file, "rb") as fh:
            tlm_sniffer(a.file, a.baud, a.duration, data=fh.read())
    else:
        tlm_sniffer(a.port, a.baud, a.duration)


def _cmd_compare(a):
    from .compare import CompareError, compare_embedded
    try:
        out = compare_embedded(a.capture, do_plot=not a.no_plot)
    except CompareError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0 if out.ok else 1


def _floats(text, n, name):
    v = [float(x) for x in text.replace(",", " ").split()]
    if len(v) != n:
        raise SystemExit(f"{name}: expected {n} numbers, got {len(v)}")
    return v


def _cmd_calib(a):
    from .magcalib import calib_mag_axes
    calib_mag_axes(a.log, a.gap,
                   map_fw=_floats(a.map, 9, "--map") if a.map else None,
                   hard_iron_fw=_floats(a.hard_iron, 3, "--hard-iron") if a.hard_iron else None)


def _cmd_magcheck(a):
    from .magcalib import mag_check
    out = mag_check(a.log, _floats(a.hard_iron, 3, "--hard-iron") if a.hard_iron else None)
    return 0 if out.verdict == "ok" else 1


def _cmd_fit(a):
    from .magcalib import fit_hard_iron
    fit_hard_iron(a.log, do_plot=not a.no_plot)


def _cmd_info(a):
    import numpy as np
    if a.file.lower().endswith(".mat"):
        from .logio import load_log
        L = load_log(a.file)
        n = L.seq.size
        dur = (L.t_us[-1] - L.t_us[0]) * 1e-6 if n > 1 else 0.0
        print(f"{a.file}: {n} STATE samples over {dur:.1f} s, {len(L.status)} STATUS, "
              f"INIT {'present' if L.init else 'missing'}, {L.resetAt.size} reboots")
        if n:
            print(f"  |mag| mean {np.mean(np.linalg.norm(L.mag, axis=0)):.1f} uT, "
                  f"|acc| mean {np.mean(np.linalg.norm(L.acc, axis=0)):.3f} g")
        return 0
    from .telemetry import decode_status, frame_counts, parse_telemetry, read_capture, status_diagnosis
    data = read_capture(a.file)
    frames, rest = parse_telemetry(data)
    c = frame_counts(frames)
    print(f"{a.file}: {len(data)} bytes, {len(frames)} valid frames, {len(rest)} trailing bytes")
    print("  " + "   ".join(f"{k} {v}" for k, v in c.items()))
    st = [fr for fr in frames if fr.type == 3]
    if st:
        print("  last STATUS: " + status_diagnosis(decode_status(st[-1].payload)))
        from .telemetry import timing_report
        print("\n".join(timing_report(frames)))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="acwing_tools", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("viewer", help="live 3D viewer (serial port, BLE, or replay of a .bin)")
    s.add_argument("port", nargs="?", help="serial port, e.g. COM6 or /dev/ttyACM0")
    s.add_argument("--ble", nargs="?", const="", metavar="NAME_OR_ADDRESS",
                   help="use BLE instead of the serial port; without a value the board is found "
                        "automatically (name ACwFC, IEKF... or STM32WB). Needs: pip install bleak")
    s.add_argument("--decim", type=int, default=4,
                   help="BLE only: STATE every N acc updates (default 4 = 60 Hz; 8 = 30 Hz)")
    s.add_argument("--baud", type=int, default=921600)
    s.add_argument("--log-dir", default=".", help="folder for the .mat log and the .bin capture")
    s.add_argument("--replay", metavar="BIN", help="replay a recorded capture instead of a port")
    s.add_argument("--rate", type=float, default=19000.0,
                   help="replay speed in bytes/s (0 = as fast as possible)")
    s.set_defaults(func=_cmd_viewer)

    s = sub.add_parser("scan", help="list the devices advertising over BLE (needs bleak)")
    s.add_argument("--timeout", type=float, default=5.0, help="scan time [s] (default 5)")
    s.set_defaults(func=_cmd_scan)

    s = sub.add_parser("capture", help="record raw telemetry to .bin, no graphics (USB or BLE)")
    s.add_argument("port", nargs="?", help="serial port (omit with --ble)")
    s.add_argument("--ble", nargs="?", const="", metavar="NAME_OR_ADDRESS",
                   help="record the BLE link instead, printing the received rates every 2 s")
    s.add_argument("--decim", type=int, default=4, help="BLE only: STATE every N acc updates")
    s.add_argument("--duration", type=float, default=60.0, help="seconds (default 60)")
    s.add_argument("--baud", type=int, default=921600)
    s.add_argument("--out-dir", default=".")
    s.set_defaults(func=_cmd_capture)

    s = sub.add_parser("sniff", help="byte-level link diagnosis")
    s.add_argument("port", nargs="?", default="")
    s.add_argument("--baud", type=int, default=921600)
    s.add_argument("--duration", type=float, default=3.0)
    s.add_argument("--file", help="analyse a recorded .bin instead of the port")
    s.set_defaults(func=_cmd_sniff)

    s = sub.add_parser("compare", help="embedded filter vs original/extended reference")
    s.add_argument("capture", help="raw capture .bin recorded with APP_TLM_REPLAY = 1")
    s.add_argument("--no-plot", action="store_true")
    s.set_defaults(func=_cmd_compare)

    s = sub.add_parser("calib-mag", help="magnetometer axis map + hard-iron from a log")
    s.add_argument("log", help="viewer log .mat (MATLAB or Python)")
    s.add_argument("--gap", type=float, default=0.2, help="pair spacing [s] (default 0.2)")
    s.add_argument("--map", metavar="'9 VALUES'",
                   help="MAG_AXIS_MAP of the firmware that recorded the log (row-major, as in "
                        "app_config.h); default identity")
    s.add_argument("--hard-iron", metavar="'X Y Z'",
                   help="MAG_HARD_IRON of the firmware that recorded the log [uT]; default 0")
    s.set_defaults(func=_cmd_calib)

    s = sub.add_parser("mag-check", help="is the firmware magnetometer calibration still valid?")
    s.add_argument("log", help="viewer log .mat recorded with the CURRENT firmware")
    s.add_argument("--hard-iron", metavar="'X Y Z'",
                   help="current MAG_HARD_IRON [uT]: prints the corrected value")
    s.set_defaults(func=_cmd_magcheck)

    s = sub.add_parser("fit-hard-iron", help="hard-iron sphere fit from a log")
    s.add_argument("log")
    s.add_argument("--no-plot", action="store_true")
    s.set_defaults(func=_cmd_fit)

    s = sub.add_parser("info", help="summary of a capture .bin or a log .mat")
    s.add_argument("file")
    s.set_defaults(func=_cmd_info)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == "viewer" and sum(x is not None and x is not False for x in
                                    (args.port, args.replay, args.ble)) != 1:
        print("viewer: give exactly one of: a serial port, --ble [NAME], --replay <file.bin>",
              file=sys.stderr)
        return 2
    rc = args.func(args)
    return int(rc or 0)


if __name__ == "__main__":
    sys.exit(main())
