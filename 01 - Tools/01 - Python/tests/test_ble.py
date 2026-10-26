"""BLE link: STATUS with the BLE fields, board discovery, notification source,
viewer on a decimated, fragmented stream (no radio needed)."""
import threading
import time

import numpy as np

from iekf_tools.ble_io import BleSource, is_board_name, pick_board
from iekf_tools.simulate import ble_notifications, pack_status, synthetic_stream
from iekf_tools.telemetry import decode_status, parse_telemetry
from iekf_tools.viewer import LiveViewer


def test_decode_status_both_lengths():
    s1 = decode_status(pack_status(1234, 100))                       # phase 1, 54 B
    assert len(pack_status(0, 0)) == 54 and s1.ble_state == 0 and s1.uptime_s == 1.234
    p2 = pack_status(1234, 100, ble=(3, 247, 7, 0x64))               # phase 2, 62 B
    assert len(p2) == 62
    s2 = decode_status(p2)
    assert (s2.ble_state, s2.ble_mtu, s2.ble_drop, s2.ble_err) == (3, 247, 7, 0x64)
    assert s2.running == s1.running and s2.n_gyro == s1.n_gyro


def test_board_discovery():
    devs = [("Phone", "AA:01", -40), ("STM32WB", "AA:02", -70), ("IEKF", "AA:03", -60),
            ("", "AA:04", -30)]
    assert is_board_name("IEKF") and is_board_name("STM32WB") and not is_board_name("Phone")
    assert pick_board(devs)[1] == "AA:03"                            # strongest board name
    assert pick_board(devs, "STM32WB")[1] == "AA:02"                  # by name
    assert pick_board(devs, "aa:01")[0] == "Phone"                    # by address
    assert pick_board(devs, "nothing") is None
    assert pick_board([("Phone", "AA:01", -40)]) is None


def test_notifications_reassemble_into_frames():
    stream = synthetic_stream(duration=5.0, state_decim=4, ble=(3, 185, 0, 0))
    src = BleSource(connect=False)
    for chunk in ble_notifications(stream, mtu=23, seed=3):          # worst case: 20 B
        src.on_notify(None, bytearray(chunk))
    data = src.read()
    assert data == stream and src.read() == b""
    frames, rest = parse_telemetry(data)
    assert rest == b"" and sum(f.type == 1 for f in frames) == 300   # 5 s at 60 Hz


def test_viewer_over_ble_source(tmp_path):
    """Notifications arrive from another thread, as with bleak; the viewer must
    count 0 lost packets on a 4x decimated stream and show the BLE line."""
    stream = synthetic_stream(duration=10.0, state_decim=4, ble=(3, 247, 0, 0))
    src = BleSource(connect=False)
    chunks = ble_notifications(stream, mtu=247, seed=5)

    def radio():
        for c in chunks:
            src.on_notify(None, bytearray(c))
        time.sleep(0.05)
        src.eof = True

    v = LiveViewer(src, log_dir=str(tmp_path), stop_at_eof=True, show=False)
    threading.Thread(target=radio, daemon=True).start()
    L = v.run()
    assert L.seq.size == 600 and v.n_lost == 0 and v.seq_step == 4
    assert np.all(np.diff(L.seq) == 4) and L.init is not None
    assert open(v.bin_file, "rb").read() == stream
    txt = v.status_text()
    assert "BLE: streaming  MTU 247  drop 0" in txt and "notifications" in txt


def test_viewer_counts_real_gaps_on_decimated_stream(tmp_path):
    """Drop one STATE of a 4x decimated stream: exactly 1 lost."""
    stream = synthetic_stream(duration=2.0, state_decim=4)
    frames, _ = parse_telemetry(stream)
    from iekf_tools.telemetry import encode_frame
    states = [i for i, f in enumerate(frames) if f.type == 1]
    drop = states[50]
    cut = b"".join(encode_frame(f.type, f.payload) for i, f in enumerate(frames) if i != drop)
    p = tmp_path / "gap.bin"
    p.write_bytes(cut)
    from iekf_tools.serial_io import FileSource
    v = LiveViewer(FileSource(str(p), rate=None, chunk=300), log_dir=str(tmp_path),
                   stop_at_eof=True, show=False)
    v.run()
    assert v.n_lost == 1
