#!/usr/bin/env python3
"""
soundcore-balance (p40i_balance.py)
Hardware-level Left/Right channel balance & per-ear 8-band EQ calibration tool
for Anker Soundcore earbuds (P40i, Liberty 4, Liberty 4 NC, Space A40, Life P3, etc.).

Writes independent Left and Right channel gains directly into the earbuds'
onboard non-volatile DSP memory (via Soundcore RFCOMM protocol command 0x03 0x87),
so the balance fix persists across ALL connected devices (Phone, PC, TV, Console).
"""

import argparse
import json
import math
import os
import re
import select
import socket
import subprocess
import sys
import termios
import time
import tty
from typing import Dict, List, Optional, Tuple

BAND_LABELS = ["100Hz", "200Hz", "400Hz", "800Hz", "1.6kHz", "3.2kHz", "6.4kHz", "12.8kHz"]
SOUNDCORE_KEYWORDS = (
    "soundcore",
    "p40i",
    "p30i",
    "p20i",
    "r50i",
    "liberty",
    "space a40",
    "life p3",
    "life a2",
    "sport x",
    "a3955",
    "a3953",
    "a3947",
    "a3936",
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BACKUP_FILE = os.path.join(BASE_DIR, "backup_state.json")


def apply_drc(adj_10: List[int]) -> List[int]:
    """
    Exact transcription of Soundcore's HearId2Utils.b (OpenSCQ30 VolumeAdjustments::apply_drc).
    Transforms 8 band adjustments (in 0.1 dB units, range -120..+134) into the 10-byte
    DSP DRC register block.
    """
    adjustments = [a / 10.0 for a in adj_10[:8]]
    S, L = 0.85, 0.95
    lows_sub = adjustments[2] * 0.81 * S
    highs_sub = adjustments[5] * 0.81 * S
    COEFFS = [
        [1.26, -0.71 * S, 0.177, -0.0494, 0.0345, -0.0197, 0.0075, -0.00217],
        [-0.71 * S, 1.73 * L, 0.0, 0.204, -0.068, 0.045, -0.0235, 0.0075],
        [0.177, -0.81 * S, 1.73 * L, -0.81 * S, 0.208, -0.07, 0.045, -0.0197],
        [-0.0494, 0.204, 0.0, 1.73 * L, -0.82 * S, 0.208, -0.068, 0.0345],
        [0.0345, -0.068, 0.208, -0.82 * S, 1.73 * L, 0.0, 0.204, -0.0494],
        [-0.0197, 0.045, -0.07, 0.208, -0.81 * S, 1.73 * L, -0.81 * S, 0.177],
        [0.0075, -0.0235, 0.045, -0.068, 0.204, 0.0, 1.83 * L, -0.71 * S],
        [-0.00217, 0.0075, -0.0197, 0.0345, -0.0494, 0.177, -0.71 * S, 1.5],
    ]
    out = []
    for b_idx in range(8):
        acc = 0.0
        for idx, adj in enumerate(adjustments):
            if (b_idx in (1, 3)) and idx == 2:
                acc -= lows_sub
            elif (b_idx in (4, 6)) and idx == 5:
                acc -= highs_sub
            else:
                acc += adj * COEFFS[b_idx][idx]
        r = int(math.copysign(math.floor(abs(acc) + 0.5), acc))
        out.append(max(-120, min(134, r)) + 120)
    out.extend([120, 0])
    return out


def make_packet(cmd: Tuple[int, int], body: List[int]) -> bytes:
    header = bytes([0x08, 0xEE, 0x00, 0x00, 0x00, cmd[0], cmd[1]])
    length = len(header) + 2 + len(body) + 1
    pkt = header + length.to_bytes(2, "little") + bytes(body)
    chk = sum(pkt) & 0xFF
    return pkt + bytes([chk])


def discover_soundcore_device() -> Optional[Tuple[str, str]]:
    """Auto-detect connected Soundcore Bluetooth device via bluetoothctl."""
    try:
        res = subprocess.run(
            ["bluetoothctl", "devices", "Connected"],
            capture_output=True,
            text=True,
            timeout=3,
        )
        lines = res.stdout.strip().splitlines()
        if not lines:
            res = subprocess.run(
                ["bluetoothctl", "devices", "Paired"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            lines = res.stdout.strip().splitlines()

        for line in lines:
            m = re.match(r"^Device\s+([0-9A-Fa-f:]{17})\s+(.+)$", line.strip())
            if m:
                mac, name = m.group(1), m.group(2)
                if any(k in name.lower() for k in SOUNDCORE_KEYWORDS):
                    return mac, name
    except Exception:
        pass

    if os.path.exists(DEFAULT_BACKUP_FILE):
        try:
            with open(DEFAULT_BACKUP_FILE, "r") as f:
                data = json.load(f)
                if "mac" in data:
                    return data["mac"], "Soundcore Device"
        except Exception:
            pass
    return None


def connect_soundcore(
    mac: str, preferred_channel: Optional[int] = None
) -> Tuple[socket.socket, int, bytes]:
    """
    Connect to the Soundcore RFCOMM command channel and return (socket, channel, state_packet).
    Automatically probes channels if preferred_channel is not specified or doesn't respond to RequestState.
    """
    channels_to_try = []
    if preferred_channel is not None:
        channels_to_try.append(preferred_channel)
    for ch in [6, 5, 4, 1, 2, 3, 7, 8, 9, 10, 11, 12, 15, 20]:
        if ch not in channels_to_try:
            channels_to_try.append(ch)

    last_err = None
    req_pkt = make_packet((1, 1), [])
    for ch in channels_to_try:
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.settimeout(1.5)
        try:
            s.connect((mac, ch))
            s.sendall(req_pkt)
            data = s.recv(1024)
            if len(data) >= 60 and data[0] == 0x09 and data[1] == 0xFF:
                s.settimeout(3.0)
                return s, ch, data
            s.close()
        except Exception as e:
            last_err = e
            try:
                s.close()
            except Exception:
                pass

    raise RuntimeError(
        f"Could not connect to Soundcore command channel on {mac}. "
        f"Make sure the earbuds are out of the case and connected. (Last error: {last_err})"
    )


def parse_state_packet(raw: bytes) -> Dict:
    """Parse A3955 / Soundcore state update packet (Command 0x01 0x01)."""
    body = raw[9:-1]
    preset_id = int.from_bytes(body[38:40], "little")
    eq_left_db = [round((x - 120) / 10.0, 1) for x in body[40:48]]
    eq_right_raw = list(body[50:58])
    eq_right_db = (
        [round((x - 120) / 10.0, 1) for x in eq_right_raw]
        if eq_right_raw[0] != 255
        else None
    )
    hearid_status = body[60]
    hearid_enabled = bool(body[61])
    hearid_left_db = [round((x - 120) / 10.0, 1) for x in body[62:70]]
    hearid_right_db = [round((x - 120) / 10.0, 1) for x in body[72:80]]
    hearid_time = int.from_bytes(body[82:86], "big")
    hearid_type = body[86]
    hearid_custom_left_db = [round((x - 120) / 10.0, 1) for x in body[87:95]]
    hearid_custom_right_db = [round((x - 120) / 10.0, 1) for x in body[97:105]]
    hearid_genre = list(body[107:109])

    return {
        "tws_host": "Right" if body[0] == 1 else "Left",
        "tws_connected": bool(body[1]),
        "battery_left": f"{body[2]}/5",
        "battery_right": f"{body[3]}/5",
        "case_battery": f"{body[37]}/5",
        "charging_left": bool(body[4]),
        "charging_right": bool(body[5]),
        "fw_left": body[6:11].decode("ascii", errors="replace"),
        "fw_right": body[11:16].decode("ascii", errors="replace"),
        "serial": body[16:32].decode("ascii", errors="replace"),
        "preset_id": f"{preset_id:#06x}",
        "eq_left_db": eq_left_db,
        "eq_right_db": eq_right_db,
        "hearid_status": hearid_status,
        "hearid_enabled": hearid_enabled,
        "hearid_left_db": hearid_left_db,
        "hearid_right_db": hearid_right_db,
        "hearid_custom_left_db": hearid_custom_left_db,
        "hearid_custom_right_db": hearid_custom_right_db,
        "hearid_time": hearid_time,
        "hearid_type": hearid_type,
        "hearid_genre": hearid_genre,
        "raw_state_hex": raw.hex(),
    }


def ensure_backup(mac: str, channel: int, state: Dict, backup_path: str = DEFAULT_BACKUP_FILE) -> None:
    """Save a backup of the device's state if no backup file exists yet."""
    if os.path.exists(backup_path):
        return
    backup_data = {
        "mac": mac,
        "rfcomm_channel": channel,
        "raw_state_hex": state["raw_state_hex"],
        "original_eq_db": state["eq_left_db"],
        "original_hear_id_enabled": 1 if state["hearid_enabled"] else 0,
        "original_hear_id_left_db": state["hearid_left_db"],
        "original_hear_id_right_db": state["hearid_right_db"],
        "original_hear_id_time": state["hearid_time"],
        "original_hear_id_type": state["hearid_type"],
        "original_hear_id_genre": state["hearid_genre"],
    }
    with open(backup_path, "w") as f:
        json.dump(backup_data, f, indent=2)


def get_base_eq_tenths(state: Dict, backup_path: str = DEFAULT_BACKUP_FILE) -> List[int]:
    """Load the user's original 8-band EQ curve from backup (or current state if no backup)."""
    if os.path.exists(backup_path):
        try:
            with open(backup_path, "r") as f:
                b = json.load(f)
                if "original_eq_db" in b and len(b["original_eq_db"]) == 8:
                    return [int(round(x * 10)) for x in b["original_eq_db"]]
        except Exception:
            pass
    return [int(round(x * 10)) for x in state["eq_left_db"]]


def write_lr_eq(
    sock: socket.socket,
    left_bands_tenths: List[int],
    right_bands_tenths: List[int],
    enable_hearid: bool = True,
    hearid_time: int = 1762257418,
    hearid_type: int = 2,
    hearid_genre: List[int] = [17, 0],
) -> bytes:
    """
    Write independent 8-band Left and Right EQ curves (in 0.1 dB tenths, range -120..+120)
    to the earbuds' onboard DSP via Command [0x03, 0x87].
    """
    left_clamped = [max(-120, min(120, x)) for x in left_bands_tenths[:8]]
    right_clamped = [max(-120, min(120, x)) for x in right_bands_tenths[:8]]

    eq_left_bytes = [x + 120 for x in left_clamped] + [120, 120]
    eq_right_bytes = [x + 120 for x in right_clamped] + [120, 120]
    hearid_left_bytes = [x + 120 for x in left_clamped] + [60, 60]
    hearid_right_bytes = [x + 120 for x in right_clamped] + [60, 60]

    drc_left = apply_drc(left_clamped)
    drc_right = apply_drc(right_clamped)

    body = (
        [254, 254]
        + list(hearid_genre[:2])
        + eq_left_bytes
        + eq_right_bytes
        + [0, 0]
        + [1 if enable_hearid else 0]
        + hearid_left_bytes
        + hearid_right_bytes
        + list(hearid_time.to_bytes(4, "big"))
        + [hearid_type]
        + hearid_left_bytes
        + hearid_right_bytes
        + drc_left
        + drc_right
        + [0]
    )
    pkt = make_packet((3, 135), body)
    sock.sendall(pkt)
    return sock.recv(1024)


def restore_from_backup(sock: socket.socket, backup_path: str = DEFAULT_BACKUP_FILE) -> bytes:
    if not os.path.exists(backup_path):
        raise FileNotFoundError(f"Backup file not found: {backup_path}")
    with open(backup_path, "r") as f:
        backup = json.load(f)

    orig_eq = [int(round(x * 10)) for x in backup["original_eq_db"]]
    orig_h_l = [int(round(x * 10)) for x in backup["original_hear_id_left_db"]]
    orig_h_r = [int(round(x * 10)) for x in backup["original_hear_id_right_db"]]

    eq_bytes = [x + 120 for x in orig_eq] + [120, 120]
    hl_bytes = [x + 120 for x in orig_h_l] + [60, 60]
    hr_bytes = [x + 120 for x in orig_h_r] + [60, 60]
    drc = apply_drc(orig_eq)

    body = (
        [254, 254]
        + backup.get("original_hear_id_genre", [17, 0])
        + eq_bytes
        + eq_bytes
        + [0, 0]
        + [backup.get("original_hear_id_enabled", 0)]
        + hl_bytes
        + hr_bytes
        + list(backup.get("original_hear_id_time", 1762257418).to_bytes(4, "big"))
        + [backup.get("original_hear_id_type", 2)]
        + hl_bytes
        + hr_bytes
        + drc
        + drc
        + [0]
    )
    pkt = make_packet((3, 135), body)
    sock.sendall(pkt)
    return sock.recv(1024)


def print_status_summary(mac: str, channel: int, st: Dict, base_tenths: List[int]) -> None:
    print(f"\n[*] Soundcore Device : {mac} (RFCOMM ch {channel}, FW {st['fw_left']} / {st['fw_right']})")
    print(
        f"[*] Battery Levels   : Left {st['battery_left']} | Right {st['battery_right']} | Case {st['case_battery']}"
    )
    print(f"[*] HearID Stereo DSP: {'ACTIVE' if st['hearid_enabled'] else 'INACTIVE'}")
    print("-" * 74)
    print(f"{'Band':<8} | {'Base EQ':>9} | {'Left (Ch0)':>12} | {'Right (Ch1)':>12} | {'L-R Diff':>10}")
    print("-" * 74)
    for i, label in enumerate(BAND_LABELS):
        base_v = base_tenths[i] / 10.0
        l_v = st["hearid_left_db"][i] if st["hearid_enabled"] else st["eq_left_db"][i]
        r_v = st["hearid_right_db"][i] if st["hearid_enabled"] else st["eq_left_db"][i]
        diff = l_v - r_v
        print(
            f"{label:<8} | {base_v:+6.1f} dB | {l_v:+9.1f} dB | {r_v:+9.1f} dB | {diff:+7.1f} dB"
        )
    print("-" * 74)


def read_key() -> str:
    """Read a single keypress or ANSI escape sequence directly from the unbuffered TTY fd."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        raw = os.read(fd, 8)
        return raw.decode("utf-8", errors="ignore")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def run_interactive_tui(mac: str, channel: int, sock: socket.socket, st: Dict) -> None:
    base_tenths = get_base_eq_tenths(st)
    cur_l = [int(round(x * 10)) for x in st["hearid_left_db"]]
    cur_r = [int(round(x * 10)) for x in st["hearid_right_db"]]

    avg_l_shift = round(sum(cur_l[i] - base_tenths[i] for i in range(8)) / 8.0 / 5.0) * 0.5
    avg_r_shift = round(sum(cur_r[i] - base_tenths[i] for i in range(8)) / 8.0 / 5.0) * 0.5
    left_offset = avg_l_shift
    right_offset = avg_r_shift
    status_msg = "[i] Play some music and use Arrow Keys to calibrate L/R balance in real time."

    while True:
        sys.stdout.write("\x1b[2J\x1b[H")
        print("==========================================================================")
        print(" :: SOUNDCORE HARDWARE L/R BALANCE CALIBRATOR (Onboard DSP Flash Memory)")
        print("==========================================================================")
        print(f" Device: {mac} (Ch {channel}) | FW: {st['fw_left']} | Battery: L {st['battery_left']} R {st['battery_right']}")
        print("--------------------------------------------------------------------------")

        diff = left_offset - right_offset
        # Visual slider (-12 dB to +12 dB)
        pos = int(round((right_offset - left_offset + 12.0) / 24.0 * 40))
        pos = max(0, min(40, pos))
        slider = ["─"] * 41
        slider[20] = "┼"
        slider[pos] = "◆"
        print(f" Balance Meter : [L] {''.join(slider)} [R]")
        print(
            f" Gain Offsets  : Left: {left_offset:+5.1f} dB   |   Right: {right_offset:+5.1f} dB   (Total L-R Diff: {diff:+5.1f} dB)"
        )
        print("--------------------------------------------------------------------------")
        print(f" {'Band':<8} | {'Base EQ':>9} | {'Left Ear DSP':>14} | {'Right Ear DSP':>14}")
        print("--------------------------------------------------------------------------")
        for i, label in enumerate(BAND_LABELS):
            b_db = base_tenths[i] / 10.0
            l_db = max(-12.0, min(12.0, b_db + left_offset))
            r_db = max(-12.0, min(12.0, b_db + right_offset))
            print(f" {label:<8} | {b_db:+6.1f} dB | {l_db:+11.1f} dB | {r_db:+11.1f} dB")
        print("--------------------------------------------------------------------------")
        print(" Controls:")
        print("  [← / Left ] Shift balance to LEFT  (+0.5 dB Left, -0.5 dB Right)")
        print("  [→ / Right] Shift balance to RIGHT (-0.5 dB Left, +0.5 dB Right)")
        print("  [↑ / Up   ] Boost LEFT only (+0.5 dB)   | [↓ / Down] Cut RIGHT only (-0.5 dB)")
        print("  [0] Center (0.0 dB / 0.0 dB)            | [r] Restore original factory backup")
        print("  [q] Quit (Settings remain saved in earbud hardware memory)")
        print("--------------------------------------------------------------------------")
        print(f" Status: {status_msg}")
        sys.stdout.flush()

        key = read_key()
        changed = False
        if key in ("\x1b[D", "\x1bOD", "h", "H", "a", "A"):  # Left arrow
            left_offset = min(12.0, left_offset + 0.5)
            right_offset = max(-12.0, right_offset - 0.5)
            changed = True
        elif key in ("\x1b[C", "\x1bOC", "l", "L", "d", "D"):  # Right arrow
            left_offset = max(-12.0, left_offset - 0.5)
            right_offset = min(12.0, right_offset + 0.5)
            changed = True
        elif key in ("\x1b[A", "\x1bOA", "w", "W"):  # Up arrow: boost left only
            left_offset = min(12.0, left_offset + 0.5)
            changed = True
        elif key in ("\x1b[B", "\x1bOB", "s", "S"):  # Down arrow: cut right only
            right_offset = max(-12.0, right_offset - 0.5)
            changed = True
        elif key == "0":
            left_offset = 0.0
            right_offset = 0.0
            changed = True
        elif key in ("r", "R"):
            try:
                restore_from_backup(sock)
                time.sleep(0.2)
                sock.sendall(make_packet((1, 1), []))
                st = parse_state_packet(sock.recv(1024))
                left_offset = 0.0
                right_offset = 0.0
                status_msg = "[+] Restored original backup state to earbud flash memory."
            except Exception as e:
                status_msg = f"[!] Restore error: {e}"
        elif key in ("q", "Q", "\x03"):
            print("\n[+] Saved in earbud hardware memory. Exiting.")
            break

        if changed:
            l_shift = int(round(left_offset * 10))
            r_shift = int(round(right_offset * 10))
            l_tenths = [max(-120, min(120, x + l_shift)) for x in base_tenths]
            r_tenths = [max(-120, min(120, x + r_shift)) for x in base_tenths]
            try:
                write_lr_eq(sock, l_tenths, r_tenths, enable_hearid=True)
                status_msg = f"[>>] Flashed to earbuds: Left {left_offset:+.1f} dB | Right {right_offset:+.1f} dB"
            except Exception as e:
                status_msg = f"[!] Connection lost, reconnecting... ({e})"
                try:
                    sock.close()
                except Exception:
                    pass
                sock, channel, _ = connect_soundcore(mac, channel)
                write_lr_eq(sock, l_tenths, r_tenths, enable_hearid=True)
                status_msg = f"[>>] Reconnected & flashed: Left {left_offset:+.1f} dB | Right {right_offset:+.1f} dB"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hardware-level Left/Right channel balance calibrator for Anker Soundcore earbuds."
    )
    parser.add_argument("--mac", help="Bluetooth MAC address (auto-detected if omitted)")
    parser.add_argument("--channel", type=int, help="RFCOMM channel (auto-detected if omitted)")
    parser.add_argument("--status", action="store_true", help="Show current earbud hardware state")
    parser.add_argument("--json", action="store_true", help="Output state as JSON")
    parser.add_argument("--interactive", "-i", action="store_true", help="Launch live interactive TUI calibrator")
    parser.add_argument("--restore", action="store_true", help="Restore original backed-up state to earbuds")
    parser.add_argument("--left", type=float, help="Left channel gain offset in dB (e.g. +4.5)")
    parser.add_argument("--right", type=float, help="Right channel gain offset in dB (e.g. -4.5)")
    parser.add_argument(
        "--flat-base",
        action="store_true",
        help="Apply L/R offsets relative to a flat 0 dB EQ curve instead of your saved custom EQ curve",
    )
    args = parser.parse_args()

    mac = args.mac
    dev_name = "Soundcore Earbuds"
    if not mac:
        found = discover_soundcore_device()
        if not found:
            sys.stderr.write(
                "Error: No connected Soundcore device found. Connect your earbuds or specify --mac XX:XX:XX:XX:XX:XX\n"
            )
            sys.exit(1)
        mac, dev_name = found

    sock, channel, raw_state = connect_soundcore(mac, args.channel)
    try:
        st = parse_state_packet(raw_state)
        ensure_backup(mac, channel, st)
        base_tenths = [0] * 8 if args.flat_base else get_base_eq_tenths(st)

        if args.restore:
            restore_from_backup(sock)
            time.sleep(0.25)
            sock.sendall(make_packet((1, 1), []))
            st = parse_state_packet(sock.recv(1024))
            if args.json:
                print(json.dumps(st, indent=2))
            else:
                print("[+] Original state restored to earbud hardware memory.")
                print_status_summary(mac, channel, st, base_tenths)
            return

        if args.left is not None or args.right is not None:
            l_db = args.left if args.left is not None else 0.0
            r_db = args.right if args.right is not None else 0.0
            l_shift = int(round(l_db * 10))
            r_shift = int(round(r_db * 10))
            l_tenths = [max(-120, min(120, x + l_shift)) for x in base_tenths]
            r_tenths = [max(-120, min(120, x + r_shift)) for x in base_tenths]
            write_lr_eq(sock, l_tenths, r_tenths, enable_hearid=True)
            time.sleep(0.25)
            sock.sendall(make_packet((1, 1), []))
            st = parse_state_packet(sock.recv(1024))
            if args.json:
                print(json.dumps(st, indent=2))
            else:
                print(
                    f"[+] Flashed to {dev_name} ({mac}) hardware DSP: Left {l_db:+.1f} dB | Right {r_db:+.1f} dB"
                )
                print_status_summary(mac, channel, st, base_tenths)
            return

        if args.interactive:
            run_interactive_tui(mac, channel, sock, st)
            return

        if args.status or args.json:
            if args.json:
                print(json.dumps(st, indent=2))
            else:
                print_status_summary(mac, channel, st, base_tenths)
            return

        # Default if no flags passed: launch interactive TUI if terminal, otherwise show status
        if sys.stdin.isatty() and sys.stdout.isatty():
            run_interactive_tui(mac, channel, sock, st)
        else:
            print_status_summary(mac, channel, st, base_tenths)
    finally:
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
