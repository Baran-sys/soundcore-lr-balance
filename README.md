# soundcore-lr-balance

Hardware-level Left/Right channel balance calibrator for Anker Soundcore earbuds.

When one earbud becomes quieter than the other due to drops, mesh clogging, or driver aging, OS-level balance sliders only fix the issue on a single source device. This tool uses Soundcore's stereo HearID DSP registers (`[0x03, 0x87]`) over Bluetooth RFCOMM to flash independent Left and Right 8-band gain offsets directly into the earbuds' onboard non-volatile flash memory. Once flashed, the balance compensation persists across all connected devices (phones, PCs, TVs, consoles).

## Hardware Support Requirements

Standard Bluetooth audio profiles (A2DP/AVRCP) do not provide a way to write persistent per-channel gain into an earbud's firmware. For a device to work with this tool, it must meet **two hardware requirements**:

1. **Anker Soundcore RFCOMM Protocol:** The device must expose the Soundcore serial command interface (`08 ee 00 00 00 ...`) over Bluetooth RFCOMM.
2. **Stereo HearID DSP Support (`[0x03, 0x87]`):** Standard Soundcore EQ (`[0x02, 0x81]`) is mono (a single 8-band curve applied equally to both ears). Only Soundcore models equipped with **HearID** have dual-channel DSP registers (`Ch0` = Left, `Ch1` = Right) and stereo Dynamic Range Compression (DRC) biquad filters in hardware.

### Supported Models

| Model ID | Device Name | Status |
| :--- | :--- | :--- |
| **A3955** | Soundcore P40i | Tested & Verified on Hardware |
| **A3953** | Soundcore Liberty 4 | Compatible (`[0x03, 0x87]`) |
| **A3947** | Soundcore Liberty 4 NC | Compatible (`[0x03, 0x87]`) |
| **A3954** | Soundcore Liberty 4 Pro | Compatible (`[0x03, 0x87]`) |
| **A3957** | Soundcore Liberty 5 | Compatible (`[0x03, 0x87]`) |
| **A3936** | Soundcore Space A40 | Compatible (`[0x03, 0x87]`) |
| **A3939** | Soundcore Life P3 | Compatible (`[0x03, 0x87]`) |
| **A3952** | Soundcore Liberty 3 Pro | Compatible (`[0x03, 0x87]`) |
| **A3951** | Soundcore Liberty Air 2 Pro | Compatible (`[0x03, 0x87]`) |
| **A3959** | Soundcore P30i / R50i NC | Compatible (`[0x03, 0x87]`) |
| **A3968** | Soundcore Sport X20 | Compatible (`[0x03, 0x87]`) |

### Unsupported Devices

- **Entry-level Soundcore models without HearID** (e.g., Soundcore P20i, P25i, R50i `A3949`, Life P2, Q20i, Q30): These models only accept a single-channel EQ curve (`[0x02, 0x81]` or `[0x02, 0x83]`) and apply it identically to both sides.
- **Non-Soundcore Bluetooth headphones/earbuds** (Sony, JBL, Samsung, Apple, Bose, QCY, etc.): These use different proprietary protocols or lack writable per-ear DSP memory.

## Installation

No external Python packages (`pip`) are required. The tool runs out of the box on **Linux, Windows, and macOS** using only the Python standard library:
- **Linux:** Connects via `socket.AF_BLUETOOTH` (`BTPROTO_RFCOMM`) and auto-discovers devices via `bluetoothctl`.
- **Windows (10/11):** Connects via Winsock `socket.AF_BLUETOOTH` and auto-discovers paired Soundcore devices via PowerShell `Get-PnpDevice`.
- **macOS:** Connects via macOS Bluetooth SPP serial device nodes (`/dev/cu.*`) and auto-discovers via `system_profiler SPBluetoothDataType`.

**Requirements:**
- **OS:** Linux, Windows 10/11, or macOS
- **Python:** 3.9+
- Earbuds connected via Bluetooth and removed from the charging case

```bash
git clone https://github.com/Baran-sys/soundcore-lr-balance.git
cd soundcore-lr-balance
python3 soundcore_balance.py
```

## Usage

### 1. Interactive Live Calibration (TUI)

Run the script without arguments while playing audio to calibrate Left/Right balance in real time:

```bash
python3 soundcore_balance.py
```

**Interactive Keys:**
- `Left Arrow` (`h` / `a`): Shift balance toward Left (`+0.5 dB` Left, `-0.5 dB` Right)
- `Right Arrow` (`l` / `d`): Shift balance toward Right (`-0.5 dB` Left, `+0.5 dB` Right)
- `Up Arrow` (`w`): Boost Left channel only (`+0.5 dB`)
- `Down Arrow` (`s`): Cut Right channel only (`-0.5 dB`)
- `0`: Reset offsets to `0.0 dB / 0.0 dB`
- `r`: Restore original factory backup from `backup_state.json`
- `q`: Quit (settings remain saved in earbud hardware memory)

### 2. Command-Line Flags

```bash
# Boost Left by +4.5 dB and cut Right by -4.5 dB (9.0 dB total L-R difference)
python3 soundcore_balance.py --left +4.5 --right -4.5

# Cut Right by -4.0 dB only (prevents clipping on bass-heavy custom EQs)
python3 soundcore_balance.py --left 0 --right -4.0

# Apply offsets over a flat 0.0 dB EQ curve instead of your saved custom EQ
python3 soundcore_balance.py --flat-base --left +3.0 --right -3.0

# Display current hardware DSP state, firmware version, and battery levels
python3 soundcore_balance.py --status

# Output device state as JSON
python3 soundcore_balance.py --status --json

# Restore the original backed-up state to the earbuds
python3 soundcore_balance.py --restore

# Manually specify Bluetooth MAC address (Linux/Windows) or serial port (macOS)
python3 soundcore_balance.py --mac 98:47:44:A6:DD:CB --channel 6 --status
python3 soundcore_balance.py --port /dev/cu.soundcoreP40i --status
```

| Flag | Description |
| :--- | :--- |
| `-i`, `--interactive` | Launch the live interactive TUI calibrator (default when run with no flags) |
| `--left <dB>` | Left channel gain offset in dB (`-12.0` to `+12.0`) |
| `--right <dB>` | Right channel gain offset in dB (`-12.0` to `+12.0`) |
| `--flat-base` | Use a flat `0.0 dB` base curve instead of preserving your existing 8-band EQ |
| `--status` | Print current earbud hardware state and per-band L/R DSP table |
| `--json` | Output state information in JSON format |
| `--restore` | Flash the original backed-up state (`backup_state.json`) back to the earbuds |
| `--mac <MAC>` | Specify target Bluetooth MAC address on Linux/Windows (auto-detected if omitted) |
| `--channel <INT>` | Specify target RFCOMM channel on Linux/Windows (auto-probed if omitted) |
| `--port <PATH>` | Specify Bluetooth serial device path on macOS/Linux (e.g. `/dev/cu.soundcoreP40i`) |

## Backup & Notes

- On its first run, the script automatically backs up your earbuds' untouched state packet to `backup_state.json` before writing any changes.
- Changing EQ presets inside the official Soundcore mobile app will overwrite the custom HearID/EQ registers on the earbuds. If you change settings in the mobile app, run `soundcore_balance.py` again to re-apply your hardware balance offset.

## License

MIT License. Protocol structure and DRC coefficient calculation adapted from [OpenSCQ30](https://github.com/Oppzippy/OpenSCQ30).
