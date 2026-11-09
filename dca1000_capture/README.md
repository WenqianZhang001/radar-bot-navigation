# Raw IQ capture with the DCA1000EVM (AWR1843BOOST)

Record the radar's raw ADC samples (complex IQ), not just its point cloud. The radar streams every chirp's samples to a TI DCA1000EVM board, which forwards them to the Jetson over Ethernet.

```text
AWR1843BOOST ──60-pin ribbon──► DCA1000EVM ──Ethernet──► Jetson: capture_raw.py ──► captures/*_iq.npy
     └──────────── USB (radar settings) ────────────────► Jetson
```

## Quick start

```bash
cd ~/radar-bot-navigation/dca1000_capture
python3 dca1000.py info               # should print the DCA1000's FPGA version
python3 capture_raw.py --frames 100   # 10 seconds of data
```

`capture_raw.py` sets up the DCA1000 over Ethernet and the radar over USB (with `awr1843_raw.cfg`), arms the DCA1000, starts the radar, records, then stops both. It prints the frame rate, any lost packets and the strongest returns.

## Output

Each capture writes three files to `captures/` (kept out of git), named after the start time:

| File | What it is |
| --- | --- |
| `*_iq.npy` | complex64 samples, shape (frame, chirp, rx, sample) |
| `*_raw.bin` | the DCA1000 byte stream, in the format TI's tools record |
| `*_meta.json` | radar settings, packet statistics, lost bytes per frame |

With `awr1843_raw.cfg`, a frame is 16 loops of 3 chirps (TX0, TX2, TX1) × 4 RX × 256 samples, at 10 frames per second, 4.36 cm per range bin and 11.2 m maximum range:

```python
import numpy as np
iq = np.load('captures/YYYYmmdd_HHMMSS_iq.npy')
x = iq.reshape(-1, 16, 3, 4, 256)     # (frame, loop, tx, rx, sample)
```

For other chirps, edit a copy of `awr1843_raw.cfg` and pass it with `--cfg`. Keep `lvdsStreamCfg -1 0 1 0`; the frame size is worked out from the file.

## Files

| File | What it is |
| --- | --- |
| `capture_raw.py` | Records a capture (above). |
| `dca1000.py` | DCA1000 commands, frame reassembly from its UDP stream, and IQ conversion; usable from your own code. `python3 dca1000.py stop` ends a recording left running by an interrupted capture. |
| `awr1843_raw.cfg` | Radar settings: the chirps of `radar_pointcloud/awr1843_profile.cfg`, with raw ADC streaming switched on. |

## Before you start

- **Wiring.** The DCA1000 connects to the radar with the blue 60-pin ribbon cable, and its Ethernet port goes to the Jetson's Ethernet port. The radar's USB cable goes to the Jetson as usual. The DCA1000's own USB port isn't used.
- **Power.** Both boards need 5 V. The DCA1000's slide switch picks its source: `DC_JACK_5V_IN` (its own barrel jack) or `RADAR_5V_IN` (shared with the radar's supply). If the radar's LEDs start blinking on `RADAR_5V_IN`, that supply can't carry both boards; give the DCA1000 its own.
- **DCA1000 DIP switch SW2, position 5 on `SW_CONFIG`**, so the Jetson can configure it over Ethernet.
- **Jetson network, once per Jetson.** The DCA1000 is at `192.168.33.180` and only talks to `192.168.33.30`. The lab Jetson already has the `dca1000` connection (check with `nmcli connection show dca1000`); elsewhere:

  ```bash
  sudo nmcli connection add type ethernet ifname enP8p1s0 con-name dca1000 \
    ipv4.method manual ipv4.addresses 192.168.33.30/24 ipv4.never-default yes \
    ipv6.method ignore connection.autoconnect-priority 10
  sudo nmcli connection up dca1000
  ```

  Wi-Fi stays the internet connection. To use the wired port on an ordinary network instead, run `sudo nmcli connection up "Wired connection 1"`.
- **A large network receive buffer**, so the burst of packets that carries each frame isn't dropped. `capture_raw.py` warns when it's too small. To set it permanently:

  ```bash
  echo 'net.core.rmem_max=33554432' | sudo tee /etc/sysctl.d/90-dca1000.conf
  sudo sysctl -w net.core.rmem_max=33554432
  ```

- **Nothing else is using the radar.** Stop `radar_viewer.py` or the ROS driver first; they set the radar up again when they restart.

## Sample order

With the SDK demo's `adcbufCfg -1 0 1 1 1`, the DCA1000 stores each pair of samples as `Q0 Q1 I0 I1`. This was checked against the radar's own range profile, computed by the radar from the same chirps. TI's MATLAB reader assumes `I0 I1 Q0 Q1`, which mirrors the spectrum for this data.
