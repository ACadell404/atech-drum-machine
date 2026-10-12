# Atech Drum Machine

A 16-step, 6-voice drum machine for the Atech 14-port board (ESP32-S3), built with the
[atech](https://pypi.org/project/atech/) Python SDK.

## Projects

| Folder | What it is |
|---|---|
| `drum-machine/` | The drum machine: firmware project (`project.yaml`), custom `drum_synth` module, and a Web Serial control page (`interface.html`) |
| `led-strip/` | Addressable LED strip module (external power). The drum machine uses it to flash the strip on kick/snare/tom hits |
| `lighting/` | Tunable-white light module (work in progress) |

## Hardware (drum-machine)

| Ports | Module | Role |
|---|---|---|
| 1–2 | Rotary encoder `step_knob` | Move cursor across steps; press toggles a step. SHIFT: tempo / clear row |
| 3–4 | Rotary encoder `drum_knob` | Pick drum row; press plays it. SHIFT: velocity / load demo beat |
| 5–6 | I2S speaker (`drum_synth`) | Drum sounds |
| 7 | NeoPixel 3×3 grid | Hit and beat lights |
| 9–10 | ST7735 TFT 160×80 | Step grid |
| 11 / 13 | Buttons | PLAY / SHIFT |
| 14 | LED strip data (via level shifter, own 24 V supply) | Beat-synced flashes |

## Build and flash

```bash
pip install atech
python -m atech.cli upload drum-machine --port COM9
```

Then serve the control page and open it in Chrome (Web Serial):

```bash
python -m http.server 8765 --directory drum-machine
```

Open http://localhost:8765/interface.html and click **Connect board**.
