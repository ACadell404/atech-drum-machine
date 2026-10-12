# drum-machine

Generated firmware for **Atech 14-Port Board** (`14port`).

## Modules

- **Rotary Encoder Knob** (`rotary_encoder`) as `step_knob` on `port_1, port_2`
- **Rotary Encoder Knob** (`rotary_encoder`) as `drum_knob` on `port_3, port_4`
- **Drum Synth (I2S Speaker)** (`drum_synth`) as `drums` on `port_5, port_6`
- **NeoPixel 3x3 Grid** (`neopixel`) as `leds` on `port_7`
- **ST7735 TFT Color Display (160x80)** (`st7735_tft`) as `screen` on `port_9, port_10`
- **Button** (`button`) as `btn_play` on `port_11`
- **Button** (`button`) as `btn_shift` on `port_13`
- **Addressable LED Strip (external power)** (`led_strip`) as `strip` on `port_14`

## Build & flash

```bash
pio run                         # build
pio run -t upload               # flash
```

Or from Python via the Atech SDK:

```python
from atech import Project
Project.load('project.yaml').upload()
```
