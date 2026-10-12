# led-strip

Generated firmware for **Atech 14-Port Board** (`14port`).

## Modules

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
