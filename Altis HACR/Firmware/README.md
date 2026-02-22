# RP2040 ESC PWM Controller

## Wiring
- GIO12 → ESC signal
- GPIO15 → Button → GND
- GPIO26 → Pot wiper
- 3V3 → Pot
- GND → common ground

## Build
```bash
mkdir build
cd build
cmake ..
make
