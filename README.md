```markdown
# STM32F103C8T6 USB-MIDI Matrix Arranger — Full Feature Build

This repo contains a PlatformIO / Arduino project for STM32F103C8T6 (Blue Pill) implementing a near‑professional MIDI keyboard arranger:

Highlights:
- Note matrix 8x8 on MCU pins (PB0..PB15) -> up to 64 keys (mapable to 61 keys)
- Separate function matrix (intro/fill/var/ending/break/keyboardset)
- 4 keyboard layers (keyboardset) with Program Change send per set
- Tap‑Chord tempo: tap a chord (>=2 notes) 2 or 3 times to set tempo
- Backup Tap button (PC13)
- Rotary encoder used to select and edit parameters (tempo, octave, transpose, PB-X rate/depth)
- Per-parameter ON/OFF buttons
- Master Volume (CC7) and Balance/Pan (CC10) pots
- Pitchbend X/Y (14-bit for X, mapped; Y -> CC1)
- Tremolo LFO for X
- MIDI Clock (24 ppqn) & Start/Stop/Continue support
- Panic / All‑Notes‑Off support (sends CC123 + note off)
- EEPROM persistence for settings and mappings (saved to emulated EEPROM)
- Mapping editor (long-press function key + encoder to change mapping)
- Robust debouncing and chord/tap handling
- PlatformIO build + GitHub Actions CI

IMPORTANT:
- Do NOT feed ADC pins from >3.3V. All pots/sensors must be 3.3V referenced.
- Each matrix switch should have a diode in series (hardware) to avoid ghosting.
- For runtime user feedback (mapping results, saved settings) use Serial (USB CDC) at 115200.
- If your Blue Pill variant does not provide EEPROM emulation, I will adapt storage to flash page usage (FlashStorage).

Build:
- platformio run -e bluepill_f103c8

If you want:
- I can add optional CD4051 / 74HC4067 support for many EQ pots (requires pin mapping) — do not change pins unless you approve.
- I can add TinyUSB backend if USB-MIDI enumeration issues occur on your hardware.

```
