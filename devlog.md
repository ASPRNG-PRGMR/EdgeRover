# EdgeRover Devlog

Running log of the ESP-NOW control rebuild — bugs, root causes, fixes. Newest at the bottom.

---

### The motors never actually stopped at center

First symptom: direction pins only went low at *exactly* raw value 2048, everything else drove a motor. Root cause turned out to be two layers deep:

1. The receiver's stop logic assumed a joystick centered on 2048. The real hardware centered closer to ~2700–2950.
2. The transmitter had a full calibration system (`applyCalibration`, min/center/max/deadzone) — but nothing ever called `inputs_begin_calibration()`/`inputs_finish_calibration()`. It had been running on hardcoded defaults the entire time, silently pretending to be calibrated.

**Lesson:** an unused calibration path with defaults that don't match the actual hardware is worse than no calibration at all — it looks like a working system while quietly producing wrong output. Either wire the calibration routine up and call it, or don't ship code that pretends it ran.

**Fix (early version):** switched motor control to a tri-band scheme instead of proportional mapping — full reverse / stop / full forward, since there was never any PWM/speed involved at that stage anyway.

---

### `packet.h` field rename broke the build on both sides

Renaming `throttle`/`steering` to `left`/`right` in `ControlPacket` meant every file that touched those fields needed updating too — not just `outputs.cpp`, but `receiver.ino`'s debug print, which was easy to miss since it wasn't part of the file set originally shared.

**Lesson:** a packed struct sent raw over the wire (`__attribute__((packed))`) is a contract, not a convenience. Grep the whole repo for every field name before renaming, on both the transmitter and receiver sides — the compiler will only catch it if the field is actually referenced in a file you remembered to check.

---

### Display: backlight on, nothing drawn

Went through a few rounds on this one:

1. **GPIO2 conflict.** `PIN_TFT_RS` (DC) was wired to GPIO2, which is the onboard LED pin on most ESP32 devkits. SPI DC toggling was flickering the LED and loading the line enough to potentially corrupt the init sequence. Moved DC to a free pin.
2. **Still blank.** Confirmed with a standalone test sketch (bypassing the rest of the project) that it wasn't a project-code bug — still white with just `tft.init()` + `fillScreen()`.
3. **Wrong resolution assumed.** Guessed 240×240 (common ST7789 size), actual panel was 240×320.
4. **Wrong chip entirely.** 240×320 SPI panels are essentially never ST7789 — that resolution belongs to a different controller family. Turned out to be an **ILI9225**. Same wiring, different init command sequence, different library (`Adafruit_ILI9225` instead of `Adafruit_ST7789`). Swapping the driver fixed it immediately.

**Lesson:** "240x320 SPI TFT" is a driver hint, not a spec. Check the chip markings or listing before assuming the controller family from resolution alone — backlight-on/nothing-drawn is a driver mismatch symptom far more often than a wiring one.

---

### ADC2 + ESP-NOW = unreliable pot readings

The speed potentiometer sits on GPIO26, which is an ADC2 pin. ESP-NOW keeps the Wi-Fi radio active continuously, and ADC2 shares hardware with the Wi-Fi driver — so ADC2 reads are undefined/unreliable any time Wi-Fi is on, which for this project is always.

**Status:** flagged, not yet fixed — GPIO26 was already committed to in the physical build. Left as-is for now with a loud warning in `inputs.h`; the fix (move to an ADC1 pin, 32–39) is straightforward whenever it's worth the rewire.

**Lesson:** know which ADC bank a pin belongs to *before* wiring, if Wi-Fi/ESP-NOW/BLE will be active. ADC1 (32–39) is always safe under Wi-Fi load; ADC2 (0, 2, 4, 12–15, 25–27) is not.

---

### Rotary encoder: direction flips, then jitter at rest

Steering felt directionally correct but occasionally counted the wrong way on a turn. First fix pass:

- Added a settle delay + double-sample of DT inside the ISR before trusting a CLK edge, on the theory that CLK and DT don't transition in perfect lockstep and an edge fired mid-transition could read DT ambiguously.

This measurably helped (turning right consistently moved right), but a *new* symptom showed up: the displayed steering value would jitter between adjacent values while the knob wasn't moving at all.

**Root cause:** time-based debounce (even with a settle-check) can only reject edges that look locally noisy — it can't catch a short, clean-looking glitch from vibration or contact chatter that individually passes every check. Each glitch was independently valid by the debounce's own rules.

**Real fix:** replaced edge-interrupt + debounce entirely with a **detent-to-detent state machine** (Buxton-style rotary decoder) that tracks CLK+DT together as one combined state. A step is only counted after the encoder passes through a complete, valid transition sequence and lands back at a rest state. A glitch that doesn't walk the whole sequence just falls back to idle and produces nothing — no timing guesswork required.

**Lesson:** mechanical rotary encoders need a state machine, not a debounce timer. Those two techniques solve different problems — debounce rejects edges that arrive too close together in time; a state machine rejects edges that don't form a complete, legitimate transition, regardless of timing. If you get direction flips *and* jitter-at-rest from the same encoder, you're almost certainly looking at the second problem, not the first.

**Side effect:** the state machine derives direction differently than the old edge-compare logic did, so left/right came out swapped after the rewrite. Fixed with the `ENCODER_INVERTED` flag rather than touching the decode logic itself.

---

### Pin conflicts introduced by the pot/encoder/display rewrite

Adding the potentiometer (GPIO26), encoder (GPIO12/13/14), and display (GPIO2/4/5/15/18) pins collided with the pre-existing button pins (`PIN_BUTTON_B` was GPIO26, `PIN_BUTTON_D` was GPIO14). Moved both buttons to free GPIOs (16, 17).

**Lesson:** when adding a new peripheral to an existing pinout, diff the full pin list before wiring, not just the new pins in isolation — collisions show up as one peripheral silently overriding another's `pinMode()`/`digitalWrite()` calls, not as a compile error.

---

### File layout: transmitter and receiver files ended up mixed together

`outputs.h`/`outputs.cpp` (receiver-side, drives the TB6612FNG) briefly ended up sitting in the transmitter's folder alongside `inputs.*`/`display.*`/`espnow_tx.*`. Both sketches also need their own identical copy of `packet.h`, since Arduino sketch folders don't share headers.

**Lesson:** keep the transmitter/receiver split enforced by folder structure, not just by convention — it's easy for debug files to drift into the wrong directory once you're iterating fast on one board at a time.

