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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.# EdgeRover Devlog

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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.# EdgeRover Devlog

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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.# EdgeRover Devlog

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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.# EdgeRover Devlog

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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.# EdgeRover Devlog

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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.# EdgeRover Devlog

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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.# EdgeRover Devlog

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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.# EdgeRover Devlog

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

---

### ESP32 resetting under motor load — despite a fully isolated battery domain

Symptom: with a dedicated 6V pack for the ESP32 and a separate 12V pack for the motors, the ESP32 still reset partway through the PWM ramp — reliably, at the same duty cycle (~27/255) every time. This looked like it should be impossible: the whole point of splitting the batteries was to remove any electrical path for motor current to reach the ESP32's supply. Assumed at first this was a supply-sag issue (weak NiMH cells under load, or a resistive ground return dragging the ESP32's effective VIN-GND differential down) and started down that path.

**The test that actually cracked it:** powered the ESP32 from a PC's USB port instead of the 6V pack — completely independent of *both* battery packs — and reran the same PWM ramp. It still reset at the same threshold. That single test eliminated the entire "supply can't source enough current" theory in one shot: USB power from a PC is about as stable and isolated as it gets, so if the reset still happens, the batteries and BEC were never the problem.

**Second clue, easy to miss:** the reset reason printed was `rst:0x1 (POWERON_RESET)`, not a brownout code. A voltage-sag/brownout theory should produce a brownout reset reason. A power-on reset pointed somewhere else entirely — something was pulling the `EN` (reset) pin low directly, not starving the chip's supply.

**Root cause:** motor switching noise (brush commutation + back-EMF from the DC motors) coupling onto the ESP32's `EN` pin via the shared ground path between the two battery domains. `EN` on most ESP32 DevKit boards is a lightly-pulled-up, high-impedance line by design (so the auto-upload circuit can yank it low quickly) — which also makes it one of the most noise-sensitive pins on the board. Even with two separate batteries, a shared ground is still an electrical connection, and if that ground wiring isn't a clean single-point (star) topology, motor return current can inject enough transient noise at the ESP32's ground reference to glitch `EN` low.

**Fix, applied in order, each one narrowing the problem further:**
1. **Capacitor (0.1–1µF) directly across `EN`→`GND`** — filters fast transients trying to yank the pin low. This alone moved the reset-free threshold from ~27/255 to ~99/255, confirming the EN-pin noise theory and showing the fix direction was right, even though it wasn't sufficient alone.
2. **Snubber capacitors (0.1µF) across each motor's terminals**, soldered as close to the motor body as possible — kills brush/commutation noise at the source instead of only filtering it downstream at `EN`.
3. **Rebuilt the ground wiring as a genuine single-point star** (all grounds — battery negatives, BEC, TB6612FNG, ESP32 — landing on one soldered node, not daisy-chained through each other) instead of stitching two separate ground domains together at an arbitrary point.
4. **Simplified the power architecture at the same time:** rather than maintaining two fully separate batteries with a fragile inter-domain ground join, switched to one 12V pack + one 5A BEC, where the BEC output feeds only logic-level current (ESP32 + TB6612FNG `VCC`) and `VM` (motor power) comes straight off the 12V pack, bypassing the BEC entirely. This keeps the ESP32 isolated from motor current without the complexity of reconciling two independent ground systems.

Combined, this fully resolved the resets — confirmed stable across the full 0–255 PWM range.

**Lesson:** "separate battery pack" only isolates you from noise/current on the *supply* side. If the two domains still share a ground (and they almost always must, for signal reference), that ground connection is a real electrical path, and poor topology there (daisy-chained instead of star, thin/long wire, unshielded routing near motor leads) can reproduce symptoms that look exactly like a voltage-sag problem, right down to resetting at a repeatable current/PWM threshold — while the actual battery and regulator are completely innocent. When a reset happens under load, check the reset reason (`rst:0x..`) before assuming brownout — a plain `POWERON_RESET` under load is a strong hint to look at `EN`-pin integrity and ground topology, not supply capacity. Powering the suspect board from a source with zero relationship to the rest of the circuit (like PC USB) is one of the fastest ways to conclusively rule the power domain in or out.

---

### Tank-turn steering + in-place pivot — `ControlPacket` v3, signed PWM

Manual control had two real gaps going into competition prep: no way to pivot in place, and turning while moving felt too soft — the inner wheel was floored at a fixed 35% minimum (`MIN_INNER_FACTOR`) rather than ever reaching zero or reversing, so even a full steering-lock turn stayed a wide arc.

**Root cause of both, really the same gap:** `ControlPacket.leftPWM`/`rightPWM` were `uint8_t` (0–255) — unsigned, forward-only by construction. There was no way to represent "drive this wheel backward" anywhere in the protocol, so neither a tight turn nor an in-place pivot was reachable no matter how the mixing math was tuned.

**Fix:** bumped `PACKET_VERSION` to `3` and changed `leftPWM`/`rightPWM` to `int16_t` (-255..255), sign = direction, magnitude = duty. This let `computeDrive()` in `inputs.cpp` replace the old floor-based tapering with a through-zero formula (`innerFactor = 1.0 - 2×turnFactor`): full forward at center, exactly zero at half steering lock, ramping into genuine reverse toward full lock. `outputs.cpp` now reads the sign of each field to set the TB6612FNG's direction pins, instead of hardcoding `AIN1`/`BIN1` HIGH on every packet. In-place pivoting came along almost for free: when the throttle pot is idle and the steering encoder is deflected, `computeDrive()` now drives both wheels at equal magnitude, opposite direction, scaled by steering angle — reusing the existing pot + encoder instead of needing new hardware.

Reverse (as a standalone drive mode, independent of pivot) was considered and deliberately dropped — button pins A-D and the mode switch are all already spoken for or reserved, and the only free momentary input left was a joystick module's click-switch, which would have worked, but reverse itself wasn't judged useful enough for this track layout to justify adding a new control (and a new failure mode next to the arm/disarm switch) for it.

**Lesson:** an unsigned "duty cycle" field in a wire protocol is a forward-only assumption baked in at the struct level, not just a UI limitation — no amount of tuning the mixing math on top of it can produce a real pivot or reverse until the underlying field can actually represent direction. Worth deciding signed-vs-unsigned deliberately up front on any field that might ever need to mean "the other way," rather than retrofitting it once the mixing logic already assumes 0 is the floor.

---

### AprilTag follower: proofread pass, servo tilt + Pythagoras range, sender task

First full read of `vision_control/apriltag_follower/` before any hardware existed to test it on. It would not have built, and if it had, it would not have been safe:

1. **`constants.h` had lost its `#ifndef` guard** — a stray `#endif` with no matching `#if`, so the very first include failed.
2. **The sketch lived one folder too deep** (`apriltag_follower/apriltag_follower/apriltag_follower.ino`) with all the `.cpp`/`.h` files in the parent. Arduino only compiles what sits in the sketch folder itself, so none of the modules were part of the build.
3. **Uncalibrated distance mapped to full speed.** `FOCAL_LENGTH_PX` defaulted to `0`, which the control code turned into "distance = infinite", which the speed map turned into 100 % duty with no stop condition. Now an uncalibrated build caps forward PWM (`UNCALIBRATED_PWM_CAP`) and the estimate still produces a stop.
4. **"Target stopped moving" heuristic stopped the rover whenever the person stood still** — even 2 m away. It predated a working distance estimate; removed.
5. **Signed PWM printed with `%u`.** Cosmetic, but reverse values showed as ~4 billion.
6. **Receiver failsafe vs. detection time.** The receiver drops to failsafe 200 ms after the last packet, and AprilTag detection at VGA takes ~100-150 ms per frame on its own — one slow frame and the motors cut out. Packets now come from a dedicated 50 Hz FreeRTOS task on core 0 that repeats the last command, slew-limits PWM changes (softer starts are also kinder to the EN-pin noise problem), and zeroes the output if the vision loop stops feeding it for 400 ms.
7. **LEDC timer collision.** esp32-camera generates the OV3660's 20 MHz XCLK on LEDC timer 0. The Arduino core 3.x `ledcAttach()` allocator only tracks its own channels, so attaching a 50 Hz servo through it can silently reprogram timer 0 and freeze the camera. The servo uses the IDF `ledc` driver on timer 1 / channel 2 explicitly.
8. **`set_framesize()` after init in grayscale mode** is a documented way to get "frame buffer size mismatch" capture failures with esp32-camera. Removed; frame size is fixed in the init config. Added the OV3660 `set_vflip(1)` the stock example applies (the sensor is mounted upside-down on these boards).

**The bigger finding was physical, not code.** The goal is "stop 15 cm behind the person" with the tag on a back pocket. Ran the geometry (`vision_control/model/follow_geometry.py`): camera ~12 cm up, tag ~85 cm up, so at 15 cm the line of sight is ~78° above horizontal — and a tag hanging flat on a pocket is then viewed ~78° off-normal. tag36h11 stops decoding somewhere past ~65-75°. Closest distance that actually works with that mounting is ~40 cm. Pitching the tag ~40° downward on a wedge, or masting the camera to ~50 cm, makes every distance from 10 cm out decodable. This is also *why* the servo matters: the pinhole range at the heels reads ~75 cm (slant), and only the tilt angle + Pythagoras turns that into 15 cm (ground).

**Lesson:** run the viewing-angle / pixel-size numbers before choosing where a fiducial goes. A follower's hardest case is the close one, and "close" for a floor-level camera means "looking almost straight up at something edge-on" — no amount of detector tuning fixes a tag you can only see edge-on. And when a firmware folder has never been compiled, assume nothing: check the guard, the folder layout and the uncalibrated-defaults path before reading the control math.
