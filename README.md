# nrf24-gamepad-link

Turn any USB gamepad into a low-latency nRF24 RC transmitter — ESP32-S3, OLED menu, live trims, axis reversal, and an adjustable throttle curve.

<img src="https://github.com/user-attachments/assets/5eb46359-2449-4e06-b082-e2bd22275353" width="40%">


This project lets a standard USB HID gamepad (wired, or via a wireless dongle — this build was tested with an **EvoFox One S**) act as a full RC transmitter for anything you'd normally fly or drive with a dedicated radio — a drone, an RC car, or any other project listening on an nRF24 link. A USB gamepad is plugged into the ESP32-S3's native USB host port; the board reads every stick, trigger, button, and d-pad press directly from the raw HID report, applies your trims/reverse/curve settings, and streams the result out over an nRF24 radio in a compact, fixed-size packet. An OLED + rotary encoder + 4-button control panel lets you adjust everything on the transmitter itself — no laptop, no phone, nothing else needed.

## Why build this instead of buying a radio

A real RC transmitter is a single-purpose device. This is a general-purpose bridge: any USB gamepad you already own becomes the stick hardware, and the actual "radio" logic — trims, reverse, curves, packet format — lives in firmware you can read, modify, and rebuild for a completely different vehicle without buying new hardware.

## Features

- **Send-on-change radio protocol** — a packet goes out only when something actually changes (plus a fixed-rate keepalive heartbeat so the receiver can tell "idle" from "dead link"), instead of blasting packets on a fixed clock regardless of whether anything moved. Lower latency, lower airtime, lower power draw.
- **Compact, fixed 11-byte packet** — 6 analog channels (raw 0–255, matching the gamepad's native USB HID resolution — no wasted precision, no wasted bits) plus a 16-bit button bitmask and a d-pad byte. No framing, no varint, nothing to parse — the receiver just casts the bytes onto a matching struct.
- **OLED + rotary encoder control panel** — a live status screen (showing real button/d-pad names, not raw hex), per-stick trim, per-axis channel reverse, and a dual-curve throttle mapping, all adjustable from the transmitter itself.
- **Settings persist to flash** (ESP32 Preferences/NVS) — survive a power cycle, no re-configuration needed after every reboot.

## Hardware used

| Component | Role |
|---|---|
| **ESP32-S3** | Main controller. Runs the USB host stack (reads the gamepad's raw HID reports directly, no separate USB host chip needed), applies trim/expo/reverse math, drives the OLED, and talks to the nRF24 radio over SPI. |
| **Ebyte E01-2G4M27D** (nRF24L01+ PA/LNA, 500mW) | The radio. See the comparison section below for why this specific module was chosen over a plain or "100mW" nRF24 module. |
| **0.96" OLED (SSD1306, I2C)** | Displays the live status screen and the full settings menu (trim, channel reverse, throttle curve, arm/reset). |
| **Rotary encoder (with push-button)** | Primary menu navigation — rotate to move the highlighted selection, press to drill in/confirm, long-press to save and exit back to the status screen. |
| **4 push buttons (d-pad layout)** | Fine control inside menu pages — nudging trim values, adjusting the throttle curve, etc. |
| **USB-C female breakout** | Passthrough for the USB gamepad into the ESP32-S3's native USB host pins. |
| **USB-C charging + boost converter module** | Charges the onboard Li-ion cell over USB-C and boosts it back up to a stable 5V rail to power the whole transmitter — the same kind of small board used in USB power banks. |

## The Ebyte E01-2G4M27D vs. a "normal" 100mW nRF24 module

Both are built around the same Nordic nRF24L01+ radio chip and speak the identical Enhanced ShockBurst protocol — the difference is entirely in the power amplifier (PA) and low-noise amplifier (LNA) stage bolted onto that same core chip:

| | Common "100mW" nRF24+PA+LNA module | Ebyte E01-2G4M27D |
|---|---|---|
| Max output power | ~20 dBm (100mW) | **27 dBm (500mW)** — 5× the power |
| Receive sensitivity | Typical nRF24+PA+LNA sensitivity | **-99 dBm** (roughly 3× more sensitive than a PA/LNA-less module) |
| Rated range (line of sight) | ~2–2.5 km | **Up to 5 km** |
| Antenna | SMA, external | SMA, external |
| Build | Common in cheap generic modules; huge variance in real-world PA/LNA quality between clones | Industrial-grade components (precision TCXO crystal), FCC/CE/RoHS certified, more consistent real-world performance than typical generic clones |

Both options are a big step up from the tiny integrated-antenna nRF24L01+ modules (0 dBm, ~1mW, maybe 20-50m realistic range) that ship on most cheap starter kits. The Ebyte module was chosen here for the extra power/sensitivity margin, which matters more once you're flying or driving something out past line-of-sight distances a plain module starts struggling with.

**Important — this is a transmitter-side choice only.** Because every one of these modules implements the exact same Nordic Enhanced ShockBurst protocol, **the receiver end does not need to match the transmitter's module.** A plain low-power nRF24L01+, a 100mW PA/LNA clone, or another E01-2G4M27D will all correctly receive packets from this transmitter — pick whatever you already have on the receiver side. (The one thing to actually match between the two ends is the radio *configuration* — channel, data rate, address, and payload size — not the physical module itself.)

## The control menu

Everything is navigated with the rotary encoder plus the 4 push buttons, entirely on the OLED — no external tool needed:

- **STATUS screen (default)** — live view of both sticks, both triggers, currently-pressed buttons (shown by name, not a raw bitmask), and the d-pad direction. Press the encoder to open the menu; long-press saves and exits back to STATUS.
- **Inside the menu**, rotating the encoder always just moves the highlighted item on the current page (it never changes pages), left/right switches between pages, and a long-press on the encoder saves and exits back to STATUS from anywhere.
  - **TRIM** — select a stick, drill in, and nudge its X/Y center offset with the 4 push buttons.
  - **LIVE DATA** — a read-only view of every value after trim/expo/reverse is applied, useful for confirming your settings actually did what you expected.
  - **CHANNEL INVERT** — flip the direction of any of the 4 stick axes.
  - **THROTTLE CURVE** — see below.
- The menu auto-saves and returns to STATUS after 10 seconds of no input (except on the LIVE DATA page, which is meant to just be watched).

## The throttle curve

A normal stick auto-centers, which is awkward for throttle/altitude — you want to *hold* a position, not have it snap back. So the throttle axis gets its own curve, separate from the plain trim/expo path the other three axes use:

- **Default (no button held)** — a smooth, adjustable non-linear (expo) curve: gentle response near center for fine altitude control, stronger response out near the stick's extremes. The strength (0% = straight linear, 100% = full soft cubic near center) is adjustable on the THROTTLE CURVE menu page, shown live as a graph against a dotted straight-line reference.
- **Aggressive button held** — a steep, near-linear curve for an instant, snappy climb/dive when you need it.

This is recalculated fresh every single loop tick from the stick's *current* position and the button's *current* state — there's no ramping or "memory" of where the stick was a moment ago, so whichever curve is active always matches exactly what your thumb is doing right now.

## Receiver

The receiver side is intentionally minimal here: an **Arduino Nano (or Uno) + any nRF24L01 module**, decoding the full 11-byte packet and printing a complete, human-readable dump straight to the Serial Monitor — no servo/PWM output, just the transmitter's full state for testing and debugging the radio link. See `receiver_serial_debug.ino`.

Every packet that actually arrives gets its own line, showing:
- a running packet count and the transmitter's own `seq` number
- whether the packet was live gamepad data or just a heartbeat, and the (display-only) armed/safe flag
- a running dropped-packet count, worked out from gaps in `seq`
- all 4 stick axes (LX/LY/RX/RY) and both analog triggers (LT/RT), raw 0–255
- the d-pad direction as text (`up`/`down`/`left`/`right`/`neutral`) rather than a raw hat value
- every currently-held button by name (`X`, `A`, `LB`, `SELECT`, `ML`, …), decoded from the 16-bit bitmask, plus the raw hex value for cross-checking

If no packet arrives for more than 500ms, it also prints a `LINK LOST` line (rate-limited to once a second so a dead link doesn't spam the monitor). Because this sketch is purely for bench-testing the radio link, it deliberately does **not** rate-limit the normal per-packet output the way the transmitter's own debug print does — you're meant to watch it while moving one stick/button at a time, so seeing literally every packet is the point.

## Using a different gamepad

This project was built and tested with an **EvoFox One S** controller, and the firmware currently reads that pad's stick/button/trigger data from **fixed byte offsets** inside its raw USB HID input report. Every gamepad model lays its report out slightly differently — not just "different button order," but genuinely different byte positions for the same piece of data — so plugging in a different pad without remapping these offsets will read garbage (or nothing) for at least some inputs, even though the USB connection itself will work fine.

The good news: this only needs to be figured out **once per controller model**, and the firmware already has a debug mode built specifically for this.

### How to map a new controller

1. **Turn on raw byte debugging.** In the transmitter sketch, find:
   ```cpp
   #define DEBUG_PRINT      0
   ```
   and
   ```cpp
   #define DEBUG_RAW_BYTES  0
   ```
   Set both to `1`, then re-flash and open the Serial Monitor.

2. **Watch the raw report change one input at a time.** With your new controller plugged into the transmitter's USB-C port, the Serial Monitor will print the full raw HID report every time it changes — e.g.:
   ```
   RAW len=8  00 00 08 80 80 80 80 00
   ```
   Press **one button at a time** (release it before pressing the next) and note which byte position changes, and by how much. Do the same for each stick (push it to each extreme, then let it re-center) and each trigger.

3. **Work out the layout.** You're looking to identify:
   - Which byte holds the **d-pad** (usually a small value like 0-8 representing a direction/neutral)
   - Which **one or two bytes** hold the **button bitmask** (a value that changes in a binary/bitwise pattern as you press different buttons — e.g. pressing button "A" alone might always add exactly `0x02` to that byte, regardless of what else is pressed)
   - Which bytes are **LX, LY, RX, RY** (these are the ones that sit near a mid-value like `0x80` at rest and sweep from roughly `0x00` to `0xFF` as you move each stick)
   - Which bytes are **LT, RT** (rest at `0x00` and rise toward `0xFF` as you pull each trigger)

4. **Update the layout in code.** The transmitter stores all of this in one small struct (`ReportLayout` — look for `layout.btnLowByte`, `layout.btnHighByte`, `layout.dpadByte`, `layout.lxByte`, `layout.lyByte`, `layout.rxByte`, `layout.ryByte`, `layout.ltByte`, `layout.rtByte`). Update these byte-offset values to match what you found in step 3 — this is the only code change needed to support a new controller.

5. **Re-check button bit assignments.** The button byte(s) are read as a bitmask (`BTN_X`, `BTN_A`, `BTN_B`, etc. — see the `#define BTN_*` block). If your controller's physical buttons don't line up with the same bit positions the EvoFox One S uses, update those defines too so the on-screen button names (and any future per-button mapping) point at the right physical button.

6. **Turn debugging back off** (`DEBUG_PRINT` and `DEBUG_RAW_BYTES` back to `0`) once everything checks out on the LIVE DATA menu page — leaving them on costs flash space and CPU cycles for no benefit once you're done calibrating.

If you map a new controller and get it working, consider opening a pull request with its `ReportLayout` values — over time this could grow into a small library of known-working configs instead of everyone re-deriving their own from scratch.

## License

MIT — free to use, modify, and share. Attribution appreciated.
