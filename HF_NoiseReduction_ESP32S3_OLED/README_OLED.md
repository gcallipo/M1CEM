# HF Noise Reduction — SH1106 OLED variant

This is a sibling build of the base `HF_NoiseReduction_ESP32S3` project with
the 4-digit TM1637 status display replaced by a 1.3" SH1106 128x64 I2C OLED.
**The DSP/audio pipeline is byte-for-byte identical** — `fft.*`, `matrix.*`,
`noise.*`, `wiener.*`, and `heterodyne.*` are unmodified copies of the base
project's files. Only three things differ:

1. **`display.h` / `display.cpp`** — completely rewritten for the OLED.
2. **`controls.cpp`** — the secondary-parameter table now stores full names
   and printf-style value/unit formats instead of the TM1637 build's
   1–9 index + `DISP_FMT_INT/FRAC1/FRAC2` scheme, and hands the whole
   status picture to the display every tick instead of three separate
   flash/value/bypass calls.
3. **`settings.h`** — the TM1637 pin/config block is replaced by an I2C
   pin/address block; **`audio.h`/`audio.cpp`** gain a small `DspStatus`
   struct (a smoothed incoming-audio level) so the OLED has a live VU
   meter to show, which the TM1637 build never tracked at all.

Everything else — I2S wiring, sample rate, FFT size, pot/encoder/button
pins and behaviour, Serial command set — is unchanged from the base
project. See the base project's `README.md` for the DSP/matrix math,
block diagram, CPU/memory budget, and I2S bring-up notes; none of that is
repeated here.

## Required libraries

Arduino IDE → Tools → Manage Libraries → install:

- **Adafruit SH110X** (provides `Adafruit_SH110X.h` / `Adafruit_SH1106G`)
- **Adafruit GFX Library** (Adafruit SH110X's rendering dependency)

(The base TM1637 build's one dependency, the TM1637Display library, is
not needed here.)

## Wiring

| Signal        | ESP32-S3 pin | Notes                                      |
|---------------|:------------:|---------------------------------------------|
| I2C SDA       | 18           | was the TM1637's DIO pin in the base build  |
| I2C SCL       | 17           | was the TM1637's CLK pin in the base build  |
| OLED VCC      | 3.3V         |                                              |
| OLED GND      | GND          |                                              |

Everything else (I2S, pots, encoder, buttons) uses the exact same pins as
the base build — see `settings.h`. Reusing the TM1637's two header pins
for I2C means switching between the two builds on the same board is just
"swap the display module, flash the other firmware," nothing else changes.

`OLED_I2C_ADDR` in `settings.h` defaults to `0x3C`, the common address for
1.3" SH1106 modules — some ship at `0x3D` instead; check your module's
silkscreen/datasheet if `display_init()` prints a "not found" warning over
Serial at boot. Most 1.3" SH1106 breakouts tie RES high on-board, so no
dedicated reset GPIO is wired (`OLED_RESET_PIN = -1`); if yours needs one,
wire a spare GPIO and update that define.

## Dashboard layout

```
+----------------------------+
| 3/9 Noise Tracking         |  <- selected parameter (inverts briefly
| 0.20                       |     on selection change); big value line
| NR    [======......] 0.65  |  <- all three pots, always visible,
| NOTCH [========....] 0.80  |     no menu/mode switch needed
| GAIN  [=====.......] +2.0  |
| LIMITER: OFF          41%  |  <- limiter state + DSP CPU load
| IN [=========........]     |  <- VU meter, INCOMING audio level
+----------------------------+
```

Bypass isn't repeated in the footer — while bypass is on, it already takes
over the big value line as `BYPASSED`, which is a lot harder to miss than
one more line of small footer text.

**The VU meter** replaced an earlier version of this build's footer, which
showed VAD-activity and confirmed-tone-count flags instead. Those turned
out to be low-value at a glance (a boolean and a small integer that rarely
change), whereas a live level bar is immediately useful: it confirms audio
is actually reaching the ADC, gives a rough sense of gain-staging, and
moves continuously enough to double as a "the OLED/I2C link is still
alive" heartbeat. It's read from the raw incoming samples (`hop_in`),
*before* the bypass/test-tone/pipeline branch in `dsp_task`, so it stays
live no matter what mode the firmware is in — including bypass, where the
big value line has switched over to showing `BYPASSED`. Ballistics are a
classic fast-attack/slower-release VU-style smoothing (see `VU_METER_*` in
`audio.cpp`), mapped from `VU_METER_MIN_DB` (bar empty, default −50 dBFS)
up to 0 dBFS (bar full).

While bypass is engaged, the big value line shows `BYPASSED` instead of a
parameter value, since none of the controls have any audible effect in
that state (same rationale as the base build's dedicated "----" bypass
indicator).

## Comparing the two builds

Both are provided so you can wire up either display and judge for
yourself which suits your enclosure/use case better. A few honest
trade-offs, since "just tell me which is better" depends on what you
value:

- **Information density**: the OLED shows all three pot positions, the
  selected secondary parameter (full name, not a numbered index you have
  to cross-reference), bypass/limiter state, and live VAD/tone-count/CPU
  telemetry simultaneously. The TM1637 can only show one short field at a
  time and needs a printed numbered list taped to the enclosure.
- **Power draw**: a 128x64 OLED with a meaningful fraction of its pixels
  lit draws more current than a 4-digit 7-segment display, which matters
  more if this is battery-powered.
- **Readability at a glance / in bright sunlight**: 7-segment digits at
  high brightness are easier to read from an angle or in direct sun than
  a small graphical OLED, which is a real consideration for a receiver
  that might sit on a desk near a window or be used in the field.
- **Parts cost and wiring complexity**: TM1637 modules are typically
  cheaper and the 2-wire protocol is marginally simpler than I2C, though
  both are trivial on this hardware.
- **Firmware footprint**: Adafruit_GFX + Adafruit_SH110X pull in more
  flash/RAM than TM1637Display, though nowhere close to being a concern on
  an ESP32-S3.

## A note on verification

This variant hasn't been compiled or run on real hardware in the process
of writing it (no ESP32 toolchain or the Adafruit libraries were available
in the environment used to generate it) — unlike the base project, whose
README documents specific compile/link verification steps taken for its
modules. Before flashing: double-check the I2C address, confirm
`Adafruit_SH1106G`'s constructor/`begin()` signature against whatever
library version you install (it has changed slightly across releases),
and watch the Serial monitor at boot for the "OLED not found" warning.
