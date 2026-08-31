/*
 * settings.h
 * ---------------------------------------------------------------------------
 * HF Communications Receiver — Digital Noise Reduction Processor
 * ESP32-S3 / PCM1808 (ADC) / PCM5102 (DAC)
 *
 * Central configuration: hardware pins, sample-rate/FFT geometry, buffer
 * sizes, and the runtime-adjustable DSP parameter set.
 *
 * Everything a user is likely to want to tune lives in this one file.
 * ---------------------------------------------------------------------------
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_PORT_NUM  I2S_NUM_0

// =============================================================================
// HARDWARE PIN MAP  (ESP32-S3 side)
// -----------------------------------------------------------------------------
// The ESP32-S3 is configured as I2S MASTER. Both the PCM1808 (ADC) and the
// PCM5102 (DAC) run as I2S SLAVES, sharing one bit clock (BCLK) and one word
// clock (WS/LRCLK). Running both codecs off the *same* master clock domain
// is what keeps capture and playback sample-locked with zero long-term drift
// -- essential for a fixed-latency real-time pipeline.
//
// PCM1808 needs a system clock (SCKI) of 256*Fs or 384*Fs; we provide this on
// the MCLK pin. PCM5102 does not strictly require MCLK (it can derive its
// internal clock from BCK when its SCK pin is grounded), but we drive it from
// the same MCLK line anyway for a clean, single-oscillator design.
//
// Pins below are example assignments only -- the ESP32-S3 GPIO matrix lets
// you route I2S to almost any pin. Avoid strapping pins (0, 3, 45, 46) and,
// if you use native USB, pins 19/20. Plain integers here (not gpio_num_t)
// because that is what I2SClass::setPins() expects.
// =============================================================================

#define PIN_I2S_BCLK        2      // -> PCM1808 BCK,  PCM5102 BCK
#define PIN_I2S_WS           42     // -> PCM1808 LRCK, PCM5102 LRCK
#define PIN_I2S_DOUT         4      // ESP32 ->  PCM5102 DIN
#define PIN_I2S_DIN          5      // PCM1808 DOUT -> ESP32
#define PIN_I2S_MCLK         1      // -> PCM1808 SCKI, PCM5102 SCK

// PCM1808 mode pins (tie in hardware, not driven by firmware):
//   FMT = 0 (I2S, Philips format), MD0/MD1 = 0,0 (Master mode disabled ->
//   PCM1808 as *slave*, since ESP32 is the I2S master here).
// PCM5102 mode pins (tie in hardware, not driven by firmware -- some boards
// label these slightly differently, e.g. "DMP" for DEMP, "XMT" for XSMT,
// and "SCL" for what this project calls MCLK; same signals either way):
//   FMT   -> GND   (I2S/Philips format, matching this firmware's I2S config)
//   DEMP  -> GND   (de-emphasis filter off -- our audio isn't pre-emphasized)
//   FLT   -> GND   (normal/flat reconstruction filter, best THD+N)
//   XSMT  -> 3.3V  (active-high un-mute -- tie high explicitly rather than
//                   leaving it floating, since board-default pull-ups vary)

// Optional status LED (onboard RGB / user LED). Set to -1 to disable.
#define PIN_STATUS_LED      -1

// =============================================================================
// AUDIO FORMAT
// =============================================================================

#define SAMPLE_RATE_HZ      44100
#define I2S_CHANNELS        2               // physical I2S bus is stereo

// PCM1808 delivers L/R on its stereo bus; in this design the receiver's audio
// is fed single-ended into (at minimum) the Left channel. Set MONO_SOURCE to
// select which physical input channel carries the receiver audio:
#define MONO_INPUT_IS_LEFT  1               // 1 = use L input, 0 = use R input

// =============================================================================
// I2S I/O CHUNK SIZE
// -----------------------------------------------------------------------------
// How many stereo frames we read/write from the legacy i2s driver per
// audio_io_task iteration (also used as dma_buf_len in i2s_setup(),
// audio.cpp). Smaller = lower latency, more task-switch overhead; larger =
// the reverse. 256 frames = ~5.8 ms per chunk.
// =============================================================================

#define I2S_CHUNK_FRAMES    256

// =============================================================================
// PHYSICAL CONTROLS  (potentiometers, buttons, rotary encoder)
// -----------------------------------------------------------------------------
// Lets the unit be tuned standalone, with no PC/Serial connection needed.
// See controls.h/controls.cpp for the full behaviour. Layout:
//
//   3 potentiometers -> direct, always-live control of the parameters you
//   most want your hand on while listening: overall NR amount, notch depth,
//   output gain. No menu, no ambiguity: knob position == parameter value,
//   at all times.
//
//   1 rotary encoder + its push-button -> a "select then adjust" interface
//   for the remaining, less frequently touched parameters (oversubtraction,
//   spectral floor, noise tracking speed, notch width/attack/release,
//   speech enhancement, overall attack/release). Press the button to cycle
//   which parameter is selected (also announced over Serial, if connected,
//   for setup convenience -- but turning the encoder to adjust the
//   currently-selected parameter works with nothing attached).
//
//   2 push buttons -> instant toggles for bypass (A/B compare) and the
//   output limiter.
//
// Pin choices avoid the I2S pins above, ESP32-S3 strapping pins (0,3,45,46),
// and the native-USB pins (19,20). The three potentiometer pins are on
// ADC1 (GPIO1-10) so analogRead() works correctly even with WiFi active
// (ADC2, GPIO11-20, conflicts with the WiFi driver; ADC1 does not).
// =============================================================================

#define PIN_POT_NR_AMOUNT     6     // ADC1_CH5
#define PIN_POT_NOTCH_DEPTH   7     // ADC1_CH6
#define PIN_POT_OUTPUT_GAIN   8     // ADC1_CH7

#define PIN_ENCODER_A         11
#define PIN_ENCODER_B         12
#define PIN_ENCODER_BUTTON    13
#define PIN_BUTTON_BYPASS     15
#define PIN_BUTTON_LIMITER    16

// Potentiometer smoothing (one-pole low-pass on the raw 12-bit ADC reading,
// so mechanical/electrical pot noise doesn't "zipper" the gain matrix).
#define POT_SMOOTH_COEFF      0.15f
// Minimum change (post-smoothing, in ADC counts out of 4095) before a pot's
// value is written into g_params -- avoids constant tiny writes from ADC
// dither when a knob isn't actually being touched.
#define POT_DEADBAND_COUNTS   6

// How often (ms) a pot's movement is allowed to print a Serial feedback
// line, at most -- independent of, and much looser than, the deadband
// above. The deadband governs how often the actual parameter value updates
// (kept tight, for smooth audio response); this only throttles how often we
// print about it, so slowly sweeping a knob doesn't flood the Serial
// monitor with a line per tick.
#define POT_PRINT_INTERVAL_MS 150

// Output-gain pot range (dB) -- full clockwise rotation = +OUTPUT_GAIN_POT_MAX_DB.
#define OUTPUT_GAIN_POT_MIN_DB  -12.0f
#define OUTPUT_GAIN_POT_MAX_DB   12.0f

// Button debounce (ms of stable reading required before accepting a change).
#define BUTTON_DEBOUNCE_MS    30

// Encoder step size, as a fraction of each secondary parameter's full
// min..max span, per detent -- i.e. ENCODER_STEP_FRACTION=0.02 means ~50
// detents to sweep a parameter end-to-end. See controls.cpp's
// s_secondary_params table for each parameter's individual min/max.
#define ENCODER_STEP_FRACTION 0.02f

// =============================================================================
// 1.3" SH1106 128x64 OLED (I2C) -- standalone feedback, OLED variant.
// -----------------------------------------------------------------------------
// This build replaces the base project's 4-digit TM1637 with a graphical
// OLED so the whole status picture (full parameter name + value, all three
// pot positions, bypass/limiter state, and live DSP telemetry) fits on
// screen simultaneously instead of being shown one field at a time. See
// display.h/.cpp for the rendering, and README_OLED.md for a side-by-side
// comparison against the TM1637 build.
//
// REQUIRES two external libraries: Arduino IDE -> Tools -> Manage Libraries:
//   "Adafruit SH110X"     (provides Adafruit_SH110X.h / Adafruit_SH1106G)
//   "Adafruit GFX Library" (Adafruit SH110X's rendering dependency)
//
// WIRING: this deliberately reuses the exact two header pins the TM1637
// build used for its CLK/DIO lines, just re-purposed for I2C -- every other
// pin in this file (I2S, pots, encoder, buttons) is IDENTICAL between the
// two builds, so switching between them on the same board is a matter of
// swapping one display module and re-flashing, nothing else on the board
// changes. Most 1.3" SH1106 breakout boards tie their RES pin high on the
// module itself, so no dedicated reset GPIO is needed (OLED_RESET_PIN=-1).
// If your particular module needs one, wire it to a spare GPIO and update
// OLED_RESET_PIN accordingly.
// =============================================================================

#define PIN_OLED_SDA        18     // I2C SDA (was TM1637 DIO in the base build)
#define PIN_OLED_SCL        17     // I2C SCL (was TM1637 CLK in the base build)
#define OLED_I2C_ADDR       0x3C   // common default for 1.3" SH1106 modules --
                                   // some ship at 0x3D instead; check your module's
                                   // datasheet/silkscreen if display_init() warns
                                   // "OLED not found" over Serial
#define OLED_RESET_PIN      -1     // -1 = no dedicated reset GPIO (see WIRING note above)
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define DISPLAY_FLASH_MS    500    // how long the header row inverts after the
                                   // selected secondary parameter changes

// =============================================================================
// FFT / OVERLAP-ADD GEOMETRY
// -----------------------------------------------------------------------------
// FFT_SIZE      : analysis/synthesis window length (samples)
// HALF_FFT      : FFT_SIZE/2 -- size of the internal complex FFT used by the
//                 real-FFT-via-half-size-complex-FFT trick (see fft.cpp)
// NUM_BINS      : FFT_SIZE/2 + 1 -- number of unique real-spectrum bins
//                 (0 .. Nyquist inclusive)
// HOP_SIZE      : samples advanced per frame (50% overlap => HOP = FFT_SIZE/2)
// BIN_HZ        : frequency spacing per FFT bin
// =============================================================================

#define FFT_SIZE            1024
#define HALF_FFT            (FFT_SIZE / 2)      // 512
#define NUM_BINS            (FFT_SIZE / 2 + 1)  // 513
#define HOP_SIZE            (FFT_SIZE / 2)       // 512  (50% overlap)
#define BIN_HZ              ((float)SAMPLE_RATE_HZ / (float)FFT_SIZE)  // ~43.07 Hz

// NOTE ON LATENCY (read this before filing a bug report!):
// A 50%-overlap STFT analysis/synthesis system has an unavoidable structural
// latency of approximately one full window (FFT_SIZE / SAMPLE_RATE) before
// the *first* fully-formed output sample can be emitted, because the Hann-
// windowed overlap-add reconstruction needs contributions from two
// consecutive frames before any output sample is final. At FFT_SIZE=1024 and
// 44.1 kHz that floor is 1024/44100 ~= 23.2 ms, plus a few ms of I2S
// buffering (dma_buf_count/dma_buf_len in i2s_setup(), audio.cpp) and
// processing time.
// That totals to roughly 26-30 ms end-to-end, which is *above* the 20 ms
// target stated in the design brief. This is a fundamental property of
// block-based spectral processing, not an implementation bug -- you cannot
// beat window-length latency with a linear STFT overlap-add scheme. Two
// honest ways to hit <20 ms if that is a hard requirement:
//   1. Drop to FFT_SIZE=512 (11.6 ms window -> ~14-17 ms total), at the cost
//      of coarser frequency resolution (86 Hz/bin instead of 43 Hz/bin),
//      which slightly blunts heterodyne notch precision.
//   2. Keep FFT_SIZE=1024 for the notch/noise-estimation *analysis* path but
//      accept the reported ~27 ms figure -- for SSB/CW listening this is
//      inaudible as delay (it is not an echo, just a fixed pipeline delay).
// Change FFT_SIZE below and rebuild to trade off the two.

// =============================================================================
// RING BUFFERS (samples, must be a power of two)
// =============================================================================

#define RING_BUFFER_SIZE    8192   // mono samples; ~185 ms of headroom at 44.1kHz

// =============================================================================
// TASK / CORE ASSIGNMENT
// =============================================================================

#define CORE_AUDIO_IO       0     // I2S read/write task
#define CORE_DSP            1     // FFT / spectral processing task
#define TASK_PRIORITY_AUDIO 23    // near real-time priority (Arduino loop runs at 1)
#define TASK_PRIORITY_DSP   22
#define STACK_SIZE_AUDIO    4096
#define STACK_SIZE_DSP      12288

// =============================================================================
// RUNTIME-ADJUSTABLE DSP PARAMETERS
// -----------------------------------------------------------------------------
// All fields here can be changed at runtime (e.g. via Serial commands parsed
// in main.ino) without recompiling. Every DSP module reads from the single
// shared instance g_params (declared extern here, defined in the main .ino).
// =============================================================================

struct DSPParams {
    // --- Master ---
    bool  bypass;                 // true = pass audio through unmodified (A/B testing)

    // --- Diagnostics ---
    float test_tone_hz;           // 0 = off; nonzero = replace the output entirely with a
                                   // synthetic sine at this frequency, bypassing the PCM1808
                                   // input path completely. See "tone <hz>" Serial command --
                                   // this isolates whether the ESP32->PCM5102->speaker chain
                                   // works independent of the input side, without touching
                                   // any wiring. "tone 0" turns it back off.

    // --- Noise reduction / spectral subtraction / Wiener stage ---
    float nr_amount;              // 0.0 (off) .. 1.0 (full strength) -- overall wet/dry mix
    float oversubtraction;        // spectral-subtraction over-subtraction factor (alpha), typ. 1.0 - 3.0
    float spectral_floor;         // spectral floor (beta), typ. 0.05 - 0.2 (fraction of noise retained)
    float noise_tracking_speed;   // 0.0 (frozen) .. 1.0 (fast) -- noise floor adaptation rate

    // --- Notch / heterodyne removal ---
    float notch_depth;            // 0.0 (no attenuation) .. 1.0 (full null) at tone center
    float notch_width_hz;         // width of each notch, in Hz
    float notch_attack_ms;        // time constant to engage a notch once a tone is confirmed
    float notch_release_ms;       // time constant to release a notch once a tone disappears

    // --- Speech preservation / enhancement ---
    float speech_enhance;         // 0.0 .. 1.0 -- extra gain applied in the speech-dominant band
    float attack_ms;              // overall gain-matrix attack time constant
    float release_ms;             // overall gain-matrix release time constant

    // --- Output stage ---
    float output_gain_db;         // final output trim, dB
    bool  limiter_enable;         // soft limiter on/off
};

extern DSPParams g_params;

// Sensible defaults (tuned for SSB voice with moderate atmospheric noise).
#define DSP_PARAMS_DEFAULT { \
    /* bypass               */ false, \
    /* test_tone_hz          */ 0.0f, \
    /* nr_amount             */ 0.65f, \
    /* oversubtraction       */ 1.6f, \
    /* spectral_floor        */ 0.10f, \
    /* noise_tracking_speed  */ 0.20f, \
    /* notch_depth           */ 0.90f, \
    /* notch_width_hz        */ 60.0f, \
    /* notch_attack_ms       */ 40.0f, \
    /* notch_release_ms      */ 400.0f, \
    /* speech_enhance        */ 0.25f, \
    /* attack_ms             */ 8.0f, \
    /* release_ms            */ 80.0f, \
    /* output_gain_db        */ 0.0f, \
    /* limiter_enable        */ true \
}

#endif // SETTINGS_H
