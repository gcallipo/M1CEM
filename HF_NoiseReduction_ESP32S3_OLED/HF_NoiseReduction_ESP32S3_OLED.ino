/*
 * HF_NoiseReduction_ESP32S3_OLED.ino
 * ---------------------------------------------------------------------------
 * Digital Noise Reduction Processor for HF Communications Receivers
 * ESP32-S3 + PCM1808 (I2S ADC) + PCM5102 (I2S DAC)
 *
 * OLED VARIANT: identical DSP/audio/physical-controls to the base project,
 * with the 4-digit 7-segment TM1637 display replaced by a 1.3" SH1106
 * 128x64 graphical OLED (I2C, via the Adafruit_SH110X + Adafruit_GFX
 * libraries) for a richer standalone readout -- full parameter names, all
 * three pot values as gauges, bypass/limiter state, and live VAD/tone/CPU
 * telemetry, all visible at once. See README_OLED.md for what's different
 * from the base (TM1637) project, wiring, and a side-by-side comparison;
 * DSP theory, matrix math, latency/CPU/memory budgets, and the I2S
 * hardware bring-up notes are unchanged and covered in the base project's
 * README.md instead of being duplicated here.
 *
 * See README.md in this project folder for:
 *   - Full DSP / matrix-mathematics explanation
 *   - Block diagram
 *   - CPU and memory budget
 *   - Per-stage design rationale
 *   - Testing & validation notes
 *   - Suggested future additions (ANR variants, AGC, EQ, noise blanker,
 *     waterfall/OLED display, SD recording, Bluetooth audio, ...)
 *
 * All of the actual signal processing lives in the other modules:
 *   settings.h    - hardware pins, geometry, runtime-adjustable parameters
 *   matrix.h/.cpp - complex numbers, diagonal "gain matrix" ops, small dense
 *                   matrix utilities, LMS adaptive vector
 *   fft.h/.cpp    - real FFT via half-size complex FFT, Hann window
 *   noise.h/.cpp  - VAD + adaptive noise-floor tracking
 *   wiener.h/.cpp - decision-directed Wiener filter + spectral subtraction
 *   heterodyne.h/.cpp - tone detection, multi-frame tracking, notch matrix
 *   audio.h/.cpp  - I2S duplex driver, ring buffers, Core0/Core1 tasks
 *   controls.h/.cpp - potentiometers, buttons, rotary encoder (standalone
 *                     tuning with no PC/Serial connection needed)
 *   display.h/.cpp  - 1.3" SH1106 128x64 OLED (I2C, via Adafruit_SH110X):
 *                     shows the selected parameter's full name/value, all
 *                     three pot values, bypass/limiter state, and live
 *                     VAD/tone/CPU telemetry at a glance
 *
 * This file only wires everything together and provides human interfaces
 * (Serial commands + physical controls + periodic status reporting).
 * ---------------------------------------------------------------------------
 */

#include <Arduino.h>
#include "settings.h"
#include "audio.h"
#include "controls.h"
#include "display.h"

// The single shared, runtime-adjustable parameter set. Every DSP module
// reads from this instance (declared extern in settings.h).
DSPParams g_params = DSP_PARAMS_DEFAULT;

static uint32_t s_last_report_ms = 0;
#define REPORT_INTERVAL_MS 2000

// -----------------------------------------------------------------------------
// Serial command interface
// -----------------------------------------------------------------------------
// Simple line-based "name value" commands, e.g.:
//   nr 0.7            -> set nr_amount
//   notch_depth 0.95
//   notch_width 80
//   tracking 0.3
//   floor 0.08
//   oversub 2.0
//   speech 0.4
//   attack 10
//   release 100
//   gain -3
//   bypass 1          -> 1 = bypass on, 0 = off
//   limiter 0
//   tone <Hz>         -> diagnostic: output a synthetic sine at <Hz>,
//                        ignoring PCM1808 input entirely. "tone 0" = off.
//                        Use this to check the ESP32->PCM5102->speaker
//                        chain in isolation, e.g. "tone 1000".
//   status            -> print current parameters
//   help              -> list commands
// -----------------------------------------------------------------------------

static void print_help() {
    Serial.println(F("Commands: nr <0-1> | notch_depth <0-1> | notch_width <Hz> |"));
    Serial.println(F("  tracking <0-1> | floor <0-1> | oversub <val> | speech <0-1> |"));
    Serial.println(F("  attack <ms> | release <ms> | gain <dB> | bypass <0/1> |"));
    Serial.println(F("  limiter <0/1> | tone <Hz, 0=off> | status | help"));
}

static void print_status() {
    Serial.println(F("--- Current DSP Parameters ---"));
    Serial.printf("  bypass               : %d\n", g_params.bypass);
    Serial.printf("  test_tone_hz         : %.1f\n", g_params.test_tone_hz);
    Serial.printf("  nr_amount            : %.2f\n", g_params.nr_amount);
    Serial.printf("  oversubtraction      : %.2f\n", g_params.oversubtraction);
    Serial.printf("  spectral_floor       : %.2f\n", g_params.spectral_floor);
    Serial.printf("  noise_tracking_speed : %.2f\n", g_params.noise_tracking_speed);
    Serial.printf("  notch_depth          : %.2f\n", g_params.notch_depth);
    Serial.printf("  notch_width_hz       : %.1f\n", g_params.notch_width_hz);
    Serial.printf("  notch_attack_ms      : %.1f\n", g_params.notch_attack_ms);
    Serial.printf("  notch_release_ms     : %.1f\n", g_params.notch_release_ms);
    Serial.printf("  speech_enhance       : %.2f\n", g_params.speech_enhance);
    Serial.printf("  attack_ms            : %.1f\n", g_params.attack_ms);
    Serial.printf("  release_ms           : %.1f\n", g_params.release_ms);
    Serial.printf("  output_gain_db       : %.1f\n", g_params.output_gain_db);
    Serial.printf("  limiter_enable       : %d\n", g_params.limiter_enable);
}

static float clamp01(float v) { if (v < 0.0f) return 0.0f; if (v > 1.0f) return 1.0f; return v; }

static void handle_command(const String &line) {
    int sp = line.indexOf(' ');
    String cmd = (sp < 0) ? line : line.substring(0, sp);
    String arg = (sp < 0) ? "" : line.substring(sp + 1);
    cmd.trim(); arg.trim();
    float v = arg.toFloat();

    if (cmd == "help")            { print_help(); }
    else if (cmd == "status")     { print_status(); }
    else if (cmd == "nr")         { g_params.nr_amount = clamp01(v); }
    else if (cmd == "notch_depth"){ g_params.notch_depth = clamp01(v); }
    else if (cmd == "notch_width"){ g_params.notch_width_hz = v; }
    else if (cmd == "tracking")   { g_params.noise_tracking_speed = clamp01(v); }
    else if (cmd == "floor")      { g_params.spectral_floor = clamp01(v); }
    else if (cmd == "oversub")    { g_params.oversubtraction = v; }
    else if (cmd == "speech")     { g_params.speech_enhance = clamp01(v); }
    else if (cmd == "attack")     { g_params.attack_ms = v; }
    else if (cmd == "release")    { g_params.release_ms = v; }
    else if (cmd == "gain")       { g_params.output_gain_db = v; }
    else if (cmd == "bypass")     { g_params.bypass = (v != 0.0f); }
    else if (cmd == "limiter")    { g_params.limiter_enable = (v != 0.0f); }
    else if (cmd == "tone")       {
        if (v < 0.0f) v = 0.0f;
        if (v > 8000.0f) v = 8000.0f;   // stay comfortably under Nyquist (22.05 kHz)
        g_params.test_tone_hz = v;
        Serial.printf("test_tone_hz -> %.1f%s\n", v, (v == 0.0f) ? " (off)" : "");
    }
    else if (cmd.length() > 0)    { Serial.println(F("Unknown command. Type 'help'.")); }
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(200);   // let USB-CDC/UART settle before printing the banner

    Serial.println();
    Serial.println(F("========================================================"));
    Serial.println(F(" HF Communications Receiver - Digital Noise Reduction"));
    Serial.println(F(" ESP32-S3 / PCM1808 / PCM5102"));
    Serial.println(F("========================================================"));
    Serial.printf("FFT size: %d  Hop: %d  Bins: %d  Bin spacing: %.2f Hz\n",
                   FFT_SIZE, HOP_SIZE, NUM_BINS, BIN_HZ);
    Serial.printf("Sample rate: %d Hz  Nominal analysis-window latency: %.1f ms\n",
                   SAMPLE_RATE_HZ, 1000.0f * FFT_SIZE / SAMPLE_RATE_HZ);
    Serial.println(F("(See settings.h for the full latency-budget discussion.)"));
    print_help();

    audio_init();
    controls_init();

    Serial.println(F("(Physical controls also active -- see settings.h pin map"));
    Serial.println(F(" and README for the potentiometer/button/encoder layout.)"));

    s_last_report_ms = millis();
}

void loop() {
    // --- Serial command handling (non-blocking) ---
    static String line;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (line.length() > 0) { handle_command(line); line = ""; }
        } else {
            line += c;
        }
    }

    // --- Physical controls (pots, buttons, encoder) ---
    controls_update();

    // --- Periodic performance report ---
    uint32_t now = millis();
    if (now - s_last_report_ms >= REPORT_INTERVAL_MS) {
        s_last_report_ms = now;
        Serial.printf(
            "[perf] frames=%lu  last=%.0fus  avg=%.0fus  max=%.0fus  fft=%.0fus  "
            "cpu=%.1f%%  i2s_over=%lu  i2s_under=%lu  freeHeap=%lu\n",
            (unsigned long)g_perf.frames_processed,
            g_perf.last_frame_us, g_perf.avg_frame_us, g_perf.max_frame_us, g_perf.fft_us,
            g_perf.cpu_load_percent,
            (unsigned long)g_perf.i2s_overruns, (unsigned long)g_perf.i2s_underruns,
            (unsigned long)ESP.getFreeHeap());
    }

    delay(5);   // loop() is low priority housekeeping only; all real-time work
                // happens in audio_io_task (Core 0) and dsp_task (Core 1)
}
