/*
 * display.cpp
 * ---------------------------------------------------------------------------
 * See display.h for the API and settings.h for the pin map/I2C address.
 *
 * LAYOUT (128x64, all coordinates in pixels, text size 1 = 6x8 px/glyph):
 *
 *   y= 0  "n/N Parameter Name"          <- header, inverts briefly on select
 *   y= 8  VALUE (text size 2)           <- selected parameter's live value,
 *                                           or "BYPASSED" while bypass is on
 *   y=24  NR    [======......] 0.65     <- always-live pot gauges
 *   y=32  NOTCH [========....] 0.80
 *   y=40  GAIN  [=====.......] +2.0
 *   y=48  LIMITER: OFF            41%   <- limiter state + DSP CPU load
 *   y=56  IN [=========........]        <- VU meter, INCOMING audio level
 *
 * Bypass state isn't repeated in the footer -- it already takes over the
 * big value line above ("BYPASSED"), which is a lot harder to miss than
 * one more line of footer text.
 *
 * The VU meter (bottom row) replaces this build's earlier VAD-activity/
 * tone-count readout: those turned out to be low-value at a glance (a
 * flag and a small integer, changing infrequently), whereas a live level
 * bar is immediately useful -- confirms audio is actually reaching the
 * ADC, gives a rough sense of gain-staging, and moves continuously so
 * it also doubles as a "the OLED/I2C link is still alive" heartbeat.
 * It reads `hop_in` (the raw incoming samples, full-scale = 1.0)
 * BEFORE the bypass/test-tone/pipeline branch in dsp_task, so it stays
 * live in every mode, not just while the spectral pipeline is running.
 * Ballistics: fast attack / slower release (see VU_METER_* in audio.cpp),
 * mapped from VU_METER_MIN_DB..0 dBFS onto an empty..full bar.
 *
 * Every field the base TM1637 build could only show one at a time (via
 * display_flash_index()/display_show_value()/display_show_bypass()) is
 * visible here simultaneously, plus a live input VU meter and DSP CPU
 * load, which the TM1637 had no room for at all (see audio.h's
 * DspStatus and g_perf).
 * ---------------------------------------------------------------------------
 */

#include "display.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <stdio.h>

static Adafruit_SH1106G s_display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);
static unsigned long s_flash_until_ms = 0;
static bool s_display_ok = false;   // false if display_init() couldn't find the panel

// True while a selection-change highlight (display_notify_selection_changed())
// is still in its display window.
static bool flash_active(void) {
    return millis() < s_flash_until_ms;
}

void display_init(void) {
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);

    if (!s_display.begin(OLED_I2C_ADDR, true)) {
        // Most likely a wrong I2C address or a wiring fault. This is a
        // standalone-tuning convenience, not required for audio processing,
        // so treat it as a warning rather than halting the whole firmware.
        Serial.println(F("WARNING: SH1106 OLED not found at OLED_I2C_ADDR -- "
                          "check wiring/address in settings.h (some 1.3\" "
                          "modules use 0x3D instead of 0x3C). Continuing "
                          "without the display."));
        s_display_ok = false;
        return;
    }

    s_display_ok = true;
    s_display.clearDisplay();
    s_display.display();
}

void display_notify_selection_changed(void) {
    s_flash_until_ms = millis() + DISPLAY_FLASH_MS;
}

// Prints `text` right-aligned against the screen's right edge at row y.
static void print_right_aligned(int y, const char *text) {
    int16_t x1, y1;
    uint16_t w, h;
    s_display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    s_display.setCursor(OLED_WIDTH - (int)w, y);
    s_display.print(text);
}

// Draws one gauge row: a short label, an outlined bar filled in proportion
// to frac (0..1, clamped), and a right-aligned numeric readout -- used for
// all three always-live pot values.
static void draw_gauge_row(int y, const char *label, float frac, const char *value_text) {
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    s_display.setTextColor(SH110X_WHITE);
    s_display.setCursor(0, y);
    s_display.print(label);

    const int bar_x = 34, bar_w = 60, bar_h = 6;
    const int bar_y = y + 1;
    s_display.drawRect(bar_x, bar_y, bar_w, bar_h, SH110X_WHITE);
    int fill_w = (int)((bar_w - 2) * frac + 0.5f);
    if (fill_w > 0) {
        s_display.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, SH110X_WHITE);
    }

    print_right_aligned(y, value_text);
}

// Draws the VU meter row: an "IN" label and a single wide bar (no numeric
// readout -- classic VU meters don't show an exact number either, just
// the bar) filled in proportion to frac (0..1, clamped). See this file's
// header comment for what frac represents and its ballistics.
static void draw_vu_bar(int y, float frac) {
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    s_display.setTextColor(SH110X_WHITE);
    s_display.setCursor(0, y);
    s_display.print("IN");

    const int bar_x = 20, bar_w = 108, bar_h = 6;
    const int bar_y = y + 1;
    s_display.drawRect(bar_x, bar_y, bar_w, bar_h, SH110X_WHITE);
    int fill_w = (int)((bar_w - 2) * frac + 0.5f);
    if (fill_w > 0) {
        s_display.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, SH110X_WHITE);
    }
}

void display_update(const DisplayStatus *st) {
    if (!s_display_ok) return;   // see display_init(): no panel found at boot

    s_display.clearDisplay();
    s_display.setTextWrap(false);

    // --- Header: "n/N Parameter Name" -- inverted briefly after a
    //     selection change so the new selection is obvious at a glance ---
    char header[24];
    snprintf(header, sizeof(header), "%d/%d %s", st->selected_index, st->selected_count, st->selected_name);
    s_display.setTextSize(1);
    if (flash_active()) {
        s_display.fillRect(0, 0, OLED_WIDTH, 9, SH110X_WHITE);
        s_display.setTextColor(SH110X_BLACK);
    } else {
        s_display.setTextColor(SH110X_WHITE);
    }
    s_display.setCursor(0, 0);
    s_display.print(header);

    // --- Big value line: the selected parameter's live value, UNLESS
    //     bypass is engaged -- while bypassed, none of these controls have
    //     any audible effect, so show that plainly instead of a value that
    //     would otherwise look "live" but isn't currently doing anything
    //     (same rationale as the base TM1637 build's display_show_bypass(),
    //     see that project's display.cpp). ---
    s_display.setTextColor(SH110X_WHITE);
    s_display.setTextSize(2);
    s_display.setCursor(0, 8);
    if (st->bypass) {
        s_display.print(F("BYPASSED"));
    } else {
        s_display.print(st->selected_value_str);
    }
    s_display.setTextSize(1);

    // --- Always-live pot values, as gauges (unlike the TM1637 build, these
    //     stay visible even while a secondary parameter's value/name is
    //     also on screen -- no menu, no mode switch needed to see them) ---
    char buf[12];

    snprintf(buf, sizeof(buf), "%.2f", st->nr_amount);
    draw_gauge_row(24, "NR", st->nr_amount, buf);

    snprintf(buf, sizeof(buf), "%.2f", st->notch_depth);
    draw_gauge_row(32, "NOTCH", st->notch_depth, buf);

    {
        float frac = (st->output_gain_db - OUTPUT_GAIN_POT_MIN_DB) /
                     (OUTPUT_GAIN_POT_MAX_DB - OUTPUT_GAIN_POT_MIN_DB);
        snprintf(buf, sizeof(buf), "%+.1f", st->output_gain_db);
        draw_gauge_row(40, "GAIN", frac, buf);
    }

    // --- Footer row 1: limiter TOGGLE STATE, spelled out, plus DSP CPU
    //     load right-aligned. Bypass isn't repeated here -- it already
    //     takes over the big value line above ("BYPASSED"). ---
    char limiter_line[16];
    snprintf(limiter_line, sizeof(limiter_line), "LIMITER: %s",
             st->limiter_enable ? "ON" : "OFF");
    s_display.setTextColor(SH110X_WHITE);
    s_display.setCursor(0, 48);
    s_display.print(limiter_line);

    char cpu_buf[8];
    snprintf(cpu_buf, sizeof(cpu_buf), "%.0f%%", st->cpu_load_percent);
    print_right_aligned(48, cpu_buf);

    // --- Footer row 2: VU meter of the INCOMING (pre-processing) audio
    //     level -- see the glossary in this file's header comment. ---
    draw_vu_bar(56, st->input_level);

    s_display.display();
}

void display_off(void) {
    if (!s_display_ok) return;
    s_display.clearDisplay();
    s_display.display();
}
