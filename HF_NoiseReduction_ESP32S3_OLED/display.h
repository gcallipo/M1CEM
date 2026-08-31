/*
 * display.h
 * ---------------------------------------------------------------------------
 * 1.3" SH1106 128x64 OLED (I2C): standalone (no-PC) feedback, OLED variant.
 * See settings.h ("1.3" SH1106 128x64 OLED") for the pin map, I2C address,
 * and required libraries; see README_OLED.md for how this compares to the
 * base project's 4-digit TM1637 build.
 *
 * WHY THE API SHAPE IS DIFFERENT FROM THE TM1637 BUILD
 * A 4-digit 7-segment display can only ever show one short field at a time,
 * which is why that build's API was three separate calls (flash an index,
 * show a value, show a "bypassed" indicator) sequenced by the caller. A
 * 128x64 graphical OLED has enough room to show the whole picture at once
 * -- the selected parameter's full name and value, all three pot
 * positions, bypass/limiter state, and live DSP telemetry -- so this API
 * is instead a single status snapshot (DisplayStatus) that the caller
 * fills in every tick and hands to display_update(), which redraws the
 * entire dashboard from scratch. Simpler to reason about than tracking
 * partial-update state, and a full 128x64 I2C redraw is still comfortably
 * fast enough to do every controls_update() tick.
 * ---------------------------------------------------------------------------
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "settings.h"
#include <stdbool.h>

// Snapshot of everything the OLED dashboard needs to render one frame.
// Built fresh by controls_update() every tick (~150-200 Hz, see controls.h)
// and handed to display_update().
typedef struct {
    char  selected_name[24];        // full parameter name, e.g. "Noise Tracking"
    char  selected_value_str[16];   // pre-formatted value + unit, e.g. "40 ms"
    int   selected_index;           // 1-based position in the secondary menu
    int   selected_count;           // total number of secondary parameters

    float nr_amount;                // 0..1, from the NR AMOUNT pot
    float notch_depth;              // 0..1, from the NOTCH DEPTH pot
    float output_gain_db;           // OUTPUT_GAIN_POT_MIN_DB..MAX_DB, from the OUTPUT GAIN pot

    bool  bypass;
    bool  limiter_enable;

    float input_level;              // 0..1, smoothed peak of the incoming audio (VU meter)
    float cpu_load_percent;         // DSP core load, shown alongside the limiter state
} DisplayStatus;

// One-time setup: starts I2C on PIN_OLED_SDA/PIN_OLED_SCL, initializes the
// SH1106 controller at OLED_I2C_ADDR, clears the screen. Call once from
// controls_init(), after g_params has its initial values. If the display
// isn't found (wrong address/wiring), this prints a warning over Serial
// and returns -- the rest of the firmware works fine without it, since the
// OLED is a standalone-tuning convenience, not required for audio
// processing.
void display_init(void);

// Call once whenever the encoder button changes the selected secondary
// parameter. Briefly inverts the header row for DISPLAY_FLASH_MS so the
// change is obvious even out of the corner of your eye; the header reverts
// to normal automatically on the first display_update() call after the
// window elapses -- callers don't need to track the timing themselves.
void display_notify_selection_changed(void);

// Redraws the entire dashboard from `status`. Safe (and intended) to call
// every controls_update() tick.
void display_update(const DisplayStatus *status);

// Blanks the display (e.g. if you want a "sleep" mode later).
void display_off(void);

#endif // DISPLAY_H
