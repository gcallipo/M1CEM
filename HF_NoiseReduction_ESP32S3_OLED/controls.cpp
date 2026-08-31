/*
 * controls.cpp
 * ---------------------------------------------------------------------------
 * See controls.h for the overview and settings.h ("PHYSICAL CONTROLS") for
 * the pin map and design rationale.
 * ---------------------------------------------------------------------------
 */

#include "controls.h"
#include "display.h"
#include "audio.h"      // g_perf, g_dsp_status -- CPU load & VU meter surfaced on the OLED
#include <Arduino.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// =============================================================================
// Secondary parameter table -- the encoder's "select then adjust" menu.
// Order here is the order the encoder button cycles through, and is also
// the 1-based index shown in the OLED header ("n/N Name"):
//   1=Oversub  2=Spectral Floor  3=Noise Tracking  4=Notch Width
//   5=Notch Attack  6=Notch Release  7=Speech Enhance  8=Gain Attack
//   9=Gain Release
//
// Unlike the base TM1637 build (which could only show 1-2 significant
// digits on a 4-digit display and so needed a fixed DISP_FMT_INT/FRAC1/
// FRAC2 enum), the OLED has room for the parameter's full name and a
// proper printf-style format string per entry, including units -- so the
// value shown is unambiguous without cross-referencing a printed list.
// =============================================================================

struct SecondaryParam {
    const char *name;    // full name, shown in the OLED header
    float *value;
    float min_val;
    float max_val;
    const char *fmt;     // printf format for *value, e.g. "%.2f" or "%.0f Hz"
};

static SecondaryParam s_secondary[] = {
    { "Oversub",        &g_params.oversubtraction,      0.5f,    4.0f,    "%.2f"    },
    { "Spectral Floor",  &g_params.spectral_floor,       0.0f,    1.0f,    "%.2f"    },
    { "Noise Tracking",  &g_params.noise_tracking_speed, 0.0f,    1.0f,    "%.2f"    },
    { "Notch Width",     &g_params.notch_width_hz,       10.0f,   300.0f,  "%.0f Hz" },
    { "Notch Attack",    &g_params.notch_attack_ms,      5.0f,    200.0f,  "%.0f ms" },
    { "Notch Release",   &g_params.notch_release_ms,     50.0f,   3000.0f, "%.0f ms" },
    { "Speech Enhance",  &g_params.speech_enhance,       0.0f,    1.0f,    "%.2f"    },
    { "Gain Attack",     &g_params.attack_ms,            1.0f,    100.0f,  "%.0f ms" },
    { "Gain Release",    &g_params.release_ms,           10.0f,   1000.0f, "%.0f ms" },
};
#define NUM_SECONDARY (int)(sizeof(s_secondary) / sizeof(s_secondary[0]))
static int s_selected = 0;

// =============================================================================
// Rotary encoder: interrupt-driven quadrature decode.
// -----------------------------------------------------------------------------
// QUAD_TABLE is the classic 4x "full step" gray-code transition table: index
// with (previous 2-bit A/B state << 2 | current 2-bit A/B state) and get
// -1/0/+1 for that particular transition. Summed over a full physical
// detent (4 raw transitions on a typical cheap encoder), this yields a
// clean +-4 per click, which controls_update() below divides back down to
// +-1 "clicks". This table is standard and widely used (e.g. PJRC's Encoder
// library) specifically because it's robust to switch bounce -- invalid/
// impossible transitions map to 0, contributing nothing.
// =============================================================================

static const int8_t QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static volatile int32_t s_encoder_raw_delta = 0;
static volatile uint8_t s_encoder_prev_state = 0;

static void IRAM_ATTR encoder_isr(void) {
    uint8_t a = digitalRead(PIN_ENCODER_A);
    uint8_t b = digitalRead(PIN_ENCODER_B);
    uint8_t curr = (uint8_t)((a << 1) | b);
    uint8_t idx = (uint8_t)((s_encoder_prev_state << 2) | curr);
    s_encoder_raw_delta += QUAD_TABLE[idx & 0x0F];
    s_encoder_prev_state = curr;
}

// =============================================================================
// Debounced button helper (active-low, INPUT_PULLUP)
// =============================================================================

struct DebouncedButton {
    int pin;
    int stable_state;
    int last_raw_state;
    unsigned long last_change_ms;
};

static DebouncedButton s_btn_encoder = { PIN_ENCODER_BUTTON, HIGH, HIGH, 0 };
static DebouncedButton s_btn_bypass  = { PIN_BUTTON_BYPASS,  HIGH, HIGH, 0 };
static DebouncedButton s_btn_limiter = { PIN_BUTTON_LIMITER, HIGH, HIGH, 0 };

// Returns true exactly once, on a debounced HIGH->LOW transition (a press).
static bool button_pressed_edge(DebouncedButton *b) {
    int raw = digitalRead(b->pin);
    unsigned long now = millis();

    if (raw != b->last_raw_state) {
        b->last_raw_state = raw;
        b->last_change_ms = now;
    }

    bool edge = false;
    if ((now - b->last_change_ms) >= BUTTON_DEBOUNCE_MS && raw != b->stable_state) {
        bool was_high = (b->stable_state == HIGH);
        b->stable_state = raw;
        if (was_high && raw == LOW) edge = true;
    }
    return edge;
}

// =============================================================================
// Potentiometers: smoothed, deadbanded, direct-mapped
// =============================================================================

static float s_pot_nr_smooth    = 0.0f;
static float s_pot_notch_smooth = 0.0f;
static float s_pot_gain_smooth  = 0.0f;
static int   s_pot_nr_last_counts    = -1000;
static int   s_pot_notch_last_counts = -1000;
static int   s_pot_gain_last_counts  = -1000;
static unsigned long s_pot_nr_last_print_ms    = 0;
static unsigned long s_pot_notch_last_print_ms = 0;
static unsigned long s_pot_gain_last_print_ms  = 0;

// Serial feedback for a pot move is rate-limited independently of the
// deadband above: the deadband governs how often the actual parameter
// value updates (kept tight, for smooth audio response), while
// POT_PRINT_INTERVAL_MS (settings.h) caps how often we print about it (kept
// loose, so slowly sweeping a knob doesn't flood the Serial monitor). A
// value change is still applied to g_params immediately every time
// regardless of the print rate -- only the Serial line itself is throttled.
// (The OLED gauges, unlike this Serial print, redraw every tick regardless
// -- see the DisplayStatus hand-off at the end of controls_update().)

static void update_pot(int pin, float *smooth_state, int *last_counts,
                        float out_min, float out_max, float *target,
                        const char *name, unsigned long *last_print_ms) {
    int raw = analogRead(pin);
    *smooth_state += POT_SMOOTH_COEFF * ((float)raw - *smooth_state);
    int counts = (int)(*smooth_state + 0.5f);

    if (abs(counts - *last_counts) < POT_DEADBAND_COUNTS) return;   // ADC dither, knob not touched
    *last_counts = counts;

    float frac = (float)counts / 4095.0f;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    *target = out_min + frac * (out_max - out_min);

    unsigned long now = millis();
    if (now - *last_print_ms >= POT_PRINT_INTERVAL_MS) {
        *last_print_ms = now;
        Serial.printf("[ctrl] %s = %.3f\n", name, *target);
    }
}

// =============================================================================
// Public API
// =============================================================================

void controls_init(void) {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);   // full 0-3.3V pot range

    pinMode(PIN_POT_NR_AMOUNT, INPUT);
    pinMode(PIN_POT_NOTCH_DEPTH, INPUT);
    pinMode(PIN_POT_OUTPUT_GAIN, INPUT);

    pinMode(PIN_ENCODER_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUTTON_BYPASS, INPUT_PULLUP);
    pinMode(PIN_BUTTON_LIMITER, INPUT_PULLUP);

    s_encoder_prev_state = (uint8_t)((digitalRead(PIN_ENCODER_A) << 1) | digitalRead(PIN_ENCODER_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoder_isr, CHANGE);

    // Seed the smoothed pot state from a real first reading, so the very
    // first controls_update() call doesn't see a spurious large "change"
    // from an initial value of 0.
    s_pot_nr_smooth    = (float)analogRead(PIN_POT_NR_AMOUNT);
    s_pot_notch_smooth = (float)analogRead(PIN_POT_NOTCH_DEPTH);
    s_pot_gain_smooth  = (float)analogRead(PIN_POT_OUTPUT_GAIN);

    display_init();
    display_notify_selection_changed();   // highlight the initial selection briefly

    Serial.println(F("Physical controls ready:"));
    Serial.println(F("  Pots    : NR amount / Notch depth / Output gain (always live)"));
    Serial.println(F("  Buttons : bypass toggle / limiter toggle"));
    Serial.println(F("  Encoder : press = select parameter, turn = adjust it"));
    Serial.println(F("  Display : 1.3\" SH1106 OLED -- shows the selected parameter's"));
    Serial.println(F("            full name/value, all three pot values, bypass/limiter"));
    Serial.println(F("            state, and live VAD/tone/CPU telemetry all at once"));
    Serial.printf("  Selected: #%d %s = ", s_selected + 1, s_secondary[s_selected].name);
    Serial.printf(s_secondary[s_selected].fmt, *s_secondary[s_selected].value);
    Serial.println();
}

void controls_update(void) {
    // --- Potentiometers: direct, always-live, no menu ---
    update_pot(PIN_POT_NR_AMOUNT,   &s_pot_nr_smooth,    &s_pot_nr_last_counts,
               0.0f, 1.0f, &g_params.nr_amount, "nr_amount", &s_pot_nr_last_print_ms);
    update_pot(PIN_POT_NOTCH_DEPTH, &s_pot_notch_smooth, &s_pot_notch_last_counts,
               0.0f, 1.0f, &g_params.notch_depth, "notch_depth", &s_pot_notch_last_print_ms);
    update_pot(PIN_POT_OUTPUT_GAIN, &s_pot_gain_smooth,  &s_pot_gain_last_counts,
               OUTPUT_GAIN_POT_MIN_DB, OUTPUT_GAIN_POT_MAX_DB, &g_params.output_gain_db,
               "output_gain_db", &s_pot_gain_last_print_ms);

    // --- Instant toggle buttons ---
    if (button_pressed_edge(&s_btn_bypass)) {
        g_params.bypass = !g_params.bypass;
        Serial.printf("[ctrl] bypass -> %d\n", (int)g_params.bypass);
    }
    if (button_pressed_edge(&s_btn_limiter)) {
        g_params.limiter_enable = !g_params.limiter_enable;
        Serial.printf("[ctrl] limiter -> %d\n", (int)g_params.limiter_enable);
    }

    // --- Encoder button: cycle the selected secondary parameter ---
    if (button_pressed_edge(&s_btn_encoder)) {
        s_selected = (s_selected + 1) % NUM_SECONDARY;
        display_notify_selection_changed();
        Serial.printf("[ctrl] selected: #%d %s = ", s_selected + 1, s_secondary[s_selected].name);
        Serial.printf(s_secondary[s_selected].fmt, *s_secondary[s_selected].value);
        Serial.println();
    }

    // --- Encoder rotation: adjust the selected secondary parameter ---
    int32_t delta;
    noInterrupts();
    delta = s_encoder_raw_delta;
    s_encoder_raw_delta = 0;
    interrupts();

    if (delta != 0) {
        // Most inexpensive mechanical encoders produce 4 raw quadrature
        // transitions per physical detent (click); accumulate a remainder
        // so partial transitions carry over correctly between calls rather
        // than being dropped, and only act once a full click's worth (4)
        // has accumulated.
        static int32_t remainder = 0;
        remainder += delta;
        int32_t clicks = remainder / 4;
        remainder -= clicks * 4;

        if (clicks != 0) {
            SecondaryParam *p = &s_secondary[s_selected];
            float step = ENCODER_STEP_FRACTION * (p->max_val - p->min_val);
            *p->value += step * (float)clicks;
            if (*p->value < p->min_val) *p->value = p->min_val;
            if (*p->value > p->max_val) *p->value = p->max_val;
            Serial.printf("[ctrl] %s = ", p->name);
            Serial.printf(p->fmt, *p->value);
            Serial.println();
        }
    }

    // --- OLED dashboard: build the full status snapshot and redraw. Unlike
    //     the base TM1637 build (which had to choose between showing the
    //     selected value or a "bypassed" indicator, one or the other), the
    //     OLED shows the selected parameter, all three pot positions,
    //     bypass/limiter state, and a live input VU meter simultaneously
    //     -- see display.h/.cpp for the layout. ---
    DisplayStatus status;
    snprintf(status.selected_name, sizeof(status.selected_name), "%s", s_secondary[s_selected].name);
    snprintf(status.selected_value_str, sizeof(status.selected_value_str),
             s_secondary[s_selected].fmt, *s_secondary[s_selected].value);
    status.selected_index    = s_selected + 1;
    status.selected_count    = NUM_SECONDARY;
    status.nr_amount         = g_params.nr_amount;
    status.notch_depth       = g_params.notch_depth;
    status.output_gain_db    = g_params.output_gain_db;
    status.bypass            = g_params.bypass;
    status.limiter_enable    = g_params.limiter_enable;
    status.input_level       = g_dsp_status.input_level;
    status.cpu_load_percent  = g_perf.cpu_load_percent;

    display_update(&status);
}
