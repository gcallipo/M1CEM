/*
 * controls.h
 * ---------------------------------------------------------------------------
 * Physical control surface: potentiometers, push buttons, and a rotary
 * encoder, so the unit can be tuned standalone with no PC/Serial connection.
 * See settings.h ("PHYSICAL CONTROLS") for the pin map and overall layout
 * rationale -- unchanged from the base TM1637 project; only the display
 * module (display.h/.cpp) and this file's secondary-parameter table
 * (now full names + printf-style value formatting, since the OLED has room
 * for that) differ between the two builds.
 *
 * This is a SECOND way to change g_params, alongside the Serial command
 * interface in the main .ino -- both write to the same shared struct, so
 * whichever you touch last "wins" for that field. That's an intentional,
 * simple design: there's no arbitration needed because a human is only ever
 * driving one interface at a time in practice.
 * ---------------------------------------------------------------------------
 */

#ifndef CONTROLS_H
#define CONTROLS_H

#include "settings.h"

// One-time setup: configures ADC attenuation/resolution, pin modes, and the
// encoder's interrupt-driven quadrature decoder. Call once from setup(),
// after g_params has its initial values (so the "currently selected
// secondary parameter" table can capture valid starting pointers).
void controls_init(void);

// Polls the potentiometers and buttons, drains the encoder's ISR-accumulated
// step count, updates g_params accordingly, and redraws the OLED dashboard
// from the current state. Call frequently (e.g. every iteration of loop(),
// which runs at ~150-200 Hz in this project) -- button debounce and pot
// smoothing are time-based, not call-count-based, so calling it more or
// less often doesn't change control behaviour, just latency (though it does
// change the OLED's effective refresh rate, since this is also where
// display_update() is called from).
void controls_update(void);

#endif // CONTROLS_H
