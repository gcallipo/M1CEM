/*
 * wiener.cpp
 * ---------------------------------------------------------------------------
 * See wiener.h for the full mathematical derivation and rationale.
 * ---------------------------------------------------------------------------
 */

#include "wiener.h"
#include <string.h>
#include <math.h>

#define DECISION_DIRECTED_ALPHA   0.98f   // Ephraim-Malah smoothing factor
#define SMOOTHING_CENTER_WEIGHT   0.6f    // cross-bin gain smoothing (see spectral_smooth3)
#define MIN_NOISE_POWER           1e-6f   // guards against divide-by-zero on silent bins

void wiener_init(WienerState *st) {
    memset(st, 0, sizeof(WienerState));
    for (int k = 0; k < NUM_BINS; k++) st->prev_gain[k] = 1.0f;
}

void wiener_compute_gain(WienerState *st,
                          const complex_t *X,
                          const float *noise_mag,
                          float *gain_out,
                          float oversubtraction,
                          float spectral_floor,
                          float nr_amount) {
    static float raw_gain[NUM_BINS];   // pre-smoothing gain (static: no heap, single-threaded caller)

    for (int k = 0; k < NUM_BINS; k++) {
        float sig_mag2   = c_mag2(X[k]);                          // |X(k,n)|^2
        float noise_pow  = noise_mag[k] * noise_mag[k];
        if (noise_pow < MIN_NOISE_POWER) noise_pow = MIN_NOISE_POWER;

        // --- A posteriori SNR ---
        float gamma = sig_mag2 / noise_pow;

        // --- Decision-directed a priori SNR (Ephraim-Malah) ---
        float prev_term = st->prev_gain[k] * st->prev_gain[k] *
                           st->prev_sig_mag2[k] / noise_pow;
        float xi = DECISION_DIRECTED_ALPHA * prev_term +
                   (1.0f - DECISION_DIRECTED_ALPHA) * fmaxf(gamma - 1.0f, 0.0f);

        // --- Wiener gain from a priori SNR:  H = Rs (Rs+Rn)^-1 -> xi/(1+xi) ---
        float g_wiener = xi / (1.0f + xi);

        // --- Classical magnitude spectral subtraction with over-subtraction
        //     and spectral floor, as an independently-tunable alternative
        //     view of the same problem:
        //         |Y| = max( |X| - alpha*|N| , beta*|X| )
        //         G_ss = |Y| / |X| = max( 1 - alpha*|N|/|X| , beta )
        float sig_mag = sqrtf(sig_mag2) + 1e-9f;
        float g_ss = 1.0f - oversubtraction * (noise_mag[k] / sig_mag);
        if (g_ss < spectral_floor) g_ss = spectral_floor;
        if (g_ss > 1.0f) g_ss = 1.0f;

        // --- Blend the two gain estimates (geometric mean -- keeps the
        //     conservative estimate from either method dominant, which in
        //     practice sounds more natural than an arithmetic mean) ---
        float g_combined = sqrtf(fmaxf(g_wiener, 0.0f) * fmaxf(g_ss, 0.0f));

        // --- Enforce the user-facing floor and the nr_amount wet/dry mix ---
        if (g_combined < spectral_floor) g_combined = spectral_floor;
        float g_final = 1.0f - nr_amount * (1.0f - g_combined);

        raw_gain[k] = g_final;

        // Update decision-directed state for next frame
        st->prev_sig_mag2[k] = sig_mag2;
        // (prev_gain is updated AFTER cross-bin smoothing below, using the
        //  smoothed value actually applied to the audio -- keeps the
        //  decision-directed loop consistent with what was really used.)
    }

    // Cross-bin smoothing (banded-matrix operation, see matrix.h) to avoid
    // an audible "comb" pattern in the gain curve, which is a major source
    // of the "watery"/robotic character the brief explicitly wants avoided.
    spectral_smooth3(raw_gain, gain_out, NUM_BINS, SMOOTHING_CENTER_WEIGHT);

    for (int k = 0; k < NUM_BINS; k++) {
        st->prev_gain[k] = gain_out[k];
    }
}
