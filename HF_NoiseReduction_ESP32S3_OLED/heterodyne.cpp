/*
 * heterodyne.cpp
 * ---------------------------------------------------------------------------
 * See heterodyne.h for the pipeline overview.
 * ---------------------------------------------------------------------------
 */

#include "heterodyne.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define PROMINENCE_THRESHOLD_DB   9.0f    // candidate peak must beat neighbours by this much
#define NEIGHBOUR_GUARD_BINS      2       // bins immediately around the peak excluded from the average
#define NEIGHBOUR_SPAN_BINS       3       // how many further bins on each side form the average
#define MATCH_TOLERANCE_BINS      2       // how far (in bins) a candidate may drift from a track
#define MAX_CANDIDATES            32      // hard cap on candidate peaks examined per frame

// How quickly a track's notch CENTRE FREQUENCY chases a newly matched bin,
// rather than snapping straight to it. Bins are BIN_HZ (~43 Hz) apart and a
// new frame arrives every ~11.6 ms (see settings.h), so a tone that's
// slowly sweeping -- e.g. someone tuning up nearby, or a drifting
// heterodyne -- crosses a bin boundary every so often. Without this, each
// crossing snaps the whole notch shape sideways by a full bin, instantly,
// while the notch is fully engaged; a steady sweep crossing several bins a
// second turns into a rapid train of these position-snaps, which is heard
// as a buzz. Smoothing the tracked frequency (and building the notch
// shape from that continuous, sub-bin position -- see step 5 below)
// turns each crossing into a gradual glide instead of a jump.
#define TONE_FREQ_SMOOTH_TC_MS    120.0f

// -----------------------------------------------------------------------------
// Local helper: neighbour-average prominence test + local maximum test.
// Returns true and sets *prominence_db if bin k is a valid candidate tone.
// -----------------------------------------------------------------------------
static bool is_candidate_peak(const float *mag, int k, float *prominence_db_out) {
    if (k < NEIGHBOUR_GUARD_BINS + NEIGHBOUR_SPAN_BINS ||
        k > NUM_BINS - 1 - NEIGHBOUR_GUARD_BINS - NEIGHBOUR_SPAN_BINS) {
        return false;
    }
    if (!(mag[k] > mag[k - 1] && mag[k] > mag[k + 1])) return false;   // local maximum

    float sum = 0.0f;
    int count = 0;
    for (int d = NEIGHBOUR_GUARD_BINS; d < NEIGHBOUR_GUARD_BINS + NEIGHBOUR_SPAN_BINS; d++) {
        sum += mag[k - d] + mag[k + d];
        count += 2;
    }
    float neighbour_avg = sum / (float)count;
    float prominence_db = 20.0f * log10f((mag[k] + 1e-9f) / (neighbour_avg + 1e-9f));

    *prominence_db_out = prominence_db;
    return prominence_db > PROMINENCE_THRESHOLD_DB;
}

// -----------------------------------------------------------------------------
// Sub-bin peak offset via parabolic (quadratic) interpolation of the
// log-magnitude around an integer-bin peak at k. Returns an offset in
// [-0.5, +0.5] bins -- the true peak is estimated to sit at (k + offset)
// bins, i.e. BIN_HZ * (k + offset) Hz.
//
// WHY THIS MATTERS: without it, the tracker's only notion of "where the
// tone is" is the integer bin index that happened to have the larger
// magnitude that frame. When a real tone sits between two bins, both get
// comparable energy from spectral leakage, and ordinary frame-to-frame
// noise can flip which of the two is technically the taller one. That
// flip changes the *target* fed into the freq_hz smoothing filter below,
// so the smoothed frequency (and the notch built from it) chases back
// and forth between two fixed points ~BIN_HZ apart forever, instead of
// settling -- heard as a buzz/warble, since the notch's depth is real and
// its position is moving. Estimating the true sub-bin location directly,
// every frame, removes the discrete "which bin wins" decision that was
// causing the flip-flop in the first place; the freq_hz smoothing then
// only has ordinary estimation noise left to damp, which is what it was
// actually meant for.
//
// k-1 and k+1 are guaranteed valid by is_candidate_peak's edge-margin
// check before this is ever called on that k.
// -----------------------------------------------------------------------------
static float parabolic_peak_offset(const float *mag, int k) {
    float lm1 = log10f(mag[k - 1] + 1e-9f);
    float l0  = log10f(mag[k]     + 1e-9f);
    float lp1 = log10f(mag[k + 1] + 1e-9f);

    float denom = lm1 - 2.0f * l0 + lp1;
    if (fabsf(denom) < 1e-9f) return 0.0f;   // degenerate (flat top) -- no correction

    float offset = 0.5f * (lm1 - lp1) / denom;
    if (offset < -0.5f) offset = -0.5f;
    if (offset > 0.5f)  offset = 0.5f;
    return offset;
}

void heterodyne_init(HeterodyneState *hs) {
    memset(hs, 0, sizeof(HeterodyneState));
}

void heterodyne_process(HeterodyneState *hs,
                         const float *mag,
                         float *notch_gain_out,
                         float notch_depth,
                         float notch_width_hz,
                         float attack_ms,
                         float release_ms,
                         float frame_period_ms) {
    // ---- 1. Peak detection ----
    int   cand_bin[MAX_CANDIDATES];
    float cand_prom[MAX_CANDIDATES];
    float cand_frac[MAX_CANDIDATES];   // sub-bin offset, see parabolic_peak_offset()
    bool  cand_claimed[MAX_CANDIDATES];
    int   num_cand = 0;

    for (int k = 0; k < NUM_BINS && num_cand < MAX_CANDIDATES; k++) {
        float prom;
        if (is_candidate_peak(mag, k, &prom)) {
            cand_bin[num_cand] = k;
            cand_prom[num_cand] = prom;
            cand_frac[num_cand] = parabolic_peak_offset(mag, k);
            cand_claimed[num_cand] = false;
            num_cand++;
        }
    }

    // ---- 2. Match existing tracks to candidates ----
    for (int t = 0; t < MAX_TONE_TRACKS; t++) {
        ToneTrack *tr = &hs->tracks[t];
        if (!tr->in_use) continue;

        int best_c = -1;
        int best_dist = MATCH_TOLERANCE_BINS + 1;
        float best_prom = -1e9f;
        for (int c = 0; c < num_cand; c++) {
            if (cand_claimed[c]) continue;
            int dist = abs(cand_bin[c] - tr->bin);
            if (dist > MATCH_TOLERANCE_BINS) continue;
            // Prefer the closest candidate; break ties by higher prominence
            // (the more clearly tonal candidate is more likely the same tone).
            if (dist < best_dist || (dist == best_dist && cand_prom[c] > best_prom)) {
                best_dist = dist;
                best_prom = cand_prom[c];
                best_c = c;
            }
        }

        if (best_c >= 0) {
            cand_claimed[best_c] = true;
            tr->bin = cand_bin[best_c];   // nearest bin -- still used for match-tolerance distance above
            // Chase the matched peak's frequency smoothly rather than
            // snapping straight to it (see TONE_FREQ_SMOOTH_TC_MS above) --
            // this is what freq_hz's "smoothed tracked frequency" comment
            // in heterodyne.h describes; it drives the notch's actual
            // (sub-bin, continuous) centre position in step 5 below.
            // target_hz uses the parabolic-interpolated sub-bin offset,
            // not just the raw integer bin -- otherwise a tone sitting
            // between two bins can flip which integer bin "wins" from one
            // frame to the next, and the smoothing filter would just chase
            // that flip back and forth forever instead of settling (see
            // parabolic_peak_offset()'s comment for why that's audible as
            // a buzz).
            float target_hz = (cand_bin[best_c] + cand_frac[best_c]) * BIN_HZ;
            float freq_c = tc_to_coeff(TONE_FREQ_SMOOTH_TC_MS, frame_period_ms);
            tr->freq_hz += freq_c * (target_hz - tr->freq_hz);
            tr->miss_count = 0;
            if (tr->hit_count < TONE_PERSIST_CONFIRM + 100) tr->hit_count++;
            if (tr->hit_count >= TONE_PERSIST_CONFIRM) tr->confirmed = true;
        } else {
            tr->miss_count++;
            if (tr->miss_count > TONE_MISS_TOLERANCE) {
                // Drop the track entirely (also lets its notch release to 0
                // via depth_env below before the slot is actually reused,
                // since we only clear in_use here and depth_env decays
                // toward 0 on the frames leading up to this point).
                tr->in_use = false;
                tr->confirmed = false;
                tr->hit_count = 0;
                tr->depth_env = 0.0f;
            }
            // else: keep the track alive through short gaps (e.g. CW keying)
        }
    }

    // ---- 3. Start new tentative tracks for unclaimed candidates ----
    // freq_hz starts at the candidate's sub-bin-interpolated frequency
    // (see parabolic_peak_offset()) rather than the raw integer bin --
    // there's nothing to smooth from yet on a brand new track, and
    // TONE_PERSIST_CONFIRM frames still have to pass (with depth_env at 0
    // the whole time, see step 4) before it can notch anything anyway, but
    // starting from the accurate position means it doesn't have to "catch
    // up" from a bin-quantized guess once it does start notching.
    for (int c = 0; c < num_cand; c++) {
        if (cand_claimed[c]) continue;
        for (int t = 0; t < MAX_TONE_TRACKS; t++) {
            if (!hs->tracks[t].in_use) {
                hs->tracks[t].in_use = true;
                hs->tracks[t].bin = cand_bin[c];
                hs->tracks[t].freq_hz = (cand_bin[c] + cand_frac[c]) * BIN_HZ;
                hs->tracks[t].hit_count = 1;
                hs->tracks[t].miss_count = 0;
                hs->tracks[t].confirmed = false;
                hs->tracks[t].depth_env = 0.0f;
                break;
            }
        }
    }

    // ---- 4. Update each track's smoothed notch-depth envelope ----
    float attack_c  = tc_to_coeff(attack_ms, frame_period_ms);
    float release_c = tc_to_coeff(release_ms, frame_period_ms);
    for (int t = 0; t < MAX_TONE_TRACKS; t++) {
        ToneTrack *tr = &hs->tracks[t];
        float target = (tr->in_use && tr->confirmed) ? notch_depth : 0.0f;
        float coeff = (target > tr->depth_env) ? attack_c : release_c;
        tr->depth_env += coeff * (target - tr->depth_env);
        if (tr->depth_env < 0.0005f && !tr->in_use) tr->depth_env = 0.0f;  // fully settle a dropped track
    }

    // ---- 5. Build the notch gain matrix (diagonal) ----
    for (int k = 0; k < NUM_BINS; k++) notch_gain_out[k] = 1.0f;

    float width_bins = notch_width_hz / BIN_HZ;
    if (width_bins < 1.0f) width_bins = 1.0f;
    int half_width = (int)ceilf(width_bins);

    for (int t = 0; t < MAX_TONE_TRACKS; t++) {
        ToneTrack *tr = &hs->tracks[t];
        if (tr->depth_env <= 0.0005f) continue;

        // Sub-bin-accurate, continuously-moving centre position -- derived
        // from the SMOOTHED freq_hz (step 2), not the raw integer bin, so
        // the notch glides as a sweeping tone crosses bin boundaries
        // instead of jumping. center_bin_f will usually sit between two
        // integer bins; the loop below still only touches the (integer)
        // bins actually in range, but each one's distance from the notch
        // centre -- and therefore its raised-cosine gain -- is computed
        // against that continuous position.
        float center_bin_f = tr->freq_hz / BIN_HZ;
        int center_i = (int)roundf(center_bin_f);
        for (int d = -half_width; d <= half_width; d++) {
            int k = center_i + d;
            if (k < 0 || k >= NUM_BINS) continue;
            // Raised-cosine dip: 1.0 at center tapering smoothly to 0.0 at
            // the notch edges, so the gain matrix has no hard discontinuity
            // (a rectangular notch would ring / sound clicky).
            float x = ((float)k - center_bin_f) / width_bins;   // -1..1 across the notch
            if (x < -1.0f) x = -1.0f;
            if (x > 1.0f)  x = 1.0f;
            float shape = 0.5f * (1.0f + cosf((float)M_PI * x));  // 1 at x=0, 0 at |x|=1
            float local_gain = 1.0f - tr->depth_env * shape;
            // Multiple overlapping notches combine multiplicatively (each
            // is an independent diagonal-matrix factor; the product of
            // diagonal matrices is diagonal, entries multiply elementwise).
            notch_gain_out[k] *= local_gain;
        }
    }
}
