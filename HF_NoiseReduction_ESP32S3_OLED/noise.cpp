/*
 * noise.cpp
 * ---------------------------------------------------------------------------
 * See noise.h for the design rationale.
 * ---------------------------------------------------------------------------
 */

#include "noise.h"
#include <string.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Tunables that are not exposed as runtime DSPParams because they rarely need
// live adjustment (they shape the VAD's internal behaviour, not its "amount").
// -----------------------------------------------------------------------------
#define VAD_ENERGY_ATTACK_MS     15.0f    // how fast the energy estimate rises
#define VAD_ENERGY_RELEASE_MS    150.0f   // how fast it falls
#define VAD_NOISE_REF_TC_MS      500.0f   // reference update rate while NOT flagged active
#define VAD_NOISE_REF_SLOW_TC_MS 15000.0f // reference update rate while flagged active (slow, but non-zero:
                                           // guarantees the reference can never get permanently stuck far
                                           // from reality, e.g. after a bad startup guess or a persistent tone)
#define VAD_SNR_THRESHOLD_DB     6.0f     // frame judged "speech" above this SNR
#define VAD_HOLD_FRAMES          8        // hangover, ~93 ms at 11.6 ms/frame
#define NOISE_ATTACK_TC_MS       60.0f    // fast: estimate drops quickly toward a new lower floor
// Release (rise) time constant is derived from tracking_speed at call time.
#define NOISE_RELEASE_TC_MIN_MS  200.0f   // tracking_speed = 1.0 -> fastest release
#define NOISE_RELEASE_TC_MAX_MS  8000.0f  // tracking_speed = 0.0 -> slowest release (~frozen)
#define NOISE_SPEECH_RELEASE_MULT 6.0f    // extra sluggishness applied to release while VAD says "speech"

void noise_init(NoiseEstimator *ns) {
    memset(ns, 0, sizeof(NoiseEstimator));
    for (int k = 0; k < NUM_BINS; k++) {
        ns->noise_mag[k] = 1.0f;     // small non-zero floor; real level converges within ~1s of audio
        ns->smooth_mag[k] = 1.0f;
    }
    ns->vad_energy_smooth = 0.0f;
    ns->vad_noise_ref = 1.0f;        // placeholder; overwritten by the bootstrap on the first real call
    ns->vad_active = false;
    ns->vad_hold_counter = 0;
    ns->vad_bootstrapped = false;
}

bool noise_vad_update(NoiseEstimator *ns, const float *mag, const float *bin_weight,
                       float frame_period_ms) {
    // Wideband energy of this frame (sum of |X[k]|^2, i.e. Parseval-style
    // energy; the DC and Nyquist bins are given half weight since, in the
    // real-FFT half-spectrum, they represent unmirrored components).
    // Bins already claimed by a confirmed heterodyne notch are down-weighted
    // via bin_weight (typically the heterodyne module's notch gain, so a
    // fully-notched bin contributes roughly (1-notch_depth)^2 as much energy
    // as it otherwise would) so a steady local tone can't monopolize the
    // VAD's "signal present" decision -- see noise.h.
    float energy = 0.0f;
    for (int k = 1; k < NUM_BINS - 1; k++) {
        float w = bin_weight ? bin_weight[k] * bin_weight[k] : 1.0f;
        energy += w * mag[k] * mag[k];
    }
    {
        float w0 = bin_weight ? bin_weight[0] * bin_weight[0] : 1.0f;
        float wN = bin_weight ? bin_weight[NUM_BINS-1] * bin_weight[NUM_BINS-1] : 1.0f;
        energy += 0.5f * w0 * mag[0] * mag[0];
        energy += 0.5f * wN * mag[NUM_BINS - 1] * mag[NUM_BINS - 1];
    }

    // Bootstrap: the very first frame seeds BOTH the smoothed energy and the
    // noise reference directly from the observed signal. Without this, a
    // fixed placeholder reference (e.g. 1.0) chosen without knowledge of the
    // real signal's amplitude scale can sit orders of magnitude away from
    // the true noise floor; since the reference is normally only allowed to
    // move while the VAD believes "no speech", and the VAD's own SNR
    // decision is computed FROM that same reference, a bad enough initial
    // mismatch is a self-sustaining deadlock (found empirically during
    // pipeline testing -- see README, "Testing & Validation"). Seeding from
    // the first frame is the standard practical fix used by most simple
    // energy-based VADs, and any resulting misclassification of that first
    // ~10 ms frame washes out quickly once tracking begins.
    if (!ns->vad_bootstrapped) {
        ns->vad_energy_smooth = energy;
        ns->vad_noise_ref = energy;
        ns->vad_bootstrapped = true;
    }

    float attack_c  = tc_to_coeff(VAD_ENERGY_ATTACK_MS, frame_period_ms);
    float release_c = tc_to_coeff(VAD_ENERGY_RELEASE_MS, frame_period_ms);
    float coeff = (energy > ns->vad_energy_smooth) ? attack_c : release_c;
    ns->vad_energy_smooth += coeff * (energy - ns->vad_energy_smooth);

    // Reference "noise-only" energy level. This uses the SAME fast-attack /
    // slow-release philosophy as the per-bin noise tracker below: it follows
    // a FALLING energy trend quickly (so it doesn't lag behind a genuinely
    // quieting band), but only creeps upward slowly -- both while flagged
    // "not active" (the normal case) and, at a much slower rate still,
    // even while flagged "active" (so the estimator is always eventually
    // self-correcting and cannot get permanently stuck, e.g. after a bad
    // startup guess, or fooled by a persistent tone bin_weight didn't fully
    // suppress). Without the slow-upward asymmetry here, a fast-tracking
    // reference will partially "chase" a genuine signal burst upward before
    // the SNR-threshold logic below has even classified it as speech,
    // silently reducing the VAD's own sensitivity to exactly the onset it
    // should be catching -- found empirically during pipeline testing with
    // a synthetic speech-like burst (see README, "Testing & Validation").
    {
        float ref_tc_ms;
        if (ns->vad_energy_smooth < ns->vad_noise_ref) {
            ref_tc_ms = VAD_NOISE_REF_TC_MS;                 // falling: follow quickly
        } else {
            ref_tc_ms = ns->vad_active ? VAD_NOISE_REF_SLOW_TC_MS
                                        : VAD_NOISE_REF_TC_MS * 4.0f;  // rising: always cautious
        }
        float ref_c = tc_to_coeff(ref_tc_ms, frame_period_ms);
        ns->vad_noise_ref += ref_c * (ns->vad_energy_smooth - ns->vad_noise_ref);
    }
    if (ns->vad_noise_ref < 1e-8f) ns->vad_noise_ref = 1e-8f;

    float snr_db = 10.0f * log10f(ns->vad_energy_smooth / ns->vad_noise_ref + 1e-12f);

    bool instantaneous_active = (snr_db > VAD_SNR_THRESHOLD_DB);

    if (instantaneous_active) {
        ns->vad_hold_counter = VAD_HOLD_FRAMES;
        ns->vad_active = true;
    } else if (ns->vad_hold_counter > 0) {
        ns->vad_hold_counter--;
        ns->vad_active = true;    // hangover: still "active" for a few more frames
    } else {
        ns->vad_active = false;
    }

    return ns->vad_active;
}

void noise_estimate_update(NoiseEstimator *ns, const float *mag, bool vad_active,
                            float tracking_speed, float frame_period_ms) {
    if (tracking_speed < 0.0f) tracking_speed = 0.0f;
    if (tracking_speed > 1.0f) tracking_speed = 1.0f;

    // Map tracking_speed -> a release time constant (ms). Higher speed = the
    // estimate is allowed to rise back up faster after a loud noise burst
    // subsides (log-scaled so the middle of the slider feels "linear").
    float release_tc_ms = NOISE_RELEASE_TC_MAX_MS *
        powf(NOISE_RELEASE_TC_MIN_MS / NOISE_RELEASE_TC_MAX_MS, tracking_speed);

    if (vad_active) release_tc_ms *= NOISE_SPEECH_RELEASE_MULT;  // "update slowly during speech"

    float attack_c  = tc_to_coeff(NOISE_ATTACK_TC_MS, frame_period_ms);
    float release_c = tc_to_coeff(release_tc_ms, frame_period_ms);

    for (int k = 0; k < NUM_BINS; k++) {
        float m = mag[k];
        float n = ns->noise_mag[k];
        // Asymmetric smoothing: fast when the signal is BELOW the current
        // noise estimate (we quickly trust that the floor really is that
        // low), slow when it is ABOVE (a loud burst, or speech, must not
        // immediately raise the estimate -- exactly the "minimum tracking"
        // behaviour of minimum-statistics / percentile methods).
        float coeff = (m < n) ? attack_c : release_c;
        ns->noise_mag[k] = n + coeff * (m - n);
    }
}
