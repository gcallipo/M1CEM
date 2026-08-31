/*
 * heterodyne.h
 * ---------------------------------------------------------------------------
 * Detects narrowband tones -- heterodynes, carrier whistles, adjacent-channel
 * CW -- and builds an adaptive per-bin notch gain matrix (again a diagonal
 * matrix; see matrix.h) that nulls them while leaving everything else alone.
 *
 * PIPELINE
 *   1. Peak detection: a bin is a *candidate* tone if it is a local maximum
 *      and its magnitude exceeds the average of its surrounding neighbour
 *      bins by a configurable ratio (prominence / signal-to-neighbour test).
 *   2. Multi-frame tracking: candidates are matched frame-to-frame to a
 *      small set of persistent tracks by proximity in frequency (allows the
 *      tone to drift, e.g. as an HF heterodyne slowly shifts). A track must
 *      be confirmed for several consecutive frames (persistence) before it
 *      is allowed to start notching -- this is what keeps transient speech
 *      harmonics/plosives from being mistaken for a steady heterodyne.
 *   3. Confirmed tracks generate a smooth (raised-cosine) notch around their
 *      centre frequency, with independently configurable depth and width.
 *      Notch depth is smoothed with a fast attack / slow release envelope
 *      per the brief ("track drifting heterodynes... attack quickly,
 *      release slowly"), so a tone is suppressed almost as soon as it's
 *      confirmed, but a notch doesn't "snap" back open the instant a CW
 *      signal pauses between dits and dahs (which would sound choppy). The
 *      notch's CENTRE FREQUENCY is smoothed the same way (see
 *      TONE_FREQ_SMOOTH_TC_MS in heterodyne.cpp): a track's matched bin can
 *      change by a couple of bins from one ~11.6 ms frame to the next as a
 *      tone drifts or someone tunes across it, and without smoothing that
 *      the notch shape would snap sideways by a full bin (~43 Hz) on every
 *      such frame -- audible as a buzz on a steadily sweeping tone. The
 *      notch is built from this smoothed, continuous (sub-bin) position
 *      rather than the raw integer bin, so it glides instead of hopping.
 * ---------------------------------------------------------------------------
 */

#ifndef HETERODYNE_H
#define HETERODYNE_H

#include "matrix.h"
#include "settings.h"

#define MAX_TONE_TRACKS       8
#define TONE_PERSIST_CONFIRM  3     // consecutive detections needed to confirm a track
#define TONE_MISS_TOLERANCE   6     // consecutive misses allowed before a track is dropped

typedef struct {
    bool  in_use;
    float freq_hz;           // smoothed tracked frequency (see TONE_FREQ_SMOOTH_TC_MS
                              // in heterodyne.cpp) -- drives the notch's actual centre
                              // position; `bin` below is only the raw match target
    int   bin;               // nearest bin index (match target only, see freq_hz above)
    int   hit_count;        // consecutive-detection counter (confirmation)
    int   miss_count;       // consecutive-miss counter (drop-out)
    bool  confirmed;        // true once hit_count >= TONE_PERSIST_CONFIRM
    float depth_env;        // current smoothed notch depth, 0..1 (envelope)
} ToneTrack;

typedef struct {
    ToneTrack tracks[MAX_TONE_TRACKS];
} HeterodyneState;

void heterodyne_init(HeterodyneState *hs);

// Runs peak detection + tracking for the current frame's magnitude spectrum,
// and writes the resulting notch gain matrix (diagonal, NUM_BINS floats,
// each in [1-notch_depth, 1]) into notch_gain_out.
void heterodyne_process(HeterodyneState *hs,
                         const float *mag,
                         float *notch_gain_out,
                         float notch_depth,
                         float notch_width_hz,
                         float attack_ms,
                         float release_ms,
                         float frame_period_ms);

#endif // HETERODYNE_H
