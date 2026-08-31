 /*
 * wiener.h
 * ---------------------------------------------------------------------------
 * Frequency-domain Wiener filter + spectral subtraction, expressed as a
 * diagonal gain matrix (see matrix.h for why "matrix" means "diagonal" here).
 *
 * SCALAR WIENER FILTER (per bin k):
 *     H(k) = S(k) / (S(k) + N(k))
 * where S(k) is the estimated clean-signal power and N(k) the estimated
 * noise power in bin k. This is the classical minimum-mean-square-error
 * (MMSE) linear estimator for a signal buried in additive, uncorrelated
 * noise, derived from Wiener-Hopf theory.
 *
 * MATRIX WIENER FILTER, AND WHY IT COLLAPSES TO THE SCALAR FORM HERE:
 *     H = Rs * (Rs + Rn)^-1
 * In general Rs and Rn are full covariance matrices over some vector of
 * observations, and computing H requires inverting (Rs + Rn). If we choose
 * our "vector of observations" to be the FFT bins, and make the (very
 * standard, and here well-justified) assumption that noise and signal
 * power are uncorrelated ACROSS bins -- i.e. Rs = diag(S(0), S(1), ..., S(M))
 * and Rn = diag(N(0), ..., N(M)) -- then:
 *     Rs + Rn = diag(S(0)+N(0), ..., S(M)+N(M))
 * and the inverse of a diagonal matrix is simply the reciprocal of each
 * diagonal entry:
 *     (Rs + Rn)^-1 = diag(1/(S(0)+N(0)), ..., 1/(S(M)+N(M)))
 * so:
 *     H = Rs * (Rs+Rn)^-1 = diag( S(0)/(S(0)+N(0)), ..., S(M)/(S(M)+N(M)) )
 * which is exactly NUM_BINS independent applications of the scalar formula
 * above. This is not a simplification made for convenience; the diagonal
 * assumption is the standard one used throughout the speech-enhancement
 * literature (it is exact for spectrally white/uncorrelated noise, and a
 * good approximation for atmospheric/HF noise), and a *dense* (non-diagonal)
 * Rs/Rn would require estimating and inverting a 513x513 covariance matrix
 * per frame, which is computationally infeasible in real time on this
 * hardware (see matrix.h header comment for the operation count).
 *
 * DECISION-DIRECTED A PRIORI SNR ESTIMATION:
 * Rather than estimate S(k) directly (hard -- we only observe signal+noise),
 * this module uses the Ephraim-Malah "decision-directed" approach, standard
 * in professional spectral-subtraction / Wiener implementations specifically
 * because it suppresses "musical noise" (the isolated, randomly-appearing
 * narrowband tones that plague naive spectral subtraction):
 *     xi(k,n)   = alpha * G(k,n-1)^2 * |X(k,n-1)|^2 / N(k,n)   +   (1-alpha) * max(gamma(k,n)-1, 0)
 *     gamma(k,n)= |X(k,n)|^2 / N(k,n)                            (a posteriori SNR)
 *     H(k,n)    = xi(k,n) / (1 + xi(k,n))                        (Wiener gain from a priori SNR)
 * alpha (~0.98) blends the previous frame's *result* back in as a smoothed
 * estimate of the true SNR, which is what damps down the random gain
 * fluctuations that create musical noise.
 * ---------------------------------------------------------------------------
 */

#ifndef WIENER_H
#define WIENER_H

#include "matrix.h"
#include "settings.h"

typedef struct {
    float prev_gain[NUM_BINS];      // G(k, n-1)
    float prev_sig_mag2[NUM_BINS];  // |X(k,n-1)|^2  (previous frame's raw power)
} WienerState;

void wiener_init(WienerState *st);

// Computes the combined noise-reduction gain matrix (diagonal, i.e. a plain
// vector of NUM_BINS real gains in [0,1]) from the current spectrum and the
// current noise magnitude estimate. Blends:
//   - decision-directed Wiener gain (musical-noise-resistant, primary method)
//   - classical magnitude spectral subtraction with over-subtraction (alpha)
//     and a spectral floor (beta), which gives explicit, independent control
//     over aggressiveness that pure Wiener gain doesn't expose
//   - cross-bin (banded-matrix) smoothing of the final gain curve, to avoid
//     "combing" artifacts between adjacent bins
// Result is written to gain_out (NUM_BINS floats, each in [floor, 1]).
void wiener_compute_gain(WienerState *st,
                          const complex_t *X,
                          const float *noise_mag,
                          float *gain_out,
                          float oversubtraction,
                          float spectral_floor,
                          float nr_amount);

#endif // WIENER_H
