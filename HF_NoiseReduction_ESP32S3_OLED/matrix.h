/*
 * matrix.h
 * ---------------------------------------------------------------------------
 * Matrix / vector algebra layer for the HF noise-reduction pipeline.
 *
 * WHY "MATRIX" MEANS WHAT IT MEANS HERE
 * -----------------------------------------------------------------------------
 * The design brief asks for the noise-reduction stage to be expressed as a
 * matrix operation  Y = W * X , with W an "adaptive weighting matrix". This
 * is implemented *honestly* rather than decoratively:
 *
 *   X (the FFT spectrum) is a complex vector of NUM_BINS = 513 elements.
 *   W, the per-bin gain matrix, is mathematically a 513x513 DIAGONAL matrix:
 *
 *       [ w0   0    0  ...   0  ]   [X0]     [w0*X0]
 *       [ 0    w1   0  ...   0  ] * [X1]  =  [w1*X1]   =  Y
 *       [ 0    0   w2  ...   0  ]   [X2]     [w2*X2]
 *       [ ...            ...   ]   [..]     [ ... ]
 *       [ 0    0    0  ... w512]  [X512]   [w512*X512]
 *
 * A diagonal matrix multiply is *identical* to an element-wise vector
 * multiply, so that is how it is implemented (mat_diag_apply below) --
 * this is not a shortcut, it is the mathematically exact and computationally
 * optimal way to apply a diagonal matrix. Storing and multiplying a *dense*
 * 513x513 matrix would need 513^2 ~= 263,000 multiply-adds per frame just 
 * for the gain stage (and a full covariance-based Wiener solution would
 * additionally require inverting a 513x513 matrix -- O(513^3) ~= 1.3x10^8
 * operations -- per 11.6 ms frame). That is roughly 11 GFLOP/s sustained,
 * which is far beyond what an ESP32-S3 (a few hundred MFLOP/s in practice)
 * can deliver in real time. No real-time embedded noise reduction system
 * (or, for that matter, desktop software) inverts a full per-bin covariance
 * matrix; they all use the diagonal (bin-independent) approximation, which
 * is mathematically the maximum-likelihood solution anyway *when the noise
 * is assumed spectrally uncorrelated between bins* -- a very good assumption
 * for atmospheric/white noise, and a fair one for narrowband heterodynes
 * once they are isolated to single bins.
 *
 * Where genuine *cross-bin* (off-diagonal) structure is useful -- smoothing
 * a gain curve across neighbouring bins so it doesn't "comb" the audio, or
 * locally correlating a small cluster of bins to improve tone detection --
 * this file also provides small (3x3 to 8x8) dense matrix routines that
 * operate on local bin neighbourhoods. These are genuinely non-diagonal
 * matrix operations, sized so they stay cheap (a KxK matrix inversion is
 * O(K^3); for K=5 that's 125 ops, done a handful of times per frame, not
 * 513 times).
 * ---------------------------------------------------------------------------
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Complex number (single precision). Plain struct, no operator overloading
// overhead surprises -- explicit function calls make the FLOP count visible.
// -----------------------------------------------------------------------------
typedef struct {
    float re;
    float im;
} complex_t;

static inline complex_t c_add(complex_t a, complex_t b) {
    complex_t r = { a.re + b.re, a.im + b.im };
    return r;
}
static inline complex_t c_sub(complex_t a, complex_t b) {
    complex_t r = { a.re - b.re, a.im - b.im };
    return r;
}
static inline complex_t c_mul(complex_t a, complex_t b) {
    complex_t r = { a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re };
    return r;
}
static inline complex_t c_conj(complex_t a) {
    complex_t r = { a.re, -a.im };
    return r;
}
static inline complex_t c_scale(complex_t a, float s) {
    complex_t r = { a.re * s, a.im * s };
    return r;
}
static inline float c_mag2(complex_t a) {          // |a|^2  (avoids sqrt where possible)
    return a.re * a.re + a.im * a.im;
}
static inline float c_mag(complex_t a) {
    return sqrtf(c_mag2(a));
}

// =============================================================================
// DIAGONAL "MATRIX" (GAIN VECTOR) OPERATIONS
// =============================================================================

// Y[k] = W[k] * X[k]   (element-wise; W real, X complex) -- the diagonal
// matrix multiply Y = W . X described above.
void mat_diag_apply(const float *W, const complex_t *X, complex_t *Y, int n);

// In-place version: X[k] *= W[k]
void mat_diag_apply_inplace(const float *W, complex_t *X, int n);

// Element-wise vector multiply of two real vectors (used to combine the
// noise-reduction gain matrix with the notch gain matrix, since applying two
// diagonal matrices in sequence, W2*(W1*X), equals applying their diagonal
// product (W2 . W1) to X once -- again exact, not an approximation).
void vec_mul_real(const float *a, const float *b, float *out, int n);

// Exponential (one-pole) smoothing of a real vector toward a target vector:
//   state[k] += coeff * (target[k] - state[k])
// Used throughout (noise floor tracking, gain-matrix attack/release, notch
// engage/release). coeff in [0,1]; coeff derived from a millisecond time
// constant via tc_to_coeff() below.
void vec_smooth(float *state, const float *target, float coeff, int n);

// Convert a time constant (ms) and the frame period (ms) into a one-pole
// smoothing coefficient: coeff = 1 - exp(-frame_period_ms / tc_ms)
float tc_to_coeff(float tc_ms, float frame_period_ms);

// Simple 3-tap (or generalizable odd-length) moving-average smoothing across
// FREQUENCY (i.e. across the bin index, not across time) -- used to keep the
// gain matrix from creating an audible "comb" between adjacent bins. This is
// a genuine (banded, tridiagonal-like) matrix operation: each output is a
// weighted sum of itself and its immediate neighbours, i.e. multiplication by
// a banded matrix with a small number of non-zero diagonals.
void spectral_smooth3(const float *in, float *out, int n, float center_weight);

// =============================================================================
// SMALL DENSE MATRIX UTILITIES (K x K, K <= MAX_LOCAL_DIM)
// -----------------------------------------------------------------------------
// Used for local (few-bin) covariance smoothing / correlation-based tone
// confirmation -- genuine off-diagonal matrix math, deliberately kept small
// so it remains real-time-feasible (see file header).
// =============================================================================

#define MAX_LOCAL_DIM 8

// out = A * B,  A,B,out are k x k, row-major float arrays
void matK_mul(const float *A, const float *B, float *out, int k);

// out = A + lambda * I   (Tikhonov / ridge regularization, keeps inversion
// numerically stable even for near-singular local covariance estimates)
void matK_add_ridge(float *A, int k, float lambda);

// Gauss-Jordan inversion of a k x k matrix (k <= MAX_LOCAL_DIM). Returns
// false if the matrix is singular even after ridge regularization (caller
// should fall back to an identity/no-op gain in that case).
bool matK_inverse(const float *A, float *out, int k);

// out = A * x   (k x k times k x 1)
void matK_apply_vec(const float *A, const float *x, float *out, int k);

// =============================================================================
// LIGHTWEIGHT LMS ADAPTIVE MATRIX (diagonal), per the design brief's optional
// "machine-learning-style" stage:  W(n+1) = W(n) + mu * e * conj(x)
// -----------------------------------------------------------------------------
// This is applied to a *reduced* feature vector (critical-band grouped
// magnitudes, NUM_LMS_BANDS elements -- see settings/heterodyne) rather than
// to all 513 raw bins, both to keep the adaptive weight vector small (fast,
// no dynamic memory) and because adapting one weight per raw FFT bin from a
// single error sample per frame converges far too slowly and noisily to be
// useful (513 independent 1-tap LMS filters, each updated once every 11.6 ms,
// each fed a single noisy real-valued error term, will not converge to
// anything meaningful within the time scales that matter for speech). Using
// a small number of perceptually-grouped bands makes each weight's update
// statistically well-conditioned.
// =============================================================================

#define NUM_LMS_BANDS 32

typedef struct {
    float w[NUM_LMS_BANDS];      // adaptive diagonal weights, init to 1.0
    float mu;                    // adaptation step size
} LmsState;

void lms_init(LmsState *s, float mu);

// One LMS update step:
//   y[k]      = w[k] * x[k]                  (apply current weight)
//   e[k]      = d[k] - y[k]                  (error vs. desired/reference)
//   w[k]     += mu * e[k] * x[k]             (gradient step)
// x, d, y are length NUM_LMS_BANDS. Weights are clamped to [w_min, w_max]
// for stability (unconstrained LMS on a non-stationary, quantized embedded
// signal can otherwise drift or blow up).
void lms_update(LmsState *s, const float *x, const float *d, float *y,
                 float w_min, float w_max);

#endif // MATRIX_H
