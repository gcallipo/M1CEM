/*
 * fft.cpp
 * ---------------------------------------------------------------------------
 * See fft.h for the algorithm overview. This file contains:
 *   1. A standard iterative radix-2 decimation-in-time complex FFT of size
 *      HALF_FFT (= FFT_SIZE/2), used as the computational core.
 *   2. The real-FFT forward/inverse wrappers that pack/unpack a real signal
 *      of length FFT_SIZE into/from that half-size complex FFT.
 *
 * All trig values (twiddle factors, bit-reversal indices, window) are
 * precomputed once in fft_init() and stored in static RAM -- no sin/cos and
 * no dynamic allocation in the real-time path, per the project's
 * no-heap-allocation-in-the-hot-path requirement.
 * ---------------------------------------------------------------------------
 */

#include "fft.h"
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// -----------------------------------------------------------------------------
// Static tables (internal RAM, computed once in fft_init)
// -----------------------------------------------------------------------------

// Twiddle factors for the HALF_FFT-point complex FFT: W[j] = exp(-2*pi*i*j/HALF_FFT),
// j = 0 .. HALF_FFT/2 - 1. Indexed with stride during each FFT stage (see fft_core).
static complex_t s_twiddle_half[HALF_FFT / 2];

// Bit-reversal permutation indices for HALF_FFT points.
static uint16_t s_bitrev[HALF_FFT];

// Twiddle factors for the FULL FFT_SIZE, but only the k=0..HALF_FFT unique
// bins we actually need for the real-FFT pack/unpack step:
// twN[k] = exp(-2*pi*i*k/FFT_SIZE), k = 0 .. HALF_FFT (NUM_BINS values).
static complex_t s_twiddle_full[NUM_BINS];

// Hann analysis window, exposed to callers (audio.cpp) via the
// `extern float g_analysis_window[FFT_SIZE]` declaration in fft.h.
// Filled in once by fft_init(); treated as read-only thereafter.
float g_analysis_window[FFT_SIZE];

// Scratch buffer used only inside real_fft_forward/inverse (static -> no
// stack pressure, no heap allocation; safe because the DSP task is
// single-threaded through this module).
static complex_t s_scratch[HALF_FFT];

// -----------------------------------------------------------------------------
// fft_init
// -----------------------------------------------------------------------------

void fft_init(void) {
    // --- Bit-reversal table for HALF_FFT points ---
    int bits = 0;
    for (int t = HALF_FFT; t > 1; t >>= 1) bits++;   // log2(HALF_FFT)
    for (int i = 0; i < HALF_FFT; i++) {
        uint16_t rev = 0;
        int v = i;
        for (int b = 0; b < bits; b++) {
            rev = (rev << 1) | (v & 1);
            v >>= 1;
        }
        s_bitrev[i] = rev;
    }

    // --- Twiddle factors for the HALF_FFT-point core FFT ---
    for (int j = 0; j < HALF_FFT / 2; j++) {
        float angle = -2.0f * (float)M_PI * (float)j / (float)HALF_FFT;
        s_twiddle_half[j].re = cosf(angle);
        s_twiddle_half[j].im = sinf(angle);
    }

    // --- Twiddle factors for the full FFT_SIZE, bins 0..HALF_FFT ---
    for (int k = 0; k <= HALF_FFT; k++) {
        float angle = -2.0f * (float)M_PI * (float)k / (float)FFT_SIZE;
        s_twiddle_full[k].re = cosf(angle);
        s_twiddle_full[k].im = sinf(angle);
    }

    // --- Hann analysis window (periodic / DFT-even form) ---
    for (int n = 0; n < FFT_SIZE; n++) {
        g_analysis_window[n] =
            0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)n / (float)FFT_SIZE);
    }
}

// -----------------------------------------------------------------------------
// fft_core: in-place radix-2 DIT complex FFT of length HALF_FFT.
// If `inverse` is true, computes the true inverse DFT (includes the 1/M
// scaling), via the conjugate-trick: IFFT(X) = conj(FFT(conj(X))) / M.
// -----------------------------------------------------------------------------

static void fft_core(complex_t *data, bool inverse) {
    const int M = HALF_FFT;

    if (inverse) {
        for (int i = 0; i < M; i++) data[i].im = -data[i].im;
    }

    // Bit-reversal permutation
    for (int i = 0; i < M; i++) {
        uint16_t r = s_bitrev[i];
        if (i < r) {
            complex_t tmp = data[i];
            data[i] = data[r];
            data[r] = tmp;
        }
    }

    // Iterative butterfly stages
    for (int len = 2; len <= M; len <<= 1) {
        int half = len >> 1;
        int step = M / len;   // stride into s_twiddle_half (size M/2)
        for (int i = 0; i < M; i += len) {
            int tw_idx = 0;
            for (int j = 0; j < half; j++) {
                complex_t w = s_twiddle_half[tw_idx];
                tw_idx += step;
                complex_t u = data[i + j];
                complex_t t = c_mul(w, data[i + j + half]);
                data[i + j]        = c_add(u, t);
                data[i + j + half] = c_sub(u, t);
            }
        }
    }

    if (inverse) {
        float inv_M = 1.0f / (float)M;
        for (int i = 0; i < M; i++) {
            data[i].re *= inv_M;
            data[i].im = -data[i].im * inv_M;
        }
    }
}

// -----------------------------------------------------------------------------
// real_fft_forward
// -----------------------------------------------------------------------------
//
// Derivation (verified numerically against NumPy's rfft to double-precision
// round-off for N=16 and N=1024 during development -- see README):
//
//   Pack: z[m] = x[2m] + j*x[2m+1],  m = 0..M-1        (M = FFT_SIZE/2)
//   Z    = FFT_M(z)                                     (core complex FFT)
//
//   For k = 0..M  (M+1 = NUM_BINS unique bins of the real spectrum):
//     Zk    = Z[k mod M]
//     Zmk   = Z[(M-k) mod M]
//     Xe[k] = 0.5 * (Zk + conj(Zmk))          -- DFT of the even samples
//     Xo[k] = 0.5 * (Zk - conj(Zmk)) / j      -- DFT of the odd samples
//           = ( 0.5*Im(diff), -0.5*Re(diff) )  where diff = Zk - conj(Zmk)
//     X[k]  = Xe[k] + W_N^k * Xo[k]            (W_N^k = exp(-2*pi*i*k/N))
//
void real_fft_forward(const float *frame_windowed, complex_t *X) {
    const int M = HALF_FFT;

    // Pack even/odd samples into the complex scratch buffer.
    for (int m = 0; m < M; m++) {
        s_scratch[m].re = frame_windowed[2 * m];
        s_scratch[m].im = frame_windowed[2 * m + 1];
    }

    fft_core(s_scratch, false);   // Z = FFT_M(z)

    for (int k = 0; k <= M; k++) {
        complex_t Zk  = s_scratch[k % M];
        complex_t Zmk = s_scratch[(M - k) % M];
        complex_t Zmk_conj = c_conj(Zmk);

        complex_t sum  = c_add(Zk, Zmk_conj);
        complex_t diff = c_sub(Zk, Zmk_conj);

        complex_t Xe = c_scale(sum, 0.5f);
        complex_t Xo = { 0.5f * diff.im, -0.5f * diff.re };

        complex_t tw = s_twiddle_full[k];
        X[k] = c_add(Xe, c_mul(tw, Xo));
    }
}

// -----------------------------------------------------------------------------
// real_fft_inverse
// -----------------------------------------------------------------------------
//
// Inverts the above exactly (see README for the full derivation):
//
//   For r = 0..M-1:
//     twr  = W_N^r
//     Xr   = X[r]
//     Xmr  = X[M-r]                         (in range because X holds 0..M)
//     Xe   = 0.5 * (Xr + conj(Xmr))
//     diff = Xr - conj(Xmr)
//     Xo   = 0.5 * diff * conj(twr)          (dividing by twr == multiplying
//                                              by conj(twr) since |twr| = 1)
//     Z[r] = Xe + j*Xo
//   z = IFFT_M(Z)
//   x[2m] = Re(z[m]),  x[2m+1] = Im(z[m])
//
void real_fft_inverse(const complex_t *X, float *frame_out) {
    const int M = HALF_FFT;

    for (int r = 0; r < M; r++) {
        complex_t twr = s_twiddle_full[r];
        complex_t Xr  = X[r];
        complex_t Xmr = X[M - r];
        complex_t Xmr_conj = c_conj(Xmr);

        complex_t Xe   = c_scale(c_add(Xr, Xmr_conj), 0.5f);
        complex_t diff = c_sub(Xr, Xmr_conj);
        complex_t twr_conj = c_conj(twr);
        complex_t Xo   = c_scale(c_mul(diff, twr_conj), 0.5f);

        // Z[r] = Xe + j*Xo  =>  re = Xe.re - Xo.im ,  im = Xe.im + Xo.re
        s_scratch[r].re = Xe.re - Xo.im;
        s_scratch[r].im = Xe.im + Xo.re;
    }

    fft_core(s_scratch, true);   // z = IFFT_M(Z), includes 1/M scaling

    for (int m = 0; m < M; m++) {
        frame_out[2 * m]     = s_scratch[m].re;
        frame_out[2 * m + 1] = s_scratch[m].im;
    }
}
