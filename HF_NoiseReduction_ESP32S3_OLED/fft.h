/*
 * fft.h
 * ---------------------------------------------------------------------------
 * Real-input FFT for the HF noise-reduction pipeline.
 *
 * ALGORITHM: "real FFT via half-size complex FFT"
 * -----------------------------------------------------------------------------
 * A real-valued signal's spectrum is conjugate-symmetric, so all information
 * is contained in bins 0 .. N/2 (NUM_BINS = N/2+1 values) rather than the
 * full N complex values a naive complex FFT of the real signal would produce.
 * We exploit that two independent real signals can be packed into one
 * complex signal for the price of one FFT, but here we go one step further
 * and pack a *single* real signal's even/odd samples into one complex
 * sequence of half the length, run one M = N/2-point complex FFT, and then
 * unpack the result into the N/2+1 unique real-spectrum bins with a small
 * amount of post-processing ("butterfly" combine per bin). This is the same
 * technique used by mature libraries (FFTW's rfft, KissFFT's real transform,
 * etc.) and halves both the FFT size and the twiddle-factor complexity
 * compared to zero-padding the imaginary part and running a full N-point
 * complex FFT.
 *
 * The forward and inverse formulas implemented in fft.cpp were derived by
 * hand and numerically verified against NumPy's rfft/irfft for N=16 and
 * N=1024 (including verifying that applying a real per-bin gain to the
 * half-spectrum and inverse-transforming reproduces the same real-valued
 * output as SciPy/NumPy's irfft on the equivalent gained full spectrum, to
 * machine precision). See the project README, "Matrix & DSP Mathematics"
 * section, for the full derivation.
 * ---------------------------------------------------------------------------
 */

#ifndef FFT_H
#define FFT_H

#include "matrix.h"
#include "settings.h"

// Precomputed periodic ("DFT-even") Hann analysis window, length FFT_SIZE:
//     w[n] = 0.5 - 0.5*cos(2*pi*n / FFT_SIZE)
// A single Hann window applied only at analysis, combined with plain
// overlap-add (no separate synthesis window) at 50% hop, satisfies the
// COLA (Constant-Overlap-Add) condition *exactly*: summed contributions
// from two overlapping shifted copies of this window equal 1.0 everywhere
// in steady state (verified numerically to double-precision machine
// epsilon during development). That is what guarantees the reconstructed
// audio has no amplitude "pumping" at the hop rate when the per-bin gain
// matrix is neutral (all 1.0).
// (Declared non-const because fft_init() fills it in at startup; treat it
// as read-only everywhere else in the codebase.)
extern float g_analysis_window[FFT_SIZE];

// One-time setup: computes twiddle factor tables, bit-reversal table, and
// the analysis window. Must be called once from setup() before any FFT call.
void fft_init(void);

// Forward transform.
//   frame_windowed : FFT_SIZE floats, time-domain samples ALREADY multiplied
//                    by g_analysis_window by the caller (see audio.cpp).
//   X              : output, NUM_BINS complex_t values (bins 0 .. FFT_SIZE/2)
void real_fft_forward(const float *frame_windowed, complex_t *X);

// Inverse transform.
//   X         : NUM_BINS complex_t values (Hermitian half-spectrum). Any
//               real, frequency-symmetric gain applied to X before calling
//               this preserves the Hermitian property, so the output is
//               guaranteed real (up to floating-point rounding).
//   frame_out : output, FFT_SIZE floats (NOT yet windowed for synthesis --
//               caller applies the synthesis window and overlap-adds).
void real_fft_inverse(const complex_t *X, float *frame_out);

#endif // FFT_H
