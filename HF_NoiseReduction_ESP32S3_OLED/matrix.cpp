/*
 * matrix.cpp
 * ---------------------------------------------------------------------------
 * See matrix.h for the design rationale (why the "gain matrix" is diagonal,
 * and when small dense matrices are used instead).
 * ---------------------------------------------------------------------------
 */

#include "matrix.h"
#include <string.h>

// =============================================================================
// Diagonal matrix / vector operations
// =============================================================================

void mat_diag_apply(const float *W, const complex_t *X, complex_t *Y, int n) {
    for (int k = 0; k < n; k++) {
        Y[k].re = W[k] * X[k].re;
        Y[k].im = W[k] * X[k].im;
    }
}

void mat_diag_apply_inplace(const float *W, complex_t *X, int n) {
    for (int k = 0; k < n; k++) {
        X[k].re *= W[k];
        X[k].im *= W[k];
    }
}

void vec_mul_real(const float *a, const float *b, float *out, int n) {
    for (int k = 0; k < n; k++) out[k] = a[k] * b[k];
}

void vec_smooth(float *state, const float *target, float coeff, int n) {
    for (int k = 0; k < n; k++) {
        state[k] += coeff * (target[k] - state[k]);
    }
}

float tc_to_coeff(float tc_ms, float frame_period_ms) {
    if (tc_ms <= 0.0f) return 1.0f;   // zero time-constant = instantaneous
    return 1.0f - expf(-frame_period_ms / tc_ms);
}

void spectral_smooth3(const float *in, float *out, int n, float center_weight) {
    // Banded (tridiagonal-weight) smoothing across frequency bins:
    //   out[k] = cw*in[k] + sw*(in[k-1] + in[k+1])
    // with sw chosen so the row sums to 1 (unity DC gain, no overall
    // loudness change from the smoothing itself).
    float side_weight = (1.0f - center_weight) * 0.5f;
    for (int k = 0; k < n; k++) {
        float left  = (k > 0)     ? in[k - 1] : in[k];
        float right = (k < n - 1) ? in[k + 1] : in[k];
        out[k] = center_weight * in[k] + side_weight * (left + right);
    }
}

// =============================================================================
// Small dense K x K matrix utilities
// =============================================================================

void matK_mul(const float *A, const float *B, float *out, int k) {
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) {
            float sum = 0.0f;
            for (int m = 0; m < k; m++) {
                sum += A[i * k + m] * B[m * k + j];
            }
            out[i * k + j] = sum;
        }
    }
}

void matK_add_ridge(float *A, int k, float lambda) {
    for (int i = 0; i < k; i++) {
        A[i * k + i] += lambda;
    }
}

bool matK_inverse(const float *A_in, float *out, int k) {
    // Gauss-Jordan elimination with partial pivoting on an augmented
    // [A | I] matrix. k is small (<= MAX_LOCAL_DIM = 8) so this is cheap
    // and we don't need anything fancier (no LU, no SIMD).
    if (k > MAX_LOCAL_DIM) return false;

    float aug[MAX_LOCAL_DIM][2 * MAX_LOCAL_DIM];
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) aug[i][j] = A_in[i * k + j];
        for (int j = 0; j < k; j++) aug[i][k + j] = (i == j) ? 1.0f : 0.0f;
    }

    for (int col = 0; col < k; col++) {
        // Partial pivot: find largest magnitude in this column at/below row col
        int pivot = col;
        float best = fabsf(aug[col][col]);
        for (int r = col + 1; r < k; r++) {
            float v = fabsf(aug[r][col]);
            if (v > best) { best = v; pivot = r; }
        }
        if (best < 1e-9f) return false;  // singular even with ridge added upstream

        if (pivot != col) {
            for (int j = 0; j < 2 * k; j++) {
                float tmp = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = tmp;
            }
        }

        float inv_piv = 1.0f / aug[col][col];
        for (int j = 0; j < 2 * k; j++) aug[col][j] *= inv_piv;

        for (int r = 0; r < k; r++) {
            if (r == col) continue;
            float factor = aug[r][col];
            if (factor == 0.0f) continue;
            for (int j = 0; j < 2 * k; j++) {
                aug[r][j] -= factor * aug[col][j];
            }
        }
    }

    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) {
            out[i * k + j] = aug[i][k + j];
        }
    }
    return true;
}

void matK_apply_vec(const float *A, const float *x, float *out, int k) {
    for (int i = 0; i < k; i++) {
        float sum = 0.0f;
        for (int j = 0; j < k; j++) sum += A[i * k + j] * x[j];
        out[i] = sum;
    }
}

// =============================================================================
// Lightweight diagonal LMS adaptive matrix
// =============================================================================

void lms_init(LmsState *s, float mu) {
    for (int i = 0; i < NUM_LMS_BANDS; i++) s->w[i] = 1.0f;
    s->mu = mu;
}

void lms_update(LmsState *s, const float *x, const float *d, float *y,
                 float w_min, float w_max) {
    for (int k = 0; k < NUM_LMS_BANDS; k++) {
        y[k] = s->w[k] * x[k];
        float e = d[k] - y[k];
        float w_new = s->w[k] + s->mu * e * x[k];
        if (w_new < w_min) w_new = w_min;
        if (w_new > w_max) w_new = w_max;
        s->w[k] = w_new;
    }
}
