/*
 * audio.cpp
 * ---------------------------------------------------------------------------
 * See audio.h for the architectural overview.
 * ---------------------------------------------------------------------------
 */

#include "audio.h"
#include "matrix.h"
#include "fft.h"
#include "noise.h"
#include "wiener.h"
#include "heterodyne.h"

#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// -----------------------------------------------------------------------------
// VU meter tunables (OLED dashboard only -- not exposed as a runtime
// g_params knob, same rationale as noise.cpp's local VAD tunables: this
// shapes the meter's *ballistics*, not something that needs live tuning).
//   VU_METER_MIN_DB    - input level (dBFS) that reads as an EMPTY bar;
//                         0 dBFS (full-scale) always reads FULL.
//   VU_METER_ATTACK_MS - near-instant rise, so the bar catches peaks
//   VU_METER_RELEASE_MS- classic VU-style decay, slow enough to be
//                         readable rather than flickering every ~11.6 ms
//                         frame, fast enough to still feel "live"
// -----------------------------------------------------------------------------
#define VU_METER_MIN_DB     -50.0f
#define VU_METER_ATTACK_MS   10.0f
#define VU_METER_RELEASE_MS 300.0f

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

AudioPerfStats g_perf;
DspStatus      g_dsp_status;

// Legacy driver/i2s.h: no persistent C++ object -- the driver is addressed
// by port number (I2S_PORT_NUM, see settings.h) after i2s_driver_install().

static RingBufferF s_input_ring;    // mono float samples, PCM1808 -> DSP
static RingBufferF s_output_ring;   // mono float samples, DSP -> PCM5102

static SemaphoreHandle_t s_data_ready_sem = NULL;   // "new input samples are available"

static TaskHandle_t s_audio_io_task_handle = NULL;
static TaskHandle_t s_dsp_task_handle = NULL;

// =============================================================================
// Ring buffer implementation
// -----------------------------------------------------------------------------
// Correctness note on lock-free SPSC without std::atomic: on Xtensa/ESP32,
// 32-bit aligned reads/writes of a uint32_t are naturally atomic at the ISA
// level, and each index is written by exactly one of the two tasks (classic
// single-producer/single-consumer discipline), so ordinary volatile uint32_t
// counters are sufficient here -- there is no read-modify-write race because
// each side only ever reads the OTHER side's index and writes its OWN. Index
// arithmetic uses unsigned (uint32_t) wraparound, which C guarantees is
// well-defined modulo 2^32; masking with (RING_BUFFER_SIZE-1) on every access
// converts the free-running counter into a valid buffer offset regardless of
// how many times it has wrapped, so the buffer works correctly for
// indefinite (multi-day) continuous runtime.
// =============================================================================

void ring_init(RingBufferF *rb) {
    memset(rb->data, 0, sizeof(rb->data));
    rb->write_idx = 0;
    rb->read_idx = 0;
    rb->overrun_count = 0;
    rb->underrun_count = 0;
}

uint32_t ring_available(const RingBufferF *rb) {
    return (uint32_t)(rb->write_idx - rb->read_idx);   // wraparound-safe unsigned subtraction
}

uint32_t ring_push(RingBufferF *rb, const float *src, uint32_t n) {
    uint32_t free_space = RING_BUFFER_SIZE - ring_available(rb);
    uint32_t to_write = (n < free_space) ? n : free_space;
    for (uint32_t i = 0; i < to_write; i++) {
        rb->data[(rb->write_idx + i) & (RING_BUFFER_SIZE - 1)] = src[i];
    }
    rb->write_idx += to_write;
    if (to_write < n) rb->overrun_count += (n - to_write);
    return to_write;
}

uint32_t ring_pop(RingBufferF *rb, float *dst, uint32_t n) {
    uint32_t avail = ring_available(rb);
    uint32_t to_read = (n < avail) ? n : avail;
    for (uint32_t i = 0; i < to_read; i++) {
        dst[i] = rb->data[(rb->read_idx + i) & (RING_BUFFER_SIZE - 1)];
    }
    rb->read_idx += to_read;
    if (to_read < n) {
        // Underrun: zero-fill the shortfall so the I2S TX side always gets a
        // full, glitch-free (if silent) buffer rather than stale/garbage data.
        for (uint32_t i = to_read; i < n; i++) dst[i] = 0.0f;
        rb->underrun_count += (n - to_read);
    }
    return to_read;
}

// =============================================================================
// I2S setup (legacy driver/i2s.h -- see README "Troubleshooting" for why
// this project uses the legacy driver rather than i2s_std/ESP_I2S: it is
// the one I2S API that has existed essentially unchanged across the whole
// arduino-esp32 2.x series, which is what maximizes compatibility with
// whatever board-package version is actually installed.)
// =============================================================================

static void i2s_setup(void) {
    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    i2s_config.sample_rate = SAMPLE_RATE_HZ;
    // 32-bit slots (-> 64 BCK cycles/frame), NOT 16-bit (-> 32 BCK/frame).
    // This matters a lot more than it might look: the PCM1808's datasheet
    // states plainly that in SLAVE mode it "accepts 64-BCK/frame or
    // 48-BCK/frame format ... not 32-BCK/frame format." A 16-bit slot width
    // here would silently violate that and the chip simply never produces
    // any DOUT activity at all -- exactly the "totally dead, zero toggling"
    // symptom this setting caused during hardware bring-up (see README,
    // "Diagnosing 'no audio'"). We still only carry 16 bits of real
    // resolution through the DSP pipeline (see MONO_INPUT_IS_LEFT handling
    // below); the extra 16 bits of slot width are just standard I2S
    // left-justified padding, present purely to satisfy the ADC/DAC's
    // expected BCK-per-frame timing.
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;   // both slots present -> stereo bus
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags = 0;   // default interrupt priority
    i2s_config.dma_buf_count = 4;
    i2s_config.dma_buf_len = I2S_CHUNK_FRAMES;
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = true;   // underflow -> silence, not garbage/repeat

    esp_err_t err = i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("FATAL: i2s_driver_install failed (err=%d)\n", (int)err);
        return;
    }

    // One shared BCLK/WS pair, one MCLK -- this is what keeps the PCM1808
    // (RX, slave) and PCM5102 (TX, slave) sample-locked to each other with
    // zero long-term drift (see settings.h pin map notes).
    i2s_pin_config_t pin_config = {};
    pin_config.mck_io_num   = PIN_I2S_MCLK;
    pin_config.bck_io_num   = PIN_I2S_BCLK;
    pin_config.ws_io_num    = PIN_I2S_WS;
    pin_config.data_out_num = PIN_I2S_DOUT;
    pin_config.data_in_num  = PIN_I2S_DIN;

    err = i2s_set_pin(I2S_PORT_NUM, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("FATAL: i2s_set_pin failed (err=%d) -- check wiring/pins "
                       "in settings.h\n", (int)err);
    }
}

// =============================================================================
// Core 0: audio_io_task
// -----------------------------------------------------------------------------
// Reads a block of stereo 32-bit-slot frames from the PCM1808, extracts the
// mono channel of interest (top 16 bits of its 32-bit slot -- the PCM1808
// left-justifies its 24-bit sample within the slot, MSB-first, per standard
// I2S convention for slot width > data width; we keep only the top 16 bits
// of that for the rest of the pipeline), pushes it to s_input_ring; pops the
// same number of processed mono samples from s_output_ring, left-justifies
// each into a 32-bit slot (mirroring the same convention on the way out),
// duplicates to stereo, and writes them to the PCM5102. Signals
// s_data_ready_sem after every read so dsp_task can wake up and check
// whether a full hop's worth of data is ready.
// =============================================================================

static void audio_io_task(void *arg) {
    (void)arg;

    static int32_t rx_buf[I2S_CHUNK_FRAMES * I2S_CHANNELS];
    static int32_t tx_buf[I2S_CHUNK_FRAMES * I2S_CHANNELS];
    static float    mono_in[I2S_CHUNK_FRAMES];
    static float    mono_out[I2S_CHUNK_FRAMES];

    for (;;) {
        size_t bytes_read = 0;
        i2s_read(I2S_PORT_NUM, rx_buf, sizeof(rx_buf), &bytes_read, portMAX_DELAY);
        uint32_t frames_read = (uint32_t)(bytes_read / (sizeof(int32_t) * I2S_CHANNELS));
        if (frames_read == 0) {
            continue;   // transient read timeout; try again next iteration
        }

        for (uint32_t i = 0; i < frames_read; i++) {
#if MONO_INPUT_IS_LEFT
            int32_t raw = rx_buf[i * I2S_CHANNELS + 0];
#else
            int32_t raw = rx_buf[i * I2S_CHANNELS + 1];
#endif
            int16_t sample = (int16_t)(raw >> 16);   // top 16 bits of the left-justified slot
            mono_in[i] = (float)sample * (1.0f / 32768.0f);
        }

        ring_push(&s_input_ring, mono_in, frames_read);
        xSemaphoreGive(s_data_ready_sem);

        uint32_t popped = ring_pop(&s_output_ring, mono_out, frames_read);
        (void)popped;   // ring_pop already zero-fills any shortfall (underrun)

        for (uint32_t i = 0; i < frames_read; i++) {
            float s = mono_out[i];
            if (s > 1.0f) s = 1.0f;
            if (s < -1.0f) s = -1.0f;
            int16_t sample = (int16_t)(s * 32767.0f);
            int32_t slot = ((int32_t)sample) << 16;   // left-justify into the 32-bit slot
            tx_buf[i * I2S_CHANNELS + 0] = slot;       // duplicate mono -> both channels
            tx_buf[i * I2S_CHANNELS + 1] = slot;
        }

        size_t bytes_written = 0;
        i2s_write(I2S_PORT_NUM, tx_buf, frames_read * sizeof(int32_t) * I2S_CHANNELS,
                  &bytes_written, portMAX_DELAY);

        g_perf.i2s_overruns  = s_input_ring.overrun_count;
        g_perf.i2s_underruns = s_output_ring.underrun_count;
    }
}

// =============================================================================
// Core 1: dsp_task
// -----------------------------------------------------------------------------
// The full spectral pipeline, run once per HOP_SIZE new input samples:
//   sliding window assembly -> Hann window -> real FFT -> magnitude ->
//   heterodyne detection/notch -> VAD -> noise floor update -> Wiener/
//   spectral-subtraction gain -> combine gains -> apply -> inverse FFT ->
//   overlap-add -> output ring buffer.
// =============================================================================

static void dsp_task(void *arg) {
    (void)arg;

    fft_init();

    static NoiseEstimator  noise_state;
    static WienerState     wiener_state;
    static HeterodyneState het_state;
    noise_init(&noise_state);
    wiener_init(&wiener_state);
    heterodyne_init(&het_state);

    // Sliding analysis window: holds the most recent FFT_SIZE input samples.
    static float analysis_buf[FFT_SIZE];
    memset(analysis_buf, 0, sizeof(analysis_buf));

    static float windowed[FFT_SIZE];
    static complex_t spectrum[NUM_BINS];
    static float magnitude[NUM_BINS];
    static float notch_gain[NUM_BINS];
    static float nr_gain[NUM_BINS];
    static float combined_gain[NUM_BINS];
    static float smoothed_gain[NUM_BINS];
    for (int k = 0; k < NUM_BINS; k++) smoothed_gain[k] = 1.0f;
    static float ifft_out[FFT_SIZE];
    static float ola_accum[FFT_SIZE];
    memset(ola_accum, 0, sizeof(ola_accum));
    static float hop_in[HOP_SIZE];
    static float hop_out[HOP_SIZE];
    static float test_tone_phase = 0.0f;   // persistent phase accumulator, radians

    const float frame_period_ms = 1000.0f * (float)HOP_SIZE / (float)SAMPLE_RATE_HZ;
    const float frame_period_us = frame_period_ms * 1000.0f;

    for (;;) {
        // Block until at least HOP_SIZE new input samples are available.
        while (ring_available(&s_input_ring) < HOP_SIZE) {
            xSemaphoreTake(s_data_ready_sem, portMAX_DELAY);
        }
        ring_pop(&s_input_ring, hop_in, HOP_SIZE);

        int64_t t_frame_start = esp_timer_get_time();

        // --- VU meter: peak level of the INCOMING audio, before any
        //     processing branch below -- stays live and meaningful whether
        //     the pipeline is running, bypassed, or replaced by the test
        //     tone (see audio.h's DspStatus). ---
        {
            float peak = 0.0f;
            for (int n = 0; n < HOP_SIZE; n++) {
                float a = fabsf(hop_in[n]);
                if (a > peak) peak = a;
            }
            float level_db = 20.0f * log10f(peak + 1e-6f);
            float frac = (level_db - VU_METER_MIN_DB) / (0.0f - VU_METER_MIN_DB);
            if (frac < 0.0f) frac = 0.0f;
            if (frac > 1.0f) frac = 1.0f;
            float coeff = (frac > g_dsp_status.input_level)
                              ? tc_to_coeff(VU_METER_ATTACK_MS, frame_period_ms)
                              : tc_to_coeff(VU_METER_RELEASE_MS, frame_period_ms);
            g_dsp_status.input_level += coeff * (frac - g_dsp_status.input_level);
        }

        // --- Slide the analysis window and insert the new hop ---
        memmove(analysis_buf, analysis_buf + HOP_SIZE, (FFT_SIZE - HOP_SIZE) * sizeof(float));
        memcpy(analysis_buf + (FFT_SIZE - HOP_SIZE), hop_in, HOP_SIZE * sizeof(float));

        if (g_params.test_tone_hz > 0.0f) {
            // Diagnostic: replace the output entirely with a synthetic sine,
            // completely independent of hop_in / the PCM1808 input path.
            // Lets you confirm the ESP32 -> PCM5102 -> speaker chain works
            // in isolation, with no wiring changes -- see "tone <hz>" in
            // the Serial command reference (main.ino) and settings.h.
            float phase_inc = 2.0f * (float)M_PI * g_params.test_tone_hz / (float)SAMPLE_RATE_HZ;
            float amplitude = 0.3f;   // conservative level, well under clipping
            for (int n = 0; n < HOP_SIZE; n++) {
                hop_out[n] = amplitude * sinf(test_tone_phase);
                test_tone_phase += phase_inc;
                if (test_tone_phase > 2.0f * (float)M_PI) test_tone_phase -= 2.0f * (float)M_PI;
            }
        } else if (g_params.bypass) {
            // Bypass: skip the spectral pipeline entirely, straight passthrough.
            memcpy(hop_out, hop_in, HOP_SIZE * sizeof(float));
        } else {
            // --- Analysis window ---
            for (int n = 0; n < FFT_SIZE; n++) windowed[n] = analysis_buf[n] * g_analysis_window[n];

            // --- Forward real FFT ---
            int64_t t_fft_start = esp_timer_get_time();
            real_fft_forward(windowed, spectrum);

            for (int k = 0; k < NUM_BINS; k++) magnitude[k] = c_mag(spectrum[k]);

            // --- Heterodyne detection + notch matrix (run BEFORE the VAD so
            //     the VAD can down-weight confirmed-tone bins; see noise.h) ---
            heterodyne_process(&het_state, magnitude, notch_gain,
                                g_params.notch_depth, g_params.notch_width_hz,
                                g_params.notch_attack_ms, g_params.notch_release_ms,
                                frame_period_ms);

            // --- VAD + noise floor tracking ---
            bool vad = noise_vad_update(&noise_state, magnitude, notch_gain, frame_period_ms);
            noise_estimate_update(&noise_state, magnitude, vad,
                                   g_params.noise_tracking_speed, frame_period_ms);

            // --- Wiener / spectral-subtraction gain matrix ---
            wiener_compute_gain(&wiener_state, spectrum, noise_state.noise_mag,
                                 nr_gain, g_params.oversubtraction,
                                 g_params.spectral_floor, g_params.nr_amount);

            // --- Combine noise-reduction gain and notch gain: applying two
            //     diagonal matrices in sequence equals applying their
            //     elementwise (diagonal) product once (see matrix.h). ---
            vec_mul_real(nr_gain, notch_gain, combined_gain, NUM_BINS);

            // --- Overall temporal smoothing of the gain matrix (separate
            //     from the Wiener stage's own decision-directed smoothing
            //     and the notch stage's own attack/release): an exponential
            //     (one-pole) update of the whole gain vector toward its new
            //     target, per the brief's "Adaptive Matrix Update ... using
            //     exponential averaging" / "Speech Preservation: temporal
            //     smoothing" requirements. This is what g_params.attack_ms /
            //     release_ms actually control. ---
            {
                float atk_c = tc_to_coeff(g_params.attack_ms, frame_period_ms);
                float rel_c = tc_to_coeff(g_params.release_ms, frame_period_ms);
                for (int k = 0; k < NUM_BINS; k++) {
                    float coeff = (combined_gain[k] < smoothed_gain[k]) ? atk_c : rel_c;
                    smoothed_gain[k] += coeff * (combined_gain[k] - smoothed_gain[k]);
                }
                memcpy(combined_gain, smoothed_gain, sizeof(smoothed_gain));
            }

            // --- Optional speech-band enhancement: a gentle, smooth boost
            //     centred on the 300-3000 Hz voice band (a raised-cosine
            //     shelf, not a hard boundary, to avoid an audible "step"). ---
            if (g_params.speech_enhance > 0.0f) {
                const float lo_hz = 300.0f, hi_hz = 3000.0f;
                for (int k = 0; k < NUM_BINS; k++) {
                    float f = k * BIN_HZ;
                    float shape = 0.0f;
                    if (f >= lo_hz && f <= hi_hz) {
                        shape = 1.0f;
                    } else if (f > hi_hz && f < hi_hz + 800.0f) {
                        shape = 0.5f * (1.0f + cosf((float)M_PI * (f - hi_hz) / 800.0f));
                    } else if (f < lo_hz && f > lo_hz - 200.0f) {
                        shape = 0.5f * (1.0f + cosf((float)M_PI * (lo_hz - f) / 200.0f));
                    }
                    combined_gain[k] *= (1.0f + g_params.speech_enhance * 0.6f * shape);
                }
            }

            // --- Apply the combined diagonal gain matrix: Y = W . X ---
            mat_diag_apply_inplace(combined_gain, spectrum, NUM_BINS);

            // --- Inverse real FFT ---
            real_fft_inverse(spectrum, ifft_out);
            g_perf.fft_us = (float)(esp_timer_get_time() - t_fft_start);

            // --- Overlap-add reconstruction ---
            for (int n = 0; n < FFT_SIZE; n++) ola_accum[n] += ifft_out[n];
            memcpy(hop_out, ola_accum, HOP_SIZE * sizeof(float));
            memmove(ola_accum, ola_accum + HOP_SIZE, (FFT_SIZE - HOP_SIZE) * sizeof(float));
            memset(ola_accum + (FFT_SIZE - HOP_SIZE), 0, HOP_SIZE * sizeof(float));

            // --- Output gain + optional soft limiter ---
            float lin_gain = powf(10.0f, g_params.output_gain_db / 20.0f);
            for (int n = 0; n < HOP_SIZE; n++) {
                float s = hop_out[n] * lin_gain;
                if (g_params.limiter_enable) {
                    // Simple smooth soft-knee limiter: tanh saturates gracefully
                    // above ~0.8 instead of hard-clipping, which is far less
                    // audible as distortion on transient speech peaks.
                    const float knee = 0.8f;
                    if (s > knee)       s =  knee + (1.0f - knee) * tanhf((s - knee) / (1.0f - knee));
                    else if (s < -knee) s = -knee - (1.0f - knee) * tanhf((-s - knee) / (1.0f - knee));
                }
                hop_out[n] = s;
            }
        }

        ring_push(&s_output_ring, hop_out, HOP_SIZE);

        // --- Performance bookkeeping ---
        float frame_us = (float)(esp_timer_get_time() - t_frame_start);
        g_perf.frames_processed++;
        g_perf.last_frame_us = frame_us;
        if (frame_us > g_perf.max_frame_us) g_perf.max_frame_us = frame_us;
        const float smooth_c = 0.02f;
        g_perf.avg_frame_us += smooth_c * (frame_us - g_perf.avg_frame_us);
        g_perf.cpu_load_percent = 100.0f * g_perf.avg_frame_us / frame_period_us;
    }
}

// =============================================================================
// Public entry point
// =============================================================================

void audio_init(void) {
    memset(&g_perf, 0, sizeof(g_perf));
    memset(&g_dsp_status, 0, sizeof(g_dsp_status));

    ring_init(&s_input_ring);
    ring_init(&s_output_ring);

    s_data_ready_sem = xSemaphoreCreateCounting(1000, 0);

    i2s_setup();

    xTaskCreatePinnedToCore(audio_io_task, "audio_io", STACK_SIZE_AUDIO, NULL,
                             TASK_PRIORITY_AUDIO, &s_audio_io_task_handle, CORE_AUDIO_IO);
    xTaskCreatePinnedToCore(dsp_task, "dsp", STACK_SIZE_DSP, NULL,
                             TASK_PRIORITY_DSP, &s_dsp_task_handle, CORE_DSP);
}
