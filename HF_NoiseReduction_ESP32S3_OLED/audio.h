/*
 * audio.h
 * ---------------------------------------------------------------------------
 * Hardware I/O layer: I2S duplex (PCM1808 RX + PCM5102 TX) driver setup,
 * inter-core lock-free ring buffers, and the two FreeRTOS tasks that move
 * samples between the codecs and the DSP pipeline.
 *
 * CORE SPLIT
 *   Core 0 (audio_io_task): owns the I2S peripheral. Blocking
 *   I2SClass::readBytes()/write() calls (ESP_I2S.h, the documented
 *   Arduino-facing I2S wrapper) live only here, so a slow
 *   or jittery DSP frame on Core 1 can never stall the hardware sample
 *   clock -- it can only cause the ring buffers to run a little fuller or
 *   emptier, which the performance counters make visible (see
 *   AudioPerfStats).
 *
 *   Core 1 (dsp_task): owns the entire spectral pipeline (window -> FFT ->
 *   noise/Wiener/heterodyne -> inverse FFT -> overlap-add). It never touches
 *   the I2S driver directly.
 *
 * The two tasks communicate only through two lock-free single-producer/
 * single-consumer ring buffers (see matrix-free "RingBufferF" below) plus one
 * counting semaphore used purely as a wake-up signal -- there is no shared
 * mutable state that needs a mutex, so there is no possibility of priority
 * inversion or one core blocking the other.
 * ---------------------------------------------------------------------------
 */

#ifndef AUDIO_H
#define AUDIO_H

#include "settings.h"
#include <stdint.h>

// =============================================================================
// Lock-free single-producer/single-consumer float ring buffer.
// Capacity is RING_BUFFER_SIZE (power of two, see settings.h); indices are
// free-running uint32_t counters masked at use, so they wrap safely forever
// (see audio.cpp for the wraparound-safety argument).
// =============================================================================

typedef struct {
    float data[RING_BUFFER_SIZE];
    volatile uint32_t write_idx;   // only written by the producer task
    volatile uint32_t read_idx;    // only written by the consumer task
    volatile uint32_t overrun_count;   // producer: samples dropped (consumer too slow)
    volatile uint32_t underrun_count;  // consumer: samples missing (producer too slow / zero-filled)
} RingBufferF;

void     ring_init(RingBufferF *rb);
uint32_t ring_available(const RingBufferF *rb);          // samples ready to read
uint32_t ring_push(RingBufferF *rb, const float *src, uint32_t n);  // returns count actually pushed
uint32_t ring_pop(RingBufferF *rb, float *dst, uint32_t n);         // returns count actually popped (zero-fills shortfall)

// =============================================================================
// Performance monitoring (read by main.ino for periodic Serial reporting)
// =============================================================================

typedef struct {
    uint32_t frames_processed;
    float    last_frame_us;         // wall-clock time of the most recent DSP frame, microseconds
    float    max_frame_us;          // worst-case frame time observed since boot
    float    avg_frame_us;          // exponentially-smoothed average
    float    fft_us;                // time spent in the two real_fft_* calls (subset of frame time)
    uint32_t i2s_overruns;          // input ring buffer overruns (DSP core falling behind)
    uint32_t i2s_underruns;         // output ring buffer underruns (DSP core falling behind)
    float    cpu_load_percent;      // avg_frame_us / (HOP period in us) * 100
} AudioPerfStats;

extern AudioPerfStats g_perf;

// =============================================================================
// DSP telemetry for the OLED dashboard (display.cpp/controls.cpp): a
// smoothed 0..1 level of the INCOMING (pre-processing) audio, for a VU-
// meter-style bar. Computed from hop_in every frame, before the
// bypass/test-tone/pipeline branch, so it stays live and meaningful in
// every mode -- not otherwise visible outside dsp_task's local state.
//
// Same single-writer (dsp_task, Core 1) / single-reader (loop() /
// controls_update(), Core 0) discipline as AudioPerfStats above: a plain
// float field, no mutex needed, for the same reason the ring buffers'
// plain volatile uint32_t counters don't need one (see the wraparound-
// safety note near the top of this file).
// =============================================================================

typedef struct {
    float input_level;   // 0..1, smoothed peak level of the incoming mic
                          // audio (before any processing) -- see the
                          // VU_METER_* tunables and level computation in
                          // audio.cpp's dsp_task for the ballistics.
} DspStatus;

extern DspStatus g_dsp_status;

// =============================================================================
// Public API
// =============================================================================

// Sets up the I2S peripheral (full-duplex STD mode, ESP32-S3 as master),
// initializes both ring buffers, and launches audio_io_task (Core 0) and
// dsp_task (Core 1). Call once from setup().
void audio_init(void);

#endif // AUDIO_H
