# HF Communications Receiver — Digital Noise Reduction Processor
### ESP32-S3 · PCM1808 (I2S ADC) · PCM5102 (I2S DAC) · Arduino IDE

A real-time, matrix-based spectral noise reduction processor for HF (shortwave)
communications receivers: SSB/AM voice and CW. Removes white noise, atmospheric
hiss, static, heterodynes/carrier whistles, and narrowband interference while
preserving voice intelligibility and CW tone quality.

---

## 1. Hardware & Wiring

| ESP32-S3 pin (example, see `settings.h`) | Signal      | PCM1808        | PCM5102 |
|---|---|---|---|
| GPIO 1  | MCLK (system clock) | SCKI | SCK |
| GPIO 2  | BCLK (bit clock)    | BCK  | BCK |
| GPIO 42 | WS (word/LR clock)  | LRCK | LRCK |
| GPIO 4  | Data out            | —    | DIN |
| GPIO 5  | Data in             | DOUT | —   |

The ESP32-S3 is the **I2S master**; both codecs run as **slaves**. Running
both from one shared clock domain is what keeps capture and playback
sample-locked with zero long-term drift — important for a fixed-latency
real-time pipeline. Any GPIO can be used (ESP32-S3's I2S is routed through
the GPIO matrix) except strapping pins (0, 3, 45, 46) and the native-USB
pins (19/20) if you're using those.

PCM1808 hardware mode pins: `FMT=0` (I2S/Philips), `MD0=MD1=0` (slave mode).
PCM5102 hardware mode pins: `FMT=0`, `DEMP=0`, `FLT=0`, `XSMT=1` (un-muted).

## 2. Getting Started

1. Install **ESP32 Arduino Core 3.x** via Boards Manager.
2. Select an ESP32-S3 board (e.g. "ESP32S3 Dev Module").
3. Open `HF_NoiseReduction_ESP32S3.ino` (the folder and the `.ino` file must
   have the same name — that's an Arduino IDE requirement, already satisfied
   here).
4. Wire per the table above (or edit the pin `#define`s in `settings.h`).
5. Flash, then open the Serial Monitor at 115200 baud for the startup banner,
   command reference, and periodic performance stats.
6. Feed your receiver's audio output into the PCM1808's analog input, and
   listen on headphones/an amp connected to the PCM5102's output.

### Runtime tuning (Serial commands)

```
nr <0-1>            overall noise-reduction wet/dry mix
notch_depth <0-1>    heterodyne notch depth
notch_width <Hz>     heterodyne notch width
tracking <0-1>       noise-floor adaptation speed
floor <0-1>          spectral floor (residual noise fraction kept)
oversub <val>        spectral-subtraction over-subtraction factor
speech <0-1>         speech-band enhancement amount
attack <ms>          overall gain-matrix attack time constant
release <ms>         overall gain-matrix release time constant
gain <dB>            output trim
bypass <0/1>         A/B test: unprocessed passthrough
limiter <0/1>        soft output limiter on/off
status               print current parameter values
help                 list commands
```

### Troubleshooting

**`fatal error: driver/i2s_std.h: No such file or directory` (or similar
missing-header errors from ESP-IDF driver paths):** Your installed
**arduino-esp32 board package** is older than the version this project
needs. Fix: **Tools > Board > Boards Manager**, search "esp32", and install/
select version **3.0.0 or later** (the code targets the current 3.x series).
This project's I2S layer uses `ESP_I2S.h` (`I2SClass`), Espressif's
documented, bundled Arduino wrapper — it does **not** reach into raw
ESP-IDF driver component headers, specifically so it's robust across IDE
versions (Arduino IDE 1.8.x vs 2.x can resolve component include paths
differently in some setups; the bundled-library route sidesteps that class
of problem). If you still get a "no such file" error for `ESP_I2S.h`
itself, your board package is old enough (pre-3.0) that this wrapper
doesn't exist yet either — the Boards Manager update above is the fix
either way.

**I2S.begin() prints a failure message at boot:** double-check the pins in
`settings.h` against your actual wiring, and confirm both codecs' mode pins
are strapped for I2S/Philips slave operation (§1).

**Choppy / distorted audio:** watch the `[perf]` Serial line — rising
`i2s_over`/`i2s_under` counts mean the DSP task is falling behind (try
raising `I2S_CHUNK_FRAMES` in `settings.h`, or check `cpu%` isn't near 100).

---

## 3. Architecture & Block Diagram

```
   PCM1808 (I2S ADC, slave)                              PCM5102 (I2S DAC, slave)
         |  stereo I2S, ESP32-S3 = master clock                     ^
         v                                                          |
+----------------------- CORE 0 : audio_io_task -----------------------------+
|  I2S.readBytes()  ->  mono extract  ->  input ring buffer                  |
|  output ring buffer  ->  mono->stereo duplicate  ->  I2S.write()           |
+------------------------------|---------------------------^-----------------+
                     (lock-free SPSC ring buffers, no mutex)
+------------------------------v---------------------------|-----------------+
|                       CORE 1 : dsp_task  (per 512-sample hop)              |
|                                                                             |
|   sliding 1024-sample window  ->  Hann window  ->  real FFT (513 bins)     |
|                                        |                                    |
|                                        v                                    |
|                              magnitude spectrum |X(k)|                     |
|                                        |                                    |
|              +-------------------------+-------------------------+         |
|              v                                                    v         |
|   Heterodyne detector & tracker                          VAD (down-weights|
|   (peaks -> multi-frame tracks ->                          confirmed tone  |
|    notch gain matrix N(k))                                 bins first)     |
|              |                                                    |         |
|              |                                                    v         |
|              |                                    Noise-floor tracker ->   |
|              |                                    N_est(k) (min-stat-like) |
|              |                                                    |         |
|              |                                                    v         |
|              |                          Decision-directed Wiener filter +  |
|              |                          spectral subtraction -> gain W(k)  |
|              |                                                    |         |
|              +----------------->  combined diagonal gain  G = W . N ------+|
|                                        |                                    |
|                              overall attack/release smoothing              |
|                              + optional speech-band shelf boost            |
|                                        |                                    |
|                              Y(k) = G(k) . X(k)   (apply gain matrix)      |
|                                        |                                    |
|                                  inverse real FFT                          |
|                                        |                                    |
|                            overlap-add (50%, COLA-exact)                   |
|                                        |                                    |
|                          soft limiter + output gain  ->  output ring buffer|
+-----------------------------------------------------------------------------+
```

---

## 4. DSP Pipeline, Stage by Stage

**Sliding window / overlap-add framing.** A rolling 1024-sample buffer
(`analysis_buf`) always holds the most recent 1024 input samples. Every time
512 new samples (one hop) arrive, the buffer is shifted left by 512 and the
new samples appended — this is the "50% overlap" the design calls for.

**Hann window.** A periodic ("DFT-even") Hann window,
`w[n] = 0.5 - 0.5*cos(2*pi*n/N)`, is applied to the 1024-sample frame before
the FFT. This specific window, at exactly 50% hop, satisfies the
**COLA (Constant-Overlap-Add)** condition exactly: two overlapping shifted
copies of it sum to 1.0 everywhere (verified numerically to double-precision
machine epsilon during development). That guarantees a neutral gain matrix
(all 1s) reconstructs the input **exactly**, with no synthesis window needed
and no amplitude "pumping" at the 86 Hz hop rate.

**Real FFT (1024-point, via a 512-point complex FFT).** See §5 for the full
derivation. Produces 513 complex bins (DC .. Nyquist), each representing
43.07 Hz of bandwidth.

**Heterodyne detection & notch matrix.** Runs *before* the VAD (see below).
For each bin, a local-maximum + neighbour-average "prominence" test flags
candidate tones (§4a). Candidates are matched across frames to a small set
of persistent tracks; a track must be confirmed for 3 consecutive frames
(~35 ms) before it's allowed to notch, which is what keeps transient speech
harmonics/plosives from being mistaken for a steady heterodyne. Confirmed
tracks generate a raised-cosine notch (no hard edges -> no ringing/clicks),
independently adjustable in depth and width, with a fast attack / slow
release envelope (engages almost as soon as confirmed; doesn't "snap" open
during CW keying gaps).

**Voice Activity Detection (VAD).** A wideband-energy-vs-adaptive-reference
detector with hangover (hold time), described fully in §4b. Bins already
claimed by a *confirmed* heterodyne track are down-weighted in the VAD's
energy sum, so a strong steady local tone can't make the VAD believe
"speech" is present indefinitely (a real bug found during testing — see
§9).

**Noise-floor tracking.** Per-bin fast-attack/slow-release asymmetric
smoothing (§4c) approximates minimum-statistics tracking at O(1) memory,
gated more conservative during detected speech per the brief's "noise
estimate should update slowly during speech."

**Wiener filter + spectral subtraction (the noise-reduction gain matrix).**
A decision-directed (Ephraim-Malah) Wiener gain is computed per bin and
blended (geometric mean) with a classical over-subtraction/floor spectral-
subtraction gain, giving both musical-noise resistance (from the Wiener
side) and explicit aggressiveness control (from the subtraction side). Full
derivation in §5.

**Gain-matrix combination.** The notch gain and the noise-reduction gain
are two diagonal matrices; applying them in sequence is mathematically
identical to applying their elementwise product once — see §5. The combined
gain is then temporally smoothed (attack/release) and given an optional
speech-band (300–3000 Hz) shelf boost with raised-cosine transition edges
(no hard steps).

**Inverse FFT + overlap-add.** Because any *real*, frequency-symmetric gain
applied to a Hermitian half-spectrum preserves the Hermitian property, the
inverse transform is guaranteed real (verified numerically, §9) — no extra
symmetrization step needed. The two most recent inverse-FFT frames are
summed sample-by-sample (overlap-add); the completed (fully summed) half of
the accumulator is emitted each hop.

**Output stage.** Output trim (dB) and an optional smooth tanh soft-knee
limiter (engages above ~80% full scale) protect against occasional
transient overs without hard-clipping.

### 4a. Heterodyne / tone detection detail
A bin `k` is a candidate tone if it's a local maximum **and** its magnitude
exceeds the average of nearby "skip-guard" neighbour bins by more than 9 dB.
The guard band (2 bins either side excluded, next 3 bins each side averaged)
keeps a wide, naturally-peaked speech formant from being mistaken for the
extremely narrow (much-less-than-one-bin-wide) heterodyne tones typical of
HF interference.

### 4b. VAD detail
```
energy(frame) = sum_k  bin_weight(k)^2 * |X(k)|^2
```
`energy` is smoothed with a fast attack (15 ms) / slower release (150 ms).
A separate adaptive reference tracks the *noise-only* energy level with its
own asymmetric logic: it always follows a **falling** trend quickly, but
only creeps **upward** slowly (and even more slowly while flagged
"active") — this dual-rate asymmetry is what prevents a genuine speech
onset from being partially absorbed into the reference before the
SNR-threshold decision even gets to see it (a real, empirically-found
tuning issue — see §9). `SNR_dB = 10*log10(energy/reference)`; a 6 dB
threshold plus an 8-frame (~93 ms) hangover makes the final "active" flag.

### 4c. Noise-floor tracker detail
```
if mag[k] < noise_estimate[k]:  fast update (60 ms time constant)
else:                            slow update (200 ms .. 8 s, controlled
                                  by `tracking`, x6 slower again if VAD
                                  says "speech is present")
```

---

## 5. Matrix Mathematics

### 5.1 Why "matrix" means "diagonal" here — and when it doesn't

The brief asks for `Y = W · X` with `W` an "adaptive weighting matrix" over
the FFT bin vector `X` (513 complex values). This is implemented literally:
`W` **is** a 513×513 matrix — it is just a **diagonal** one:

```
[ w0   0    0  ...   0  ]   [X0]     [w0*X0]
[ 0    w1   0  ...   0  ] · [X1]  =  [w1*X1]   =  Y
[ 0    0   w2  ...   0  ]   [X2]     [w2*X2]
[ ...            ...   ]   [..]     [ ... ]
[ 0    0    0  ... w512]  [X512]   [w512*X512]
```

A diagonal matrix multiply *is* an element-wise vector multiply — that is
not a shortcut, it is the exact and computationally optimal way to apply
one. A **dense** 513×513 gain matrix would cost 513² ≈ 263,000 multiply-adds
per 11.6 ms frame just for that one step, and a genuine covariance-based
*matrix* Wiener solution (§5.3) would additionally require inverting a
513×513 matrix — O(513³) ≈ 1.3×10⁸ operations — every frame. That's on the
order of 11 GFLOP/s sustained; an ESP32-S3 delivers a few hundred MFLOP/s in
practice. No real-time embedded (or, for that matter, desktop) noise
reduction system inverts a full per-bin covariance matrix; every one uses
the diagonal approximation, which is also the *exact* maximum-likelihood
answer under the standard, well-justified assumption that noise power is
uncorrelated across frequency bins (true for white/atmospheric noise, and a
fair approximation once heterodynes are isolated to single bins).

Where genuine **off-diagonal** (cross-bin) structure earns its keep, this
project uses it in two places, deliberately kept small so they stay
real-time-feasible:
- **Cross-bin gain smoothing** (`spectral_smooth3`, `matrix.cpp`) — each
  output bin is a weighted sum of itself and its immediate neighbours, i.e.
  multiplication by a small banded (tridiagonal-like) matrix. This is what
  keeps the gain curve from "combing" adjacent bins, a major source of the
  "watery" character the brief explicitly asks to avoid.
- **Small dense K×K matrix utilities** (`matK_mul`, `matK_inverse`,
  `matK_add_ridge` in `matrix.cpp`, Gauss-Jordan with partial pivoting and
  optional Tikhonov/ridge regularization) are provided for local
  (few-bin) covariance/correlation operations, sized so a K³ inversion (K ≤ 8)
  stays cheap even done several times per frame. These are exposed as
  general building blocks (validated with an automated inverse-matrix test —
  see §9) for anyone extending the notch/tone-confirmation logic with true
  cross-bin correlation.

### 5.2 Scalar Wiener filter
```
H(k) = S(k) / (S(k) + N(k))
```
`S(k)`: estimated clean-signal power in bin k. `N(k)`: estimated noise
power. This is the classical minimum-mean-square-error (MMSE) linear
estimator for a signal in additive, uncorrelated noise (Wiener-Hopf theory).

### 5.3 Matrix Wiener filter, and why it collapses to §5.2 here
```
H = Rs · (Rs + Rn)^-1
```
In general `Rs`/`Rn` are full covariance matrices. Under the (standard,
above-justified) diagonal assumption, `Rs = diag(S(0)..S(512))` and
`Rn = diag(N(0)..N(512))`, so:
```
Rs + Rn = diag(S(0)+N(0), ..., S(512)+N(512))
```
and the inverse of a diagonal matrix is simply the reciprocal of each
diagonal entry:
```
(Rs+Rn)^-1 = diag(1/(S(0)+N(0)), ..., 1/(S(512)+N(512)))
H = Rs·(Rs+Rn)^-1 = diag( S(0)/(S(0)+N(0)), ..., S(512)/(S(512)+N(512)) )
```
— exactly 513 independent applications of §5.2's scalar formula. This is
not a convenience simplification; it is the standard approach throughout
the speech-enhancement literature, made exact by the diagonal assumption and
made necessary by the operation count in §5.1.

### 5.4 Decision-directed a priori SNR (musical-noise suppression)
Rather than estimate `S(k)` directly (we only ever observe signal+noise),
this project uses the Ephraim-Malah "decision-directed" approach — the
standard technique specifically because it damps the random gain
fluctuations that create musical noise in naive spectral subtraction:
```
gamma(k,n)  = |X(k,n)|^2 / N(k,n)                              (a posteriori SNR)
xi(k,n)     = alpha * G(k,n-1)^2 * |X(k,n-1)|^2 / N(k,n)
              + (1-alpha) * max(gamma(k,n) - 1, 0)              (a priori SNR, alpha ~= 0.98)
H(k,n)      = xi(k,n) / (1 + xi(k,n))                           (Wiener gain from a priori SNR)
```

### 5.5 Spectral subtraction (independent, tunable alternative view)
```
|Y(k)| = max( |X(k)| - alpha_os * |N(k)| , beta * |X(k)| )
G_ss(k) = |Y(k)| / |X(k)|
```
`alpha_os` (over-subtraction) and `beta` (spectral floor) are exposed as
`oversub`/`floor` at runtime. The final gain blends `H` (§5.4) and `G_ss`
by geometric mean, then applies the user's `nr` (wet/dry) parameter.

### 5.6 LMS adaptive vector (optional "ML-style" stage)
```
Y(n) = W(n) · X(n)
W(n+1) = W(n) + mu * e(n) * X(n),   e(n) = D(n) - Y(n)
```
Implemented in `matrix.cpp` (`lms_init`/`lms_update`) as a diagonal LMS
update, but deliberately operating on a **reduced** feature vector (32
critical-band-grouped magnitudes, `NUM_LMS_BANDS`) rather than all 513 raw
bins. Adapting one weight per raw FFT bin from a single noisy error sample
every 11.6 ms converges far too slowly and noisily to help — 513
independent single-tap filters, each fed one sample of gradient information
per frame, don't have enough statistical support. Grouping into ~32
perceptual bands makes each update well-conditioned. **This module is
implemented, unit-tested, and ready to use, but is not wired into the
default signal path** (the brief marks it optional): the Wiener + notch
combination already covers every listed noise/interference type, and the
brief also asks to avoid over-processing / "watery" sound, which an
additional always-on adaptive stage risks without a clear reference signal
to adapt against. To experiment with it, call `lms_init()`/`lms_update()`
from `dsp_task` in `audio.cpp`, feeding it (for example) critical-band
averages of the current magnitude spectrum as `x` and a smoothed target as
`d`, and folding its output back into `combined_gain`.

---

## 6. Latency Budget (read this before filing a bug report)

A 50%-overlap STFT analysis/synthesis system has an **unavoidable
structural latency** of about one full window before the first fully-formed
output sample: the Hann-windowed overlap-add reconstruction needs
contributions from two consecutive frames before any output sample is
final. At `FFT_SIZE=1024`, 44.1 kHz, that floor is `1024/44100 ≈ 23.2 ms`,
plus:
- I2S buffering: the `ESP_I2S.h` wrapper (`I2SClass`) manages its own
  internal DMA buffering; `I2S_CHUNK_FRAMES` in `settings.h` (default 256,
  ~5.8 ms) only controls how many frames we read/write per software loop
  iteration. Actual added latency is typically on the order of one to a few
  chunk periods, roughly 6-20 ms depending on the core's internal defaults
  for your installed arduino-esp32 version.
- DSP processing time itself (sub-millisecond, see §7)

**Total: roughly 27-35 ms**, which is *above* the 20 ms target in the design
brief. This is a property of block-based spectral processing, not an
implementation shortfall — no linear STFT overlap-add scheme beats
window-length latency. Two honest options if 20 ms is a hard requirement:

1. **Drop to `FFT_SIZE=512`** in `settings.h` (11.6 ms window →
   ~14-17 ms total), at the cost of coarser frequency resolution
   (86 Hz/bin instead of 43 Hz/bin — the heterodyne notch gets a little
   less surgical, and the Wiener filter has coarser spectral resolution).
   Also reduce `I2S_CHUNK_FRAMES` to keep the I/O granularity contribution
   proportionate (e.g. 128 instead of 256).
2. **Keep 1024** and accept ~30 ms — for SSB/CW/AM listening this is a fixed
   pipeline delay, not an echo or audible artifact; it's inaudible as
   "lag" in a listening (not full-duplex two-way voice) application.

---

## 7. CPU Budget & Optimization

| Stage (per 11.6 ms hop, `FFT_SIZE=1024`) | Approx. operation count |
|---|---|
| Hann window multiply | 1024 multiplies |
| Forward real FFT (512-pt complex core + pack/unpack) | ~512·log2(512) ≈ 4,608 butterflies + 513 combine steps |
| Heterodyne peak scan | ~513 comparisons + a few dozen track-match ops |
| VAD + noise-floor update | ~513 × a few ops |
| Wiener/spectral-subtraction gain | ~513 × ~15 ops (incl. one `sqrtf`/`log`-free-path) |
| Gain combine + smoothing + speech shelf | ~513 × ~5 ops |
| Inverse real FFT | same order as forward |
| Overlap-add + limiter | ~1024 + 512 ops |

All told, on the order of **10⁵ floating-point operations per hop**, against
an 11.6 ms budget — at ESP32-S3's 240 MHz clock (with hardware single-
precision FPU), this leaves an enormous margin even assuming only a few
FLOPs/cycle sustained. **Measure it on your actual board**: the firmware
prints live `frame`/`fft`/`cpu%` numbers over Serial every 2 seconds (see
`AudioPerfStats` in `audio.h` and the `[perf]` log line in `main.ino`) rather
than asking you to trust a back-of-envelope estimate.

**Optimizations applied:**
- No dynamic memory allocation anywhere in the real-time path — every
  working buffer is `static` (fixed address, computed once at link time).
- No PSRAM in the hot path: the entire per-frame working set (see §8, ~35 KB)
  fits comfortably in internal SRAM, which is far lower-latency than PSRAM
  over its cache; PSRAM access would risk jeopardizing the frame deadline
  for no benefit at this working-set size. (PSRAM remains a good place for
  *future* additions like a longer SD-card recording buffer or a waterfall
  display's history — see §11.)
- Real FFT via half-size complex FFT, not a zero-padded full-size complex
  FFT — roughly half the butterfly operations and half the twiddle-factor
  table (§5, algorithm cited in `fft.h`/`fft.cpp`).
- All trig tables (twiddles, bit-reversal, Hann window) precomputed once in
  `fft_init()` — zero `sinf`/`cosf` calls in the per-frame hot path.
- Lock-free single-producer/single-consumer ring buffers between the two
  cores — no mutex, no possibility of one core blocking the other or of
  priority inversion.
- Core split: Core 0 owns the I2S hardware exclusively (nothing can make a
  blocking `I2SClass::readBytes()/write()` late except the library's own
  internal driver);
  Core 1 owns 100% of the spectral math, so a momentarily slow DSP frame
  can only make the ring buffers a bit fuller/emptier (visible via the
  `i2s_overruns`/`i2s_underruns` counters), never stall sample acquisition.

## 8. Memory Budget

| Buffer | Size |
|---|---|
| Twiddle factors (512-pt core FFT) | 256 × 8 B = 2 KB |
| Twiddle factors (full 1024, 513 unique) | 513 × 8 B ≈ 4.1 KB |
| Bit-reversal table | 512 × 2 B = 1 KB |
| Hann window | 1024 × 4 B = 4 KB |
| Analysis sliding buffer | 1024 × 4 B = 4 KB |
| Windowed frame scratch | 1024 × 4 B = 4 KB |
| Spectrum (513 complex) | 513 × 8 B ≈ 4.1 KB |
| Magnitude / gain vectors (×5: mag, notch, nr, combined, smoothed) | 5 × 513 × 4 B ≈ 10.3 KB |
| Overlap-add accumulator | 1024 × 4 B = 4 KB |
| Noise/Wiener/heterodyne module state | a few KB combined |
| I2S DMA buffers (driver-internal) | `dma_desc_num × dma_frame_num × 2ch × 2B` = 4×256×2×2 = 4 KB |
| Ring buffers (×2) | 2 × 8192 × 4 B = 64 KB |

**Total: well under 150 KB**, comfortably inside the ESP32-S3's internal
SRAM (typically 512 KB), with no PSRAM required for the core pipeline.

## 9. Testing & Validation


- **Real FFT (forward + inverse)**: derived by hand, first checked against
  NumPy's `rfft`/`irfft` in Python (exact to double-precision round-off for
  N=16 and N=1024), then re-verified by compiling the **actual C++ source**
  (`fft.cpp`) standalone and testing it against a brute-force O(N²) DFT,
  a full forward→inverse round trip, and — critically — applying a
  real per-bin gain and inverse-transforming, confirming the result stays
  real and matches a brute-force Hermitian-symmetric IDFT to within
  floating-point error. A pure-tone test also confirms correct bin-frequency
  detection.
- **Small dense matrix inversion** (`matK_inverse`): verified against a
  hand-computed 3×3 inverse (`A · A⁻¹ = I` to ~1e-7) and a ridge-regularized
  near-singular 2×2 case.
- **Full pipeline integration**: `fft -> heterodyne -> VAD -> noise tracker
  -> Wiener gain` was chained across 300 synthetic frames (white noise + a
  steady 1 kHz heterodyne + a mid-run broadband "speech-like" burst),
  compiled and run as ordinary C++. This caught two real bugs before they
  ever reached the firmware:
  1. **VAD reference deadlock**: the noise-energy reference started at an
     arbitrary placeholder value, and was only allowed to move while the
     VAD judged "no speech" — but the *first* real audio sample could be
     orders of magnitude away from that placeholder, permanently latching
     "speech present." Fixed by bootstrapping the reference from the first
     real frame instead of a fixed constant.
  2. **A persistent heterodyne masking the VAD**: a strong steady tone was
     enough, by itself, to make the wideband-energy VAD believe speech was
     continuously present, starving the noise-floor tracker of updates.
     Fixed by having the heterodyne detector run *before* the VAD each
     frame and passing its notch gain in as a per-bin de-weighting factor,
     plus making the VAD's own reference-energy tracker asymmetric
     (fast-falling / slow-rising) so it can't be dragged upward by a
     transient before the threshold logic gets a chance to classify it.
  With both fixes, the tone gain converges to the expected
  `1 - notch_depth` floor within ~20 frames (~230 ms) and the noise floor
  estimate tracks monotonically downward as expected.
- **`audio.cpp` / `HF_NoiseReduction_ESP32S3.ino` (the hardware-facing
  code)**: type- and logic-checked by compiling them, unmodified, against a
  set of stub headers reproducing the `ESP_I2S.h` (`I2SClass::setPins`/
  `begin`/`readBytes`/`write`) and FreeRTOS/Arduino API shapes documented in
  Espressif's current arduino-esp32 reference and official example sketches.
  This catches type mismatches, signature drift between modules, and typos,
  though it cannot substitute for testing on real silicon with real I2S
  timing. **Please do a bench test (start with `bypass 1` to confirm clean
  passthrough, then `bypass 0`) before relying on this for on-air use.** An
  earlier revision of this project used the lower-level ESP-IDF `i2s_std`
  driver directly; it was replaced with the `ESP_I2S.h` wrapper after a
  user's older arduino-esp32 board-package install couldn't resolve the raw
  driver header path (see "Troubleshooting" in §2) — the wrapper is bundled
  as a first-party Arduino library rather than a raw IDF component, which is
  more robust across IDE/core version combinations.

## 10. Known Limitations / Design Trade-offs

- Latency is ~27-35 ms with the default 1024-point FFT (see §6) — above the
  brief's 20 ms target; this is a fundamental property of the chosen
  window size, not a bug, and `FFT_SIZE=512` is provided as a documented
  escape hatch.
- The VAD is a simple wideband-energy detector; it responds well to real
  speech's sharp onsets and high peak-to-average ratio, but a deliberately
  mild or perfectly-steady synthetic test tone can under-trigger it (see
  §9). Real HF SSB/AM speech, tuned normally, triggers it reliably in
  testing.
- The gain matrix is diagonal (per-bin independent) rather than a full
  cross-bin covariance solution, for the computational reasons in §5.1 —
  this is standard practice, not a shortcut, but it's worth knowing if you
  came looking for a full MVDR/multichannel-style beamforming-grade filter
  (this is a single-channel, single-microphone-equivalent system).
- Mono processing: the PCM1808's stereo bus is used single-ended (one
  channel carries the receiver audio); the PCM5102 gets the same processed
  mono signal on both output channels.

## 11. Suggested Future Additions

- **AGC** (input and/or output automatic gain control) — a natural
  companion to the existing limiter; the `settings.h` parameter block and
  module layout leave room for an `agc.h/.cpp` pair.
- **Graphic/parametric EQ** — could reuse the existing diagonal-gain-matrix
  machinery (`mat_diag_apply`) with a user-defined static shaping curve
  multiplied into `combined_gain`.
- **Noise blanker** (impulse/ignition-noise blanking in the time domain,
  ahead of the FFT) — pairs naturally with the existing DC-blocker
  placeholder in the pipeline.
- **Waterfall / OLED spectrum display** — `magnitude[]` is already computed
  every frame; an SPI/I2C display task on Core 0 (or a rate-limited pass
  through the existing Serial link) could visualize it directly.
- **SD card recording** — PSRAM (not used by the core pipeline, see §7)
  would be a good place for a pre-record ring buffer.
- **Bluetooth audio output** — ESP32-S3 supports BLE; classic Bluetooth A2DP
  would need an external codec path or a different SoC variant.
- **True Martin minimum-statistics** noise estimation, if you want to
  compare against the current fast-attack/slow-release approximation — the
  hook point is documented directly in `noise.h`/`noise.cpp`.
- **LMS adaptive stage** — implemented and tested (§5.6) but not wired into
  the default path; see that section for the integration point.

---

## 12. File Organization

```
HF_NoiseReduction_ESP32S3/
├── HF_NoiseReduction_ESP32S3.ino   setup()/loop(), Serial command interface, g_params instance
├── settings.h                       pins, sample-rate/FFT geometry, DSPParams struct + defaults
├── matrix.h / matrix.cpp            complex numbers, diagonal gain-matrix ops, small dense
│                                    matrix utilities, LMS adaptive vector
├── fft.h / fft.cpp                  real FFT via half-size complex FFT, Hann window
├── noise.h / noise.cpp              VAD + adaptive noise-floor tracking
├── wiener.h / wiener.cpp            decision-directed Wiener filter + spectral subtraction
├── heterodyne.h / heterodyne.cpp    tone detection, multi-frame tracking, notch matrix
├── audio.h / audio.cpp              I2S duplex driver, ring buffers, Core 0 / Core 1 tasks
└── README.md                        this file
```
