# Native IMU antialias relay

The pinned Classic MAVLink interface takes the latest 1000 Hz IMU measurement
every four physics steps. A contact impulse shorter than four steps can be lost
or overrepresented depending on the sample phase. The relay averages the last
four native measured acceleration and gyro samples before that existing
250 Hz decimator. It still publishes every native sample, retaining its original
sequence, measurement time, orientation, covariance, bias and nonzero noise.
At startup the first three outputs average only the samples actually received.
There is no fabricated sample, interpolated timestamp or zero-noise override.

The original PX4 IMU plugin publishes `/imu_native`; the relay publishes `/imu`
for the unchanged MAVLink interface. Generated protobuf descriptors from the
original loaded PX4 plugins are used, avoiding duplicate definitions or a second
protocol implementation. The original producer must load first. Unsupported
physics rates, schema mismatch or an invalid sample fail explicitly. A source
gap/time discontinuity latches a stopped relay; it never conceals missing data.
Only the reviewed 1 ms physics / 250 Hz MAVLink path is supported.

The unit test verifies all six channels, nonzero noise, phase-independent contact
impulse integration and refusal of skipped/repeated/nonfinite samples. Explicit
native fixtures compare actual producer/relay protobufs and actual rigid-body
velocity. The fixture keeps the body enabled, matching the jointed Iris model;
a sleeping bare box can retain stale engine force and is not aircraft evidence.
Real scene qualification is separate; native fixture success alone does not
mark a warehouse as flight tested. See `imu-sampling-reference.json` in the model
license/reference records and the per-scene flight evidence for exact versions.

The acceleration output also models the reviewed PX4 1.16.2 accel-0 FIFO's ADC
capacity. That simulator divides by g/2048 and narrows directly to int16. An
unbounded contact impulse can wrap sign without setting the driver's clipping
counter. After averaging, the relay saturates acceleration to the symmetric
signed capacity, one float ULP inside 32767 counts, so FLU/FRD sign changes remain
representable and the existing 99.9% clipping detector is triggered. This does
not change the firmware range or its guards. The raw native topic is preserved.
Ordinary samples are unchanged at MAVLink float precision. Clipped impulses
necessarily lose area; full velocity-integral accuracy is only a claim for the
linear averaging stage, while real flights separately verify estimator recovery.
The native ADC fixture checks generated message identity, source metadata,
clipping detection, safe integer narrowing and nonzero freefall measurement
noise. The original range-conversion failures remain in the raw evidence.
