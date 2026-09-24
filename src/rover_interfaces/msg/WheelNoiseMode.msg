# Per-wheel control message for rover_ekf wheel relay nodes.
#
# When noisy_mode is true, the receiving wheel relay stops relaying the real
# encoder reading and instead publishes pure Gaussian noise (mean, stddev)
# as its linear velocity, clamped to [-2.0, 2.0] m/s.
# When noisy_mode is false, the relay returns to normal operation (real
# encoder reading + its usual small injected noise); mean/stddev are ignored.

bool noisy_mode
float64 mean
float64 stddev
