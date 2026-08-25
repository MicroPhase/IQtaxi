/*
 * E206-specific entry point for the shared IQ replay test implementation.
 * The compile-time profile selects E206Impl, its single 64-bit LO readback,
 * ten supported sample rates, and the 240 MiB replay limit.
 */
#define IQTAXI_REPLAY_DEVICE_E206 1
#include "../e200/e200_replay_iq_test.cpp"
