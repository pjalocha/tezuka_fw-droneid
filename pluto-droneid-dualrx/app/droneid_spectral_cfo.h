#ifndef SCAN2400_DRONEID_SPECTRAL_CFO_H
#define SCAN2400_DRONEID_SPECTRAL_CFO_H

#include <cstddef>

struct DroneIdSpectralCfoEstimate {
    bool valid = false;
    double cfo_hz = 0.0;
    double low_edge_hz = 0.0;
    double high_edge_hz = 0.0;
    double notch_hz = 0.0;
    double notch_depth_db = 0.0;
    double packet_snr_db = 0.0;
    bool packet_snr_valid = false;
    bool used_notch_cfo = false;
    std::size_t packet_samples = 0;
    std::size_t fft_size = 0;
};

/* Estimate CFO from the high-resolution spectrum of an observed packet interval.
 * active_first/active_end select observed RF activity, not the protocol packet
 * start; they are relative to the supplied I/Q buffer. */
bool EstimateDroneIdSpectralCfo(const float *iq_interleaved,
                                std::size_t sample_count,
                                std::size_t active_first,
                                std::size_t active_end,
                                double sample_rate_hz,
                                double coarse_cfo_hz,
                                DroneIdSpectralCfoEstimate &estimate);

#endif
