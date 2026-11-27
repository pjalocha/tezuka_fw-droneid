/*
 * Streaming, signal-agnostic chirp and occupied-band edge detector.
 *
 * The caller supplies consecutive, unshifted complex FFT slices. The detector
 * owns only a bounded ring of power slices; it can therefore be called from a
 * live FFT worker without retaining the original I/Q stream.
 */
#ifndef SCAN2400_CHIRP_SPECTRUM_DETECTOR_H
#define SCAN2400_CHIRP_SPECTRUM_DETECTOR_H

#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

enum class ChirpSignature {
    Unknown,
    DroneId,
    O2
};

struct ChirpEdgeCrossing {
    bool fitted = false;
    bool clipped = false;
    double frequency_offset_hz = 0.0;
    double frequency_hz = 0.0;
    double crossing_time_s = 0.0;
};

struct ChirpDetection {
    ChirpSignature signature = ChirpSignature::Unknown;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    double start_frequency_offset_hz = 0.0;
    double end_frequency_offset_hz = 0.0;
    double slope_hz_per_s = 0.0;
    double slope_mhz_per_us = 0.0;
    double duration_us = 0.0;
    double sweep_width_mhz = 0.0;
    double ridge_snr_db = 0.0;
    bool ridge_snr_valid = false;
    double mean_peak_snr_db = 0.0;
    std::size_t matched_slices = 0;
    double mean_peak_width_bins = 0.0;
    double line_reference_time_s = 0.0;
    double line_reference_frequency_offset_hz = 0.0;
    double min_frequency_offset_hz = 0.0;
    double max_frequency_offset_hz = 0.0;
    std::uint64_t first_slice_index = 0;
    std::uint64_t last_slice_index = 0;
    ChirpEdgeCrossing low_edge;
    ChirpEdgeCrossing high_edge;
    double fitted_band_width_mhz = 0.0;
    bool band_width_valid = false; // true only when both edges are fitted
    double packet_start_time_s = 0.0;
    double packet_end_time_s = 0.0;
    double packet_duration_us = 0.0;
    double packet_band_low_frequency_offset_hz = 0.0;
    double packet_band_high_frequency_offset_hz = 0.0;
    double packet_band_cfo_offset_hz = 0.0;
    double packet_snr_db = 0.0;
    bool packet_band_fit_valid = false;
    bool packet_snr_valid = false;
    std::string packet_fit_reject_reason = "not_run";
    std::size_t packet_fit_context_rows = 0;
    std::size_t packet_fit_pre_baseline_rows = 0;
    std::size_t packet_fit_post_baseline_rows = 0;
    double packet_fit_context_first_offset_us = 0.0;
    double packet_fit_context_last_offset_us = 0.0;
    double droneid_symbol_snr_db = 0.0;
    bool droneid_symbol_snr_valid = false;
};

// Copyable, time-ordered linear-power FFT history for debug rendering or
// offline inspection. `power` is row-major, one fft_size row per time slice.
struct ChirpSpectrumWindow {
    std::size_t fft_size = 0;
    std::uint64_t slice_hop_samples = 0;
    std::vector<std::uint64_t> sample_indices;
    std::vector<float> noise;
    std::vector<float> power;
};

struct ChirpSpectrumConfig {
    double sample_rate_hz = 0.0;
    double center_frequency_hz = 0.0;
    std::size_t fft_size = 0;
    std::size_t slice_hop_samples = 0;
    // Keep the rising O2 signature out of the live candidate search when only
    // falling DroneID chirps are needed. Offline tools retain the default.
    bool enable_o2 = true;
    // Live scanner mode: export candidate plus FFT context and let a separate
    // worker do ridge/edge/packet fitting. Offline tools keep this false.
    bool defer_refinement = false;

    // Candidate ridge search limits. Either slope direction is accepted.
    double min_abs_slope_mhz_per_us = 0.08;
    double max_abs_slope_mhz_per_us = 0.32;
    double min_duration_us = 55.0;
    double max_duration_us = 90.0;
    float peak_snr_db = 8.0f;
    // Universal post-fit ridge SNR gate, applied to every chirp signature.
    float min_ridge_snr_db = 8.0f;
    std::size_t noise_smoothing_slices = 9;
    std::size_t noise_update_stride_slices = 16;
    float noise_percentile = 50.0f;
    float peak_outer_flank_ratio = 0.25f;
    float line_match_tolerance_bins = 2.0f;
    std::size_t min_matched_slices = 6;
    std::size_t search_stride_slices = 3; // seed ridge hypotheses every Nth slice
    float edge_threshold_fraction = 0.15f;
    double edge_fit_slices_per_side = 2.0;
    // For falling DroneID chirps, retain enough future/past FFT rows to fit
    // the packet activity envelope and its occupied-band edges.
    double droneid_packet_context_after_us = 750.0;
    std::size_t minimum_history_slices = 0; // 0 selects the computed >=5T size
    std::size_t maximum_peaks_per_slice = 8;
    std::size_t maximum_queued_detections = 64;
    std::size_t maximum_queued_candidates = 8;
};

// Candidate plus the bounded FFT context needed by the slower edge, ridge,
// and packet-envelope fits. The first-stage detector creates these; a worker
// can refine them without holding up per-slice processing.
struct ChirpCandidateWork {
    ChirpSpectrumConfig config;
    ChirpDetection detection;
    ChirpSpectrumWindow context;
    std::uint64_t context_first_slice = 0;
    double reference_bin = 0.0;
    double reference_time_s = 0.0;
    std::uint64_t first_slice = 0;
    std::uint64_t last_slice = 0;

    // Acquisition metadata anchored at the last context row. These are filled
    // by the scanner when the work item leaves the detector.
    std::uint64_t rf_center_frequency_hz = 0;
    std::uint32_t sample_rate_hz = 0;
    std::uint64_t anchor_sample_index = 0;
    std::int64_t anchor_time_ns = 0;
    float gain_db = 0.0f;
    std::uint8_t gain_set = 0;
};

class ChirpSpectrumDetector {
public:
    bool Setup(const ChirpSpectrumConfig &config, std::string *error = nullptr);
    bool ProcessFftSlice(const std::complex<float> *bins, std::size_t bin_count,
                         std::uint64_t first_sample_index,
                         std::string *error = nullptr);
    std::vector<ChirpDetection> TakeDetections();
    std::vector<ChirpCandidateWork> TakeCandidateWork();
    // Load and refine one candidate using its private FFT snapshot. Reuses
    // allocated history when the FFT/rate configuration is unchanged.
    bool RefineCandidateWork(ChirpCandidateWork &work, std::string *error = nullptr);
    bool CopySpectrumWindow(std::uint64_t first_slice,
                            std::uint64_t last_slice,
                            std::size_t margin_slices,
                            ChirpSpectrumWindow &window) const;
    bool CopySpectrumRange(std::uint64_t first_slice,
                           std::uint64_t last_slice,
                           ChirpSpectrumWindow &window) const;
    // Complete candidates whose required future context has arrived and drop
    // any remaining candidates that were interrupted by a stream boundary.
    std::size_t Finish();
    std::size_t HistoryCapacity() const { return history_capacity_; }
    std::size_t SliceCount() const { return slice_count_; }
    std::size_t LastPeakCount() const { return last_peak_count_; }
    std::size_t DroppedDetectionCount() const { return dropped_detections_; }
    std::size_t DroppedCandidateCount() const { return dropped_candidates_; }
    const ChirpSpectrumConfig &Config() const { return config_; }

private:
    struct Peak {
        float bin;
        std::size_t power_bin;
        float power;
        float snr;
        float width;
        bool frequency_sharp;
    };
    struct Frame {
        std::uint64_t absolute_slice;
        std::uint64_t sample_index;
        float noise;
        std::vector<Peak> peaks;
        std::vector<float> power;
    };
    struct Candidate {
        ChirpDetection detection;
        double reference_bin = 0.0;
        double reference_time_s = 0.0;
        std::uint64_t first_slice = 0;
        std::uint64_t last_slice = 0;
        std::uint64_t ready_slice = 0;
    };

    void FindPeaks(const float *power, float noise, std::vector<Peak> &peaks) const;
    void ValidateTemporalPeaks(Frame &frame, const Frame *previous,
                               const Frame &next) const;
    void MatchLines(const Frame &current, std::uint64_t absolute_slice);
    bool RefineCandidate(Candidate &candidate) const;
    bool RefineDroneIdPacket(Candidate &candidate) const;
    void CompleteReady(std::uint64_t absolute_slice);
    const Frame *GetFrame(std::uint64_t absolute_slice) const;

    ChirpSpectrumConfig config_;
    std::vector<Frame> history_;
    std::vector<float> shifted_power_;
    std::vector<float> noise_scratch_;
    std::vector<float> noise_history_;
    std::vector<float> noise_median_scratch_;
    float smoothed_noise_ = 1.0e-20f;
    std::vector<Candidate> pending_;
    std::vector<ChirpDetection> detections_;
    std::vector<ChirpDetection> recent_;
    std::vector<ChirpCandidateWork> candidate_work_;
    std::size_t history_capacity_ = 0;
    std::size_t history_write_ = 0;
    std::size_t history_count_ = 0;
    std::size_t noise_write_ = 0;
    std::size_t noise_count_ = 0;
    std::size_t dropped_detections_ = 0;
    std::size_t dropped_candidates_ = 0;
    std::uint64_t slice_count_ = 0;
    std::size_t last_peak_count_ = 0;
    bool configured_ = false;
};

#endif
