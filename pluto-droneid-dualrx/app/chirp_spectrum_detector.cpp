#include "chirp_spectrum_detector.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
void Error(std::string *out, const char *message) { if (out) *out = message; }
double Db(double value) { return 10.0 * std::log10(std::max(value, 1.0e-30)); }
}

bool ChirpSpectrumDetector::Setup(const ChirpSpectrumConfig &c, std::string *error) {
    configured_ = false;
    if (!(c.sample_rate_hz > 0.0) || !std::isfinite(c.sample_rate_hz) ||
        !std::isfinite(c.center_frequency_hz) || c.fft_size < 16 ||
        (c.fft_size & (c.fft_size - 1)) || c.slice_hop_samples == 0 ||
        !(c.min_abs_slope_mhz_per_us > 0.0) ||
        c.max_abs_slope_mhz_per_us <= c.min_abs_slope_mhz_per_us ||
        c.min_duration_us <= 0.0 || c.max_duration_us <= c.min_duration_us ||
        c.peak_snr_db < 0.0f || c.min_ridge_snr_db < 0.0f ||
        c.noise_percentile <= 0.0f || c.noise_percentile >= 100.0f ||
        c.peak_outer_flank_ratio <= 0.0f ||
        c.peak_outer_flank_ratio >= 1.0f || c.line_match_tolerance_bins <= 0.0f ||
        c.min_matched_slices < 4 || c.search_stride_slices == 0 || c.edge_threshold_fraction <= 0.0f ||
        c.edge_threshold_fraction >= 1.0f || c.edge_fit_slices_per_side < 0.0 ||
        c.maximum_peaks_per_slice < 2 || c.noise_smoothing_slices == 0 ||
        c.noise_smoothing_slices > 4096 || c.noise_update_stride_slices == 0 ||
        c.maximum_queued_detections == 0 ||
        c.maximum_queued_candidates == 0 ||
        !std::isfinite(c.min_abs_slope_mhz_per_us) ||
        !std::isfinite(c.max_abs_slope_mhz_per_us) ||
        !std::isfinite(c.min_duration_us) || !std::isfinite(c.max_duration_us) ||
        !std::isfinite(c.peak_snr_db) || !std::isfinite(c.min_ridge_snr_db) ||
        !std::isfinite(c.noise_percentile) ||
        !std::isfinite(c.peak_outer_flank_ratio) ||
        !std::isfinite(c.line_match_tolerance_bins) ||
        !std::isfinite(c.edge_threshold_fraction) ||
        !std::isfinite(c.edge_fit_slices_per_side) ||
        !std::isfinite(c.droneid_packet_context_after_us) ||
        c.droneid_packet_context_after_us < 0.0) {
        Error(error, "invalid chirp detector configuration");
        return false;
    }
    config_ = c;
    const double slice_s = static_cast<double>(c.slice_hop_samples) / c.sample_rate_hz;
    const std::size_t max_t = static_cast<std::size_t>(std::ceil(c.max_duration_us * 1.0e-6 / slice_s));
    const std::size_t required = std::max<std::size_t>(16,
        2 * static_cast<std::size_t>(std::ceil(c.edge_fit_slices_per_side * max_t)) + max_t + 8);
    const std::size_t packet_context = static_cast<std::size_t>(std::ceil(
        (c.droneid_packet_context_after_us + 400.0) * 1.0e-6 / slice_s)) + max_t + 8;
    const std::size_t required_history = std::max(required, packet_context);
    history_capacity_ = c.minimum_history_slices ? c.minimum_history_slices : required_history;
    if (history_capacity_ < required_history) {
        Error(error, "history is too short for chirp edge and DroneID packet context");
        return false;
    }
    history_.assign(history_capacity_, Frame{});
    shifted_power_.resize(c.fft_size);
    noise_scratch_.reserve(c.fft_size);
    noise_history_.assign(c.noise_smoothing_slices, 0.0f);
    noise_median_scratch_.resize(c.noise_smoothing_slices);
    history_write_ = history_count_ = 0;
    noise_write_ = noise_count_ = 0;
    smoothed_noise_ = 1.0e-20f;
    dropped_detections_ = 0;
    dropped_candidates_ = 0;
    slice_count_ = 0;
    last_peak_count_ = 0;
    pending_.clear();
    detections_.clear();
    recent_.clear();
    candidate_work_.clear();
    configured_ = true;
    return true;
}

void ChirpSpectrumDetector::FindPeaks(const float *p, float noise,
                                      std::vector<Peak> &peaks) const {
    peaks.clear();
    const std::size_t n = config_.fft_size;
    const float gate = noise * std::pow(10.0f, config_.peak_snr_db / 10.0f);
    for (std::size_t k = 3; k + 3 < n; ++k) {
        const float v = p[k];
        if (v < gate || v <= p[k - 1] || v < p[k + 1]) continue;
        unsigned width = 1;
        if (p[k - 1] >= v * 0.25f) ++width;
        if (p[k + 1] >= v * 0.25f) ++width;
        if (p[k - 2] >= v * 0.25f) ++width;
        if (p[k + 2] >= v * 0.25f) ++width;
        const bool frequency_sharp =
            p[k - 3] <= v * config_.peak_outer_flank_ratio &&
            p[k + 3] <= v * config_.peak_outer_flank_ratio;
        const float a = std::log(std::max(p[k - 1], 1.0e-30f));
        const float b = std::log(std::max(v, 1.0e-30f));
        const float d = std::log(std::max(p[k + 1], 1.0e-30f));
        const float curvature = a - 2.0f * b + d;
        float sub = curvature < -1.0e-6f ? 0.5f * (a - d) / curvature : 0.0f;
        sub = std::max(-0.5f, std::min(0.5f, sub));
        peaks.push_back({static_cast<float>(k) + sub, k, v, v / noise,
                         static_cast<float>(width), frequency_sharp});
    }
    if (peaks.size() > config_.maximum_peaks_per_slice) {
        std::nth_element(peaks.begin(), peaks.begin() + config_.maximum_peaks_per_slice,
                         peaks.end(), [](const Peak &a, const Peak &b) { return a.snr > b.snr; });
        peaks.resize(config_.maximum_peaks_per_slice);
    }
    std::sort(peaks.begin(), peaks.end(),
              [](const Peak &a, const Peak &b) { return a.bin < b.bin; });
}

void ChirpSpectrumDetector::ValidateTemporalPeaks(
    Frame &frame, const Frame *previous, const Frame &next) const {
    if (frame.peaks.empty()) return;
    if (!previous || previous->power.size() != config_.fft_size ||
        next.power.size() != config_.fft_size || frame.power.size() != config_.fft_size) {
        frame.peaks.clear();
        return;
    }

    const float frame_noise = std::max(frame.noise, 1.0e-20f);
    const float previous_noise = std::max(previous->noise, 1.0e-20f);
    const float next_noise = std::max(next.noise, 1.0e-20f);
    const float sharp_ratio = config_.peak_outer_flank_ratio;
    std::size_t kept = 0;
    for (const Peak &peak : frame.peaks) {
        const std::size_t k = peak.power_bin;
        if (k >= config_.fft_size) continue;
        const float center = frame.power[k] / frame_noise;
        const float before = previous->power[k] / previous_noise;
        const float after = next.power[k] / next_noise;
        const bool time_peak = center >= before && center >= after &&
                               (center > before || center > after);
        if (!time_peak) continue;

        const bool time_sharp = before <= center * sharp_ratio &&
                                after <= center * sharp_ratio;
        if (!peak.frequency_sharp && !time_sharp) continue;
        frame.peaks[kept++] = peak;
    }
    frame.peaks.resize(kept);
}

const ChirpSpectrumDetector::Frame *ChirpSpectrumDetector::GetFrame(
    std::uint64_t absolute_slice) const {
    if (absolute_slice >= slice_count_) return nullptr;
    const Frame &f = history_[absolute_slice % history_capacity_];
    return f.absolute_slice == absolute_slice ? &f : nullptr;
}

bool ChirpSpectrumDetector::ProcessFftSlice(const std::complex<float> *bins,
    std::size_t bin_count, std::uint64_t first_sample_index, std::string *error) {
    if (!configured_) { Error(error, "Setup() must be called first"); return false; }
    if (!bins || bin_count != config_.fft_size) {
        Error(error, "FFT slice size does not match setup"); return false;
    }
    const std::size_t n = config_.fft_size;
    noise_scratch_.clear();
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t bin = (i + n / 2) % n;
        shifted_power_[i] = std::norm(bins[bin]);
        if (i > n / 20 && i < n - n / 20 &&
            (i + 2 < n / 2 || i > n / 2 + 2) && (i & 3u) == 0)
            noise_scratch_.push_back(shifted_power_[i]);
    }
    if (noise_scratch_.empty()) { Error(error, "not enough bins for noise estimate"); return false; }
    const std::size_t mid = std::min(noise_scratch_.size() - 1,
        static_cast<std::size_t>((config_.noise_percentile / 100.0f) * noise_scratch_.size()));
    std::nth_element(noise_scratch_.begin(), noise_scratch_.begin() + mid, noise_scratch_.end());
    const float slice_noise = std::max(noise_scratch_[mid], 1.0e-20f);
    const std::uint64_t current_slice = slice_count_;
    noise_history_[noise_write_] = slice_noise;
    noise_write_ = (noise_write_ + 1) % noise_history_.size();
    noise_count_ = std::min(noise_count_ + 1, noise_history_.size());
    if (noise_count_ == 1 || current_slice % config_.noise_update_stride_slices == 0) {
        std::copy(noise_history_.begin(), noise_history_.begin() + noise_count_,
                  noise_median_scratch_.begin());
        const std::size_t noise_mid = noise_count_ / 2;
        std::nth_element(noise_median_scratch_.begin(),
                         noise_median_scratch_.begin() + noise_mid,
                         noise_median_scratch_.begin() + noise_count_);
        smoothed_noise_ = std::max(noise_median_scratch_[noise_mid], 1.0e-20f);
    }
    const float noise = smoothed_noise_;
    Frame &frame = history_[history_write_];
    frame.absolute_slice = slice_count_;
    frame.sample_index = first_sample_index + config_.fft_size / 2;
    frame.noise = noise;
    frame.power.assign(shifted_power_.begin(), shifted_power_.end());
    if (slice_count_ % config_.search_stride_slices == 0)
        FindPeaks(shifted_power_.data(), noise, frame.peaks);
    else
        frame.peaks.clear();
    history_write_ = (history_write_ + 1) % history_capacity_;
    history_count_ = std::min(history_count_ + 1, history_capacity_);
    ++slice_count_;

    // Peak hypotheses are seeded every search_stride_slices. Compare their
    // temporal power at that same spacing so a peak between seed slices does
    // not incorrectly disqualify both neighboring hypotheses.
    last_peak_count_ = 0;
    const std::size_t stride = config_.search_stride_slices;
    if (current_slice >= stride && current_slice % stride == 0) {
        const std::uint64_t target_slice = current_slice - stride;
        Frame &target = history_[target_slice % history_capacity_];
        if (target.absolute_slice == target_slice) {
            const Frame *previous = target_slice >= stride
                ? GetFrame(target_slice - stride) : nullptr;
            ValidateTemporalPeaks(target, previous, frame);
            last_peak_count_ = target.peaks.size();
            MatchLines(target, target_slice);
        }
    }
    CompleteReady(current_slice);
    return true;
}

void ChirpSpectrumDetector::MatchLines(const Frame &current, std::uint64_t now) {
    if (now % config_.search_stride_slices != 0) return;
    const double slice_s = static_cast<double>(config_.slice_hop_samples) / config_.sample_rate_hz;
    const double bin_hz = config_.sample_rate_hz / config_.fft_size;
    const std::size_t min_lag = std::max<std::size_t>(2,
        static_cast<std::size_t>(std::floor(config_.min_duration_us * 1.0e-6 / slice_s)));
    const std::size_t max_lag = static_cast<std::size_t>(std::ceil(
        config_.max_duration_us * 1.0e-6 / slice_s));
    for (std::size_t lag = min_lag; lag <= max_lag && lag <= now; ++lag) {
        const Frame *old = GetFrame(now - lag);
        if (!old || old->peaks.empty()) continue;
        const double duration_s = static_cast<double>(current.sample_index - old->sample_index) /
                                  config_.sample_rate_hz;
        if (duration_s <= 0.0 || duration_s * 1.0e6 < config_.min_duration_us ||
            duration_s * 1.0e6 > config_.max_duration_us) continue;
        for (const Peak &a : old->peaks) for (const Peak &b : current.peaks) {
            const double slope = (static_cast<double>(b.bin) - a.bin) * bin_hz / duration_s;
            const double slope_mhz_us = slope / 1.0e12;
            if (std::fabs(slope_mhz_us) < config_.min_abs_slope_mhz_per_us ||
                std::fabs(slope_mhz_us) > config_.max_abs_slope_mhz_per_us) continue;
            const double sweep_mhz = std::fabs(b.bin - a.bin) * bin_hz / 1.0e6;
            const bool droneid_width = slope_mhz_us < 0.0 &&
                                       sweep_mhz >= 6.0 && sweep_mhz <= 12.0;
            const bool video_width = config_.enable_o2 && slope_mhz_us > 0.0 &&
                                     sweep_mhz >= 13.0 && sweep_mhz <= 23.0;
            if (!droneid_width && !video_width) continue;

            std::size_t matched = 2;
            double sum_snr = a.snr + b.snr;
            double sum_width = a.width + b.width;
            double t_sum = 0.0, f_sum = 0.0, tt_sum = 0.0, tf_sum = 0.0;
            for (std::size_t j = 0; j <= lag; ++j) {
                const Frame *f = GetFrame(now - lag + j);
                if (!f || f->peaks.empty()) continue;
                const double rel_s = static_cast<double>(f->sample_index - old->sample_index) /
                                     config_.sample_rate_hz;
                const double predicted = a.bin + slope * rel_s / bin_hz;
                const auto near = std::lower_bound(f->peaks.begin(), f->peaks.end(), predicted,
                    [](const Peak &p, double value) { return p.bin < value; });
                const Peak *best = nullptr;
                if (near != f->peaks.end() && std::fabs(near->bin - predicted) <=
                                               config_.line_match_tolerance_bins) best = &*near;
                if (near != f->peaks.begin()) {
                    const Peak &before = *(near - 1);
                    if (std::fabs(before.bin - predicted) <= config_.line_match_tolerance_bins &&
                        (!best || before.snr > best->snr)) best = &before;
                }
                if (!best) continue;
                if (j != 0 && j != lag) {
                    ++matched;
                    sum_snr += best->snr;
                    sum_width += best->width;
                }
                t_sum += rel_s;
                f_sum += best->bin;
                tt_sum += rel_s * rel_s;
                tf_sum += rel_s * best->bin;
            }
            const std::size_t expected = lag / config_.search_stride_slices + 1;
            if (matched < config_.min_matched_slices || matched * 4 < expected * 3) continue;
            const double count = static_cast<double>(matched);
            const double den = count * tt_sum - t_sum * t_sum;
            if (std::fabs(den) < 1.0e-20) continue;
            const double fitted_bin_s = (count * tf_sum - t_sum * f_sum) / den;
            const double intercept = (f_sum - fitted_bin_s * t_sum) / count;
            const double fitted_slope = fitted_bin_s * bin_hz;
            const double fitted_mhz_us = fitted_slope / 1.0e12;
            if (std::fabs(fitted_mhz_us) < config_.min_abs_slope_mhz_per_us ||
                std::fabs(fitted_mhz_us) > config_.max_abs_slope_mhz_per_us) continue;

            Candidate c;
            c.first_slice = now - lag;
            c.last_slice = now;
            c.detection.first_slice_index = c.first_slice;
            c.detection.last_slice_index = c.last_slice;
            c.reference_time_s = static_cast<double>(old->sample_index) / config_.sample_rate_hz;
            c.reference_bin = intercept;
            c.detection.line_reference_time_s = c.reference_time_s;
            c.detection.line_reference_frequency_offset_hz =
                (intercept - config_.fft_size / 2.0) * bin_hz;
            c.detection.start_time_s = c.reference_time_s;
            c.detection.end_time_s = static_cast<double>(current.sample_index) / config_.sample_rate_hz;
            c.detection.start_frequency_offset_hz =
                (intercept - config_.fft_size / 2.0) * bin_hz;
            c.detection.end_frequency_offset_hz = (intercept + fitted_bin_s * duration_s -
                                                    config_.fft_size / 2.0) * bin_hz;
            c.detection.min_frequency_offset_hz = std::min(
                c.detection.start_frequency_offset_hz,
                c.detection.end_frequency_offset_hz);
            c.detection.max_frequency_offset_hz = std::max(
                c.detection.start_frequency_offset_hz,
                c.detection.end_frequency_offset_hz);
            c.detection.slope_hz_per_s = fitted_slope;
            c.detection.slope_mhz_per_us = fitted_mhz_us;
            c.detection.duration_us = duration_s * 1.0e6;
            c.detection.sweep_width_mhz = std::fabs(
                c.detection.end_frequency_offset_hz - c.detection.start_frequency_offset_hz) / 1.0e6;
            c.detection.matched_slices = matched;
            c.detection.mean_peak_snr_db = Db(sum_snr / count);
            c.detection.mean_peak_width_bins = sum_width / count;
            const std::size_t future = static_cast<std::size_t>(std::ceil(
                config_.edge_fit_slices_per_side * c.detection.duration_us * 1.0e-6 / slice_s));
            const std::size_t packet_future = static_cast<std::size_t>(std::ceil(
                config_.droneid_packet_context_after_us * 1.0e-6 / slice_s));
            c.ready_slice = now + std::max(future,
                droneid_width ? packet_future : std::size_t(0));
            bool merged = false;
            for (Candidate &prior : pending_) {
                const double tol = std::max(c.detection.duration_us,
                    prior.detection.duration_us) * 0.45e-6;
                if (std::fabs(prior.detection.start_time_s - c.detection.start_time_s) < tol &&
                    std::fabs(prior.detection.slope_mhz_per_us - fitted_mhz_us) <
                        std::max(0.015, std::fabs(fitted_mhz_us) * 0.30)) {
                    if (matched > prior.detection.matched_slices) prior = c;
                    merged = true;
                    break;
                }
            }
            if (!merged && pending_.size() < 128) pending_.push_back(c);
        }
    }
}

bool ChirpSpectrumDetector::RefineCandidate(Candidate &c) const {
    const std::uint64_t t_slices = std::max<std::uint64_t>(1, c.last_slice - c.first_slice);
    const std::size_t side = static_cast<std::size_t>(std::ceil(
        config_.edge_fit_slices_per_side * static_cast<double>(t_slices)));
    const std::uint64_t lo = c.first_slice > side ? c.first_slice - side : 0;
    const std::uint64_t hi = c.last_slice + side;
    const std::size_t n = config_.fft_size;
    std::vector<double> profile(n, 0.0);
    std::size_t frame_count = 0;
    for (std::uint64_t s = lo; s <= hi; ++s) {
        const Frame *frame = GetFrame(s);
        if (!frame) continue;
        for (std::size_t k = 0; k < n; ++k) profile[k] += frame->power[k];
        ++frame_count;
    }
    if (frame_count < 4) return false;
    for (double &v : profile) v /= frame_count;
    std::vector<double> ordered(profile);
    const std::size_t floor_index = std::min(ordered.size() - 1,
        static_cast<std::size_t>((config_.noise_percentile / 100.0) * ordered.size()));
    std::nth_element(ordered.begin(), ordered.begin() + floor_index, ordered.end());
    const double floor = ordered[floor_index];
    std::vector<double> smooth(n, 0.0);
    double top = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t first = k > 2 ? k - 2 : 0;
        const std::size_t last = std::min(n - 1, k + 2);
        for (std::size_t j = first; j <= last; ++j) smooth[k] += profile[j] - floor;
        smooth[k] = std::max(0.0, smooth[k] / static_cast<double>(last - first + 1));
        top = std::max(top, smooth[k]);
    }
    if (!(top > 0.0)) return false;

    // Find the strongest above-floor connected band containing the chirp ridge.
    const double bin_hz = config_.sample_rate_hz / n;
    const double ridge_center = n / 2.0 +
        (0.5 * (c.detection.start_frequency_offset_hz +
                c.detection.end_frequency_offset_hz)) / bin_hz;
    std::size_t peak_bin = static_cast<std::size_t>(std::max(0.0, std::min(
        static_cast<double>(n - 1), ridge_center)));
    const std::size_t search = std::max<std::size_t>(8,
        static_cast<std::size_t>(std::fabs(c.detection.sweep_width_mhz * 1.0e6 / bin_hz)));
    const std::size_t search_lo = peak_bin > search ? peak_bin - search : 0;
    const std::size_t search_hi = std::min(n - 1, peak_bin + search);
    peak_bin = search_lo + static_cast<std::size_t>(std::max_element(
        smooth.begin() + search_lo, smooth.begin() + search_hi + 1) -
        (smooth.begin() + search_lo));
    const double threshold = top * config_.edge_threshold_fraction;
    std::size_t left = peak_bin, right = peak_bin;
    while (left > 0 && smooth[left - 1] >= threshold) --left;
    while (right + 1 < n && smooth[right + 1] >= threshold) ++right;
    if (right <= left + 2) return false;
    const bool low_clipped = left <= 1;
    const bool high_clipped = right + 2 >= n;
    double left_bin = static_cast<double>(left), right_bin = static_cast<double>(right);
    if (!low_clipped && smooth[left] != smooth[left - 1])
        left_bin = left - 1 + (threshold - smooth[left - 1]) /
                   (smooth[left] - smooth[left - 1]);
    if (!high_clipped && smooth[right] != smooth[right + 1])
        right_bin = right + (smooth[right] - threshold) /
                    (smooth[right] - smooth[right + 1]);
    const double low_hz = (left_bin - n / 2.0) * bin_hz;
    const double high_hz = (right_bin - n / 2.0) * bin_hz;
    const double intercept_hz = c.detection.start_frequency_offset_hz;
    const double slope = c.detection.slope_hz_per_s;
    if (!low_clipped) {
        c.detection.low_edge.fitted = true;
        c.detection.low_edge.frequency_offset_hz = low_hz;
        c.detection.low_edge.frequency_hz = config_.center_frequency_hz + low_hz;
        c.detection.low_edge.crossing_time_s = c.reference_time_s +
            (low_hz - intercept_hz) / slope;
    } else c.detection.low_edge.clipped = true;
    if (!high_clipped) {
        c.detection.high_edge.fitted = true;
        c.detection.high_edge.frequency_offset_hz = high_hz;
        c.detection.high_edge.frequency_hz = config_.center_frequency_hz + high_hz;
        c.detection.high_edge.crossing_time_s = c.reference_time_s +
            (high_hz - intercept_hz) / slope;
    } else c.detection.high_edge.clipped = true;
    if (c.detection.low_edge.fitted && c.detection.high_edge.fitted) {
        c.detection.band_width_valid = true;
        c.detection.fitted_band_width_mhz = (high_hz - low_hz) / 1.0e6;
    }

    // Compare the integrated ridge density with a robust per-slice estimate
    // of the occupied-band floor. A mean over the whole O2 band is biased by
    // OFDM carriers and does not represent the background under the chirp.
    double ridge_density_sum = 0.0, off_median_sum = 0.0;
    std::size_t contrast_frames = 0;
    // A linear chirp sweeps during each FFT window, so its energy is spread
    // across abs(slope) * window_duration / bin_width bins. Integrate half
    // that span on either side of the ridge center, plus two bins for window
    // leakage. A fixed one-bin radius misses most of the chirp energy at the
    // 20 MHz / 128-point live setting.
    const double window_duration_s = static_cast<double>(n) /
                                     config_.sample_rate_hz;
    const double swept_bins = std::fabs(slope) * window_duration_s / bin_hz;
    const long radius = std::max<long>(1, static_cast<long>(
        std::ceil(0.5 * swept_bins + 2.0)));
    const double band_lo_bin = c.detection.low_edge.fitted
        ? std::floor(n / 2.0 + low_hz / bin_hz)
        : std::floor(std::min(c.detection.start_frequency_offset_hz,
                              c.detection.end_frequency_offset_hz) / bin_hz + n / 2.0);
    const double band_hi_bin = c.detection.high_edge.fitted
        ? std::ceil(n / 2.0 + high_hz / bin_hz)
        : std::ceil(std::max(c.detection.start_frequency_offset_hz,
                             c.detection.end_frequency_offset_hz) / bin_hz + n / 2.0);
    // A clipped edge can place the estimated band wholly outside the captured
    // FFT, or noisy edge fits can reverse its bounds. Validate in floating
    // point before converting to an integer: converting a negative bin count
    // to size_t made vector::reserve request nearly SIZE_MAX bytes.
    const double last_bin = static_cast<double>(n - 1);
    const bool valid_band = std::isfinite(band_lo_bin) &&
        std::isfinite(band_hi_bin) && band_hi_bin >= band_lo_bin &&
        band_hi_bin >= 0.0 && band_lo_bin <= last_bin;
    if (valid_band) {
        const long band_lo = static_cast<long>(std::max(0.0, band_lo_bin));
        const long band_hi = static_cast<long>(std::min(last_bin, band_hi_bin));
        const std::size_t band_bins = static_cast<std::size_t>(band_hi - band_lo) + 1;
        std::vector<double> off_ridge_bins;
        off_ridge_bins.reserve(band_bins);
        for (std::uint64_t s = c.first_slice; s <= c.last_slice; ++s) {
            const Frame *frame = GetFrame(s);
            if (!frame) continue;
            const double t = static_cast<double>(frame->sample_index) / config_.sample_rate_hz;
            const double ridge_hz = intercept_hz + slope * (t - c.reference_time_s);
            const long center = static_cast<long>(std::llround(n / 2.0 + ridge_hz / bin_hz));
            double ridge_frame_sum = 0.0;
            std::size_t ridge_frame_count = 0;
            for (long k = std::max<long>(band_lo, center - radius);
                 k <= std::min<long>(band_hi, center + radius); ++k) {
                ridge_frame_sum += frame->power[static_cast<std::size_t>(k)];
                ++ridge_frame_count;
            }
            off_ridge_bins.clear();
            for (long k = band_lo; k <= band_hi; ++k) {
                if (std::abs(k - center) <= radius + 1) continue;
                off_ridge_bins.push_back(frame->power[static_cast<std::size_t>(k)]);
            }
            if (!ridge_frame_count || off_ridge_bins.size() < 3) continue;
            const std::size_t median_index = off_ridge_bins.size() / 2;
            std::nth_element(off_ridge_bins.begin(),
                             off_ridge_bins.begin() + median_index,
                             off_ridge_bins.end());
            double off_median = off_ridge_bins[median_index];
            if ((off_ridge_bins.size() & 1u) == 0) {
                const double lower = *std::max_element(off_ridge_bins.begin(),
                    off_ridge_bins.begin() + median_index);
                off_median = 0.5 * (off_median + lower);
            }
            ridge_density_sum += ridge_frame_sum / ridge_frame_count;
            off_median_sum += off_median;
            ++contrast_frames;
        }
    }
    if (contrast_frames) {
        const double ridge = ridge_density_sum / contrast_frames;
        const double off = off_median_sum / contrast_frames;
        if (ridge > 0.0 && off > 0.0) {
            c.detection.ridge_snr_db = Db(ridge) - Db(off);
            c.detection.ridge_snr_valid = std::isfinite(c.detection.ridge_snr_db);
        }
    }

    const double abs_slope = std::fabs(c.detection.slope_mhz_per_us);
    double signature_duration_us = c.detection.duration_us;
    double signature_width_mhz = c.detection.sweep_width_mhz;
    if (c.detection.low_edge.fitted && c.detection.high_edge.fitted) {
        const bool rising = c.detection.slope_hz_per_s > 0.0;
        const ChirpEdgeCrossing &start_edge = rising ? c.detection.low_edge : c.detection.high_edge;
        const ChirpEdgeCrossing &end_edge = rising ? c.detection.high_edge : c.detection.low_edge;
        signature_duration_us = (end_edge.crossing_time_s - start_edge.crossing_time_s) * 1.0e6;
        signature_width_mhz = c.detection.fitted_band_width_mhz;
        if (signature_duration_us > 0.0 && signature_duration_us <= 150.0) {
            c.detection.start_time_s = start_edge.crossing_time_s;
            c.detection.end_time_s = end_edge.crossing_time_s;
            c.detection.start_frequency_offset_hz = start_edge.frequency_offset_hz;
            c.detection.end_frequency_offset_hz = end_edge.frequency_offset_hz;
            c.detection.min_frequency_offset_hz = std::min(
                low_hz, high_hz);
            c.detection.max_frequency_offset_hz = std::max(
                low_hz, high_hz);
            c.detection.duration_us = signature_duration_us;
            c.detection.sweep_width_mhz = signature_width_mhz;
        }
    }
    if (c.detection.slope_mhz_per_us < 0.0 && abs_slope >= 0.10 && abs_slope <= 0.16 &&
        signature_duration_us >= 55.0 && signature_duration_us <= 90.0 &&
        (!c.detection.band_width_valid ||
         (signature_width_mhz >= 6.0 && signature_width_mhz <= 12.0)))
        c.detection.signature = ChirpSignature::DroneId;
    else if (c.detection.slope_mhz_per_us > 0.0 && abs_slope >= 0.20 && abs_slope <= 0.31 &&
             signature_duration_us >= 55.0 && signature_duration_us <= 90.0 &&
             (!c.detection.band_width_valid ||
              (signature_width_mhz >= 13.0 && signature_width_mhz <= 23.0)))
        c.detection.signature = ChirpSignature::O2;
    if (c.detection.signature != ChirpSignature::Unknown &&
        (!c.detection.ridge_snr_valid ||
         c.detection.ridge_snr_db < config_.min_ridge_snr_db))
        return false;
    if (c.detection.signature == ChirpSignature::DroneId)
        RefineDroneIdPacket(c);
    return true;
}

bool ChirpSpectrumDetector::RefineDroneIdPacket(Candidate &c) const {
    struct Row {
        const Frame *frame;
        double time_s;
        double total_power;
    };
    ChirpDetection &d = c.detection;
    const auto reject = [&d](const char *reason) {
        d.packet_fit_reject_reason = reason;
        return false;
    };
    d.packet_fit_reject_reason = "in_progress";
    const double ref = c.reference_time_s;
    const double first_s = ref - 320.0e-6;
    const double last_s = ref + config_.droneid_packet_context_after_us * 1.0e-6;
    std::vector<Row> rows;
    rows.reserve(history_count_);
    const std::uint64_t oldest = slice_count_ - history_count_;
    for (std::uint64_t i = oldest; i < slice_count_; ++i) {
        const Frame *f = GetFrame(i);
        if (!f) continue;
        const double time_s = static_cast<double>(f->sample_index) /
                              config_.sample_rate_hz;
        if (time_s < first_s || time_s > last_s) continue;
        double total = 0.0;
        for (float value : f->power) total += value;
        rows.push_back({f, time_s, total});
    }
    d.packet_fit_context_rows = rows.size();
    if (!rows.empty()) {
        d.packet_fit_context_first_offset_us = (rows.front().time_s - ref) * 1.0e6;
        d.packet_fit_context_last_offset_us = (rows.back().time_s - ref) * 1.0e6;
    }
    if (rows.size() < 16) return reject("insufficient_context_rows");

    std::vector<double> baseline_rows;
    for (const Row &r : rows) {
        if (r.time_s >= ref - 300.0e-6 && r.time_s <= ref - 220.0e-6) {
            baseline_rows.push_back(r.total_power);
            ++d.packet_fit_pre_baseline_rows;
        } else if (r.time_s >= ref + 600.0e-6 && r.time_s <= ref + 740.0e-6) {
            baseline_rows.push_back(r.total_power);
            ++d.packet_fit_post_baseline_rows;
        }
    }
    if (baseline_rows.size() < 8) {
        if (d.packet_fit_pre_baseline_rows < 8 && d.packet_fit_post_baseline_rows < 8)
            return reject("insufficient_pre_and_post_baseline_rows");
        if (d.packet_fit_pre_baseline_rows < 8)
            return reject("insufficient_pre_baseline_rows");
        return reject("insufficient_post_baseline_rows");
    }
    const auto median = [](std::vector<double> &values) {
        const std::size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        double result = values[middle];
        if ((values.size() & 1u) == 0)
            result = 0.5 * (result + *std::max_element(values.begin(),
                                                       values.begin() + middle));
        return result;
    };
    const double baseline = median(baseline_rows);
    if (!(baseline > 0.0)) return reject("invalid_baseline_power");
    std::vector<double> smoothed(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const std::size_t lo = i > 2 ? i - 2 : 0;
        const std::size_t hi = std::min(rows.size(), i + 3);
        double sum = 0.0;
        for (std::size_t j = lo; j < hi; ++j) sum += rows[j].total_power;
        smoothed[i] = sum / static_cast<double>(hi - lo);
    }
    const std::size_t peak_i = static_cast<std::size_t>(
        std::max_element(smoothed.begin(), smoothed.end()) - smoothed.begin());
    const double peak = smoothed[peak_i];
    if (peak < baseline * 1.2) return reject("packet_activity_below_1p2x_baseline");
    const double activity_threshold = baseline + 0.45 * (peak - baseline);
    std::size_t active_lo = peak_i, active_hi = peak_i;
    while (active_lo && smoothed[active_lo - 1] > activity_threshold) --active_lo;
    while (active_hi + 1 < rows.size() &&
           smoothed[active_hi + 1] > activity_threshold) ++active_hi;
    if (active_hi <= active_lo + 4) return reject("packet_activity_too_short");
    double packet_start = rows[active_lo].time_s;
    if (active_lo) {
        const double a = smoothed[active_lo - 1], b = smoothed[active_lo];
        if (b > a) packet_start = rows[active_lo - 1].time_s +
            std::max(0.0, std::min(1.0, (activity_threshold - a) / (b - a))) *
            (rows[active_lo].time_s - rows[active_lo - 1].time_s);
    }
    double packet_end = rows[active_hi].time_s;
    if (active_hi + 1 < rows.size()) {
        const double a = smoothed[active_hi], b = smoothed[active_hi + 1];
        if (a > b) packet_end = rows[active_hi].time_s +
            std::max(0.0, std::min(1.0, (a - activity_threshold) / (a - b))) *
            (rows[active_hi + 1].time_s - rows[active_hi].time_s);
    }
    const double packet_duration_us = (packet_end - packet_start) * 1.0e6;
    d.packet_start_time_s = packet_start;
    d.packet_end_time_s = packet_end;
    d.packet_duration_us = packet_duration_us;
    if (packet_duration_us < 400.0 || packet_duration_us > 850.0)
        return reject("packet_duration_outside_400_850us");

    const std::size_t n = config_.fft_size;
    std::vector<double> background(n, 0.0), integrated(n, 0.0);
    std::size_t nbg = 0, nactive = 0;
    for (const Row &r : rows) {
        const float *power = r.frame->power.data();
        if ((r.time_s >= ref - 300.0e-6 && r.time_s <= ref - 220.0e-6) ||
            (r.time_s >= ref + 600.0e-6 && r.time_s <= ref + 740.0e-6)) {
            for (std::size_t k = 0; k < n; ++k) background[k] += power[k];
            ++nbg;
        }
        if (r.time_s >= packet_start && r.time_s <= packet_end) {
            for (std::size_t k = 0; k < n; ++k) integrated[k] += power[k];
            ++nactive;
        }
    }
    if (nbg < 8 || nactive < 8)
        return reject(nbg < 8 ? "insufficient_spectral_background_rows"
                              : "insufficient_packet_spectrum_rows");
    for (std::size_t k = 0; k < n; ++k) {
        background[k] /= nbg;
        integrated[k] /= nactive;
    }
    std::vector<double> excess(n), smooth(n);
    double top = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        excess[k] = std::max(0.0, integrated[k] - background[k]);
        const std::size_t lo = k > 2 ? k - 2 : 0;
        const std::size_t hi = std::min(n - 1, k + 2);
        double sum = 0.0;
        for (std::size_t j = lo; j <= hi; ++j) sum += excess[j];
        smooth[k] = sum / static_cast<double>(hi - lo + 1);
        top = std::max(top, smooth[k]);
    }
    if (!(top > 0.0)) return reject("no_packet_band_excess_power");
    const std::size_t peak_bin = static_cast<std::size_t>(
        std::max_element(smooth.begin(), smooth.end()) - smooth.begin());
    const double edge_level = 0.15 * top;
    std::size_t left = peak_bin, right = peak_bin;
    while (left && smooth[left - 1] >= edge_level) --left;
    while (right + 1 < n && smooth[right + 1] >= edge_level) ++right;
    if (right <= left + 4) return reject("packet_band_too_narrow");
    if (left <= 1) return reject("packet_low_edge_clipped");
    if (right + 2 >= n) return reject("packet_high_edge_clipped");
    double left_bin = static_cast<double>(left), right_bin = static_cast<double>(right);
    if (smooth[left] != smooth[left - 1])
        left_bin = left - 1 + (edge_level - smooth[left - 1]) /
                   (smooth[left] - smooth[left - 1]);
    if (smooth[right] != smooth[right + 1])
        right_bin = right + (smooth[right] - edge_level) /
                    (smooth[right] - smooth[right + 1]);
    const double bin_hz = config_.sample_rate_hz / n;
    const double low_hz = (left_bin - n / 2.0) * bin_hz;
    const double high_hz = (right_bin - n / 2.0) * bin_hz;
    if (high_hz <= low_hz) return reject("invalid_packet_band_edges");
    d.packet_band_low_frequency_offset_hz = low_hz;
    d.packet_band_high_frequency_offset_hz = high_hz;
    d.packet_band_cfo_offset_hz = 0.5 * (low_hz + high_hz);
    d.packet_band_fit_valid = d.low_edge.fitted && d.high_edge.fitted;
    if (!d.packet_band_fit_valid) {
        if (!d.low_edge.fitted && d.low_edge.clipped)
            return reject("low_chirp_edge_clipped");
        if (!d.high_edge.fitted && d.high_edge.clipped)
            return reject("high_chirp_edge_clipped");
        if (!d.low_edge.fitted) return reject("low_chirp_edge_not_fitted");
        return reject("high_chirp_edge_not_fitted");
    }
    double inband = 0.0;
    std::size_t inband_count = 0;
    std::vector<double> outband;
    for (std::size_t k = 0; k < n; ++k) {
        if (k >= left && k <= right) {
            inband += std::max(0.0, integrated[k] - background[k]);
            ++inband_count;
        } else if ((k + 2 < left) || (k > right + 2)) {
            outband.push_back(background[k]);
        }
    }
    if (inband_count && outband.size() >= 8) {
        inband /= inband_count;
        const double noise_density = median(outband);
        if (inband > 0.0 && noise_density > 0.0) {
            d.packet_snr_db = 10.0 * std::log10(inband / noise_density);
            d.packet_snr_valid = std::isfinite(d.packet_snr_db);
        }
    }
    d.packet_fit_reject_reason = "ok";
    return true;
}

void ChirpSpectrumDetector::CompleteReady(std::uint64_t now) {
    for (std::size_t i = 0; i < pending_.size();) {
        if (pending_[i].ready_slice > now) { ++i; continue; }
        Candidate c = pending_[i];
        if (config_.defer_refinement) {
            bool duplicate = false;
            for (const ChirpDetection &prior : recent_) {
                const double overlap = std::max(0.0,
                    std::min(prior.end_time_s, c.detection.end_time_s) -
                    std::max(prior.start_time_s, c.detection.start_time_s));
                const double shorter = std::max(1.0e-9,
                    std::min(prior.end_time_s - prior.start_time_s,
                             c.detection.end_time_s - c.detection.start_time_s));
                if (overlap / shorter > 0.45 &&
                    std::fabs(prior.slope_mhz_per_us - c.detection.slope_mhz_per_us) <
                        std::max(0.015, std::fabs(c.detection.slope_mhz_per_us) * 0.50)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate && candidate_work_.size() < config_.maximum_queued_candidates) {
                const double slice_s = static_cast<double>(config_.slice_hop_samples) /
                                       config_.sample_rate_hz;
                const std::uint64_t chirp_slices = std::max<std::uint64_t>(
                    1, c.last_slice - c.first_slice);
                const std::size_t edge_side = static_cast<std::size_t>(std::ceil(
                    config_.edge_fit_slices_per_side * chirp_slices));
                const std::size_t pre = std::max(edge_side, static_cast<std::size_t>(std::ceil(
                    320.0e-6 / slice_s)));
                const std::size_t post = std::max(edge_side, static_cast<std::size_t>(std::ceil(
                    config_.droneid_packet_context_after_us * 1.0e-6 / slice_s)));
                const std::uint64_t first = c.first_slice > pre ? c.first_slice - pre : 0;
                const std::uint64_t last = c.last_slice + post;
                ChirpCandidateWork work;
                if (CopySpectrumRange(first, last, work.context)) {
                    work.config = config_;
                    work.detection = c.detection;
                    work.context_first_slice = first;
                    work.reference_bin = c.reference_bin;
                    work.reference_time_s = c.reference_time_s;
                    work.first_slice = c.first_slice;
                    work.last_slice = c.last_slice;
                    candidate_work_.push_back(std::move(work));
                    recent_.push_back(c.detection);
                } else {
                    ++dropped_candidates_;
                }
            } else if (!duplicate) {
                ++dropped_candidates_;
            }
        } else if (RefineCandidate(c) && c.detection.ridge_snr_valid &&
                   c.detection.ridge_snr_db >= config_.min_ridge_snr_db) {
            bool duplicate = false;
            for (const ChirpDetection &prior : recent_) {
                const double overlap = std::max(0.0,
                    std::min(prior.end_time_s, c.detection.end_time_s) -
                    std::max(prior.start_time_s, c.detection.start_time_s));
                const double shorter = std::max(1.0e-9,
                    std::min(prior.end_time_s - prior.start_time_s,
                             c.detection.end_time_s - c.detection.start_time_s));
                if (overlap / shorter > 0.45 &&
                    std::fabs(prior.slope_mhz_per_us - c.detection.slope_mhz_per_us) <
                        std::max(0.015, std::fabs(c.detection.slope_mhz_per_us) * 0.50)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                if (detections_.size() < config_.maximum_queued_detections)
                    detections_.push_back(c.detection);
                else
                    ++dropped_detections_;
                recent_.push_back(c.detection);
            }
        }
        pending_.erase(pending_.begin() + i);
    }
    const double now_s = static_cast<double>(now * config_.slice_hop_samples) /
                         config_.sample_rate_hz;
    recent_.erase(std::remove_if(recent_.begin(), recent_.end(), [now_s](const ChirpDetection &d) {
        return now_s - d.end_time_s > 0.01;
    }), recent_.end());
}

std::vector<ChirpDetection> ChirpSpectrumDetector::TakeDetections() {
    std::vector<ChirpDetection> out;
    out.swap(detections_);
    return out;
}

std::vector<ChirpCandidateWork> ChirpSpectrumDetector::TakeCandidateWork() {
    std::vector<ChirpCandidateWork> out;
    out.swap(candidate_work_);
    return out;
}

bool ChirpSpectrumDetector::RefineCandidateWork(ChirpCandidateWork &work,
                                                 std::string *error) {
    const ChirpSpectrumConfig &incoming = work.config;
    const bool same_shape = configured_ &&
        config_.sample_rate_hz == incoming.sample_rate_hz &&
        config_.fft_size == incoming.fft_size &&
        config_.slice_hop_samples == incoming.slice_hop_samples &&
        history_capacity_ >= work.context.sample_indices.size();
    if (!same_shape) {
        ChirpSpectrumConfig setup = incoming;
        setup.defer_refinement = false;
        const double slice_s = static_cast<double>(setup.slice_hop_samples) /
                               setup.sample_rate_hz;
        const std::size_t max_t = static_cast<std::size_t>(std::ceil(
            setup.max_duration_us * 1.0e-6 / slice_s));
        const std::size_t edge_history = std::max<std::size_t>(16,
            2 * static_cast<std::size_t>(std::ceil(
                setup.edge_fit_slices_per_side * max_t)) + max_t + 8);
        const std::size_t packet_history = static_cast<std::size_t>(std::ceil(
            (setup.droneid_packet_context_after_us + 400.0) * 1.0e-6 / slice_s)) +
            max_t + 8;
        setup.minimum_history_slices = std::max(setup.minimum_history_slices,
            std::max({edge_history, packet_history,
                      work.context.sample_indices.size()}));
        if (!Setup(setup, error)) return false;
    } else {
        config_ = incoming;
        config_.defer_refinement = false;
    }

    const std::size_t row_count = work.context.sample_indices.size();
    if (!row_count || work.context.fft_size != config_.fft_size ||
        work.context.power.size() != row_count * config_.fft_size ||
        work.context.noise.size() != row_count) {
        Error(error, "candidate FFT context has inconsistent dimensions");
        return false;
    }
    history_count_ = row_count;
    const std::uint64_t end_slice = work.context_first_slice + row_count;
    slice_count_ = end_slice;
    history_write_ = static_cast<std::size_t>(end_slice % history_capacity_);
    for (std::size_t row = 0; row < row_count; ++row) {
        const std::uint64_t absolute = work.context_first_slice + row;
        Frame &frame = history_[absolute % history_capacity_];
        frame.absolute_slice = absolute;
        frame.sample_index = work.context.sample_indices[row];
        frame.noise = work.context.noise[row];
        frame.peaks.clear();
        const float *begin = work.context.power.data() + row * config_.fft_size;
        frame.power.assign(begin, begin + config_.fft_size);
    }

    Candidate candidate;
    candidate.detection = work.detection;
    candidate.reference_bin = work.reference_bin;
    candidate.reference_time_s = work.reference_time_s;
    candidate.first_slice = work.first_slice;
    candidate.last_slice = work.last_slice;
    if (!RefineCandidate(candidate) || !candidate.detection.ridge_snr_valid ||
        candidate.detection.ridge_snr_db < config_.min_ridge_snr_db) {
        work.detection = std::move(candidate.detection);
        return false;
    }
    work.detection = std::move(candidate.detection);
    return true;
}

bool ChirpSpectrumDetector::CopySpectrumWindow(
    std::uint64_t first_slice, std::uint64_t last_slice,
    std::size_t margin_slices, ChirpSpectrumWindow &window) const {
    if (!configured_ || !history_count_ || first_slice > last_slice ||
        first_slice >= slice_count_) return false;

    const std::uint64_t oldest = slice_count_ - history_count_;
    const std::uint64_t newest = slice_count_ - 1;
    first_slice = first_slice > margin_slices ? first_slice - margin_slices : 0;
    last_slice = std::min(last_slice, newest);
    const std::uint64_t room_after = newest - last_slice;
    last_slice += std::min<std::uint64_t>(room_after, margin_slices);
    first_slice = std::max(first_slice, oldest);
    if (first_slice > last_slice) return false;

    return CopySpectrumRange(first_slice, last_slice, window);
}

bool ChirpSpectrumDetector::CopySpectrumRange(
    std::uint64_t first_slice, std::uint64_t last_slice,
    ChirpSpectrumWindow &window) const {
    if (!configured_ || !history_count_ || first_slice > last_slice ||
        first_slice >= slice_count_ || last_slice >= slice_count_) return false;
    const std::uint64_t oldest = slice_count_ - history_count_;
    if (first_slice < oldest) return false;

    ChirpSpectrumWindow copy;
    copy.fft_size = config_.fft_size;
    copy.slice_hop_samples = config_.slice_hop_samples;
    const std::uint64_t row_count = last_slice - first_slice + 1;
    if (row_count > std::numeric_limits<std::size_t>::max() / config_.fft_size)
        return false;
    copy.sample_indices.reserve(static_cast<std::size_t>(row_count));
    copy.noise.reserve(static_cast<std::size_t>(row_count));
    copy.power.reserve(static_cast<std::size_t>(row_count) * config_.fft_size);
    for (std::uint64_t index = first_slice; index <= last_slice; ++index) {
        const Frame *frame = GetFrame(index);
        if (!frame || frame->power.size() != config_.fft_size) return false;
        copy.sample_indices.push_back(frame->sample_index);
        copy.noise.push_back(frame->noise);
        copy.power.insert(copy.power.end(), frame->power.begin(), frame->power.end());
    }
    if (copy.sample_indices.empty()) return false;
    window = std::move(copy);
    return true;
}

std::size_t ChirpSpectrumDetector::Finish() {
    if (slice_count_) CompleteReady(slice_count_ - 1);
    const std::size_t incomplete = pending_.size();
    pending_.clear();
    return incomplete;
}
