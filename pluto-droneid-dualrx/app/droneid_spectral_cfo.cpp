#include "droneid_spectral_cfo.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <vector>

namespace {

bool NextPowerOfTwo(std::size_t value, std::size_t &result) {
    if (!value) return false;
    result = 1;
    while (result < value) {
        if (result > std::numeric_limits<std::size_t>::max() / 2) return false;
        result <<= 1;
    }
    return true;
}

void ForwardRadix2(std::vector<std::complex<float>> &data) {
    const std::size_t n = data.size();
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(data[i], data[j]);
    }
    for (std::size_t length = 2; length <= n; length <<= 1) {
        const float angle = static_cast<float>(-2.0 * M_PI / length);
        const std::complex<float> step(std::cos(angle), std::sin(angle));
        for (std::size_t base = 0; base < n; base += length) {
            std::complex<float> twiddle(1.0f, 0.0f);
            const std::size_t half = length >> 1;
            for (std::size_t k = 0; k < half; ++k) {
                const std::complex<float> even = data[base + k];
                const std::complex<float> odd = data[base + k + half] * twiddle;
                data[base + k] = even + odd;
                data[base + k + half] = even - odd;
                twiddle *= step;
            }
        }
        if (length == n) break;
    }
}

double Median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    if (values.size() & 1) return values[middle];
    const double upper = values[middle];
    std::nth_element(values.begin(), values.begin() + middle - 1,
                     values.begin() + middle);
    return 0.5 * (values[middle - 1] + upper);
}

} // namespace

bool EstimateDroneIdSpectralCfo(const float *iq_interleaved,
                                std::size_t sample_count,
                                std::size_t active_first,
                                std::size_t active_end,
                                double sample_rate_hz,
                                double coarse_cfo_hz,
                                DroneIdSpectralCfoEstimate &estimate) {
    estimate = DroneIdSpectralCfoEstimate();
    if (!iq_interleaved || sample_rate_hz <= 0.0 || active_end > sample_count ||
        active_end <= active_first)
        return false;

    const std::size_t packet_samples = active_end - active_first;
    if (packet_samples < 256) return false;
    std::size_t fft_size = 0;
    if (!NextPowerOfTwo(packet_samples, fft_size)) return false;

    std::vector<std::complex<float>> spectrum(fft_size, {0.0f, 0.0f});
    const double window_denominator = packet_samples > 1
        ? static_cast<double>(packet_samples - 1) : 1.0;
    for (std::size_t i = 0; i < packet_samples; ++i) {
        const double phase = 2.0 * M_PI * static_cast<double>(i) /
                             window_denominator;
        const float window = static_cast<float>(0.5 - 0.5 * std::cos(phase));
        spectrum[i] = std::complex<float>(iq_interleaved[2 * (active_first + i)],
                                          iq_interleaved[2 * (active_first + i) + 1]) *
                      window;
    }
    ForwardRadix2(spectrum);

    const double bin_hz = sample_rate_hz / static_cast<double>(fft_size);
    std::vector<double> power(fft_size);
    std::vector<double> prefix(fft_size + 1, 0.0);
    for (std::size_t i = 0; i < fft_size; ++i) {
        const std::size_t shifted = (i + fft_size / 2) % fft_size;
        power[i] = std::norm(spectrum[shifted]);
        prefix[i + 1] = prefix[i] + power[i];
    }

    std::size_t smooth_width = static_cast<std::size_t>(std::llround(15000.0 / bin_hz));
    smooth_width = std::max<std::size_t>(3, smooth_width);
    if ((smooth_width & 1) == 0) ++smooth_width;
    const std::size_t half_width = smooth_width / 2;
    std::vector<double> smooth(fft_size);
    for (std::size_t i = 0; i < fft_size; ++i) {
        const std::size_t first = i > half_width ? i - half_width : 0;
        const std::size_t end = std::min(fft_size, i + half_width + 1);
        smooth[i] = (prefix[end] - prefix[first]) / static_cast<double>(end - first);
    }

    const double nyquist = sample_rate_hz / 2.0;
    std::vector<double> background_values;
    std::vector<double> band_values;
    for (std::size_t i = 0; i < fft_size; ++i) {
        const double frequency = (static_cast<double>(i) - fft_size / 2.0) * bin_hz;
        const double distance = std::fabs(frequency - coarse_cfo_hz);
        if (distance >= 5.8e6 && distance <= std::min(9.0e6, nyquist))
            background_values.push_back(smooth[i]);
        if (distance <= 4.5e6) band_values.push_back(smooth[i]);
    }
    const double background = Median(background_values);
    const double band_level = Median(band_values);
    if (background_values.size() < 16 || band_values.size() < 16 ||
        !(band_level > background))
        return false;
    estimate.packet_snr_db = 10.0 * std::log10((band_level - background) / background);
    estimate.packet_snr_valid = std::isfinite(estimate.packet_snr_db);
    const double threshold = background + 0.20 * (band_level - background);

    const double left_min = std::max(-nyquist, coarse_cfo_hz - 6.0e6);
    const double left_max = std::min(nyquist, coarse_cfo_hz - 3.0e6);
    const double right_min = std::max(-nyquist, coarse_cfo_hz + 3.0e6);
    const double right_max = std::min(nyquist, coarse_cfo_hz + 6.0e6);
    std::size_t left_index = fft_size, right_index = fft_size;
    for (std::size_t i = 0; i < fft_size; ++i) {
        const double frequency = (static_cast<double>(i) - fft_size / 2.0) * bin_hz;
        if (left_index == fft_size && frequency >= left_min && frequency <= left_max &&
            smooth[i] >= threshold)
            left_index = i;
        if (frequency >= right_min && frequency <= right_max && smooth[i] >= threshold)
            right_index = i;
    }
    if (left_index == fft_size || right_index == fft_size || right_index <= left_index)
        return false;

    const auto bin_frequency = [bin_hz, fft_size](std::size_t index) {
        return (static_cast<double>(index) - fft_size / 2.0) * bin_hz;
    };
    double low_edge = bin_frequency(left_index);
    if (left_index > 0 && smooth[left_index] != smooth[left_index - 1]) {
        const double fraction = (threshold - smooth[left_index - 1]) /
                                (smooth[left_index] - smooth[left_index - 1]);
        low_edge = bin_frequency(left_index - 1) + fraction * bin_hz;
    }
    double high_edge = bin_frequency(right_index);
    if (right_index + 1 < fft_size && smooth[right_index] != smooth[right_index + 1]) {
        const double fraction = (smooth[right_index] - threshold) /
                                (smooth[right_index] - smooth[right_index + 1]);
        high_edge = bin_frequency(right_index) + fraction * bin_hz;
    }
    const double width_hz = high_edge - low_edge;
    const double edge_cfo = 0.5 * (low_edge + high_edge);
    if (width_hz < 7.0e6 || width_hz > 12.0e6 ||
        std::fabs(edge_cfo - coarse_cfo_hz) > 500.0e3)
        return false;

    std::size_t notch_index = fft_size;
    const double notch_search_hz = 100.0e3;
    for (std::size_t i = 0; i < fft_size; ++i) {
        const double frequency = bin_frequency(i);
        if (std::fabs(frequency - edge_cfo) <= notch_search_hz &&
            (notch_index == fft_size || smooth[i] < smooth[notch_index]))
            notch_index = i;
    }
    if (notch_index == fft_size) return false;
    double notch = bin_frequency(notch_index);
    if (notch_index > 0 && notch_index + 1 < fft_size) {
        const double y0 = std::log(std::max(smooth[notch_index - 1], 1.0e-30));
        const double y1 = std::log(std::max(smooth[notch_index], 1.0e-30));
        const double y2 = std::log(std::max(smooth[notch_index + 1], 1.0e-30));
        const double denominator = y0 - 2.0 * y1 + y2;
        if (std::fabs(denominator) > 1.0e-12) {
            const double delta = std::max(-1.0, std::min(1.0,
                0.5 * (y0 - y2) / denominator));
            notch += delta * bin_hz;
        }
    }
    const std::size_t side_bins = std::max<std::size_t>(1,
        static_cast<std::size_t>(std::llround(75.0e3 / bin_hz)));
    std::vector<double> notch_sides;
    for (int direction : {-1, 1}) {
        const long center = static_cast<long>(notch_index) +
                            direction * static_cast<long>(side_bins * 2);
        const long first = std::max<long>(0, center - static_cast<long>(side_bins / 2));
        const long end = std::min<long>(static_cast<long>(fft_size),
                                        center + static_cast<long>(side_bins / 2 + 1));
        for (long i = first; i < end; ++i) notch_sides.push_back(smooth[static_cast<std::size_t>(i)]);
    }
    const double side_level = Median(notch_sides);
    const double notch_depth_db = side_level > smooth[notch_index]
        ? 10.0 * std::log10(side_level / smooth[notch_index]) : 0.0;

    /* The edge midpoint is preferred when the independently located carrier
     * notch supports it. Under noise, one edge may wander; a strong notch can
     * then provide a safer CFO. Reject a result if neither agrees with the
     * original coarse estimate, leaving the caller to use that estimate. */
    double cfo = edge_cfo;
    bool used_notch_cfo = false;
    const double edge_notch_delta = std::fabs(edge_cfo - notch);
    const double edge_coarse_delta = std::fabs(edge_cfo - coarse_cfo_hz);
    const double notch_coarse_delta = std::fabs(notch - coarse_cfo_hz);
    if (edge_notch_delta > 15.0e3) {
        if (notch_depth_db >= 5.0 && notch_coarse_delta <= 100.0e3) {
            cfo = notch;
            used_notch_cfo = true;
        } else if (edge_coarse_delta <= 15.0e3) {
            cfo = edge_cfo;
        } else {
            return false;
        }
    }
    if (std::fabs(cfo - coarse_cfo_hz) > 100.0e3) return false;

    estimate.valid = true;
    estimate.cfo_hz = cfo;
    estimate.low_edge_hz = low_edge;
    estimate.high_edge_hz = high_edge;
    estimate.notch_hz = notch;
    estimate.notch_depth_db = notch_depth_db;
    estimate.used_notch_cfo = used_notch_cfo;
    estimate.packet_samples = packet_samples;
    estimate.fft_size = fft_size;
    return true;
}
