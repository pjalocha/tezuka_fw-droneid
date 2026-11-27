#include <iio.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <vector>

namespace {
const double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) {
    stop_requested = 1;
}

struct Options {
    uint64_t channel_hz = 0;
    uint64_t sample_rate_hz = 30720000;
    uint64_t offset_hz = 5000000;
    uint64_t rf_bandwidth_hz = 20000000;
    std::string side;
    std::string waveform;
    double gain_db = -89.75;
    unsigned repeat_ms = 0;
};

void usage(const char *program) {
    std::cerr << "Usage: " << program
              << " --channel-hz HZ --side upper|lower --waveform FILE"
                 " [--sample-rate-hz HZ] [--offset-hz HZ]"
                 " [--rf-bandwidth-hz HZ] [--gain-db DB] [--repeat-ms MS]\n";
}

bool parse_options(int argc, char **argv, Options &o) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            std::exit(0);
        }
        if (i + 1 >= argc) return false;
        const char *value = argv[++i];
        if (arg == "--channel-hz") {
            char *end = NULL;
            errno = 0;
            const unsigned long long parsed = std::strtoull(value, &end, 10);
            if (errno || !end || *end || parsed == 0) return false;
            o.channel_hz = static_cast<uint64_t>(parsed);
        } else if (arg == "--side") {
            o.side = value;
        } else if (arg == "--sample-rate-hz" || arg == "--offset-hz" ||
                   arg == "--rf-bandwidth-hz") {
            char *end = NULL;
            errno = 0;
            const unsigned long long parsed = std::strtoull(value, &end, 10);
            if (errno || !end || *end || parsed == 0) return false;
            if (arg == "--sample-rate-hz") o.sample_rate_hz = parsed;
            else if (arg == "--offset-hz") o.offset_hz = parsed;
            else o.rf_bandwidth_hz = parsed;
        } else if (arg == "--waveform") {
            o.waveform = value;
        } else if (arg == "--gain-db") {
            char *end = NULL;
            errno = 0;
            o.gain_db = std::strtod(value, &end);
            if (errno || !end || *end || !std::isfinite(o.gain_db)) return false;
        } else if (arg == "--repeat-ms") {
            char *end = NULL;
            errno = 0;
            const unsigned long parsed = std::strtoul(value, &end, 10);
            if (errno || !end || *end || parsed > 3600000) return false;
            o.repeat_ms = static_cast<unsigned>(parsed);
        } else {
            return false;
        }
    }
    const bool supported_sample_rate = o.sample_rate_hz == 15360000ULL ||
                                       o.sample_rate_hz == 30720000ULL;
    const bool bandwidth_in_range = o.rf_bandwidth_hz >= 200000ULL &&
                                    o.rf_bandwidth_hz <= 20000000ULL;
    const bool offset_fits_sample_rate = o.offset_hz <= 20000000ULL &&
        o.offset_hz + 5000000ULL < o.sample_rate_hz / 2;
    return o.channel_hz >= 2400000000ULL && o.channel_hz <= 2500000000ULL &&
           supported_sample_rate && bandwidth_in_range && offset_fits_sample_rate &&
           (o.side == "upper" || o.side == "lower") && !o.waveform.empty() &&
           o.gain_db >= -89.75 && o.gain_db <= 0.0;
}

bool load_cf32(const std::string &path, std::vector<float> &iq) {
    std::ifstream file(path.c_str(), std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "Cannot open waveform: " << path << "\n";
        return false;
    }
    const std::streamoff bytes = file.tellg();
    if (bytes <= 0 || bytes % (2 * static_cast<std::streamoff>(sizeof(float))) != 0) {
        std::cerr << "Waveform must contain interleaved complex float32 IQ samples\n";
        return false;
    }
    file.seekg(0);
    iq.resize(static_cast<size_t>(bytes) / sizeof(float));
    if (!file.read(reinterpret_cast<char *>(iq.data()), bytes)) {
        std::cerr << "Short read from waveform\n";
        return false;
    }
    for (size_t i = 0; i < iq.size(); ++i) {
        if (!std::isfinite(iq[i])) {
            std::cerr << "Waveform contains non-finite samples\n";
            return false;
        }
    }
    return true;
}

int write_attr(struct iio_channel *channel, const char *name, long long value) {
    const int ret = iio_channel_attr_write_longlong(channel, name, value);
    if (ret < 0) {
        char error[128];
        iio_strerror(-ret, error, sizeof(error));
        std::cerr << "IIO write " << name << " failed: " << error << "\n";
    }
    return ret;
}

int write_attr(struct iio_channel *channel, const char *name, const char *value) {
    const int ret = iio_channel_attr_write(channel, name, value);
    if (ret < 0) {
        char error[128];
        iio_strerror(-ret, error, sizeof(error));
        std::cerr << "IIO write " << name << " failed: " << error << "\n";
    }
    return ret;
}

} // namespace

int main(int argc, char **argv) {
    Options options;
    if (!parse_options(argc, argv, options)) {
        usage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    std::vector<float> iq;
    if (!load_cf32(options.waveform, iq)) return 1;
    const size_t sample_count = iq.size() / 2;
    if (sample_count < 2048 || sample_count > 1000000) {
        std::cerr << "Unexpected waveform sample count: " << sample_count << "\n";
        return 1;
    }

    const double offset = options.side == "upper" ?
                          static_cast<double>(options.offset_hz) :
                          -static_cast<double>(options.offset_hz);
    const long long lo_hz = static_cast<long long>(options.channel_hz) -
                            static_cast<long long>(offset);
    if (lo_hz < 70000000LL || lo_hz > 6000000000LL) {
        std::cerr << "Calculated TX LO is outside the Pluto AD936x tuning range\n";
        return 2;
    }

    struct iio_context *ctx = iio_create_default_context();
    if (!ctx) {
        std::cerr << "Cannot create local IIO context\n";
        return 1;
    }
    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    struct iio_device *tx = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (!phy || !tx) {
        std::cerr << "Required IIO devices ad9361-phy and cf-ad9361-dds-core-lpc not found\n";
        iio_context_destroy(ctx);
        return 1;
    }
    struct iio_channel *lo = iio_device_find_channel(phy, "altvoltage1", true);
    struct iio_channel *phy_tx0 = iio_device_find_channel(phy, "voltage0", true);
    struct iio_channel *i_chan = iio_device_find_channel(tx, "voltage0", true);
    struct iio_channel *q_chan = iio_device_find_channel(tx, "voltage1", true);
    if (!lo || !phy_tx0 || !i_chan || !q_chan) {
        std::cerr << "Required AD9361 or TX DMA channels are missing\n";
        iio_context_destroy(ctx);
        return 1;
    }

    int ret = write_attr(lo, "frequency", lo_hz);
    if (ret >= 0) ret = write_attr(phy_tx0, "sampling_frequency",
                                   static_cast<long long>(options.sample_rate_hz));
    if (ret >= 0) ret = write_attr(phy_tx0, "rf_bandwidth",
                                   static_cast<long long>(options.rf_bandwidth_hz));
    if (ret >= 0) {
        char gain[32];
        std::snprintf(gain, sizeof(gain), "%.2f", options.gain_db);
        ret = write_attr(phy_tx0, "hardwaregain", gain);
    }
    if (ret < 0) {
        iio_context_destroy(ctx);
        return 1;
    }

    iio_channel_enable(i_chan);
    iio_channel_enable(q_chan);
    struct iio_buffer *buffer = iio_device_create_buffer(tx, sample_count, false);
    if (!buffer) {
        std::cerr << "Cannot create finite TX DMA buffer\n";
        iio_channel_disable(i_chan);
        iio_channel_disable(q_chan);
        iio_context_destroy(ctx);
        return 1;
    }

    const ptrdiff_t step = iio_buffer_step(buffer);
    char *i_ptr = static_cast<char *>(iio_buffer_first(buffer, i_chan));
    char *q_ptr = static_cast<char *>(iio_buffer_first(buffer, q_chan));
    if (!i_ptr || !q_ptr || step <= 0) {
        std::cerr << "Invalid TX IIO buffer layout\n";
        iio_buffer_destroy(buffer);
        iio_channel_disable(i_chan);
        iio_channel_disable(q_chan);
        iio_context_destroy(ctx);
        return 1;
    }

    const double sample_rate = static_cast<double>(options.sample_rate_hz);
    const double phase_step = 2.0 * kPi * offset / sample_rate;
    const double phase_inc_re = std::cos(phase_step);
    const double phase_inc_im = std::sin(phase_step);
    double osc_re = 1.0, osc_im = 0.0;
    const float scale = 0.70f * 32767.0f;
    for (size_t n = 0; n < sample_count; ++n) {
        const double in_re = iq[2 * n];
        const double in_im = iq[2 * n + 1];
        const int16_t sample_i = static_cast<int16_t>(std::lrint(
            std::max(-1.0, std::min(1.0, in_re * osc_re - in_im * osc_im)) * scale));
        const int16_t sample_q = static_cast<int16_t>(std::lrint(
            std::max(-1.0, std::min(1.0, in_re * osc_im + in_im * osc_re)) * scale));
        iio_channel_convert_inverse(i_chan, i_ptr + n * step, &sample_i);
        iio_channel_convert_inverse(q_chan, q_ptr + n * step, &sample_q);
        const double next_re = osc_re * phase_inc_re - osc_im * phase_inc_im;
        osc_im = osc_re * phase_inc_im + osc_im * phase_inc_re;
        osc_re = next_re;
        if ((n & 4095U) == 4095U) {
            const double mag = std::hypot(osc_re, osc_im);
            if (mag > 0.0) { osc_re /= mag; osc_im /= mag; }
        }
    }

    unsigned transmissions = 0;
    unsigned interval_transmissions = 0;
    const auto tx_start = std::chrono::steady_clock::now();
    auto interval_start = tx_start;
    std::cout << "Starting TX: channel=" << options.channel_hz
              << " Hz side=" << options.side << " LO=" << lo_hz
              << " Hz sample_rate=" << options.sample_rate_hz
              << " Hz rf_bandwidth=" << options.rf_bandwidth_hz << " Hz"
              << "; reporting successful IIO buffer pushes every second"
              << std::endl;
    do {
        const ssize_t pushed = iio_buffer_push(buffer);
        if (pushed < 0) {
            char error[128];
            iio_strerror(static_cast<int>(-pushed), error, sizeof(error));
            std::cerr << "IIO TX push failed: " << error << "\n";
            ret = static_cast<int>(pushed);
            break;
        }
        ++transmissions;
        ++interval_transmissions;

        const auto now = std::chrono::steady_clock::now();
        const double interval_seconds =
            std::chrono::duration<double>(now - interval_start).count();
        if (interval_seconds >= 1.0) {
            std::cout << "IIO TX pushes: " << interval_transmissions
                      << " in " << interval_seconds << " s ("
                      << (interval_transmissions / interval_seconds)
                      << "/s), total=" << transmissions
                      << ", last_buffer=" << pushed << " bytes"
                      << std::endl;
            interval_transmissions = 0;
            interval_start = now;
        }

        if (!options.repeat_ms) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(options.repeat_ms));
    } while (transmissions < 1000000 && !stop_requested);

    const double elapsed_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - tx_start).count();
    std::cout << "TX stopped: successful IIO buffer pushes=" << transmissions
              << " elapsed=" << elapsed_seconds << " s"
              << " average="
              << (elapsed_seconds > 0.0 ? transmissions / elapsed_seconds : 0.0)
              << "/s; channel=" << options.channel_hz
              << " Hz side=" << options.side
              << " LO=" << lo_hz
              << " Hz sample_rate=" << options.sample_rate_hz
              << " samples=" << sample_count
              << " gain=" << options.gain_db << " dB"
              << (ret < 0 ? " status=IIO-error" :
                  (stop_requested ? " status=stopped-by-signal" : " status=complete"))
              << std::endl;
    iio_buffer_destroy(buffer);
    iio_channel_disable(i_chan);
    iio_channel_disable(q_chan);
    iio_context_destroy(ctx);
    return ret < 0 ? 1 : 0;
}
