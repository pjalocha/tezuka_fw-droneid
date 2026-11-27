#include <iio.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace {
constexpr std::uint64_t kDefaultBase = 0x7c460000;
constexpr std::size_t kMapSize = 0x1000;
constexpr std::uint32_t kDetectorControl = 0x3c;
constexpr std::uint32_t kCaptureControl = 0x44;
constexpr std::uint32_t kCaptureStatus = 0x48;
constexpr std::uint32_t kCaptureComplete = 1u << 2;
constexpr std::uint32_t kTriggerSample = 0x4c;
constexpr std::uint32_t kDroppedTriggers = 0x50;
constexpr std::uint32_t kEventCount = 0x54;
constexpr std::uint32_t kEventBase = 0x80;
constexpr std::size_t kPreSamples = 15360;
constexpr std::size_t kPostSamples = 15360;
constexpr std::size_t kTotalSamples = kPreSamples + kPostSamples;
volatile std::sig_atomic_t g_stop = 0;

void stop_handler(int) { g_stop = 1; }

struct Options {
    std::uint64_t frequency_hz = 2444500000ULL;
    std::uint64_t sample_rate_hz = 15360000ULL;
    std::uint64_t bandwidth_hz = 10000000ULL;
    std::uint64_t register_base = kDefaultBase;
    double gain_db = 30.0;
    unsigned count = 0;
    unsigned software_interval_ms = 0;
    bool software_once = false;
    bool noise_monitor = false;
    std::string output_dir = ".";
    std::string mem_device = "/dev/mem";
};

void usage(const char *name) {
    std::cerr << "Usage: " << name << " [--channel-hz HZ] [--sample-rate-hz HZ]"
              << " [--bandwidth-hz HZ] [--gain-db DB (-3..71 dB at 2.4 GHz)] [--count N]"
              << " [--software-trigger | --noise-monitor |"
                 " --software-trigger-interval-ms MS]"
              << " [--output-dir DIR] [--mem /dev/mem] [--phys ADDRESS]\n"
              << "Default: hardware detector triggers, 15.36 Ms/s, 10 MHz BW,"
              << " 1 ms pre + 1 ms post on both RX chains.\n";
}

bool number(const char *s, std::uint64_t &value) {
    char *end = nullptr;
    errno = 0;
    const unsigned long long parsed = std::strtoull(s, &end, 0);
    if (errno || !end || *end) return false;
    value = static_cast<std::uint64_t>(parsed);
    return true;
}

bool parse(int argc, char **argv, Options &o) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h") { usage(argv[0]); std::exit(0); }
        if (arg == "--software-trigger") { o.software_once = true; continue; }
        if (arg == "--noise-monitor") {
            o.noise_monitor = true;
            o.software_interval_ms = 1000;
            continue;
        }
        if (i + 1 >= argc) return false;
        const char *value = argv[++i];
        std::uint64_t parsed = 0;
        if (arg == "--output-dir") o.output_dir = value;
        else if (arg == "--mem") o.mem_device = value;
        else if (arg == "--channel-hz" || arg == "--sample-rate-hz" ||
                 arg == "--bandwidth-hz" || arg == "--phys" ||
                 arg == "--count" || arg == "--software-trigger-interval-ms") {
            if (!number(value, parsed)) return false;
            if (arg == "--channel-hz") o.frequency_hz = parsed;
            else if (arg == "--sample-rate-hz") o.sample_rate_hz = parsed;
            else if (arg == "--bandwidth-hz") o.bandwidth_hz = parsed;
            else if (arg == "--phys") o.register_base = parsed;
            else if (arg == "--count") o.count = static_cast<unsigned>(parsed);
            else o.software_interval_ms = static_cast<unsigned>(parsed);
        } else if (arg == "--gain-db") {
            char *end = nullptr;
            errno = 0;
            o.gain_db = std::strtod(value, &end);
            if (errno || !end || *end) return false;
        } else return false;
    }
    const bool valid_rate = o.sample_rate_hz == 15360000;
    const bool valid = o.frequency_hz >= 2400000000ULL && o.frequency_hz <= 2500000000ULL &&
           valid_rate && o.bandwidth_hz >= 200000 && o.bandwidth_hz <= 20000000 &&
           o.gain_db >= -3.0 && o.gain_db <= 71.0 &&
           (!o.noise_monitor || o.software_interval_ms != 0) &&
           !(o.software_once && o.software_interval_ms != 0);
    if (valid && o.software_once && o.count == 0) o.count = 1;
    return valid;
}

class Registers {
public:
    bool open_map(const Options &o) {
        fd_ = ::open(o.mem_device.c_str(), O_RDWR | O_SYNC);
        if (fd_ < 0) { std::cerr << "open " << o.mem_device << ": " << std::strerror(errno) << '\n'; return false; }
        map_ = mmap(nullptr, kMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd_, static_cast<off_t>(o.register_base));
        if (map_ == MAP_FAILED) { map_ = nullptr; std::cerr << "mmap: " << std::strerror(errno) << '\n'; return false; }
        regs_ = static_cast<volatile std::uint32_t *>(map_);
        return true;
    }
    ~Registers() { if (map_) munmap(map_, kMapSize); if (fd_ >= 0) close(fd_); }
    std::uint32_t read(std::uint32_t offset) const { return regs_[offset / 4]; }
    void write(std::uint32_t offset, std::uint32_t value) { regs_[offset / 4] = value; }
private:
    int fd_ = -1;
    void *map_ = nullptr;
    volatile std::uint32_t *regs_ = nullptr;
};

bool write_exact(const std::string &path, const std::vector<std::int16_t> &samples) {
    std::ofstream out(path.c_str(), std::ios::binary | std::ios::trunc);
    if (!out) { std::cerr << "Cannot create " << path << '\n'; return false; }
    out.write(reinterpret_cast<const char *>(samples.data()),
              static_cast<std::streamsize>(samples.size() * sizeof(std::int16_t)));
    if (!out) { std::cerr << "Write failed: " << path << '\n'; return false; }
    return true;
}

std::string timestamp() {
    const std::time_t now = std::time(nullptr);
    char text[32];
    std::tm tm_value;
    localtime_r(&now, &tm_value);
    std::strftime(text, sizeof(text), "%Y%m%d-%H%M%S", &tm_value);
    return text;
}
} // namespace

int main(int argc, char **argv) {
    Options o;
    if (!parse(argc, argv, o)) { usage(argv[0]); return 2; }
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
    if (mkdir(o.output_dir.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "mkdir " << o.output_dir << ": " << std::strerror(errno) << '\n'; return 1;
    }

    Registers regs;
    if (!regs.open_map(o)) return 1;
    struct iio_context *ctx = iio_create_default_context();
    if (!ctx) { std::cerr << "Cannot create local IIO context\n"; return 1; }
    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    struct iio_device *rx = iio_context_find_device(ctx, "cf-ad9361-lpc");
    struct iio_channel *rx_cfg = phy ? iio_device_find_channel(phy, "voltage0", false) : nullptr;
    struct iio_channel *rx_cfg1 = phy ? iio_device_find_channel(phy, "voltage1", false) : nullptr;
    struct iio_channel *rx_lo = phy ? iio_device_find_channel(phy, "altvoltage0", true) : nullptr;
    if (!phy || !rx || !rx_cfg || !rx_cfg1 || !rx_lo) {
        std::cerr << "Missing ad9361-phy / cf-ad9361-lpc RX devices or channels\n";
        iio_context_destroy(ctx); return 1;
    }
    int ret = iio_channel_attr_write_longlong(rx_cfg, "sampling_frequency", o.sample_rate_hz);
    if (ret >= 0) ret = iio_channel_attr_write_longlong(rx_cfg, "rf_bandwidth", o.bandwidth_hz);
    if (ret >= 0) ret = iio_channel_attr_write_longlong(rx_lo, "frequency", o.frequency_hz);
    if (ret >= 0) ret = iio_channel_attr_write(rx_cfg, "gain_control_mode", "manual");
    if (ret >= 0) ret = iio_channel_attr_write_double(rx_cfg, "hardwaregain", o.gain_db);
    // AD9361 has independent gain control for its two receive chains.
    if (ret >= 0) ret = iio_channel_attr_write(rx_cfg1, "gain_control_mode", "manual");
    if (ret >= 0) ret = iio_channel_attr_write_double(rx_cfg1, "hardwaregain", o.gain_db);
    if (ret < 0) {
        char err[128]; iio_strerror(-ret, err, sizeof(err));
        std::cerr << "Configure RX failed: " << err << '\n'; iio_context_destroy(ctx); return 1;
    }

    struct iio_channel *channels[4] = {};
    for (unsigned i = 0; i < 4; ++i) {
        char id[16]; std::snprintf(id, sizeof(id), "voltage%u", i);
        channels[i] = iio_device_find_channel(rx, id, false);
        if (!channels[i]) { std::cerr << "RX scan channel " << id << " missing; FPGA must expose both RX chains\n"; iio_context_destroy(ctx); return 1; }
        iio_channel_enable(channels[i]);
    }
    iio_device_set_kernel_buffers_count(rx, 1);
    struct iio_buffer *buffer = iio_device_create_buffer(rx, kTotalSamples, false);
    if (!buffer) { std::cerr << "Cannot create RX IIO buffer for " << kTotalSamples << " scans\n"; iio_context_destroy(ctx); return 1; }
    const ptrdiff_t step = iio_buffer_step(buffer);
    if (step < 8) {
        std::cerr << "Unexpected four-lane RX scan layout (step=" << step << ")\n";
        iio_buffer_destroy(buffer); iio_context_destroy(ctx); return 1;
    }

    // Start the FPGA detector and arm fixed-size capture mode.
    // Acknowledge a capture left ready by an earlier interrupted process.
    const std::uint32_t old_detector_enable = regs.read(kDetectorControl) & 1u;
    regs.write(kDetectorControl,
               (o.software_once || o.software_interval_ms != 0) ? 0u : 1u);
    regs.write(kCaptureControl, 5u);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    regs.write(kCaptureControl, 1u);
    std::cout << "Armed dual-RX capture at " << o.frequency_hz << " Hz, "
              << o.sample_rate_hz << " Sa/s; each file contains 1 ms before and after the trigger.\n";
    unsigned captures = 0;
    while (!g_stop && (o.count == 0 || captures < o.count)) {
        if (o.software_once || o.software_interval_ms != 0) {
            // Let the 1 ms history ring fill after arming/reset, then trigger.
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            regs.write(kCaptureControl, 3u); // capture enable + software-trigger toggle
            o.software_once = false;
        }
        const ssize_t bytes = iio_buffer_refill(buffer);
        const std::size_t expected = kTotalSamples * static_cast<std::size_t>(step);
        if (bytes < 0 || static_cast<std::size_t>(bytes) != expected) {
            char err[128] = {};
            if (bytes < 0) iio_strerror(static_cast<int>(-bytes), err, sizeof(err));
            std::cerr << "IIO capture returned " << bytes << " bytes (expected " << expected << ")"
                      << (bytes < 0 ? std::string(": ") + err : "") << '\n';
            break;
        }
        // For high-speed local IIO RX buffers, the DMA buffer is mapped by
        // iio_buffer_refill(). Its pointer may change on each refill, so only
        // resolve channel pointers after refill and use them before refilling
        // again.
        char *lane_ptr[4];
        char *buffer_end = static_cast<char *>(iio_buffer_end(buffer));
        if (!buffer_end) {
            std::cerr << "IIO RX buffer has no data pointer after refill\n";
            break;
        }
        bool valid_layout = true;
        for (unsigned i = 0; i < 4; ++i) {
            lane_ptr[i] = static_cast<char *>(iio_buffer_first(buffer, channels[i]));
            if (!lane_ptr[i] || lane_ptr[i] >= buffer_end) {
                std::cerr << "RX scan channel voltage" << i
                          << " is absent from the refilled IIO buffer (step="
                          << step << ", bytes=" << bytes << ")\n";
                valid_layout = false;
                break;
            }
        }
        if (!valid_layout) break;
        std::vector<std::int16_t> rx0(kTotalSamples * 2), rx1(kTotalSamples * 2);
        for (std::size_t n = 0; n < kTotalSamples; ++n) {
            std::memcpy(&rx0[2*n], lane_ptr[0] + n*step, sizeof(std::int16_t));
            std::memcpy(&rx0[2*n+1], lane_ptr[1] + n*step, sizeof(std::int16_t));
            std::memcpy(&rx1[2*n], lane_ptr[2] + n*step, sizeof(std::int16_t));
            std::memcpy(&rx1[2*n+1], lane_ptr[3] + n*step, sizeof(std::int16_t));
        }
        const std::uint32_t seq = ++captures;
        char base[64]; std::snprintf(base, sizeof(base), "droneid-%s-%06u", timestamp().c_str(), seq);
        const std::string stem = o.output_dir + "/" + base;
        if (!write_exact(stem + ".rx0.cx16", rx0) || !write_exact(stem + ".rx1.cx16", rx1)) break;
        const unsigned events = regs.read(kEventCount) & 0xfu;
        std::ofstream meta((stem + ".json").c_str());
        meta << "{\n  \"format\": \"signed int16 little-endian I,Q\",\n"
             << "  \"sample_rate_hz\": " << o.sample_rate_hz << ",\n"
             << "  \"center_frequency_hz\": " << o.frequency_hz << ",\n"
             << "  \"samples_per_chain\": " << kTotalSamples << ",\n"
             << "  \"pre_samples\": " << kPreSamples << ",\n"
             << "  \"post_samples\": " << kPostSamples << ",\n"
             << "  \"trigger_sample_index\": " << regs.read(kTriggerSample) << ",\n"
             << "  \"dropped_triggers\": " << regs.read(kDroppedTriggers) << ",\n"
             << "  \"events\": [";
        for (unsigned i = 0; i < events && i < 8; ++i) {
            const std::uint32_t word = regs.read(kEventBase + 4*i);
            if (i) meta << ',';
            meta << "{\"source\":" << ((word >> 18) & 3u)
                 << ",\"rx_mask\":" << ((word >> 16) & 3u)
                 << ",\"sample_offset\":" << (word & 0xffffu) << '}';
        }
        meta << "]\n}\n";
        std::cout << "Saved " << stem << " (" << events << " trigger event(s), status=0x"
                  << std::hex << regs.read(kCaptureStatus) << std::dec << ")\n";
        // This follows a full fixed-size IIO refill, so the CPU has consumed
        // this capture. The FPGA accepts ACK in any completed state; its DMAC
        // may stop requesting words before the replay FSM reaches ST_READY.
        regs.write(kCaptureControl, 5u); // enable + capture acknowledgement toggle
        const auto ack_deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(100);
        std::uint32_t capture_status = regs.read(kCaptureStatus);
        while ((capture_status & kCaptureComplete) != 0 &&
               std::chrono::steady_clock::now() < ack_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            capture_status = regs.read(kCaptureStatus);
        }
        if ((capture_status & kCaptureComplete) != 0) {
            std::cerr << "FPGA did not acknowledge capture (status=0x"
                      << std::hex << capture_status << std::dec << ")\n";
            break;
        }
        if (o.software_interval_ms != 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(o.software_interval_ms));
    }

    regs.write(kCaptureControl, 0u); // disable capture, no trigger/ack pulse
    regs.write(kDetectorControl, old_detector_enable);
    iio_buffer_destroy(buffer);
    for (unsigned i = 0; i < 4; ++i) iio_channel_disable(channels[i]);
    iio_context_destroy(ctx);
    return g_stop ? 0 : (captures ? 0 : 1);
}
