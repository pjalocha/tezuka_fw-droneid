#include <iio.h>
#include "chirp_spectrum_detector.h"
#include "droneid_spectral_cfo.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace {
constexpr std::uint64_t kDefaultBase = 0x7c460000;
constexpr std::size_t kMapSize = 0x1000;
constexpr std::uint32_t kDetectorControl = 0x3c;
constexpr std::uint32_t kTriggerCount0 = 0x38;
constexpr std::uint32_t kTriggerCount1 = 0x40;
constexpr std::uint32_t kCaptureControl = 0x44;
constexpr std::uint32_t kCaptureStatus = 0x48;
constexpr std::uint32_t kTriggerSample = 0x4c;
constexpr std::uint32_t kDroppedTriggers = 0x50;
constexpr std::uint32_t kEventCount = 0x54;
constexpr std::uint32_t kEventBase = 0x80;
constexpr std::uint32_t kCaptureComplete = 1u << 2;
constexpr std::uint32_t kPretriggerReady = 1u << 3;
constexpr std::size_t kFftSize = 64;
constexpr std::size_t kFftHop = 32;
constexpr std::size_t kPreSamples = 15360;
constexpr std::size_t kPostSamples = 15360;
constexpr std::size_t kCaptureSamples = kPreSamples + kPostSamples;
constexpr std::size_t kHistoryColumns = 1024;
constexpr std::uint32_t kRate = 15360000;
constexpr std::uint32_t kBandwidth = 10000000;
constexpr double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t g_stop = 0;
void StopHandler(int) { g_stop = 1; }

struct Options {
    std::vector<std::uint64_t> frequencies;
    unsigned dwell_ms = 250;
    unsigned settle_ms = 20;
    unsigned http_port = 8080;
    double gain_db = 30.0;
    std::uint64_t register_base = kDefaultBase;
    std::string mem_device = "/dev/mem";
    std::string save_triggers_dir;
};

bool ParseUnsigned(const char *s, std::uint64_t &value) {
    char *end = nullptr;
    errno = 0;
    const unsigned long long n = std::strtoull(s, &end, 0);
    if (errno || !end || *end) return false;
    value = static_cast<std::uint64_t>(n);
    return true;
}

bool ParseFrequencies(const std::string &text, std::vector<std::uint64_t> &out) {
    std::istringstream input(text);
    std::string token;
    std::vector<std::uint64_t> values;
    while (std::getline(input, token, ',')) {
        std::uint64_t hz = 0;
        if (!ParseUnsigned(token.c_str(), hz) || hz < 2400000000ULL ||
            hz > 2500000000ULL) return false;
        if (std::find(values.begin(), values.end(), hz) == values.end())
            values.push_back(hz);
    }
    if (values.empty()) return false;
    out.swap(values);
    return true;
}

void Usage(const char *program) {
    std::cerr << "Usage: " << program
              << " [--hops HZ[,HZ...]] [--dwell-ms N] [--settle-ms N]"
                 " [--gain-db DB] [--http PORT]"
                 " [--save-triggers DIR] [--mem /dev/mem] [--phys ADDRESS]\n"
              << "Default 2.4 GHz channels: 2399.5 + N*15 MHz (N=0..5); "
                 "15.36 Ms/s, 10 MHz BW, 1 ms pre/post capture.\n";
}

bool ParseArgs(int argc, char **argv, Options &o) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h") { Usage(argv[0]); std::exit(0); }
        if (i + 1 >= argc) return false;
        const char *value = argv[++i];
        std::uint64_t n = 0;
        if (arg == "--hops") {
            if (!ParseFrequencies(value, o.frequencies)) return false;
        } else if (arg == "--gain-db") {
            char *end = nullptr;
            errno = 0;
            o.gain_db = std::strtod(value, &end);
            if (errno || !end || *end) return false;
        } else if (arg == "--mem") {
            o.mem_device = value;
        } else if (arg == "--save-triggers") {
            o.save_triggers_dir = value;
        } else if (arg == "--dwell-ms" || arg == "--settle-ms" ||
                   arg == "--http" || arg == "--phys") {
            if (!ParseUnsigned(value, n)) return false;
            if (arg == "--dwell-ms") o.dwell_ms = static_cast<unsigned>(n);
            else if (arg == "--settle-ms") o.settle_ms = static_cast<unsigned>(n);
            else if (arg == "--http") o.http_port = static_cast<unsigned>(n);
            else o.register_base = n;
        } else return false;
    }
    if (o.frequencies.empty())
        for (std::uint64_t hz = 2399500000ULL; hz <= 2474500000ULL; hz += 15000000ULL)
            o.frequencies.push_back(hz);
    return o.dwell_ms >= 25 && o.dwell_ms <= 60000 &&
           o.settle_ms <= 5000 && o.http_port > 0 && o.http_port <= 65535 &&
           o.gain_db >= -3.0 && o.gain_db <= 71.0;
}

class Registers {
public:
    bool Open(const Options &o) {
        fd_ = ::open(o.mem_device.c_str(), O_RDWR | O_SYNC);
        if (fd_ < 0) { std::cerr << "open " << o.mem_device << ": " << std::strerror(errno) << '\n'; return false; }
        map_ = mmap(nullptr, kMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd_, static_cast<off_t>(o.register_base));
        if (map_ == MAP_FAILED) { map_ = nullptr; std::cerr << "mmap: " << std::strerror(errno) << '\n'; return false; }
        regs_ = static_cast<volatile std::uint32_t *>(map_);
        return true;
    }
    ~Registers() { if (map_) munmap(map_, kMapSize); if (fd_ >= 0) close(fd_); }
    std::uint32_t Read(std::uint32_t offset) const { return regs_[offset / 4]; }
    void Write(std::uint32_t offset, std::uint32_t value) { regs_[offset / 4] = value; }
private:
    int fd_ = -1;
    void *map_ = nullptr;
    volatile std::uint32_t *regs_ = nullptr;
};

struct ChainHistory {
    double noise_dbfs = -120.0;
    bool have_noise = false;
    std::array<std::uint8_t, kHistoryColumns * kFftSize> pixels{};
    std::size_t write_column = 0;
    std::size_t valid_columns = 0;
};

struct Channel {
    explicit Channel(std::uint64_t hz) : frequency_hz(hz) {}
    std::uint64_t frequency_hz;
    std::uint64_t hardware_triggers_rx0 = 0;
    std::uint64_t hardware_triggers_rx1 = 0;
    std::uint64_t probes = 0;
    std::uint64_t captures = 0;
    std::uint64_t saved_candidates = 0;
    std::array<ChainHistory, 2> rx;
};

struct TemperatureReading { std::string label; double celsius = 0.0; };

bool StartsWith(const std::string &text, const std::string &prefix) {
    return text.compare(0, prefix.size(), prefix) == 0;
}
bool EndsWith(const std::string &text, const std::string &suffix) {
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}
std::vector<std::string> ListDir(const std::string &path) {
    std::vector<std::string> result;
    DIR *dir = opendir(path.c_str());
    if (!dir) return result;
    while (dirent *entry = readdir(dir))
        if (entry->d_name[0] != '.') result.push_back(entry->d_name);
    closedir(dir);
    return result;
}
bool ReadText(const std::string &path, std::string &value) {
    std::ifstream in(path.c_str());
    if (!in) return false;
    std::getline(in, value);
    return !value.empty();
}
bool ReadNumber(const std::string &path, double &value) {
    std::ifstream in(path.c_str());
    return in && (in >> value) && std::isfinite(value);
}
double MilliOrCelsius(double value) { return std::abs(value) > 200.0 ? value / 1000.0 : value; }
void AddTemperature(std::vector<TemperatureReading> &out,
                    const std::string &label, double celsius) {
    if (out.size() < 16 && std::isfinite(celsius) && celsius >= -80.0 && celsius <= 180.0)
        out.push_back({label, celsius});
}
std::vector<TemperatureReading> ReadTemperatures() {
    std::vector<TemperatureReading> out;
    for (const std::string &zone : ListDir("/sys/class/thermal")) {
        if (!StartsWith(zone, "thermal_zone")) continue;
        const std::string dir = "/sys/class/thermal/" + zone;
        std::string type = zone; ReadText(dir + "/type", type);
        double temp = 0;
        if (ReadNumber(dir + "/temp", temp)) AddTemperature(out, "thermal:" + type, MilliOrCelsius(temp));
    }
    for (const std::string &hwmon : ListDir("/sys/class/hwmon")) {
        const std::string dir = "/sys/class/hwmon/" + hwmon;
        std::string name = hwmon; ReadText(dir + "/name", name);
        for (const std::string &file : ListDir(dir)) {
            if (!StartsWith(file, "temp") || !EndsWith(file, "_input")) continue;
            const std::string id = file.substr(0, file.size() - 6);
            std::string label = id; ReadText(dir + "/" + id + "_label", label);
            double temp = 0;
            if (ReadNumber(dir + "/" + file, temp)) AddTemperature(out, "hwmon:" + name + ":" + label, MilliOrCelsius(temp));
        }
    }
    for (const std::string &dev : ListDir("/sys/bus/iio/devices")) {
        if (!StartsWith(dev, "iio:device")) continue;
        const std::string dir = "/sys/bus/iio/devices/" + dev;
        std::string name = dev; ReadText(dir + "/name", name);
        for (const std::string &file : ListDir(dir)) {
            if (!StartsWith(file, "in_temp")) continue;
            if (EndsWith(file, "_input")) {
                const std::string id = file.substr(0, file.size() - 6);
                std::string label = id; ReadText(dir + "/" + id + "_label", label);
                double temp = 0;
                if (ReadNumber(dir + "/" + file, temp)) AddTemperature(out, "iio:" + name + ":" + label, MilliOrCelsius(temp));
            } else if (EndsWith(file, "_raw")) {
                const std::string id = file.substr(0, file.size() - 4);
                std::string label = id; ReadText(dir + "/" + id + "_label", label);
                double raw = 0, scale = 0, offset = 0;
                if (ReadNumber(dir + "/" + file, raw) && ReadNumber(dir + "/" + id + "_scale", scale)) {
                    ReadNumber(dir + "/" + id + "_offset", offset);
                    AddTemperature(out, "iio:" + name + ":" + label, MilliOrCelsius((raw + offset) * scale));
                }
            }
        }
    }
    return out;
}

class Shared {
public:
    explicit Shared(const std::vector<std::uint64_t> &freqs) {
        for (std::uint64_t f : freqs) channels.emplace_back(f);
    }
    std::mutex mutex;
    std::vector<Channel> channels;
    std::size_t current = 0;
    std::uint64_t sample_rate = kRate;
    double gain_db = 0;
    std::atomic<double> requested_gain_db{30.0};
    unsigned dwell_ms = 0;
    std::atomic<std::uint64_t> software_probes{0};
    std::atomic<std::uint64_t> hardware_events{0};
    std::atomic<std::uint64_t> dropped_software_probes{0};
    std::atomic<std::uint64_t> candidate_queue_waits{0};
    std::atomic<std::uint64_t> candidate_save_failures{0};
    std::atomic<std::uint64_t> fpga_dropped_triggers{0};
    std::atomic<unsigned> candidate_queue_depth{0};
    std::atomic<unsigned> candidate_queue_high_water{0};
    std::atomic<bool> running{true};
};

std::string StatusJson(Shared &s) {
    const std::vector<TemperatureReading> temperatures = ReadTemperatures();
    std::lock_guard<std::mutex> lock(s.mutex);
    std::ostringstream out;
    out << "{\"running\":" << (s.running.load() ? "true" : "false")
        << ",\"sample_rate_hz\":" << s.sample_rate
        << ",\"gain_db\":" << std::fixed << std::setprecision(1) << s.gain_db
        << ",\"requested_gain_db\":" << s.requested_gain_db.load()
        << ",\"dwell_ms\":" << s.dwell_ms
        << ",\"current_frequency_hz\":"
        << (s.channels.empty() ? 0 : s.channels[s.current].frequency_hz)
        << ",\"software_probes_total\":" << s.software_probes.load()
        << ",\"hardware_events_total\":" << s.hardware_events.load()
        << ",\"fpga_dropped_triggers_total\":" << s.fpga_dropped_triggers.load()
        << ",\"candidate_queue_depth\":" << s.candidate_queue_depth.load()
        << ",\"candidate_queue_high_water\":" << s.candidate_queue_high_water.load()
        << ",\"candidate_queue_waits\":" << s.candidate_queue_waits.load()
        << ",\"candidate_save_failures\":" << s.candidate_save_failures.load()
        << ",\"dropped_software_probes\":" << s.dropped_software_probes.load()
        << ",\"temperatures\":[";
    for (std::size_t i = 0; i < temperatures.size(); ++i) {
        if (i) out << ',';
        out << "{\"label\":\"" << temperatures[i].label
            << "\",\"celsius\":" << std::fixed << std::setprecision(1)
            << temperatures[i].celsius << '}';
    }
    out << "],\"channels\":[";
    for (std::size_t i = 0; i < s.channels.size(); ++i) {
        const Channel &c = s.channels[i];
        if (i) out << ',';
        out << "{\"frequency_hz\":" << c.frequency_hz << ",\"rx0_noise_dbfs_per_bin\":";
        if (c.rx[0].have_noise) out << std::fixed << std::setprecision(2) << c.rx[0].noise_dbfs;
        else out << "null";
        out << ",\"rx1_noise_dbfs_per_bin\":";
        if (c.rx[1].have_noise) out << std::fixed << std::setprecision(2) << c.rx[1].noise_dbfs;
        else out << "null";
        out << ",\"detector_triggers_rx0\":" << c.hardware_triggers_rx0
            << ",\"detector_triggers_rx1\":" << c.hardware_triggers_rx1
            << ",\"software_probes\":" << c.probes
            << ",\"captures\":" << c.captures
            << ",\"saved_candidates\":" << c.saved_candidates << '}';
    }
    out << "]}";
    return out.str();
}

void SendAll(int fd, const void *data, std::size_t size) {
    const char *p = static_cast<const char *>(data);
    while (size) {
        const ssize_t n = send(fd, p, size, MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) break;
        p += n; size -= static_cast<std::size_t>(n);
    }
}

void Reply(int fd, const char *type, const std::string &body) {
    std::ostringstream head;
    head << "HTTP/1.1 200 OK\r\nContent-Type: " << type
         << "\r\nContent-Length: " << body.size()
         << "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
    const std::string h = head.str();
    SendAll(fd, h.data(), h.size()); SendAll(fd, body.data(), body.size());
}

void Reply404(int fd) {
    static const char response[] = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    SendAll(fd, response, sizeof(response) - 1);
}

void Reply400(int fd, const char *message) {
    const std::string body = std::string("{\"error\":\"") + message + "\"}";
    std::ostringstream head;
    head << "HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nContent-Length: "
         << body.size() << "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
    const std::string h = head.str();
    SendAll(fd, h.data(), h.size()); SendAll(fd, body.data(), body.size());
}

void PutLE16(std::string &s, std::uint16_t x) { s.push_back(static_cast<char>(x)); s.push_back(static_cast<char>(x >> 8)); }
void PutLE32(std::string &s, std::uint32_t x) { for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>(x >> (8 * i))); }

std::string MakeBmp(Shared &shared, int selected, unsigned rx) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    if (shared.channels.empty()) return {};
    const std::size_t count = selected >= 0 ? 1 : shared.channels.size();
    const std::uint32_t width = static_cast<std::uint32_t>(kHistoryColumns);
    const std::uint32_t height = static_cast<std::uint32_t>(count * kFftSize);
    const std::uint32_t row_bytes = (width + 3u) & ~3u;
    const std::uint32_t offset = 14 + 40 + 256 * 4;
    const std::uint32_t image_bytes = row_bytes * height;
    std::string bmp;
    bmp.reserve(offset + image_bytes);
    bmp.append("BM", 2); PutLE32(bmp, offset + image_bytes); PutLE16(bmp, 0); PutLE16(bmp, 0); PutLE32(bmp, offset);
    PutLE32(bmp, 40); PutLE32(bmp, width); PutLE32(bmp, height); PutLE16(bmp, 1); PutLE16(bmp, 8);
    PutLE32(bmp, 0); PutLE32(bmp, image_bytes); PutLE32(bmp, 2835); PutLE32(bmp, 2835); PutLE32(bmp, 256); PutLE32(bmp, 256);
    for (unsigned i = 0; i < 256; ++i) { bmp.push_back(static_cast<char>(i)); bmp.push_back(static_cast<char>(i)); bmp.push_back(static_cast<char>(i)); bmp.push_back(0); }
    bmp.resize(offset, 0);
    for (std::size_t out_y = 0; out_y < height; ++out_y) {
        const std::size_t source_y = height - 1 - out_y;
        const std::size_t channel_slot = source_y / kFftSize;
        const std::size_t bin = source_y % kFftSize;
        const Channel &c = shared.channels[selected >= 0 ? static_cast<std::size_t>(selected) : channel_slot];
        const ChainHistory &history = c.rx[rx];
        for (std::size_t x = 0; x < width; ++x) {
            const std::size_t source_col = (history.write_column + kHistoryColumns - history.valid_columns + x) % kHistoryColumns;
            const std::uint8_t px = x < kHistoryColumns - history.valid_columns ? 0 : history.pixels[source_col * kFftSize + (kFftSize - 1 - bin)];
            bmp.push_back(static_cast<char>(px));
        }
        while ((bmp.size() - offset) % row_bytes) bmp.push_back(0);
    }
    return bmp;
}

std::string MakeCombinedBmp(Shared &shared) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    if (shared.channels.empty()) return {};
    constexpr std::uint32_t label_width = 96;
    constexpr std::uint32_t panel_width = kHistoryColumns / 2;
    constexpr std::uint32_t panel_gap = 2;
    constexpr std::uint32_t panel_height = kFftSize + 1;
    const std::uint32_t width = label_width + 2 * panel_width + panel_gap;
    const std::uint32_t height = panel_height * static_cast<std::uint32_t>(shared.channels.size());
    const std::uint32_t row_bytes = (width + 3u) & ~3u;
    const std::uint32_t offset = 14 + 40 + 256 * 4;
    const std::uint32_t image_bytes = row_bytes * height;
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height, 0);
    static const std::uint8_t font[11][5] = {
        {7,5,5,5,7}, {2,6,2,2,7}, {7,1,7,4,7}, {7,1,7,1,7},
        {5,5,7,1,1}, {7,4,7,1,7}, {7,4,7,5,7}, {7,1,1,1,1},
        {7,5,7,5,7}, {7,5,7,1,7}, {0,0,0,0,2}
    };
    for (std::size_t channel = 0; channel < shared.channels.size(); ++channel) {
        const Channel &c = shared.channels[channel];
        const std::uint32_t y_base = static_cast<std::uint32_t>(channel) * panel_height;
        std::ostringstream label;
        label << std::fixed << std::setprecision(2) << c.frequency_hz / 1.0e6;
        const std::string text = label.str();
        const std::uint32_t scale = 3;
        std::uint32_t x_text = label_width > text.size() * 4 * scale
            ? label_width - static_cast<std::uint32_t>(text.size() * 4 * scale) : 0;
        const std::uint32_t y_text = y_base + (kFftSize - 5 * scale) / 2;
        for (char ch : text) {
            const unsigned glyph = ch == '.' ? 10u :
                (ch >= '0' && ch <= '9' ? static_cast<unsigned>(ch - '0') : 10u);
            for (std::uint32_t gy = 0; gy < 5; ++gy) {
                for (std::uint32_t gx = 0; gx < 3; ++gx) {
                    if (!(font[glyph][gy] & (1u << (2 - gx)))) continue;
                    for (std::uint32_t sy = 0; sy < scale; ++sy)
                        for (std::uint32_t sx = 0; sx < scale; ++sx)
                            pixels[static_cast<std::size_t>(y_text + gy * scale + sy) * width +
                                   x_text + gx * scale + sx] = 255;
                }
            }
            x_text += 4 * scale;
        }
        for (unsigned rx = 0; rx < 2; ++rx) {
            const ChainHistory &history = c.rx[rx];
            const std::uint32_t x_base = label_width + rx * (panel_width + panel_gap);
            const std::size_t first_valid = kHistoryColumns - history.valid_columns;
            for (std::uint32_t x = 0; x < panel_width; ++x) {
                for (std::uint32_t y = 0; y < kFftSize; ++y) {
                    const std::size_t bin = kFftSize - 1 - y;
                    std::uint8_t value = 0;
                    for (std::size_t t = 0; t < 2; ++t) {
                        const std::size_t logical = 2 * x + t;
                        if (logical < first_valid) continue;
                        const std::size_t source = (history.write_column + logical) % kHistoryColumns;
                        value = std::max(value, history.pixels[source * kFftSize + bin]);
                    }
                    pixels[static_cast<std::size_t>(y_base + y) * width + x_base + x] = value;
                }
            }
        }
        if (channel + 1 < shared.channels.size()) {
            const std::size_t separator = static_cast<std::size_t>(y_base + kFftSize) * width;
            std::fill(pixels.begin() + separator, pixels.begin() + separator + width, 64);
        }
    }

    std::string bmp;
    bmp.reserve(offset + image_bytes);
    bmp.append("BM", 2); PutLE32(bmp, offset + image_bytes); PutLE16(bmp, 0); PutLE16(bmp, 0); PutLE32(bmp, offset);
    PutLE32(bmp, 40); PutLE32(bmp, width); PutLE32(bmp, height); PutLE16(bmp, 1); PutLE16(bmp, 8);
    PutLE32(bmp, 0); PutLE32(bmp, image_bytes); PutLE32(bmp, 2835); PutLE32(bmp, 2835); PutLE32(bmp, 256); PutLE32(bmp, 256);
    for (unsigned i = 0; i < 256; ++i) { bmp.push_back(static_cast<char>(i)); bmp.push_back(static_cast<char>(i)); bmp.push_back(static_cast<char>(i)); bmp.push_back(0); }
    bmp.resize(offset, 0);
    for (std::uint32_t file_y = 0; file_y < height; ++file_y) {
        const std::uint32_t image_y = height - 1 - file_y;
        const std::size_t row = static_cast<std::size_t>(image_y) * width;
        for (std::uint32_t x = 0; x < width; ++x) bmp.push_back(static_cast<char>(pixels[row + x]));
        while ((bmp.size() - offset) % row_bytes) bmp.push_back(0);
    }
    return bmp;
}

std::string ChannelSpectrumPage(std::uint64_t frequency_hz) {
    std::ostringstream out;
    out << "<!doctype html><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width\">"
        << "<title>DroneID spectrum comparison</title><style>body{background:#111;color:#ddd;font:14px sans-serif;margin:1.5em}"
        << "a{color:#7db7ff}.pair{display:flex;gap:1em;flex-wrap:wrap}.pair figure{margin:0;flex:1 1 480px}"
        << ".pair img{width:100%;height:256px;object-fit:fill;image-rendering:pixelated;background:#000}</style><h1>"
        << std::fixed << std::setprecision(3) << frequency_hz / 1.0e6 << " MHz</h1><div class=\"pair\">";
    for (unsigned rx = 0; rx < 2; ++rx) {
        out << "<figure><figcaption>RX" << rx << "</figcaption><img src=\"/spectrogram.bmp?frequency_hz="
            << frequency_hz << "&amp;rx=" << rx << "\"></figure>";
    }
    out << "</div><p><a href=\"/\">Back to channel scan</a></p>";
    return out.str();
}

const char kPage[] = R"HTML(<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Pluto DroneID channel scanner</title>
<style>body{background:#111;color:#ddd;font:14px sans-serif;margin:1.5em}a{color:#7db7ff}table{border-collapse:collapse;margin:1em 0}th,td{border:1px solid #555;padding:.4em .7em;text-align:right}th:first-child,td:first-child{text-align:left}img{width:min(100%,1024px);image-rendering:pixelated;background:#000}small{color:#aaa}input,button{font:inherit;padding:.35em;background:#222;color:#ddd;border:1px solid #777;border-radius:3px}button{cursor:pointer}</style>
<h1>Pluto DroneID channel scanner</h1><p id="state">Starting…</p><section><label for="gain">RX gain, both chains [dB]:</label> <input id="gain" type="number" min="-3" max="71" step="1" value="30"> <button id="set-gain">Apply</button> <span id="gain-result">Gain changes apply at the next channel hop.</span></section><p>Noise is the median 64-point FFT-bin power across all bins and FFT slices in each 2 ms probe, in dBFS/bin. Spectrogram brightness uses that same capture-wide reference. FPGA detector counters count hardware trigger events; software probes are not included.</p>
<table><thead><tr><th>Channel [MHz]</th><th>RX0 noise [dBFS/bin]</th><th>RX1 noise [dBFS/bin]</th><th>FPGA triggers RX0 / RX1</th><th>Saved candidates</th><th>Spectrum</th></tr></thead><tbody id="channels"></tbody></table>
<h2>Combined channel spectrogram</h2><small>Each row is labeled with frequency; RX0 is on the left and RX1 on the right. Time runs left-to-right.</small><br><img id="all" src="/spectrogram-all.bmp">
<script>document.querySelector('#set-gain').addEventListener('click',async()=>{const input=document.querySelector('#gain'),out=document.querySelector('#gain-result'),gain=Number(input.value);if(!Number.isFinite(gain)||gain < -3||gain > 71){out.textContent='Enter a gain from -3 to 71 dB.';return}out.textContent='Applying at next channel hop…';try{const r=await fetch('/gain?db='+encodeURIComponent(gain),{method:'POST'}),v=await r.json();if(!r.ok)throw new Error(v.error||r.statusText);out.textContent='Requested '+v.requested_gain_db.toFixed(1)+' dB; applies at next hop.'}catch(e){out.textContent='Gain request failed: '+e}});async function refresh(){try{const r=await fetch('/status.json',{cache:'no-store'}),s=await r.json();const temps=(s.temperatures||[]).map(t=>t.label+': '+t.celsius.toFixed(1)+' °C').join(' · ');const pending=Math.abs(s.requested_gain_db-s.gain_db)>0.05?` · ${s.requested_gain_db.toFixed(1)} dB pending`:'';document.querySelector('#state').textContent=`${s.running?'Scanning':'Stopped'} · ${Number(s.current_frequency_hz/1e6).toFixed(3)} MHz current · ${s.sample_rate_hz/1e6} MS/s · RX gain ${s.gain_db.toFixed(1)} dB${pending}${temps?' · '+temps:' · temperature sensors unavailable'}`;if(document.activeElement!==document.querySelector('#gain'))document.querySelector('#gain').value=s.requested_gain_db.toFixed(1);
const body=document.querySelector('#channels');body.replaceChildren();for(const c of s.channels){const tr=document.createElement('tr');for(const value of [Number(c.frequency_hz/1e6).toFixed(3),c.rx0_noise_dbfs_per_bin===null?'--':c.rx0_noise_dbfs_per_bin.toFixed(2),c.rx1_noise_dbfs_per_bin===null?'--':c.rx1_noise_dbfs_per_bin.toFixed(2),c.detector_triggers_rx0+' / '+c.detector_triggers_rx1,String(c.saved_candidates||0)]){const td=document.createElement('td');td.textContent=value;tr.appendChild(td)}const td=document.createElement('td'),a=document.createElement('a');a.href='/spectrum.html?frequency_hz='+c.frequency_hz;a.textContent='view RX0 + RX1';td.appendChild(a);tr.appendChild(td);body.appendChild(tr)}document.querySelector('#all').src='/spectrogram-all.bmp?t='+Date.now()}catch(e){document.querySelector('#state').textContent='HTTP status unavailable: '+e}setTimeout(refresh,1000)}refresh();</script>)HTML";

int StartHttp(unsigned port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int yes = 1; setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_ANY); addr.sin_port = htons(static_cast<std::uint16_t>(port));
    if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 || listen(fd, 8) < 0) { std::cerr << "HTTP bind/listen: " << std::strerror(errno) << '\n'; close(fd); return -1; }
    return fd;
}

void HttpLoop(int server, Shared &shared) {
    while (shared.running.load()) {
        fd_set set; FD_ZERO(&set); FD_SET(server, &set); timeval timeout{0, 250000};
        if (select(server + 1, &set, nullptr, nullptr, &timeout) <= 0) continue;
        int client = accept(server, nullptr, nullptr); if (client < 0) continue;
        timeval io_timeout{1, 0}; setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &io_timeout, sizeof(io_timeout));
        char request[2048] = {}; const ssize_t n = recv(client, request, sizeof(request) - 1, 0);
        if (n <= 0) { close(client); continue; }
        std::istringstream line(std::string(request, static_cast<std::size_t>(n)));
        std::string method, target, version; line >> method >> target >> version;
        const std::size_t q = target.find('?'); const std::string route = target.substr(0, q);
        if (method == "POST" && route == "/gain" && q != std::string::npos) {
            const std::string query = target.substr(q + 1);
            const std::string key = "db=";
            const std::size_t pos = query.find(key);
            if (pos == std::string::npos) {
                Reply400(client, "missing db parameter");
            } else {
                const std::size_t begin = pos + key.size(), end = query.find('&', begin);
                const std::string value = query.substr(begin, end == std::string::npos ? end : end - begin);
                char *parsed_end = nullptr;
                errno = 0;
                const double gain_db = std::strtod(value.c_str(), &parsed_end);
                if (errno || !parsed_end || parsed_end == value.c_str() || *parsed_end || !std::isfinite(gain_db) ||
                    gain_db < -3.0 || gain_db > 71.0) {
                    Reply400(client, "gain must be between -3 and 71 dB");
                } else {
                    shared.requested_gain_db.store(gain_db);
                    std::ostringstream response;
                    response << "{\"accepted\":true,\"requested_gain_db\":"
                             << std::fixed << std::setprecision(1) << gain_db
                             << ",\"applies_at_next_hop\":true}";
                    Reply(client, "application/json", response.str());
                }
            }
        } else if (method != "GET") Reply404(client);
        else if (route == "/") Reply(client, "text/html; charset=utf-8", kPage);
        else if (route == "/status.json") Reply(client, "application/json", StatusJson(shared));
        else if (route == "/spectrogram-all.bmp") {
            const std::string bmp = MakeCombinedBmp(shared); if (bmp.empty()) Reply404(client); else Reply(client, "image/bmp", bmp);
        } else if (route == "/spectrum.html" && q != std::string::npos) {
            const std::string query = target.substr(q + 1); const std::string key = "frequency_hz=";
            const std::size_t pos = query.find(key); std::uint64_t hz = 0; bool found = false;
            if (pos != std::string::npos) {
                const std::size_t begin = pos + key.size(), end = query.find('&', begin);
                const std::string value = query.substr(begin, end == std::string::npos ? end : end - begin);
                if (ParseUnsigned(value.c_str(), hz)) {
                    std::lock_guard<std::mutex> lock(shared.mutex);
                    for (const Channel &channel : shared.channels) if (channel.frequency_hz == hz) found = true;
                }
            }
            if (found) Reply(client, "text/html; charset=utf-8", ChannelSpectrumPage(hz)); else Reply404(client);
        } else if (route == "/spectrogram.bmp" && q != std::string::npos) {
            const std::string query = target.substr(q + 1);
            auto query_value = [&query](const std::string &key) {
                const std::size_t pos = query.find(key + "=");
                if (pos == std::string::npos) return std::string();
                const std::size_t begin = pos + key.size() + 1;
                const std::size_t end = query.find('&', begin);
                return query.substr(begin, end == std::string::npos ? end : end - begin);
            };
            const std::string frequency = query_value("frequency_hz");
            const std::string rx_text = query_value("rx");
            std::uint64_t hz = 0, rx_value = 0; int index = -1;
            if (!frequency.empty() && ParseUnsigned(frequency.c_str(), hz) &&
                (rx_text.empty() || (ParseUnsigned(rx_text.c_str(), rx_value) && rx_value <= 1))) {
                std::lock_guard<std::mutex> lock(shared.mutex);
                for (std::size_t i = 0; i < shared.channels.size(); ++i) if (shared.channels[i].frequency_hz == hz) index = static_cast<int>(i);
            }
            if (index < 0) Reply404(client);
            else { const std::string bmp = MakeBmp(shared, index, static_cast<unsigned>(rx_value)); Reply(client, "image/bmp", bmp); }
        } else Reply404(client);
        shutdown(client, SHUT_RDWR); close(client);
    }
}

bool ConfigureRx(iio_context *ctx, const Options &o, iio_device *&rx,
                 iio_channel *&rx_lo, iio_channel *&rx_cfg0,
                 iio_channel *&rx_cfg1, std::array<iio_channel *, 4> &channels,
                 iio_buffer *&buffer) {
    iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    rx = iio_context_find_device(ctx, "cf-ad9361-lpc");
    rx_cfg0 = phy ? iio_device_find_channel(phy, "voltage0", false) : nullptr;
    rx_cfg1 = phy ? iio_device_find_channel(phy, "voltage1", false) : nullptr;
    rx_lo = phy ? iio_device_find_channel(phy, "altvoltage0", true) : nullptr;
    if (!phy || !rx || !rx_cfg0 || !rx_cfg1 || !rx_lo) { std::cerr << "Missing ad9361-phy / cf-ad9361-lpc RX IIO channels\n"; return false; }
    int ret = iio_channel_attr_write_longlong(rx_cfg0, "sampling_frequency", kRate);
    if (ret >= 0) ret = iio_channel_attr_write_longlong(rx_cfg0, "rf_bandwidth", kBandwidth);
    if (ret >= 0) ret = iio_channel_attr_write(rx_lo, "frequency", "2399500000");
    if (ret >= 0) ret = iio_channel_attr_write(rx_cfg0, "gain_control_mode", "manual");
    if (ret >= 0) ret = iio_channel_attr_write_double(rx_cfg0, "hardwaregain", o.gain_db);
    if (ret >= 0) ret = iio_channel_attr_write(rx_cfg1, "gain_control_mode", "manual");
    if (ret >= 0) ret = iio_channel_attr_write_double(rx_cfg1, "hardwaregain", o.gain_db);
    if (ret < 0) { char err[128] = {}; iio_strerror(-ret, err, sizeof(err)); std::cerr << "Configure RX: " << err << '\n'; return false; }
    for (unsigned i = 0; i < channels.size(); ++i) {
        char id[16]; std::snprintf(id, sizeof(id), "voltage%u", i);
        channels[i] = iio_device_find_channel(rx, id, false);
        if (!channels[i]) { std::cerr << "Missing RX IIO scan channel " << id << '\n'; return false; }
        iio_channel_enable(channels[i]);
    }
    iio_device_set_kernel_buffers_count(rx, 1);
    buffer = iio_device_create_buffer(rx, kCaptureSamples, false);
    if (!buffer) { std::cerr << "Cannot create two-millisecond RX buffer\n"; return false; }
    if (iio_buffer_step(buffer) < 8) { std::cerr << "Unexpected RX scan layout step=" << iio_buffer_step(buffer) << '\n'; return false; }
    return true;
}

bool ReadCapture(iio_buffer *buffer, const std::array<iio_channel *, 4> &channels,
                 std::array<std::vector<std::int16_t>, 2> &iq) {
    const ptrdiff_t step = iio_buffer_step(buffer);
    const ssize_t bytes = iio_buffer_refill(buffer);
    const std::size_t expected = kCaptureSamples * static_cast<std::size_t>(step);
    if (bytes < 0 || static_cast<std::size_t>(bytes) != expected) {
        char err[128] = {}; if (bytes < 0) iio_strerror(static_cast<int>(-bytes), err, sizeof(err));
        std::cerr << "IIO capture returned " << bytes << " bytes (expected " << expected << ')'
                  << (bytes < 0 ? std::string(": ") + err : "") << '\n'; return false;
    }
    char *end = static_cast<char *>(iio_buffer_end(buffer));
    if (!end) { std::cerr << "RX buffer has no end pointer after refill\n"; return false; }
    for (unsigned chain = 0; chain < 2; ++chain) {
        char *i_ptr = static_cast<char *>(iio_buffer_first(buffer, channels[chain * 2]));
        char *q_ptr = static_cast<char *>(iio_buffer_first(buffer, channels[chain * 2 + 1]));
        if (!i_ptr || !q_ptr || i_ptr >= end || q_ptr >= end) { std::cerr << "RX lane pointer invalid after refill\n"; return false; }
        iq[chain].resize(kCaptureSamples * 2);
        for (std::size_t n = 0; n < kCaptureSamples; ++n) {
            std::memcpy(&iq[chain][2*n], i_ptr + n * step, 2);
            std::memcpy(&iq[chain][2*n + 1], q_ptr + n * step, 2);
        }
    }
    return true;
}

using SpectrumPowerColumn = std::array<double, kFftSize>;

SpectrumPowerColumn SpectrumColumn(const std::vector<std::int16_t> &iq,
                                   std::size_t start) {
    std::array<double, kFftSize> re{}, im{};
    for (std::size_t n = 0; n < kFftSize; ++n) {
        const double window = 0.5 - 0.5 * std::cos(2.0 * kPi * n / (kFftSize - 1));
        re[n] = iq[2 * (start + n)] * window;
        im[n] = iq[2 * (start + n) + 1] * window;
    }
    for (std::size_t i = 1, j = 0; i < kFftSize; ++i) {
        std::size_t bit = kFftSize >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { std::swap(re[i], re[j]); std::swap(im[i], im[j]); }
    }
    for (std::size_t len = 2; len <= kFftSize; len <<= 1) {
        const double angle = -2.0 * kPi / len;
        const double wr0 = std::cos(angle), wi0 = std::sin(angle);
        for (std::size_t base = 0; base < kFftSize; base += len) {
            double wr = 1.0, wi = 0.0;
            for (std::size_t j = 0; j < len / 2; ++j) {
                const std::size_t a = base + j, b = a + len / 2;
                const double tr = wr * re[b] - wi * im[b], ti = wr * im[b] + wi * re[b];
                re[b] = re[a] - tr; im[b] = im[a] - ti; re[a] += tr; im[a] += ti;
                const double next_wr = wr * wr0 - wi * wi0; wi = wr * wi0 + wi * wr0; wr = next_wr;
            }
        }
    }
    SpectrumPowerColumn powers{};
    for (std::size_t k = 0; k < kFftSize; ++k) {
        const std::size_t shifted = (k + kFftSize / 2) % kFftSize;
        powers[shifted] = (re[k] * re[k] + im[k] * im[k]) / (kFftSize * kFftSize);
    }
    return powers;
}

struct ChirpMeasurement {
    ChirpDetection chirp;
    bool high_resolution_fit_valid = false;
    DroneIdSpectralCfoEstimate high_resolution;
};

struct ChainCandidateAnalysis {
    std::string error;
    std::vector<ChirpMeasurement> chirps;
};

std::array<std::complex<float>, kFftSize> CandidateFft(
    const std::vector<std::int16_t> &iq, std::size_t start) {
    std::array<std::complex<float>, kFftSize> bins{};
    for (std::size_t n = 0; n < kFftSize; ++n) {
        const double window = 0.5 - 0.5 * std::cos(2.0 * kPi * n / (kFftSize - 1));
        bins[n] = std::complex<float>(
            static_cast<float>(iq[2 * (start + n)] * window),
            static_cast<float>(iq[2 * (start + n) + 1] * window));
    }
    for (std::size_t i = 1, j = 0; i < kFftSize; ++i) {
        std::size_t bit = kFftSize >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(bins[i], bins[j]);
    }
    for (std::size_t length = 2; length <= kFftSize; length <<= 1) {
        const double angle = -2.0 * kPi / length;
        const std::complex<float> step(static_cast<float>(std::cos(angle)),
                                       static_cast<float>(std::sin(angle)));
        for (std::size_t base = 0; base < kFftSize; base += length) {
            std::complex<float> twiddle(1.0f, 0.0f);
            for (std::size_t j = 0; j < length / 2; ++j) {
                const std::size_t a = base + j, b = a + length / 2;
                const std::complex<float> even = bins[a];
                const std::complex<float> odd = bins[b] * twiddle;
                bins[a] = even + odd;
                bins[b] = even - odd;
                twiddle *= step;
            }
        }
    }
    return bins;
}

ChainCandidateAnalysis AnalyzeCandidateChain(const std::vector<std::int16_t> &iq,
                                             std::uint64_t center_frequency_hz) {
    ChainCandidateAnalysis result;
    ChirpSpectrumConfig config;
    config.sample_rate_hz = kRate;
    config.center_frequency_hz = static_cast<double>(center_frequency_hz);
    config.fft_size = kFftSize;
    config.slice_hop_samples = kFftHop;
    config.enable_o2 = false;
    ChirpSpectrumDetector detector;
    if (!detector.Setup(config, &result.error)) return result;

    for (std::size_t start = 0; start + kFftSize <= kCaptureSamples; start += kFftHop) {
        const std::array<std::complex<float>, kFftSize> bins = CandidateFft(iq, start);
        if (!detector.ProcessFftSlice(bins.data(), bins.size(), start, &result.error))
            return result;
    }
    detector.Finish();
    const std::vector<ChirpDetection> detections = detector.TakeDetections();
    std::vector<float> normalized_iq(iq.size());
    for (std::size_t i = 0; i < iq.size(); ++i)
        normalized_iq[i] = static_cast<float>(iq[i]) / 32768.0f;
    for (const ChirpDetection &detection : detections) {
        ChirpMeasurement measurement;
        measurement.chirp = detection;
        const long long first_signed = std::llround(detection.packet_start_time_s * kRate);
        const long long end_signed = static_cast<long long>(std::ceil(
            detection.packet_end_time_s * kRate));
        if (first_signed >= 0 && end_signed > first_signed &&
            static_cast<std::size_t>(end_signed) <= kCaptureSamples) {
            const std::size_t first = static_cast<std::size_t>(first_signed);
            const std::size_t end = static_cast<std::size_t>(end_signed);
            if (detection.packet_band_fit_valid) {
                const bool fitted = EstimateDroneIdSpectralCfo(
                    normalized_iq.data(), kCaptureSamples, first, end, kRate,
                    detection.packet_band_cfo_offset_hz,
                    measurement.high_resolution);
                measurement.high_resolution_fit_valid = fitted;
            }
        }
        result.chirps.push_back(measurement);
    }
    return result;
}

const char *SignatureName(ChirpSignature signature) {
    if (signature == ChirpSignature::DroneId) return "droneid";
    if (signature == ChirpSignature::O2) return "o2";
    return "unknown";
}

void WriteNumberOrNull(std::ostream &out, double value) {
    if (std::isfinite(value)) out << std::setprecision(9) << value;
    else out << "null";
}

void WriteChainAnalysis(std::ostream &out, const ChainCandidateAnalysis &analysis) {
    out << "{\"error\":\"" << analysis.error << "\",\"chirps\":[";
    for (std::size_t i = 0; i < analysis.chirps.size(); ++i) {
        if (i) out << ',';
        const ChirpMeasurement &m = analysis.chirps[i];
        const ChirpDetection &d = m.chirp;
        out << "{\"signature\":\"" << SignatureName(d.signature)
            << "\",\"start_us\":"; WriteNumberOrNull(out, d.start_time_s * 1.0e6);
        out << ",\"end_us\":"; WriteNumberOrNull(out, d.end_time_s * 1.0e6);
        out << ",\"duration_us\":"; WriteNumberOrNull(out, d.duration_us);
        out << ",\"slope_mhz_per_us\":"; WriteNumberOrNull(out, d.slope_mhz_per_us);
        out << ",\"sweep_width_mhz\":"; WriteNumberOrNull(out, d.sweep_width_mhz);
        out << ",\"ridge_snr_db\":";
        if (d.ridge_snr_valid) WriteNumberOrNull(out, d.ridge_snr_db); else out << "null";
        out << ",\"mean_peak_snr_db\":"; WriteNumberOrNull(out, d.mean_peak_snr_db);
        out << ",\"matched_slices\":" << d.matched_slices
            << ",\"chirp_band_edges_valid\":" << (d.band_width_valid ? "true" : "false")
            << ",\"low_edge_offset_hz\":";
        if (d.low_edge.fitted) WriteNumberOrNull(out, d.low_edge.frequency_offset_hz); else out << "null";
        out << ",\"high_edge_offset_hz\":";
        if (d.high_edge.fitted) WriteNumberOrNull(out, d.high_edge.frequency_offset_hz); else out << "null";
        out << ",\"packet_start_us\":"; WriteNumberOrNull(out, d.packet_start_time_s * 1.0e6);
        out << ",\"packet_end_us\":"; WriteNumberOrNull(out, d.packet_end_time_s * 1.0e6);
        out << ",\"packet_duration_us\":"; WriteNumberOrNull(out, d.packet_duration_us);
        out << ",\"packet_edge_fit\":\"" << d.packet_fit_reject_reason << "\""
            << ",\"packet_band_edges_valid\":" << (d.packet_band_fit_valid ? "true" : "false")
            << ",\"packet_band_low_edge_offset_hz\":";
        if (d.packet_band_fit_valid) WriteNumberOrNull(out, d.packet_band_low_frequency_offset_hz); else out << "null";
        out << ",\"packet_band_high_edge_offset_hz\":";
        if (d.packet_band_fit_valid) WriteNumberOrNull(out, d.packet_band_high_frequency_offset_hz); else out << "null";
        out << ",\"packet_band_cfo_offset_hz\":";
        if (d.packet_band_fit_valid) WriteNumberOrNull(out, d.packet_band_cfo_offset_hz); else out << "null";
        out
            << ",\"packet_snr_db\":";
        if (d.packet_snr_valid) WriteNumberOrNull(out, d.packet_snr_db); else out << "null";
        out << ",\"highres_fft_fit_valid\":"
            << (m.high_resolution_fit_valid ? "true" : "false")
            << ",\"highres_fft_size\":" << m.high_resolution.fft_size
            << ",\"highres_packet_snr_db\":";
        if (m.high_resolution.packet_snr_valid)
            WriteNumberOrNull(out, m.high_resolution.packet_snr_db);
        else out << "null";
        out << ",\"highres_low_edge_offset_hz\":";
        if (m.high_resolution_fit_valid) WriteNumberOrNull(out, m.high_resolution.low_edge_hz); else out << "null";
        out << ",\"highres_high_edge_offset_hz\":";
        if (m.high_resolution_fit_valid) WriteNumberOrNull(out, m.high_resolution.high_edge_hz); else out << "null";
        out << ",\"highres_cfo_offset_hz\":";
        if (m.high_resolution_fit_valid) WriteNumberOrNull(out, m.high_resolution.cfo_hz); else out << "null";
        out << '}';
    }
    out << "]}";
}

void ProcessProbe(Shared &shared, std::size_t channel_index,
                  const std::array<std::vector<std::int16_t>, 2> &iq) {
    std::array<double, 2> dbfs{{-120.0, -120.0}};
    std::array<std::vector<SpectrumPowerColumn>, 2> spectra;
    std::array<double, 2> noise_power{{1.0e-12, 1.0e-12}};
    for (unsigned chain = 0; chain < 2; ++chain) {
        std::vector<double> all_bin_power;
        const std::size_t column_count =
            (kCaptureSamples - kFftSize) / kFftHop + 1;
        all_bin_power.reserve(column_count * kFftSize);
        spectra[chain].reserve(column_count);
        for (std::size_t start = 0; start + kFftSize <= kCaptureSamples; start += kFftHop) {
            spectra[chain].push_back(SpectrumColumn(iq[chain], start));
            all_bin_power.insert(all_bin_power.end(), spectra[chain].back().begin(),
                                 spectra[chain].back().end());
        }
        if (all_bin_power.empty()) continue;
        std::nth_element(all_bin_power.begin(),
                         all_bin_power.begin() + all_bin_power.size() / 2,
                         all_bin_power.end());
        noise_power[chain] = std::max(all_bin_power[all_bin_power.size() / 2], 1.0e-12);
        // 12-bit ADC full scale is 2048 counts; report the capture-wide median bin power.
        dbfs[chain] = 10.0 * std::log10(
            std::max(noise_power[chain] / (2048.0 * 2048.0), 1.0e-20));
    }
    std::lock_guard<std::mutex> lock(shared.mutex);
    Channel &c = shared.channels[channel_index];
    for (unsigned chain = 0; chain < 2; ++chain) {
        ChainHistory &history = c.rx[chain];
        for (const SpectrumPowerColumn &spectrum : spectra[chain]) {
            for (std::size_t bin = 0; bin < kFftSize; ++bin) {
                const double ratio = std::max(spectrum[bin] / noise_power[chain], 1.0e-12);
                const double intensity = 32.0 + 32.0 * std::log(ratio);
                history.pixels[history.write_column * kFftSize + bin] =
                    static_cast<std::uint8_t>(std::max(0.0, std::min(255.0, intensity)));
            }
            history.write_column = (history.write_column + 1) % kHistoryColumns;
            if (history.valid_columns < kHistoryColumns) ++history.valid_columns;
        }
        if (!spectra[chain].empty()) {
            history.noise_dbfs = dbfs[chain];
            history.have_noise = true;
        }
    }
    ++c.probes;
}

bool WaitStatus(Registers &regs, std::uint32_t mask, std::chrono::steady_clock::time_point deadline) {
    while (!g_stop && std::chrono::steady_clock::now() < deadline) {
        if ((regs.Read(kCaptureStatus) & mask) == mask) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

struct TriggerEvent {
    unsigned source = 0;
    unsigned rx_mask = 0;
    unsigned sample_offset = 0;
};

bool EnsureDirectory(const std::string &path) {
    struct stat st{};
    if (stat(path.c_str(), &st) != 0) {
        if (errno != ENOENT || mkdir(path.c_str(), 0755) != 0) {
            std::cerr << "Cannot create trigger output directory " << path << ": "
                      << std::strerror(errno) << '\n';
            return false;
        }
        if (stat(path.c_str(), &st) != 0) {
            std::cerr << "Cannot inspect trigger output directory " << path << ": "
                      << std::strerror(errno) << '\n';
            return false;
        }
    }
    if (!S_ISDIR(st.st_mode)) {
        std::cerr << "Trigger output path is not a directory: " << path << '\n';
        return false;
    }
    if (path == "/boot" || StartsWith(path, "/boot/")) {
        struct stat root{};
        if (stat("/", &root) != 0 || st.st_dev == root.st_dev) {
            std::cerr << "Trigger output is under /boot, but /boot does not appear to be "
                         "the mounted SD card: " << path << '\n';
            return false;
        }
    }
    return true;
}

std::string CaptureBase(std::uint64_t frequency_hz, std::uint64_t sequence) {
    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    const std::time_t seconds = static_cast<std::time_t>(ms / 1000);
    std::tm tm_value{};
    localtime_r(&seconds, &tm_value);
    char timestamp[32]{};
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d-%H%M%S", &tm_value);
    char suffix[96]{};
    std::snprintf(suffix, sizeof(suffix), "-%03lld-%lluHz-%ld-%06llu",
                  static_cast<long long>(ms % 1000),
                  static_cast<unsigned long long>(frequency_hz),
                  static_cast<long>(getpid()),
                  static_cast<unsigned long long>(sequence));
    return std::string("droneid-") + timestamp + suffix;
}

bool WriteCandidate(const Options &o, std::uint64_t frequency_hz,
                    std::uint64_t sequence,
                    const std::array<std::vector<std::int16_t>, 2> &iq,
                    const std::vector<TriggerEvent> &events,
                    const std::array<ChainCandidateAnalysis, 2> &analysis,
                    std::uint32_t trigger_sample, std::uint32_t dropped) {
    const std::string stem = o.save_triggers_dir + "/" +
                             CaptureBase(frequency_hz, sequence);
    const std::string suffix = ".tmp." + std::to_string(static_cast<long>(getpid()));
    const std::string tmp0 = stem + ".rx0.cx16" + suffix;
    const std::string tmp1 = stem + ".rx1.cx16" + suffix;
    const std::string tmp_json = stem + ".json" + suffix;
    const std::string path0 = stem + ".rx0.cx16";
    const std::string path1 = stem + ".rx1.cx16";
    const std::string path_json = stem + ".json";
    auto cleanup = [&]() {
        unlink(tmp0.c_str()); unlink(tmp1.c_str()); unlink(tmp_json.c_str());
        unlink(path0.c_str()); unlink(path1.c_str()); unlink(path_json.c_str());
    };
    auto write_iq = [](const std::string &path, const std::vector<std::int16_t> &samples) {
        std::ofstream out(path.c_str(), std::ios::binary | std::ios::trunc);
        if (!out) return false;
        out.write(reinterpret_cast<const char *>(samples.data()),
                  static_cast<std::streamsize>(samples.size() * sizeof(std::int16_t)));
        out.flush();
        return static_cast<bool>(out);
    };
    if (!write_iq(tmp0, iq[0]) || !write_iq(tmp1, iq[1])) {
        std::cerr << "Failed writing triggered I/Q candidate in " << o.save_triggers_dir << '\n';
        cleanup(); return false;
    }
    {
        std::ofstream meta(tmp_json.c_str(), std::ios::trunc);
        if (!meta) { std::cerr << "Cannot create candidate metadata " << tmp_json << '\n'; cleanup(); return false; }
        meta << "{\n  \"format\": \"signed int16 little-endian I,Q\",\n"
             << "  \"sample_rate_hz\": " << kRate << ",\n"
             << "  \"center_frequency_hz\": " << frequency_hz << ",\n"
             << "  \"samples_per_chain\": " << kCaptureSamples << ",\n"
             << "  \"pre_samples\": " << kPreSamples << ",\n"
             << "  \"post_samples\": " << kPostSamples << ",\n"
             << "  \"trigger_sample_index\": " << trigger_sample << ",\n"
             << "  \"dropped_triggers\": " << dropped << ",\n"
             << "  \"events\": [";
        for (std::size_t i = 0; i < events.size(); ++i) {
            if (i) meta << ',';
            meta << "{\"source\":\"" << (events[i].source == 1 ? "detector" : "software")
                 << "\",\"rx_mask\":" << events[i].rx_mask
                 << ",\"sample_offset\":" << events[i].sample_offset << '}';
        }
        meta << "],\n  \"analysis\": {\"rx0\":";
        WriteChainAnalysis(meta, analysis[0]);
        meta << ",\"rx1\":";
        WriteChainAnalysis(meta, analysis[1]);
        meta << "}\n}\n";
        meta.flush();
        if (!meta) { std::cerr << "Failed writing candidate metadata " << tmp_json << '\n'; cleanup(); return false; }
    }
    if (rename(tmp0.c_str(), path0.c_str()) != 0 ||
        rename(tmp1.c_str(), path1.c_str()) != 0 ||
        rename(tmp_json.c_str(), path_json.c_str()) != 0) {
        std::cerr << "Failed committing candidate files for " << stem << ": "
                  << std::strerror(errno) << '\n';
        cleanup(); return false;
    }
    return true;
}

struct CaptureWorkItem {
    bool software_probe = false;
    std::size_t channel_index = 0;
    std::uint64_t frequency_hz = 0;
    std::uint64_t sequence = 0;
    std::array<std::vector<std::int16_t>, 2> iq;
    std::vector<TriggerEvent> events;
    std::uint32_t trigger_sample = 0;
    std::uint32_t dropped_triggers = 0;
};

class AnalysisWorker {
public:
    AnalysisWorker(Shared &shared, const Options &options)
        : shared_(shared), options_(options) {}

    void Start() { thread_ = std::thread(&AnalysisWorker::Run, this); }

    bool QueueCandidate(CaptureWorkItem item) {
        std::unique_lock<std::mutex> lock(mutex_);
        bool waited = false;
        while (!stopping_ && candidates_.size() >= kCandidateQueueCapacity) {
            waited = true;
            space_.wait(lock);
        }
        if (waited) shared_.candidate_queue_waits.fetch_add(1);
        if (stopping_) return false;
        candidates_.push_back(std::move(item));
        const unsigned depth = static_cast<unsigned>(candidates_.size());
        shared_.candidate_queue_depth.store(depth);
        unsigned high = shared_.candidate_queue_high_water.load();
        while (depth > high && !shared_.candidate_queue_high_water.compare_exchange_weak(high, depth)) {}
        lock.unlock();
        available_.notify_one();
        return true;
    }

    bool QueueProbe(CaptureWorkItem item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || probes_.size() >= kProbeQueueCapacity) {
            shared_.dropped_software_probes.fetch_add(1);
            return false;
        }
        probes_.push_back(std::move(item));
        available_.notify_one();
        return true;
    }

    void StopAndJoin() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        available_.notify_all();
        space_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    ~AnalysisWorker() { StopAndJoin(); }

private:
    static constexpr std::size_t kCandidateQueueCapacity = 32;
    static constexpr std::size_t kProbeQueueCapacity = 2;

    void Run() {
        for (;;) {
            CaptureWorkItem item;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                available_.wait(lock, [this] {
                    return stopping_ || !candidates_.empty() || !probes_.empty();
                });
                if (candidates_.empty() && probes_.empty() && stopping_) return;
                if (!candidates_.empty()) {
                    item = std::move(candidates_.front());
                    candidates_.pop_front();
                    shared_.candidate_queue_depth.store(static_cast<unsigned>(candidates_.size()));
                    space_.notify_one();
                } else {
                    item = std::move(probes_.front());
                    probes_.pop_front();
                }
            }
            if (item.software_probe) {
                ProcessProbe(shared_, item.channel_index, item.iq);
                shared_.software_probes.fetch_add(1);
                continue;
            }

            std::array<ChainCandidateAnalysis, 2> analysis;
            analysis[0] = AnalyzeCandidateChain(item.iq[0], item.frequency_hz);
            analysis[1] = AnalyzeCandidateChain(item.iq[1], item.frequency_hz);
            if (!WriteCandidate(options_, item.frequency_hz, item.sequence, item.iq,
                                item.events, analysis, item.trigger_sample,
                                item.dropped_triggers)) {
                shared_.candidate_save_failures.fetch_add(1);
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(shared_.mutex);
                ++shared_.channels[item.channel_index].saved_candidates;
            }
            std::cout << "Saved hardware-triggered candidate " << item.sequence << " at "
                      << std::fixed << std::setprecision(3) << item.frequency_hz / 1.0e6
                      << " MHz\n";
        }
    }

    Shared &shared_;
    const Options &options_;
    std::mutex mutex_;
    std::condition_variable available_;
    std::condition_variable space_;
    std::deque<CaptureWorkItem> candidates_;
    std::deque<CaptureWorkItem> probes_;
    bool stopping_ = false;
    std::thread thread_;
};

bool DrainComplete(Registers &regs, iio_buffer *buffer,
                   const std::array<iio_channel *, 4> &channels,
                   Shared &shared, std::size_t channel_index,
                   const Options &options, std::uint64_t frequency_hz,
                   std::uint64_t &save_sequence, AnalysisWorker &worker,
                   bool *saw_software) {
    if (!(regs.Read(kCaptureStatus) & kCaptureComplete)) return false;
    std::array<std::vector<std::int16_t>, 2> iq;
    const bool read_ok = ReadCapture(buffer, channels, iq);
    const unsigned event_count = regs.Read(kEventCount) & 0xfu;
    bool software = false;
    bool detector = false;
    const std::uint32_t trigger_sample = regs.Read(kTriggerSample);
    const std::uint32_t dropped_triggers = regs.Read(kDroppedTriggers);
    std::vector<TriggerEvent> events;
    events.reserve(std::min(event_count, 8u));
    for (unsigned i = 0; i < event_count && i < 8; ++i) {
        const std::uint32_t word = regs.Read(kEventBase + 4 * i);
        TriggerEvent event;
        event.source = (word >> 18) & 3u;
        event.rx_mask = (word >> 16) & 3u;
        event.sample_offset = word & 0xffffu;
        events.push_back(event);
        if (event.source == 2u) software = true;
        else if (event.source == 1u) detector = true;
    }
    if (saw_software) *saw_software = software;
    regs.Write(kCaptureControl, 5u); // enable + ACK toggle; accepted in READY/STREAM/DRAIN
    const auto ack_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while ((regs.Read(kCaptureStatus) & kCaptureComplete) && std::chrono::steady_clock::now() < ack_deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (regs.Read(kCaptureStatus) & kCaptureComplete) { std::cerr << "FPGA capture ACK timeout\n"; return false; }
    if (read_ok && event_count) {
        std::lock_guard<std::mutex> lock(shared.mutex);
        ++shared.channels[channel_index].captures;
    }
    if (read_ok && software && !detector) {
        CaptureWorkItem item;
        item.software_probe = true;
        item.channel_index = channel_index;
        item.iq = std::move(iq);
        worker.QueueProbe(std::move(item));
    } else if (read_ok && detector && !options.save_triggers_dir.empty()) {
        CaptureWorkItem item;
        item.channel_index = channel_index;
        item.frequency_hz = frequency_hz;
        item.sequence = ++save_sequence;
        item.iq = std::move(iq);
        item.events = std::move(events);
        item.trigger_sample = trigger_sample;
        item.dropped_triggers = dropped_triggers;
        if (!worker.QueueCandidate(std::move(item))) {
            std::cerr << "Candidate worker stopped before capture " << save_sequence << " was queued\n";
            return false;
        }
    }
    return read_ok;
}

} // namespace

int main(int argc, char **argv) {
    Options o;
    if (!ParseArgs(argc, argv, o)) { Usage(argv[0]); return 2; }
    if (!o.save_triggers_dir.empty() && !EnsureDirectory(o.save_triggers_dir)) return 1;
    std::signal(SIGINT, StopHandler); std::signal(SIGTERM, StopHandler);
    Registers regs; if (!regs.Open(o)) return 1;
    iio_context *ctx = iio_create_default_context();
    if (!ctx) { std::cerr << "Cannot create local IIO context\n"; return 1; }
    iio_device *rx = nullptr; iio_channel *rx_lo = nullptr;
    iio_channel *rx_cfg0 = nullptr, *rx_cfg1 = nullptr;
    std::array<iio_channel *, 4> channels{}; iio_buffer *buffer = nullptr;
    if (!ConfigureRx(ctx, o, rx, rx_lo, rx_cfg0, rx_cfg1, channels, buffer)) { iio_context_destroy(ctx); return 1; }
    Shared shared(o.frequencies); shared.gain_db = o.gain_db;
    shared.requested_gain_db.store(o.gain_db); shared.dwell_ms = o.dwell_ms;
    int server = StartHttp(o.http_port);
    if (server < 0) { std::cerr << "Could not start HTTP server\n"; iio_buffer_destroy(buffer); iio_context_destroy(ctx); return 1; }
    std::thread http(HttpLoop, server, std::ref(shared));
    AnalysisWorker worker(shared, o);
    worker.Start();
    regs.Write(kCaptureControl, 5u); std::this_thread::sleep_for(std::chrono::milliseconds(2));
    regs.Write(kCaptureControl, 1u);
    regs.Write(kDetectorControl, 0u);
    std::cout << "DroneID channel scanner: " << o.frequencies.size() << " channels, "
              << kRate << " Sa/s, dwell " << o.dwell_ms << " ms, HTTP :" << o.http_port << '\n';
    for (std::size_t i = 0; i < o.frequencies.size(); ++i)
        std::cout << "  " << std::fixed << std::setprecision(3) << o.frequencies[i] / 1.0e6 << " MHz\n";

    std::size_t hop = 0;
    std::uint64_t save_sequence = 0;
    double applied_gain_db = o.gain_db;
    while (!g_stop) {
        const std::uint64_t frequency = o.frequencies[hop];
        regs.Write(kDetectorControl, 0u); // don't charge LO-settling transients to either channel
        const double requested_gain_db = shared.requested_gain_db.load();
        if (std::abs(requested_gain_db - applied_gain_db) >= 0.05) {
            int gain_ret = iio_channel_attr_write_double(rx_cfg0, "hardwaregain", requested_gain_db);
            if (gain_ret >= 0)
                gain_ret = iio_channel_attr_write_double(rx_cfg1, "hardwaregain", requested_gain_db);
            if (gain_ret < 0) {
                // Keep the two receive chains matched if the second write fails.
                iio_channel_attr_write_double(rx_cfg0, "hardwaregain", applied_gain_db);
                iio_channel_attr_write_double(rx_cfg1, "hardwaregain", applied_gain_db);
                char err[128] = {}; iio_strerror(-gain_ret, err, sizeof(err));
                std::cerr << "RX gain update to " << requested_gain_db << " dB failed: " << err << '\n';
                shared.requested_gain_db.store(applied_gain_db);
            } else {
                applied_gain_db = requested_gain_db;
                { std::lock_guard<std::mutex> lock(shared.mutex); shared.gain_db = applied_gain_db; }
                std::cout << "RX0/RX1 gain set to " << applied_gain_db << " dB\n";
            }
        }
        int ret = iio_channel_attr_write_longlong(rx_lo, "frequency", static_cast<long long>(frequency));
        if (ret < 0) { char err[128] = {}; iio_strerror(-ret, err, sizeof(err)); std::cerr << "Tune failed: " << err << '\n'; break; }
        {
            std::lock_guard<std::mutex> lock(shared.mutex); shared.current = hop;
        }
        const auto tune_done = std::chrono::steady_clock::now() + std::chrono::milliseconds(o.settle_ms);
        while (!g_stop && std::chrono::steady_clock::now() < tune_done)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const std::uint32_t count0_before = regs.Read(kTriggerCount0), count1_before = regs.Read(kTriggerCount1);
        regs.Write(kDetectorControl, 1u);
        const auto dwell_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(o.dwell_ms);
        bool probe_pending = true;
        while (!g_stop && std::chrono::steady_clock::now() < dwell_end) {
            if (!WaitStatus(regs, kPretriggerReady, std::min(dwell_end, std::chrono::steady_clock::now() + std::chrono::milliseconds(5))))
                continue;
            if (regs.Read(kCaptureStatus) & kCaptureComplete) {
                bool software = false;
                if (!DrainComplete(regs, buffer, channels, shared, hop, o, frequency,
                                   save_sequence, worker, &software)) { g_stop = 1; break; }
                if (software) probe_pending = false;
                continue;
            }
            if (probe_pending) {
                // A full history ring can remain marked ready while capture
                // is in ST_POST. Do not inject a probe into that event's
                // capture; wait for its completion/ACK and then probe cleanly.
                if (regs.Read(kCaptureStatus) & (1u << 1)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                regs.Write(kCaptureControl, 3u); // capture enable + software-trigger edge
                probe_pending = false;
                // If a hardware event wins the race, DrainComplete leaves the probe pending.
                const auto trigger_deadline = std::min(dwell_end, std::chrono::steady_clock::now() + std::chrono::milliseconds(10));
                while (!g_stop && std::chrono::steady_clock::now() < trigger_deadline &&
                       !(regs.Read(kCaptureStatus) & kCaptureComplete))
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (regs.Read(kCaptureStatus) & kCaptureComplete) {
                    bool software = false;
                    if (!DrainComplete(regs, buffer, channels, shared, hop, o, frequency,
                                       save_sequence, worker, &software)) { g_stop = 1; break; }
                    if (!software) probe_pending = true;
                } else probe_pending = true;
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        regs.Write(kDetectorControl, 0u);
        // Let an in-flight detector pulse cross the RX/AXI synchronizers and
        // finish its 1 ms post-trigger capture before changing the LO.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
        while (!g_stop && (regs.Read(kCaptureStatus) & (1u << 1)) &&
               std::chrono::steady_clock::now() < drain_deadline) {
            if (regs.Read(kCaptureStatus) & kCaptureComplete) {
                if (!DrainComplete(regs, buffer, channels, shared, hop, o, frequency,
                                       save_sequence, worker, nullptr)) { g_stop = 1; break; }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        if (regs.Read(kCaptureStatus) & (1u << 1)) {
            std::cerr << "Capture remained busy at channel boundary; skipping LO retune\n";
            g_stop = 1;
        }
        const std::uint32_t count0_after = regs.Read(kTriggerCount0), count1_after = regs.Read(kTriggerCount1);
        shared.fpga_dropped_triggers.store(regs.Read(kDroppedTriggers));
        const std::uint32_t delta0 = count0_after - count0_before, delta1 = count1_after - count1_before;
        {
            std::lock_guard<std::mutex> lock(shared.mutex);
            shared.channels[hop].hardware_triggers_rx0 += delta0;
            shared.channels[hop].hardware_triggers_rx1 += delta1;
        }
        shared.hardware_events.fetch_add(static_cast<std::uint64_t>(delta0) + delta1);
        std::cout << std::fixed << std::setprecision(3) << frequency / 1.0e6 << " MHz: "
                  << "FPGA triggers RX0=" << delta0 << " RX1=" << delta1
                  << ", software probes=";
        { std::lock_guard<std::mutex> lock(shared.mutex); std::cout << shared.channels[hop].probes << '\n'; }
        hop = (hop + 1) % o.frequencies.size();
    }

    regs.Write(kDetectorControl, 0u); regs.Write(kCaptureControl, 0u);
    worker.StopAndJoin();
    shared.running.store(false); shutdown(server, SHUT_RDWR); close(server); if (http.joinable()) http.join();
    iio_buffer_destroy(buffer); for (iio_channel *c : channels) if (c) iio_channel_disable(c); iio_context_destroy(ctx);
    return 0;
}
