#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <thread>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

constexpr std::size_t kMapSize = 0x1000;
constexpr std::size_t kCounter0Offset = 0x38;
constexpr std::size_t kCounter1Offset = 0x40;
constexpr std::size_t kControlOffset = 0x3c;
constexpr std::uint32_t kEnableAndClear = 0x3;

void Usage(const char *program) {
    std::cerr << "Usage: " << program
              << " [--clear] [--watch-ms N] [--mem /dev/mem]"
              << " [--phys ADDRESS] [--channel 0|1 | --both]\n";
}

}  // namespace

int main(int argc, char **argv) {
    bool clear = false;
    unsigned int watch_ms = 0;
    unsigned int channel = 0;
    bool both = false;
    std::string mem_device = "/dev/mem";
    std::uint64_t physical_base = 0x7c460000;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--clear") {
            clear = true;
        } else if (arg == "--watch-ms" && i + 1 < argc) {
            char *end = nullptr;
            const unsigned long value = std::strtoul(argv[++i], &end, 10);
            if (!end || *end || value == 0 || value > 3600000) {
                Usage(argv[0]);
                return 2;
            }
            watch_ms = static_cast<unsigned int>(value);
        } else if (arg == "--mem" && i + 1 < argc) {
            mem_device = argv[++i];
        } else if (arg == "--phys" && i + 1 < argc) {
            char *end = nullptr;
            physical_base = std::strtoull(argv[++i], &end, 0);
            if (!end || *end) {
                Usage(argv[0]);
                return 2;
            }
        } else if (arg == "--channel" && i + 1 < argc) {
            char *end = nullptr;
            const unsigned long value = std::strtoul(argv[++i], &end, 10);
            if (!end || *end || value > 1) {
                Usage(argv[0]);
                return 2;
            }
            channel = static_cast<unsigned int>(value);
            both = false;
        } else if (arg == "--both") {
            both = true;
        } else {
            Usage(argv[0]);
            return 2;
        }
    }

    const int fd = open(mem_device.c_str(), O_RDWR | O_SYNC);
    if (fd < 0) {
        std::cerr << "open(" << mem_device << "): " << std::strerror(errno)
                  << '\n';
        return 1;
    }
    void *mapping = mmap(nullptr, kMapSize, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, static_cast<off_t>(physical_base));
    if (mapping == MAP_FAILED) {
        std::cerr << "mmap(" << mem_device << "): " << std::strerror(errno)
                  << '\n';
        close(fd);
        return 1;
    }

    volatile std::uint32_t *registers =
        static_cast<volatile std::uint32_t *>(mapping);
    if (clear) {
        // Enable is bit 0 and the write-pulse clear is bit 1. Writing both
        // leaves counting enabled while resetting the event counter.
        registers[kControlOffset / sizeof(std::uint32_t)] = kEnableAndClear;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const std::size_t counter_offset = channel == 0 ? kCounter0Offset : kCounter1Offset;
    std::uint32_t previous = registers[counter_offset / sizeof(std::uint32_t)];
    std::uint32_t previous0 = registers[kCounter0Offset / sizeof(std::uint32_t)];
    std::uint32_t previous1 = registers[kCounter1Offset / sizeof(std::uint32_t)];
    std::uint32_t control =
        registers[kControlOffset / sizeof(std::uint32_t)];
    if (both) {
        std::cout << "trigger_count_rx0=" << previous0
                  << " trigger_count_rx1=" << previous1;
    } else {
        std::cout << "trigger_count_rx" << channel << '=' << previous;
    }
    std::cout << " enabled=" << (control & 1u)
              << " overrun=" << ((control >> 2) & 1u) << '\n';
    while (watch_ms != 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(watch_ms));
        control = registers[kControlOffset / sizeof(std::uint32_t)];
        if (both) {
            const std::uint32_t current0 = registers[kCounter0Offset / sizeof(std::uint32_t)];
            const std::uint32_t current1 = registers[kCounter1Offset / sizeof(std::uint32_t)];
            std::cout << "trigger_count_rx0=" << current0
                      << " new_rx0=" << static_cast<std::uint32_t>(current0 - previous0)
                      << " trigger_count_rx1=" << current1
                      << " new_rx1=" << static_cast<std::uint32_t>(current1 - previous1);
            previous0 = current0;
            previous1 = current1;
        } else {
            const std::uint32_t current = registers[counter_offset / sizeof(std::uint32_t)];
            std::cout << "trigger_count_rx" << channel << '=' << current
                      << " new=" << static_cast<std::uint32_t>(current - previous);
            previous = current;
        }
        std::cout << " enabled=" << (control & 1u)
                  << " overrun=" << ((control >> 2) & 1u) << '\n';
    }

    munmap(mapping, kMapSize);
    close(fd);
    return 0;
}
