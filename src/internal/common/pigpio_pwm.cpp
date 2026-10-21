#include "internal/common/pigpio_pwm.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
struct Fifo {
    int fd;
    struct stat info{};
    Fifo(const std::string& path, int mode) : fd(open(path.c_str(), mode | O_NONBLOCK | O_CLOEXEC)) {
        if (fd < 0)
            throw std::runtime_error("Cannot open pigpio FIFO " + path + ": " + std::strerror(errno) +
                                     ". Start pigpiod with FIFO enabled and check permissions.");
        if (fstat(fd, &info) != 0 || !S_ISFIFO(info.st_mode)) {
            close(fd);
            throw std::runtime_error(path + " is not an accessible pigpio FIFO");
        }
    }
    ~Fifo() { close(fd); }
    Fifo(const Fifo&) = delete;
    Fifo& operator=(const Fifo&) = delete;
};

// A daemon disappearing during write must produce an exception, not SIGPIPE
// terminating the application. Only change this thread's signal mask.
ssize_t write_command(int fd, const std::string& data) {
    sigset_t mask, previous, pending;
    sigemptyset(&mask);
    sigaddset(&mask, SIGPIPE);
    const int error = pthread_sigmask(SIG_BLOCK, &mask, &previous);
    if (error) throw std::runtime_error("Cannot block SIGPIPE: " + std::string(std::strerror(error)));
    sigpending(&pending);
    const bool already_pending = sigismember(&pending, SIGPIPE) == 1;
    ssize_t count;
    do { count = write(fd, data.data(), data.size()); } while (count < 0 && errno == EINTR);
    const int saved_errno = errno;
    if (count < 0 && saved_errno == EPIPE && !already_pending) {
        timespec zero{};
        while (sigtimedwait(&mask, nullptr, &zero) < 0 && errno == EINTR) {}
    }
    pthread_sigmask(SIG_SETMASK, &previous, nullptr);
    errno = saved_errno;
    return count;
}
}
#endif

PigpioPWM::PigpioPWM(std::string pipe, std::string response, int hz, double seconds)
    : pipe_path(std::move(pipe)), response_path(std::move(response)), frequency(hz), timeout(seconds) {
    if (frequency <= 0) throw std::invalid_argument("PWM frequency must be positive");
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > std::numeric_limits<int>::max() / 1000.0)
        throw std::invalid_argument("pigpio timeout must be positive and fit in milliseconds");
}

int PigpioPWM::command(int command_fd, int response_fd, const std::string& text) {
#ifdef __linux__
    const std::string data = text + "\n";
    if (write_command(command_fd, data) != static_cast<ssize_t>(data.size()))
        throw std::runtime_error("Cannot write pigpio command " + text + ": " + std::strerror(errno));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    std::string response;
    while (response.find('\n') == std::string::npos) {
        const double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) throw std::runtime_error("Timed out waiting for pigpiod: " + text);
        pollfd poll_fd{response_fd, POLLIN, 0};
        int ready = poll(&poll_fd, 1, static_cast<int>(std::ceil(remaining * 1000)));
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0) throw std::runtime_error("Cannot poll pigpio response FIFO");
        if (ready == 0) continue;
        char buffer[64];
        const ssize_t count = read(response_fd, buffer, sizeof(buffer));
        if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
        if (count <= 0) throw std::runtime_error("pigpiod closed its response FIFO");
        response.append(buffer, static_cast<size_t>(count));
        if (response.size() > 64) throw std::runtime_error("Oversized pigpio response");
    }
    std::istringstream input(response);
    int result;
    if (!(input >> result) || !(input >> std::ws).eof())
        throw std::runtime_error("Invalid pigpio response: " + response);
    if (result < 0)
        throw std::runtime_error("pigpio command '" + text + "' failed with error " + std::to_string(result));
    return result;
#else
    throw std::runtime_error("pigpio FIFO control requires Linux");
#endif
}

void PigpioPWM::set_dutycycle(int gpio, double duty) {
    if (gpio < 0 || gpio > 31) throw std::invalid_argument("pigpio GPIO must be from 0 to 31");
    if (!(duty >= 0 && duty <= 1)) throw std::invalid_argument("PWM duty cycle must be between 0 and 1");
    std::lock_guard<std::recursive_mutex> guard(mutex);
    if (stopped && duty != 0) throw std::runtime_error("PWM controller has been stopped");
    try {
#ifdef __linux__
        Fifo input(pipe_path, O_WRONLY), output(response_path, O_RDONLY);
        if (!have_identity || pipe_device != input.info.st_dev || pipe_inode != input.info.st_ino) {
            configured.clear();
            pipe_device = input.info.st_dev;
            pipe_inode = input.info.st_ino;
            have_identity = true;
        }
        char buffer[4096];
        const auto drain_start = std::chrono::steady_clock::now();
        for (;;) {
            const ssize_t count = read(output.fd, buffer, sizeof(buffer));
            if (count == 0 || (count < 0 && errno == EAGAIN)) break;
            if (count < 0 && errno != EINTR) throw std::runtime_error("Cannot drain pigpio response FIFO");
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - drain_start).count() >= timeout)
                throw std::runtime_error("pigpio response FIFO is busy; use one controlling application");
        }
        const auto send = [&](const std::string& text) { return command(input.fd, output.fd, text); };
        const std::string pin = std::to_string(gpio);
        if (duty == 0) {
            // WRITE cancels PWM/servo generation and actively drives LOW.
            send("W " + pin + " 0");
            configured.erase(gpio);
            used.erase(gpio);
            return;
        }
        // Inodes can be reused, or a killed daemon may reuse existing FIFOs.
        // Check its actual settings before trusting our per-pin cache.
        if (configured.count(gpio) && (send("PFG " + pin) != frequency || send("PRG " + pin) != PWM_RANGE))
            configured.erase(gpio);
        if (!configured.count(gpio)) {
            send("P " + pin + " 0");
            const int actual = send("PFS " + pin + " " + std::to_string(frequency));
            if (actual != frequency)
                throw std::runtime_error("pigpio selected " + std::to_string(actual) + " Hz instead of " +
                                         std::to_string(frequency) + "; check sample rate (100 Hz works with -s 5)");
            send("PRS " + pin + " " + std::to_string(PWM_RANGE));
            configured.insert(gpio);
        }
        used.insert(gpio); // A lost reply does not mean PWM was not applied.
        const double scaled = duty * PWM_RANGE;
        const auto floor_value = static_cast<int>(std::floor(scaled));
        // Match Python round(): ties to even, independent of the process FPU mode.
        const int value = floor_value + (scaled - floor_value > 0.5 ||
                          (scaled - floor_value == 0.5 && floor_value % 2 != 0));
        send("P " + pin + " " + std::to_string(value));
#else
        throw std::runtime_error("pigpio FIFO control requires Linux");
#endif
    } catch (...) {
        configured.clear();
        throw;
    }
}

void PigpioPWM::stop(const std::vector<int>& gpios) {
    std::lock_guard<std::recursive_mutex> guard(mutex);
    stopped = true;
    auto targets = used;
    targets.insert(gpios.begin(), gpios.end());
    std::string errors;
    for (int gpio : targets) {
        try { set_dutycycle(gpio, 0); }
        catch (const std::exception& error) { errors += std::string(error.what()) + "; "; }
    }
    if (!errors.empty()) throw std::runtime_error("Failed to disable HCDIO PWM: " + errors);
}
