#include "shared.hpp"

#ifdef __linux__
#include <dlfcn.h>
#endif

#include <stdexcept>
#include <cstring>

LibHolder::LibHolder(const std::string& first_path) {
    std::string lib_path = first_path + "/CommonRPiLibrary/CommonRPiLibrary/build/libCommonRPiLibrary.so";

#ifdef __linux__
    handle = dlopen(lib_path.c_str(), RTLD_LAZY);
    if (!handle) {
        throw std::runtime_error("Failed to load library: " + std::string(dlerror()));
    }
    dlerror();

    start_spi_extern = reinterpret_cast<StartSPIFunc>(dlsym(handle, "StartSPI"));
    start_usb_extern = reinterpret_cast<StartUSBFunc>(dlsym(handle, "StartUSB"));
    read_write_spi_extern = reinterpret_cast<ReadWriteSPIFunc>(dlsym(handle, "ReadWriteSPI"));
    read_write_usb_extern = reinterpret_cast<ReadWriteUSBFunc>(dlsym(handle, "ReadWriteUSB"));
    stop_spi_extern = reinterpret_cast<StopSPIFunc>(dlsym(handle, "StopSPI"));
    stop_usb_extern = reinterpret_cast<StopUSBFunc>(dlsym(handle, "StopUSB"));

    const char* err = dlerror();
    if (err) {
        dlclose(handle);
        throw std::runtime_error("Failed to load symbols: " + std::string(err));
    }
#endif
}

LibHolder::~LibHolder() {
#ifdef __linux__
    if (handle) {
        dlclose(handle);
    }
#endif
}

int LibHolder::init_spi(const std::string& path, int channel, int speed, int mode) {
    if (!start_spi_extern) throw std::runtime_error("start_spi not available");
    return start_spi_extern(path.c_str(), channel, speed, mode);
}

int LibHolder::init_usb(const std::string& path, int baud) {
    if (!start_usb_extern) throw std::runtime_error("start_usb not available");
    return start_usb_extern(path.c_str(), baud);
}

uint8_t* LibHolder::rw_spi(uint8_t* data, unsigned int len) {
    if (!read_write_spi_extern) throw std::runtime_error("read_write_spi not available");

    unsigned char* result_ptr = read_write_spi_extern(data, len);
    return result_ptr;
}

uint8_t* LibHolder::rw_usb(uint8_t* data, unsigned int len) {
    if (!read_write_usb_extern) throw std::runtime_error("read_write_usb not available");

    unsigned char* result_ptr = read_write_usb_extern(data, len);
    return result_ptr;
}

void LibHolder::stop_spi() {
    if (stop_spi_extern) stop_spi_extern();
}

void LibHolder::stop_usb() {
    if (stop_usb_extern) stop_usb_extern();
}