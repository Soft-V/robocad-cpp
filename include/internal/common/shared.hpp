#pragma once

#include <string>
#include <vector>
#include <cstdint>

class LibHolder {
public:
    explicit LibHolder(const std::string& first_path);
    ~LibHolder();

    LibHolder(const LibHolder&) = delete;
    LibHolder& operator=(const LibHolder&) = delete;

    int init_spi(const std::string& path, int channel, int speed, int mode);
    int init_usb(const std::string& path, int baud);

    uint8_t* rw_spi(uint8_t* data, unsigned int len);
    uint8_t* rw_usb(uint8_t* data, unsigned int len);

    void stop_spi();
    void stop_usb();

private:
    void* handle;

    using StartSPIFunc = int (*)(const char*, int, int, int);
    using StartUSBFunc = int (*)(const char*, int);
    using ReadWriteSPIFunc = unsigned char* (*)(unsigned char*, unsigned int);
    using ReadWriteUSBFunc = unsigned char* (*)(unsigned char*, unsigned int);
    using StopSPIFunc = void (*)();
    using StopUSBFunc = void (*)();

    StartSPIFunc start_spi_extern = nullptr;
    StartUSBFunc start_usb_extern = nullptr;
    ReadWriteSPIFunc read_write_spi_extern = nullptr;
    ReadWriteUSBFunc read_write_usb_extern = nullptr;
    StopSPIFunc stop_spi_extern = nullptr;
    StopUSBFunc stop_usb_extern = nullptr;
};