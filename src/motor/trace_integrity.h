#pragma once
#include <stddef.h>
#include <stdint.h>

// IEEE CRC-32, identical to Python zlib.crc32(payload.encode('ascii')).
inline uint32_t traceCrc32(const char* data, size_t length) {
    uint32_t crc = 0xffffffffU;
    for (size_t i=0; i<length; ++i) {
        crc ^= uint8_t(data[i]);
        for (unsigned bit=0; bit<8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}
