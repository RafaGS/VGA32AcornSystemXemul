#include "DiskImage.h"

#include <string.h>
#if defined(ARDUINO_ARCH_ESP32) || defined(__unix__) || defined(__APPLE__)
#include <unistd.h>
#define S3_HAVE_FSYNC 1
#endif

bool MemoryDiskImage::read(uint32_t off, uint8_t* dst, uint16_t n) {
    if ((uint64_t)off + n > data_.size()) return false;
    memcpy(dst, &data_[off], n);
    return true;
}

bool MemoryDiskImage::write(uint32_t off, const uint8_t* src, uint16_t n) {
    if (ro_ || (uint64_t)off + n > data_.size()) return false;
    memcpy(&data_[off], src, n);
    return true;
}

bool FileDiskImage::open(const char* path) {
    close();
    ro_ = false;
    f_ = fopen(path, "r+b");
    if (!f_) { f_ = fopen(path, "rb"); ro_ = true; }
    if (!f_) return false;
    fseek(f_, 0, SEEK_END);
    size_ = (uint32_t)ftell(f_);
    return true;
}

void FileDiskImage::close() {
    if (f_) fclose(f_);
    f_ = 0;
    size_ = 0;
}

bool FileDiskImage::read(uint32_t off, uint8_t* dst, uint16_t n) {
    if (!f_ || (uint64_t)off + n > size_) return false;
    if (fseek(f_, (long)off, SEEK_SET) != 0) return false;
    return fread(dst, 1, n, f_) == n;
}

bool FileDiskImage::write(uint32_t off, const uint8_t* src, uint16_t n) {
    if (!f_ || ro_ || (uint64_t)off + n > size_) return false;
    if (fseek(f_, (long)off, SEEK_SET) != 0) return false;
    if (fwrite(src, 1, n, f_) != n) return false;
    return fflush(f_) == 0;
}

void FileDiskImage::flush() {
    if (!f_) return;
    fflush(f_);
#ifdef S3_HAVE_FSYNC
    fsync(fileno(f_));
#endif
}

bool FileDiskImage::grow(uint32_t newSize, uint8_t fill) {
    if (!f_ || ro_ || newSize < size_) return false;
    if (fseek(f_, (long)size_, SEEK_SET) != 0) return false;
    uint8_t buf[256];
    memset(buf, fill, sizeof buf);
    uint32_t left = newSize - size_;
    while (left) {
        const uint32_t n = left > sizeof buf ? (uint32_t)sizeof buf : left;
        if (fwrite(buf, 1, n, f_) != n) return false;
        left -= n;
    }
    if (fflush(f_) != 0) return false;
    size_ = newSize;
    return true;
}
