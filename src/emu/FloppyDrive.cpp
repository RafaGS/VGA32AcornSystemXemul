#include "FloppyDrive.h"

#include <string.h>

FloppyDrive::FloppyDrive()
    : img_(0), tracks_(40), spt_(10), skew_(0), head_(0), ssize_(256), wp_(false), dirty_(false),
      fault_(FAULT_NONE), ftrack_(0), fsector_(0) { unfmt_[0] = unfmt_[1] = 0; }

bool FloppyDrive::setGeometry(uint8_t tracks, uint8_t spt, uint16_t ssize) {
    if (img_ || tracks == 0 || tracks > kPhysicalTracks || spt == 0 || ssize != 256) return false;   // el 8271 de esta fase usa N=1
    tracks_ = tracks; spt_ = spt; ssize_ = ssize;
    return true;
}

FloppyDrive::InsertResult FloppyDrive::insert(DiskImage* img, bool wp, bool formatted) {
    if (!img) return INSERT_BAD_SIZE;
    const uint32_t track = (uint32_t)spt_ * ssize_;
    const uint32_t sz = img->size();
    if (sz == 0 || sz % track != 0 || sz / track > kPhysicalTracks) return INSERT_BAD_SIZE;
    sync();
    tracks_ = (uint8_t)(sz / track);
    img_ = img;
    wp_ = wp;
    unfmt_[0] = unfmt_[1] = formatted ? 0 : ~(uint64_t)0;
    fault_ = FAULT_NONE;
    dirty_ = false;
    return INSERT_OK;
}

bool FloppyDrive::step(bool inward) {
    if (inward) {
        if (head_ + 1 >= kPhysicalTracks) return false;
        head_++;
    } else {
        if (head_ == 0) return false;
        head_--;
    }
    return true;
}

void FloppyDrive::setFormatted(uint8_t track, bool f) {
    if (track >= 128) return;
    if (f) unfmt_[track >> 6] &= ~((uint64_t)1 << (track & 63)); else unfmt_[track >> 6] |= (uint64_t)1 << (track & 63);
}

uint32_t FloppyDrive::cyclesToIndex(uint32_t now) const {
    const uint32_t ph = now % kRevolutionCycles;
    return ph == 0 ? 0 : kRevolutionCycles - ph;
}

uint32_t FloppyDrive::cyclesToSlotStart(uint32_t now, uint8_t slot) const {
    const uint32_t ph = now % kRevolutionCycles;
    const uint32_t start = (uint32_t)slot * slotCycles();
    return start >= ph ? start - ph : start + kRevolutionCycles - ph;
}

uint8_t FloppyDrive::nextSlot(uint32_t now) const {
    const uint32_t ph = now % kRevolutionCycles;
    const uint32_t sc = slotCycles();
    return (uint8_t)(((ph + sc - 1) / sc) % spt_);
}

FloppyDrive::IoResult FloppyDrive::readSector(uint8_t sector, uint8_t* buf) {
    if (!img_) return IO_NOT_READY;
    if (sector >= spt_ || !formatted(head_)) return IO_NOT_FOUND;
    return img_->read(offset(head_, sector), buf, ssize_) ? IO_OK : IO_ERROR;
}

FloppyDrive::IoResult FloppyDrive::writeSector(uint8_t sector, const uint8_t* buf) {
    if (!img_) return IO_NOT_READY;
    if (writeProtected()) return IO_WRITE_PROTECT;
    if (sector >= spt_ || !formatted(head_)) return IO_NOT_FOUND;
    if (!img_->write(offset(head_, sector), buf, ssize_)) return IO_ERROR;
    dirty_ = true;
    return IO_OK;
}

FloppyDrive::IoResult FloppyDrive::formatTrack(uint8_t fill) {
    if (!img_) return IO_NOT_READY;
    if (writeProtected()) return IO_WRITE_PROTECT;
    if (head_ >= tracks_) {                                   // pista fuera de la imagen: la imagen crece hasta incluirla
        const uint32_t newSize = (uint32_t)(head_ + 1) * spt_ * ssize_;
        if (!img_->grow(newSize, fill)) return IO_ERROR;
        for (uint8_t t = tracks_; t <= head_; t++) setFormatted(t, true);
        tracks_ = (uint8_t)(head_ + 1);
    }
    uint8_t buf[256];
    memset(buf, fill, sizeof buf);
    for (uint8_t s = 0; s < spt_; s++)
        if (!img_->write(offset(head_, s), buf, ssize_)) return IO_ERROR;
    setFormatted(head_, true);
    dirty_ = true;
    return IO_OK;
}
