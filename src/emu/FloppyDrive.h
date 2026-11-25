#pragma once

#include <stdint.h>

#include "DiskImage.h"

// Unidad de disquete de 5 1/4" (una cara) tal como la ve la controladora 8271.
//
// Modelo (alineado con el emulador de referencia y con la descripcion del DOS):
//  * La unidad FISICA tiene 80 pistas (la cabeza se mueve entre 0 y 79); la IMAGEN tiene entre 1 y 80 pistas segun
//    su tamano (multiplo de 2.560 bytes: 40 pistas = 102.400, 35 = 89.600, 80 = 204.800). Leer una pista que la imagen
//    no tiene da SECTOR NOT FOUND, y FORMAT de una pista posterior al final hace crecer la imagen.
//  * El disco gira siempre a 300 RPM mientras haya soporte (el motor no se modela: READY = hay disco). Una vuelta =
//    200 ms = 200.000 ciclos de 1 MHz.
//  * Cada pista tiene 'sectorsPerTrack' huecos de sector de la misma duracion; el sector logico s de la pista t ocupa
//    el hueco (s + skew*t) mod spt. skew/intercalado son configurables (por defecto 0).
//  * La imagen es plana: sector (pista, s) en el desplazamiento (pista*spt + s)*sectorSize. Todos los IDs de una pista
//    son (C=pista, H=0, R=s, N=1), de modo que FORMAT solo reescribe el contenido y marca la pista como formateada.
class FloppyDrive {
public:
    enum Fault : uint8_t { FAULT_NONE, FAULT_ID_CRC, FAULT_DATA_CRC };   // inyeccion para pruebas
    enum InsertResult : uint8_t { INSERT_OK, INSERT_BAD_SIZE, INSERT_IO };
    enum IoResult : uint8_t { IO_OK, IO_NOT_READY, IO_NOT_FOUND, IO_WRITE_PROTECT, IO_ERROR };

    static const uint32_t kRevolutionCycles = 200000;   // 300 RPM a 1 MHz
    static const uint8_t kPhysicalTracks = 80;

    FloppyDrive();

    // Geometria de pista (solo cambia sin disco insertado). Las pistas de la imagen salen de su tamano al insertar.
    bool setGeometry(uint8_t tracks, uint8_t sectorsPerTrack, uint16_t sectorSize = 256);
    uint8_t tracks() const { return tracks_; }               // pistas de la IMAGEN
    uint8_t sectorsPerTrack() const { return spt_; }
    uint16_t sectorSize() const { return ssize_; }
    uint32_t imageBytes() const { return (uint32_t)tracks_ * spt_ * ssize_; }
    void setSkew(uint8_t s) { skew_ = s; }

    // Soporte. El tamano debe ser multiplo de una pista y de 1 a 80 pistas (si no, INSERT_BAD_SIZE).
    // 'formatted' = false deja todas las pistas sin formato.
    InsertResult insert(DiskImage* img, bool writeProtect = false, bool formatted = true);
    InsertResult insertAuto(DiskImage* img, bool writeProtect = false) { return insert(img, writeProtect, true); }
    void eject() { sync(); img_ = 0; }
    bool hasDisk() const { return img_ != 0; }
    bool ready() const { return img_ != 0; }
    bool writeProtected() const { return img_ && (wp_ || img_->readOnly()); }
    void setWriteProtect(bool wp) { wp_ = wp; }

    // Cabeza
    uint8_t track() const { return head_; }
    bool atTrack0() const { return head_ == 0; }
    bool step(bool inward);              // false si topa con el tope (pista 0 o la ultima fisica)
    void recalibrate() { head_ = 0; }

    // Pistas con formato
    bool formatted(uint8_t track) const { return track < tracks_ && !(unfmt_[track >> 6] >> (track & 63) & 1); }
    void setFormatted(uint8_t track, bool f);

    // Rotacion
    uint32_t slotCycles() const { return kRevolutionCycles / spt_; }
    uint8_t slotOfSector(uint8_t sector) const { return (uint8_t)((sector + (uint16_t)skew_ * head_) % spt_); }
    uint8_t sectorOfSlot(uint8_t slot) const { return (uint8_t)((slot + spt_ * 8u - (uint16_t)skew_ * head_ % spt_) % spt_); }
    uint32_t cyclesToSlotStart(uint32_t now, uint8_t slot) const;   // 0 si empieza ahora
    uint8_t nextSlot(uint32_t now) const;                           // primer hueco que empieza en >= now
    uint32_t cyclesToIndex(uint32_t now) const;

    // Acceso a sectores en la pista actual (la controladora ya ha comprobado el ID).
    IoResult readSector(uint8_t sector, uint8_t* buf);
    IoResult writeSector(uint8_t sector, const uint8_t* buf);
    IoResult formatTrack(uint8_t fill);    // si la pista esta fuera de la imagen, esta crece hasta incluirla

    // Escrituras pendientes de forzar al medio: el firmware llama a sync() con la 8271 inactiva.
    bool dirty() const { return dirty_; }
    void sync() { if (img_ && dirty_) img_->flush(); dirty_ = false; }

    void setFault(uint8_t track, uint8_t sector, Fault f) { ftrack_ = track; fsector_ = sector; fault_ = f; }
    Fault faultFor(uint8_t track, uint8_t sector) const {
        return (fault_ != FAULT_NONE && ftrack_ == track && fsector_ == sector) ? fault_ : FAULT_NONE;
    }

private:
    uint32_t offset(uint8_t track, uint8_t sector) const { return ((uint32_t)track * spt_ + sector) * ssize_; }

    DiskImage* img_;
    uint8_t tracks_, spt_, skew_, head_;
    uint16_t ssize_;
    bool wp_, dirty_;
    uint64_t unfmt_[2];                    // bit a 1 = pista sin formato (0-127)
    Fault fault_;
    uint8_t ftrack_, fsector_;
};
