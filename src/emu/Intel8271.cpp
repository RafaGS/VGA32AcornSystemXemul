#include "Intel8271.h"

#include <string.h>

// Ver Intel8271.h para el mapa de registros y las ordenes.

enum { SR_SCANSEC = 0x06, SR_BAD0A = 0x10, SR_BAD0B = 0x11, SR_PCN0 = 0x12, SR_SCNT0 = 0x13, SR_SCNT1 = 0x14, SR_MODE = 0x17,
       SR_BAD1A = 0x18, SR_BAD1B = 0x19, SR_PCN1 = 0x1A, SR_DRVIN = 0x22, SR_DRVOUT = 0x23 };

void Intel8271::powerOn() {
    memset(pcn_, 0, sizeof pcn_);
    memset(bad_, 0xFF, sizeof bad_);
    scanSector_ = 0; scanCount_[0] = scanCount_[1] = 0; outPort_ = 0;
    // valores por defecto antes del primer SPECIFY (los mismos que el emulador de referencia)
    stepCycles_ = 12 * 2000; settleCycles_ = 10 * 2000; headLoadCycles_ = 8 * 8000; unloadRevs_ = 12;
    unknown_ = 0;
    cur_ = drives_[0]; unit_ = 0;
    reset();
}

void Intel8271::reset() {
    wait_ = 0; paramWait_ = 0;
    phase_ = P_IDLE; cls_ = C_INVALID;
    status_ = 0; result_ = 0; cmd_ = 0; nparams_ = need_ = 0;
    data_ = 0; idx_ = 0; track_ = sector_ = remaining_ = pending_ = 0;
    idByte_ = idCount_ = fmtSectors_ = 0;
    memset(params_, 0, sizeof params_);
    memset(buf_, 0, sizeof buf_);
    memset(fmtSeen_, 0, sizeof fmtSeen_);
    moder_ = 0xC0;                    // modo DMA hasta que el DOS escribe $C1 (no-DMA)
    headLoaded_ = false; unloadWait_ = 0;
    paramPending_ = false; pendingParam_ = 0;
    cur_ = drives_[0]; unit_ = 0;
    lastLine_ = false;
    updateLine();
}

uint8_t Intel8271::status() const {
    return (uint8_t)((status_ & ST_NDRQ) && !(moder_ & 1) ? (status_ & ~ST_NDRQ) : status_);   // NDRQ solo en modo no-DMA
}

void Intel8271::updateLine() {
    const bool l = lineLevel();
    if (l != lastLine_) { lastLine_ = l; if (host_) host_->line(l); }
}

void Intel8271::setNdrq(bool v) {
    if (v) status_ |= ST_NDRQ; else status_ &= (uint8_t)~ST_NDRQ;
    updateLine();
}

Intel8271::Class Intel8271::classify(uint8_t c, uint8_t* need) {
    switch (c) {
        case 0x13: case 0x17: *need = 3; return C_READ;
        case 0x12: case 0x16: *need = 2; return C_READ;
        case 0x0B: case 0x0F: *need = 3; return C_WRITE;
        case 0x0A: case 0x0E: *need = 2; return C_WRITE;
        case 0x1F: *need = 3; return C_VERIFY;
        case 0x1E: *need = 2; return C_VERIFY;
        case 0x1B: *need = 3; return C_READID;
        case 0x23: *need = 5; return C_FORMAT;
        case 0x29: *need = 1; return C_SEEK;
        case 0x2C: *need = 0; return C_DRVSTAT;
        case 0x35: *need = 4; return C_SPECIFY;
        case 0x3A: *need = 2; return C_WSPECIAL;
        case 0x3D: *need = 1; return C_RSPECIAL;
        case 0x00: case 0x04: *need = 5; return C_SCAN;
        default: *need = 0; return C_INVALID;
    }
}

uint8_t Intel8271::read(uint8_t reg) {
    reg &= 7;
    if (reg == 0) return status();
    if (reg == 1) {
        const uint8_t r = result_;
        status_ &= (uint8_t)~(ST_RESFULL | ST_INT);
        updateLine();
        return r;
    }
    if (reg >= 4) {
        if (phase_ == P_XFER_READ || phase_ == P_ID_BYTES) setNdrq(false);
        return data_;
    }
    return 0xFF;
}

void Intel8271::write(uint8_t reg, uint8_t v) {
    reg &= 7;
    if (reg == 2) {                    // RESET
        if (v & 1) reset();
        return;
    }
    if (reg == 0) {                    // orden
        if (phase_ != P_IDLE) return;  // con BUSY la orden se ignora
        cmd_ = v;
        nparams_ = 0;
        cls_ = classify(v & 0x3F, &need_);
        status_ |= ST_BUSY;
        status_ &= (uint8_t)~(ST_RESFULL | ST_INT);
        updateLine();
        if (cls_ == C_INVALID) {       // orden no valida: resultado de error con interrupcion
            unknown_++;
            phase_ = P_FINISH_WAIT; pending_ = RES_NOT_FOUND; wait_ = kParamAcceptCycles;
            return;
        }
        phase_ = P_PARAMS;
        if (need_ == 0) startCommand();
        return;
    }
    if (reg == 1) {                    // parametro
        if (phase_ != P_PARAMS || (status_ & ST_PARAMFULL)) return;
        status_ |= ST_PARAMFULL;
        paramPending_ = true; pendingParam_ = v;
        paramWait_ = kParamAcceptCycles;
        return;
    }
    if (reg >= 4) {                    // dato
        data_ = v;
        if (phase_ == P_XFER_WRITE && (status_ & ST_NDRQ) && idx_ < sizeof buf_) {
            buf_[idx_++] = v;
            setNdrq(false);
        } else if (phase_ == P_FORMAT_BYTES && (status_ & ST_NDRQ)) {
            idByte_ = v;               // byte de ID (C,H,R,N) en data_; lo procesa onTimeout
            setNdrq(false);
        }
    }
}

void Intel8271::acceptParam() {
    paramPending_ = false;
    params_[nparams_++] = pendingParam_;
    status_ &= (uint8_t)~ST_PARAMFULL;
    if (nparams_ == need_) startCommand();
}

void Intel8271::finish(uint8_t result, bool interrupt) {
    phase_ = P_IDLE;
    wait_ = 0;
    result_ = result;
    status_ &= (uint8_t)~(ST_BUSY | ST_NDRQ);
    status_ |= ST_RESFULL;
    if (interrupt) status_ |= ST_INT;
    if (headLoaded_) unloadWait_ = unloadRevs_ * FloppyDrive::kRevolutionCycles;   // descarga el cabezal tras un rato inactivo
    updateLine();
}

// Registros especiales (mapa del emulador de referencia / datasheet).
uint8_t Intel8271::readSpecial(uint8_t r) const {
    switch (r) {
        case SR_SCANSEC: return scanSector_;
        case SR_BAD0A: return bad_[0][0];
        case SR_BAD0B: return bad_[0][1];
        case SR_PCN0: return pcn_[0];
        case SR_SCNT0: return scanCount_[0];
        case SR_SCNT1: return scanCount_[1];
        case SR_MODE: return moder_;
        case SR_BAD1A: return bad_[1][0];
        case SR_BAD1B: return bad_[1][1];
        case SR_PCN1: return pcn_[1];
        case SR_DRVIN: return driveStatus();
        case SR_DRVOUT: return (uint8_t)((cmd_ & 0xC0) | outPort_);
        default: return 0;
    }
}

void Intel8271::writeSpecial(uint8_t r, uint8_t v) {
    switch (r) {
        case SR_SCANSEC: scanSector_ = v; break;
        case SR_BAD0A: bad_[0][0] = v; break;
        case SR_BAD0B: bad_[0][1] = v; break;
        case SR_PCN0: pcn_[0] = v; break;
        case SR_SCNT0: scanCount_[0] = v; break;
        case SR_SCNT1: scanCount_[1] = v; break;
        case SR_MODE: moder_ = (uint8_t)(v | 0xC0); updateLine(); break;
        case SR_BAD1A: bad_[1][0] = v; break;
        case SR_BAD1B: bad_[1][1] = v; break;
        case SR_PCN1: pcn_[1] = v; break;
        case SR_DRVOUT:
            outPort_ = (uint8_t)(v & 0x3F);
            headLoaded_ = (v & 0x08) != 0;                     // bit 3 = cargar cabezal
            if (headLoaded_) unloadWait_ = 0;
            if (v & 0x02) drv().step((v & 0x04) != 0);         // pulso de paso manual (bit 2 = hacia adentro)
            break;
        default: break;                                        // $22 es de solo lectura
    }
}

// READ DRIVE STATUS: bit 2 RDY0, bit 6 RDY1 (confirmado por el DOS, rutina $F796: para la unidad 1 desplaza el resultado
// 4 bits y prueba AND #$04), bit 1 pista 0 y bit 3 protegida (o sin disco) de la unidad seleccionada, bit 4 pulso de indice.
uint8_t Intel8271::driveStatus() const {
    uint8_t v = 0;
    if (drives_[1] && drives_[1]->ready()) v |= 0x40;
    if (drives_[0] && drives_[0]->ready()) v |= 0x04;
    const FloppyDrive* sd = cur_;
    if (sd) {
        if (!sd->hasDisk() || sd->writeProtected()) v |= 0x08;
        if (sd->atTrack0()) v |= 0x02;
    }
    if (headLoaded_ && (now_ % FloppyDrive::kRevolutionCycles) < 4000) v |= 0x10;
    return v;
}

void Intel8271::startCommand() {
    const uint8_t sel = cmd_ >> 6;
    const int unit = sel == 1 ? 0 : sel == 2 ? 1 : -1;
    FloppyDrive* sd = unit >= 0 ? drives_[unit] : 0;          // unidad seleccionada (nula si no existe)
    if (sd) { cur_ = sd; unit_ = (uint8_t)unit; }
    const bool needsDrive = cls_ == C_READ || cls_ == C_WRITE || cls_ == C_VERIFY || cls_ == C_READID || cls_ == C_FORMAT || cls_ == C_SEEK;
    // SEEK solo necesita la unidad (se mueve aunque no haya disco); el resto exige disco (READY).
    if (needsDrive && (!sd || (cls_ != C_SEEK && !sd->ready()))) {
        phase_ = P_FINISH_WAIT; pending_ = RES_NOT_READY; wait_ = kParamAcceptCycles;
        return;
    }
    if ((cls_ == C_READ || cls_ == C_WRITE || cls_ == C_VERIFY) && need_ == 2) params_[2] = 0x21;   // variante de un registro
    // Carga de cabezal: si estaba descargado, la primera orden con movimiento o transferencia espera el tiempo de carga.
    uint32_t pre = 0;
    if (needsDrive) { pre = headLoaded_ ? 0 : (headLoadCycles_ ? headLoadCycles_ : 10); headLoaded_ = true; unloadWait_ = 0; }
    if (pre) { phase_ = P_HEADLOAD; wait_ = pre; return; }       // la posicion de rotacion se calcula DESPUES de cargar el cabezal
    execCommand();
}

void Intel8271::execCommand() {
    switch (cls_) {
        case C_SPECIFY:
            if (params_[0] == 0x0D) {
                // p1 = ritmo de paso (x2 ms), p2 = asentamiento (x2 ms), p3 = descarga (nibble alto) / carga (nibble bajo, x8 ms)
                stepCycles_ = (uint32_t)params_[1] * 2000u;
                settleCycles_ = (uint32_t)params_[2] * 2000u;
                headLoadCycles_ = (uint32_t)(params_[3] & 0x0F) * 8000u;
                const uint32_t u = (uint32_t)(params_[3] >> 4) * 4u;
                unloadRevs_ = u ? (u < 4 ? 4 : u) : 12;
            } else if (params_[0] == 0x10) { bad_[0][0] = params_[1]; bad_[0][1] = params_[2]; pcn_[0] = params_[3]; }
            else if (params_[0] == 0x18) { bad_[1][0] = params_[1]; bad_[1][1] = params_[2]; pcn_[1] = params_[3]; }
            finish(RES_OK, false);
            status_ &= (uint8_t)~ST_RESFULL;        // SPECIFY no devuelve resultado
            break;
        case C_WSPECIAL:
            writeSpecial(params_[0], params_[1]);
            finish(RES_OK, false);
            status_ &= (uint8_t)~ST_RESFULL;
            break;
        case C_RSPECIAL:
            finish(readSpecial(params_[0]), false);
            break;
        case C_DRVSTAT:
            finish(driveStatus(), false);
            break;
        case C_SCAN:                                // no implementado
            phase_ = P_FINISH_WAIT; pending_ = RES_NOT_FOUND; wait_ = kParamAcceptCycles;
            break;
        case C_SEEK: {
            const uint8_t target = params_[0];
            uint32_t n = 0;
            if (target == 0) {
                // SEEK 0 = recalibrado: la cabeza retrocede hasta el sensor de pista 0 (sin mirar el registro de pista).
                while (!drv().atTrack0() && n < 255) { drv().step(false); n++; }
            } else {
                const int delta = (int)target - (int)trackReg();
                n = (uint32_t)(delta < 0 ? -delta : delta);
                // Mueve la cabeza 'delta' pasos desde donde esta fisicamente (el registro solo guarda lo que el chip cree).
                for (uint32_t i = 0; i < n; i++) if (!drv().step(delta > 0)) break;
            }
            trackReg() = target;
            wait_ = n ? n * stepCycles_ + settleCycles_ : kParamAcceptCycles;
            phase_ = P_SEEK;
            break;
        }
        case C_READ: case C_VERIFY: case C_WRITE:
            track_ = params_[0]; sector_ = params_[1];
            remaining_ = (uint8_t)(params_[2] & 0x1F);
            if (remaining_ == 0) remaining_ = 1;
            if ((params_[2] >> 5) != 1) { phase_ = P_FINISH_WAIT; pending_ = RES_NOT_FOUND; wait_ = kParamAcceptCycles; break; }   // solo N=1 (256 B)
            if (cls_ == C_WRITE && drv().writeProtected()) { phase_ = P_FINISH_WAIT; pending_ = RES_WRITE_PROTECT; wait_ = kParamAcceptCycles; break; }
            startSearch();
            break;
        case C_READID:
            track_ = params_[0];
            idCount_ = params_[2] ? params_[2] : 1;
            idByte_ = 0;
            if (!implicitSeek()) beginReadId();
            break;
        case C_FORMAT:
            track_ = params_[0];
            fmtSectors_ = (uint8_t)(params_[2] & 0x1F);
            if (drv().writeProtected()) { phase_ = P_FINISH_WAIT; pending_ = RES_WRITE_PROTECT; wait_ = kParamAcceptCycles; break; }
            memset(fmtSeen_, 0, sizeof fmtSeen_);
            idx_ = 0; idByte_ = 0; pending_ = RES_OK;
            if (!implicitSeek()) beginFormat();
            break;
        default: break;
    }
}

// El 8271 incluye un posicionamiento implicito en READ/WRITE/VERIFY/READ ID/FORMAT:
// si el registro de pista actual difiere de la pista pedida, mueve la cabeza (el
// DOS de Acorn lo usa: p. ej. WRITE DATA en la pista 25 sin SEEK previo).
bool Intel8271::implicitSeek() {
    if (trackReg() == track_) return false;
    const int delta = (int)track_ - (int)trackReg();
    const uint32_t n = (uint32_t)(delta < 0 ? -delta : delta);
    for (uint32_t i = 0; i < n; i++) if (!drv().step(delta > 0)) break;
    trackReg() = track_;
    wait_ = n * stepCycles_ + settleCycles_;
    if (wait_ == 0) wait_ = 1;
    phase_ = P_ISEEK;
    return true;
}

void Intel8271::beginReadId() {
    const uint8_t s = drv().nextSlot(now_);
    sector_ = s; wait_ = drv().cyclesToSlotStart(now_, s) + kByteCycles; phase_ = P_ID_SEARCH;
}

void Intel8271::beginFormat() {
    wait_ = drv().cyclesToIndex(now_);
    if (wait_ == 0) wait_ = 1;                      // un timer a 0 no dispara
    phase_ = P_FORMAT_WAIT;
}

void Intel8271::startSearch() {
    if (!implicitSeek()) locate();
}

// Localiza el sector 'sector_' de la pista 'track_': el ID debe coincidir con la pista fisica; si no aparece en 2 vueltas
// -> SECTOR NOT FOUND.
void Intel8271::locate() {
    if (track_ != drv().track() || sector_ >= drv().sectorsPerTrack() || !drv().formatted(drv().track())) {
        phase_ = P_NOTFOUND;
        wait_ = 2 * FloppyDrive::kRevolutionCycles;
        return;
    }
    const uint8_t slot = drv().slotOfSector(sector_);
    wait_ = drv().cyclesToSlotStart(now_, slot) + kHeaderBytes * kByteCycles;
    phase_ = P_SEARCH;
}

void Intel8271::searchDone() {
    if (drv().faultFor(track_, sector_) == FloppyDrive::FAULT_ID_CRC) { finish(RES_ID_CRC, true); return; }
    idx_ = 0;
    if (cls_ == C_WRITE) {
        setNdrq(true);                           // pide el primer byte
        phase_ = P_XFER_WRITE; wait_ = kByteCycles;
        return;
    }
    if (drv().readSector(sector_, buf_) != FloppyDrive::IO_OK) { finish(RES_NOT_FOUND, true); return; }
    if (cls_ == C_VERIFY) { phase_ = P_VERIFY; wait_ = (uint32_t)sizeof buf_ * kByteCycles; return; }
    data_ = buf_[idx_++];                        // primer byte disponible ya
    setNdrq(true);
    phase_ = P_XFER_READ; wait_ = kByteCycles;
}

void Intel8271::sectorDone() {
    if (cls_ != C_WRITE && drv().faultFor(track_, sector_) == FloppyDrive::FAULT_DATA_CRC) { finish(RES_DATA_CRC, true); return; }
    if (cls_ == C_WRITE) {
        const FloppyDrive::IoResult r = drv().writeSector(sector_, buf_);
        if (r == FloppyDrive::IO_WRITE_PROTECT) { finish(RES_WRITE_PROTECT, true); return; }
        if (r != FloppyDrive::IO_OK) { finish(r == FloppyDrive::IO_NOT_FOUND ? RES_NOT_FOUND : RES_WRITE_FAULT, true); return; }
    }
    if (--remaining_ == 0) { finish(RES_OK, true); return; }
    sector_++;
    startSearch();
}

void Intel8271::idNext() {
    // entrega los 4 bytes C,H,R,N del hueco 'sector_' (idByte_ = 0..3)
    const uint8_t slot = (uint8_t)sector_;
    const uint8_t s = drv().sectorOfSlot(slot);
    const uint8_t v[4] = { drv().track(), 0, s, 1 };
    data_ = v[idByte_];
}

void Intel8271::onTimeout() {
    switch (phase_) {
        case P_FINISH_WAIT: finish(pending_, true); break;
        case P_HEADLOAD: execCommand(); break;
        case P_SEEK:
            if (drv().track() != trackReg()) finish(trackReg() == 0 ? RES_TRACK0 : RES_NOT_FOUND, true);
            else finish(RES_OK, true);
            break;
        case P_ISEEK:
            if (!drv().ready()) { finish(RES_NOT_READY, true); break; }
            switch (cls_) {
                case C_READID: beginReadId(); break;
                case C_FORMAT: beginFormat(); break;
                default: locate(); break;
            }
            break;
        case P_NOTFOUND: finish(RES_NOT_FOUND, true); break;
        case P_SEARCH: searchDone(); break;
        case P_VERIFY: sectorDone(); break;
        case P_XFER_READ:
            if (status_ & ST_NDRQ) { finish(RES_LATE_DMA, true); break; }   // el byte anterior no se leyo
            if (idx_ < sizeof buf_) { data_ = buf_[idx_++]; setNdrq(true); wait_ = kByteCycles; }
            else sectorDone();
            break;
        case P_XFER_WRITE:
            if (idx_ >= sizeof buf_) { sectorDone(); break; }
            if (status_ & ST_NDRQ) { finish(RES_LATE_DMA, true); break; }   // la CPU no dio el byte
            setNdrq(true); wait_ = kByteCycles;
            break;
        case P_ID_SEARCH:
            idByte_ = 0; idNext(); setNdrq(true); phase_ = P_ID_BYTES; wait_ = kByteCycles;
            break;
        case P_ID_BYTES:
            if (status_ & ST_NDRQ) { finish(RES_LATE_DMA, true); break; }
            if (++idByte_ < 4) { idNext(); setNdrq(true); wait_ = kByteCycles; break; }
            if (--idCount_ == 0) { finish(RES_OK, true); break; }
            sector_ = (uint8_t)((sector_ + 1) % drv().sectorsPerTrack());
            wait_ = drv().cyclesToSlotStart(now_, (uint8_t)sector_) + kByteCycles;
            phase_ = P_ID_SEARCH;
            break;
        case P_FORMAT_WAIT:
            // pasa el indice: empieza a pedir los 4 bytes de ID de cada sector
            idx_ = 0; setNdrq(true); phase_ = P_FORMAT_BYTES; wait_ = kByteCycles;
            break;
        case P_FORMAT_BYTES: {
            if (status_ & ST_NDRQ) { finish(RES_LATE_DMA, true); break; }
            // idByte_ contiene el byte recien escrito por la CPU; idx_ cuenta bytes (4 por sector)
            const uint8_t field = (uint8_t)(idx_ & 3);
            if (field == 0 && idByte_ != track_) pending_ = RES_NOT_FOUND;
            if (field == 1 && idByte_ != 0) pending_ = RES_NOT_FOUND;
            if (field == 2) { const uint8_t r = idByte_; if (r >= drv().sectorsPerTrack() || (fmtSeen_[r >> 3] >> (r & 7) & 1)) pending_ = RES_NOT_FOUND; else fmtSeen_[r >> 3] |= (uint8_t)(1 << (r & 7)); }
            if (field == 3 && idByte_ != 1) pending_ = RES_NOT_FOUND;
            idx_++;
            if (idx_ < (uint16_t)fmtSectors_ * 4) { setNdrq(true); wait_ = kByteCycles; break; }
            if (fmtSectors_ != drv().sectorsPerTrack() || track_ != drv().track()) pending_ = RES_NOT_FOUND;
            if (pending_ == RES_OK && drv().formatTrack(0xE5) != FloppyDrive::IO_OK) pending_ = RES_WRITE_FAULT;
            phase_ = P_FORMAT_END;
            wait_ = drv().cyclesToIndex(now_);               // termina al llegar el siguiente indice
            if (wait_ == 0) wait_ = FloppyDrive::kRevolutionCycles;
            break;
        }
        case P_FORMAT_END: finish(pending_, true); break;
        default: break;
    }
}

void Intel8271::tick(uint32_t cycles) {
    while (cycles) {
        uint32_t step = cycles;
        if (paramPending_ && paramWait_ < step) step = paramWait_;
        if (wait_ && phase_ != P_IDLE && phase_ != P_PARAMS && wait_ < step) step = wait_;
        if (unloadWait_ && unloadWait_ < step) step = unloadWait_;
        if (step == 0) step = 1;
        now_ += step;
        cycles -= step;
        // Primero se descuenta el temporizador que YA existia al empezar el paso; despues se aceptan parametros (que pueden
        // arrancar una orden con un temporizador nuevo y corto: descontarle el paso anterior lo desbordaria).
        if (phase_ != P_IDLE && phase_ != P_PARAMS && wait_) {
            wait_ -= step;
            if (wait_ == 0) onTimeout();
        }
        if (paramPending_) {
            paramWait_ -= (step < paramWait_ ? step : paramWait_);
            if (paramWait_ == 0) acceptParam();
        }
        if (unloadWait_) {
            unloadWait_ -= (step < unloadWait_ ? step : unloadWait_);
            if (unloadWait_ == 0 && phase_ == P_IDLE) headLoaded_ = false;   // descarga el cabezal tras un rato inactivo
        }
    }
}
