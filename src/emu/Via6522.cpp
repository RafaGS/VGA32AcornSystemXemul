#include "Via6522.h"

// Registros: 0 ORB 1 ORA 2 DDRB 3 DDRA 4 T1C-L 5 T1C-H 6 T1L-L 7 T1L-H 8 T2C-L 9 T2C-H
//            A SR  B ACR  C PCR  D IFR  E IER  F ORA (sin enclavamiento)
// T1: al escribir T1C-H se carga el contador con el latch y se arranca. Cuenta N..0, la
// transicion 0 -> $FFFF activa IFR6 (N+1 ciclos tras cargar) y en modo libre (ACR bit 6)
// el siguiente ciclo recarga el latch: periodo N+2 ciclos. En un disparo la bandera solo
// se activa una vez por carga.
void Via6522::reset() {
    orb_ = ora_ = ddrb_ = ddra_ = acr_ = pcr_ = sr_ = ifr_ = ier_ = 0;
    t1ll_ = t1lh_ = t2ll_ = 0xFF;
    t1c_ = t2c_ = 0xFFFF;
    t1Running_ = t1Under_ = t1Armed_ = t2Running_ = t2Armed_ = false;
    lastIrq_ = false;
    if (host_) host_->irq(false);
}

void Via6522::updateIrq() {
    const bool a = irqActive();
    if (a != lastIrq_) { lastIrq_ = a; if (host_) host_->irq(a); }
}

uint8_t Via6522::read(uint8_t reg) {
    switch (reg & 0x0F) {
        case 0x0: return (uint8_t)((orb_ & ddrb_) | ((host_ ? host_->readPortB() : 0xFF) & ~ddrb_));
        case 0x1: case 0xF: return (uint8_t)((ora_ & ddra_) | ((host_ ? host_->readPortA() : 0xFF) & ~ddra_));
        case 0x2: return ddrb_;
        case 0x3: return ddra_;
        case 0x4: ifr_ &= (uint8_t)~IFR_T1; updateIrq(); return (uint8_t)(t1c_ & 0xFF);
        case 0x5: return (uint8_t)(t1c_ >> 8);
        case 0x6: return t1ll_;
        case 0x7: return t1lh_;
        case 0x8: ifr_ &= (uint8_t)~IFR_T2; updateIrq(); return (uint8_t)(t2c_ & 0xFF);
        case 0x9: return (uint8_t)(t2c_ >> 8);
        case 0xA: return sr_;
        case 0xB: return acr_;
        case 0xC: return pcr_;
        case 0xD: return ifr();
        default:  return (uint8_t)(ier_ | 0x80);          // 0xE
    }
}

void Via6522::write(uint8_t reg, uint8_t v) {
    switch (reg & 0x0F) {
        case 0x0: orb_ = v; break;
        case 0x1: case 0xF: ora_ = v; break;
        case 0x2: ddrb_ = v; break;
        case 0x3: ddra_ = v; break;
        case 0x4: case 0x6: t1ll_ = v; break;
        case 0x5:
            t1lh_ = v; t1c_ = (uint16_t)(t1ll_ | (v << 8));
            ifr_ &= (uint8_t)~IFR_T1; t1Running_ = true; t1Under_ = false; t1Armed_ = true;
            updateIrq();
            break;
        case 0x7: t1lh_ = v; ifr_ &= (uint8_t)~IFR_T1; updateIrq(); break;
        case 0x8: t2ll_ = v; break;
        case 0x9:
            t2c_ = (uint16_t)(t2ll_ | (v << 8)); ifr_ &= (uint8_t)~IFR_T2; t2Running_ = true; t2Armed_ = true;
            updateIrq();
            break;
        case 0xA: sr_ = v; break;
        case 0xB: acr_ = v; break;
        case 0xC: pcr_ = v; break;
        case 0xD: ifr_ &= (uint8_t)~(v & 0x7F); updateIrq(); break;       // los 1 borran banderas
        default:  // 0xE
            if (v & 0x80) ier_ |= (uint8_t)(v & 0x7F); else ier_ &= (uint8_t)~(v & 0x7F);
            updateIrq();
            break;
    }
}

void Via6522::tick(uint32_t cycles) {
    while (cycles) {
        // T2 (un disparo): cuenta hacia atras sin parar; la bandera se activa una vez en 0 -> $FFFF
        if (t2Running_) {
            if (t2c_ == 0) { if (t2Armed_) { ifr_ |= IFR_T2; t2Armed_ = false; } t2c_ = 0xFFFF; cycles--; continue; }
        }
        if (t1Running_) {
            if (t1Under_) {
                cycles--;
                if (acr_ & 0x40) { t1c_ = (uint16_t)(t1ll_ | (t1lh_ << 8)); t1Under_ = false; t1Armed_ = true; }
                else { t1c_ = 0xFFFF; t1Under_ = false; }       // un disparo: sigue contando sin nueva bandera
                continue;
            }
            if (t1c_ == 0) {
                if (t1Armed_) { ifr_ |= IFR_T1; if (!(acr_ & 0x40)) t1Armed_ = false; }
                t1c_ = 0xFFFF; t1Under_ = (acr_ & 0x40) != 0; cycles--;
                continue;
            }
        }
        // salto de varios ciclos si ningun evento ocurre antes
        uint32_t step = cycles;
        if (t1Running_ && !t1Under_ && t1c_ < step) step = t1c_;
        if (t2Running_ && t2c_ < step) step = t2c_;
        if (step == 0) step = 1;
        if (t1Running_) t1c_ = (uint16_t)(t1c_ - step);
        if (t2Running_) t2c_ = (uint16_t)(t2c_ - step);
        cycles -= step;
    }
    updateIrq();
}
