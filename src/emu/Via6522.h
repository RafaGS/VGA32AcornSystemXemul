#pragma once

#include <stdint.h>

// VIA 6522 con temporizadores T1/T2, avanzado por ciclos de CPU (tick()).
// Hallazgo: el juego "Space Invaders" del disco original sincroniza su bucle con T1 en
// modo libre en $0E00 (ACR=$40, T1=$4E1E -> 20.000 ciclos = 50 Hz) y espera con
// BIT $0E0D / BVC. Este dispositivo NO esta en la tabla de memoria del encargo: su
// presencia y direccion se infieren del programa (ver docs/SYSTEM3.md).
//
// Implementado: puertos A/B con DDR (sin pines externos), T1 (un disparo y libre), T2
// (un disparo), IFR/IER y linea IRQ. NO implementado: registro de desplazamiento,
// lineas CA1/CA2/CB1/CB2 y modos de enclavamiento, T2 en modo de conteo de pulsos.
class Via6522 {
public:
    class Host {
    public:
        virtual void irq(bool level) = 0;
        virtual uint8_t readPortA() { return 0xFF; }   // pines de entrada (sin nada conectado: 1)
        virtual uint8_t readPortB() { return 0xFF; }
    };

    enum { IFR_T2 = 0x20, IFR_T1 = 0x40 };

    explicit Via6522(Host* host) : host_(host) { reset(); }

    void reset();
    uint8_t read(uint8_t reg);
    void write(uint8_t reg, uint8_t v);
    void tick(uint32_t cycles);

    bool irqActive() const { return (ifr_ & ier_ & 0x7F) != 0; }
    uint8_t ifr() const { return (uint8_t)((ifr_ & 0x7F) | (irqActive() ? 0x80 : 0)); }
    uint8_t acr() const { return acr_; }

private:
    void updateIrq();
    Host* host_;
    uint8_t orb_, ora_, ddrb_, ddra_, acr_, pcr_, sr_, ifr_, ier_;
    uint8_t t1ll_, t1lh_, t2ll_;
    uint16_t t1c_, t2c_;
    bool t1Running_, t1Under_, t1Armed_, t2Running_, t2Armed_;
    bool lastIrq_;
};
