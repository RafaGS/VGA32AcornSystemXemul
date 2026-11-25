#pragma once

#include <stdint.h>
#include <vector>

#include "Cpu6502.h"
#include "Crtc6845.h"
#include "FloppyDrive.h"
#include "Ins8154.h"
#include "Intel8271.h"
#include "Model.h"
#include "Via6522.h"

// Acorn System 3 / System 4 (6502; el modelo se elige en compilacion, ver Model.h): CPU, mapa de memoria, ROM, perifericos y
// temporizacion. Sin dependencias de la plataforma: lo usan el firmware de la TTGO VGA32 (src/main.cpp) y las pruebas del PC.
//
// Mapa de los Systems 3 y 4, segun las tarjetas de la definicion del emulador de referencia (elaverick/SystemEmulator) y
// confirmado al ejecutar la ROM del DOS:
//   0000-03FF  RAM 1K (tarjeta de CPU 200,000)
//   0400-07FF  RAM de video de 40 columnas (tarjeta VDU 200,002)       0800-08FF  6845 de 40 columnas (a&1: direccion/dato)
//   0A00-0BFF  Intel 8271 (+0 estado/orden, +1 resultado/parametro, +2 reset, +4 dato; tarjeta 200,004)
//   0E00-0EFF  INS8154 de la tarjeta de CPU, repetido hasta 0FFF: con A7=0 son sus registros de E/S (a&$7F; teclado en el
//              puerto B, $0E21) y con A7=1 sus 128 bytes de RAM
//   2000-3FFF  RAM 8K      C000-DFFF  RAM 8K (solo con 16 KB; el BASIC se carga desde disco)      F000-FFFF  Acorn DOS (ROM)
//   Todo lo demas esta sin decodificar: el bus flota a nivel bajo y se lee $00; las escrituras se ignoran.
// Con -DS3_SYSTEM5_EXTRAS (NO autentico) se anaden las tarjetas del System 5: VIA 6522 en $0E00-$0E1F (con el INS8154 solo en
// $0E20-$0E7F), RAM de video de 80 columnas en $1000-$17FF, su 6845 en $1840-$1841 y 48 KB de DRAM contiguos en $2000-$DFFF.
// La ROM del DOS se carga desde fichero o memoria; si falta, el vector de reset lee $0000 y la maquina no arranca.
class System3Machine : private Ins8154::Pins, private Intel8271::Host, private Via6522::Host {
public:
    static const uint16_t kDosBase = 0xF000, kRomSize = 0x1000;
    static const uint8_t kKeyQueueSize = 16;
    static const bool kExtras = S3_EXTRAS != 0;

    explicit System3Machine(unsigned ramKB = S3_RAM_KB);

    void reset(bool cold);
    // Ejecuta ~n instrucciones en tramos de <= 8 (56 ciclos < 64 us) y avanza el 8271 con los
    // ciclos reales de la CPU tras cada tramo, para que el NMI llegue a tiempo.
    void run(uint32_t instructions);
    // Ejecuta al menos 'cycles' ciclos de CPU (1 ciclo = 1 us a 1 MHz) y devuelve los realmente ejecutados.
    uint32_t runCycles(uint32_t cycles);

    bool loadRom(const uint8_t* data, size_t n);   // ROM del DOS: exige 4096 bytes
    bool loadRomFile(const char* path);
    bool romLoaded() const { return loaded_; }

    // Unidades de disquete: 1 en el System 3 y 2 en el System 4 (seleccion en compilacion, Model.h). drive(1) existe
    // siempre como objeto, pero solo esta conectada al 8271 si kDrives >= 2.
    static const unsigned kDrives = S3_DRIVES;
    static const char* modelName() { return S3_MODEL_NAME; }
    FloppyDrive& drive(unsigned n = 0) { return n ? drive1_ : drive_; }
    Intel8271& fdc() { return fdc_; }
    typedef Cpu6502<System3Machine> Cpu;
    Cpu& cpu() { return cpu_; }
    unsigned ramKB() const { return ramKB_; }
    bool ramMapped(uint16_t a) const;
    static uint16_t ramIndex(uint16_t a) { return kExtras || a < 0x4000 ? (uint16_t)(a - 0x2000) : (uint16_t)(0x2000 + (a - 0xC000)); }

    bool queueKey(uint8_t code);
    const uint8_t* videoRam() const { return mem_ + 0x0400; }
    const Crtc6845& crtc() const { return crtc_; }
    // Tarjeta VDU de 80 columnas (solo con S3_SYSTEM5_EXTRAS): RAM de video 2 KB en $1000 y 6845 en $1840. 'display80Active'
    // pasa a true cuando el software programa su 6845 y vuelve a false cuando se programa el de 40 columnas (p. ej. el DOS al
    // reiniciarse tras salir de un juego) o con el reset.
    const uint8_t* videoRam80() const { return vram80_; }
    const Crtc6845& crtc80() const { return crtc80_; }
    bool display80Active() const { return active80_; }
    uint32_t nmiCount() const { return nmis_; }
    // Traza opcional de accesos a E/S (depuracion): w=true escritura.
    typedef void (*TraceFn)(bool w, uint16_t addr, uint8_t v);
    void setTrace(TraceFn f) { trace_ = f; }

    // Interfaz de memoria para Cpu6502<System3Machine>
    uint8_t rd(uint16_t a);
    void wr(uint16_t a, uint8_t v);
    uint8_t fetchOp(uint16_t pc) { return rd(pc); }
    uint8_t fetchByte(uint16_t pc) { return rd(pc); }
    uint8_t* lowRam() { return mem_; }
    uint8_t read(uint16_t a) { return rd(a); }
    void write(uint16_t a, uint8_t v) { wr(a, v); }

private:
    // Ins8154::Pins: el puerto B del INS8154 es el teclado ASCII; el puerto A (cinta) no esta conectado.
    uint8_t readA() override { return 0; }
    uint8_t readB() override { return readKbd(); }
    uint8_t peekA() override { return 0; }
    uint8_t peekB() override { return (uint8_t)(kbdData_ | (kbdStrobe_ ? 0x80 : 0)); }
    void writeA(uint8_t) override {}
    void writeB(uint8_t) override {}
    // Intel8271::Host: la linea INT/NDRQ va al NMI (flanco de subida).
    void line(bool level) override { if (level) { cpu_.nmi(); nmis_++; } }
    // Via6522::Host: IRQ del VIA de $0E00 (solo con S3_SYSTEM5_EXTRAS).
    void irq(bool level) override { cpu_.setIrq(level); }
    uint8_t readPortA() override { return portKey_; }

    uint8_t rdIo(uint16_t a);
    uint8_t readKbd();
    void pumpKeyboard(uint32_t cycles);

    unsigned ramKB_;
    uint8_t mem_[0x0800];                    // RAM + video
    uint8_t vram80_[0x0800];                 // RAM de video de 80 columnas (1000-17FF)
    Crtc6845 crtc80_;
    bool active80_;
    uint8_t portKey_;                        // tecla presentada en el puerto A del 6522 (bit 7 = 1: ninguna)
    uint32_t portKeyHold_, strobeAge_;
    std::vector<uint8_t> ram_;               // 2000-3FFF y, con 16 KB, C000-DFFF (en ese orden)
    uint8_t rom_[kRomSize];
    bool loaded_;
    FloppyDrive drive_, drive1_;
    Intel8271 fdc_;
    Cpu cpu_;
    Ins8154 ins_;
    Crtc6845 crtc_;
    Via6522 timer_;                          // solo con S3_SYSTEM5_EXTRAS: 6522 en $0E00 (temporizador de 50 Hz del juego)
    uint8_t key_[kKeyQueueSize], kh_, kc_, kbdData_;
    bool kbdStrobe_;
    uint32_t kbdGap_, nmis_, lastCycles_;
    TraceFn trace_ = 0;
};
