#include "System3Machine.h"

#include <stdio.h>
#include <string.h>

// Estado de la RAM al encender: pseudoaleatorio, como la RAM estatica real y como el emulador de referencia (xorshift32 con
// una semilla por tarjeta: CPU 0x6502, RAM de video 40 col 0x5050, RAM 8K a base+99, RAM de video 80 col 0x8025). Con
// este patron el byte $F0 (interruptor MON/NOMON del DOS, que la ROM no inicializa) vale $51 y el DOS arranca en NOMON.
static void fillPowerOn(uint8_t* p, size_t n, uint32_t seed) {
    uint32_t s = seed ? seed : 1;
    for (size_t i = 0; i < n; i++) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; p[i] = (uint8_t)(s & 0xFF); }
}

System3Machine::System3Machine(unsigned ramKB)
    : ramKB_(ramKB == 8 ? 8 : 16), active80_(false), portKey_(0xFF), portKeyHold_(0), strobeAge_(0),
      ram_((size_t)(kExtras ? 48 : (ramKB == 8 ? 8 : 16)) * 1024, 0), loaded_(false), basicLoaded_(false),
      drive_(S3_DRIVE_TRACKS), drive1_(S3_DRIVE_TRACKS),
      fdc_(drive_, this, kDrives >= 2 ? &drive1_ : 0),
      cpu_(*this), ins_(*this), timer_(this), kh_(0), kc_(0), kbdData_(0), kbdStrobe_(false),
      kbdGap_(0), nmis_(0), lastCycles_(0) {
    memset(rom_, 0, sizeof rom_);
    memset(basicRom_, 0, sizeof basicRom_);
    reset(true);
}

void System3Machine::reset(bool cold) {
    if (cold) {                                       // encendido: la RAM queda con su patron de arranque
        fillPowerOn(mem_, 0x0400, 0x6502);
        fillPowerOn(mem_ + 0x0400, 0x0400, 0x5050);
        if (kExtras) fillPowerOn(&ram_[0], ram_.size(), 0x2000 + 99);
        else {
            fillPowerOn(&ram_[0], 0x2000, 0x2000 + 99);
            if (ramKB_ >= 16) fillPowerOn(&ram_[0x2000], 0x2000, 0xC000 + 99);
        }
        fillPowerOn(vram80_, sizeof vram80_, 0x8025);
    }
    ins_.reset(cold);
    crtc_.reset(); crtc80_.reset(); active80_ = false; portKey_ = 0xFF; portKeyHold_ = 0; strobeAge_ = 0;
    timer_.reset();
    if (cold) fdc_.powerOn(); else fdc_.reset();        // el RESET conserva los registros de pista; el DOS recalibra con SEEK 0
    kh_ = kc_ = 0; kbdData_ = 0; kbdStrobe_ = false; kbdGap_ = 0;
    cpu_.reset();
    lastCycles_ = cpu_.cycles();
}

bool System3Machine::loadRom(const uint8_t* d, size_t n) {
    if (n != kRomSize) return false;
    memcpy(rom_, d, n);
    loaded_ = true;
    return true;
}

bool System3Machine::loadRomFile(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    uint8_t b[kRomSize + 1];
    const size_t n = fread(b, 1, sizeof b, f);
    fclose(f);
    return loadRom(b, n);
}

bool System3Machine::loadBasicRom(const uint8_t* d, size_t n) {
    if (n != kRomSize) return false;
    memcpy(basicRom_, d, n);
    basicLoaded_ = true;
    return true;
}

bool System3Machine::loadBasicRomFile(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    uint8_t b[kRomSize + 1];
    const size_t n = fread(b, 1, sizeof b, f);
    fclose(f);
    return loadBasicRom(b, n);
}

bool System3Machine::ramMapped(uint16_t a) const {
    if (kExtras) return a >= 0x2000 && a < 0xE000;                          // System 5: 48 KB contiguos
    return (a >= 0x2000 && a < 0x4000) || (ramKB_ >= 16 && a >= 0xC000 && a < 0xE000);
}

uint8_t System3Machine::readKbd() {
    if (!kbdStrobe_) return kbdData_;
    kbdStrobe_ = false;
    kbdGap_ = 2000;                  // margen entre teclas tras consumir el estrobo (evita que el DOS pierda teclas)
    return (uint8_t)(kbdData_ | 0x80);
}

bool System3Machine::queueKey(uint8_t code) {
    if (kc_ >= kKeyQueueSize) return false;
    key_[(kh_ + kc_) % kKeyQueueSize] = (uint8_t)(code & 0x7F);
    kc_++;
    return true;
}

// Cola de teclado con dos consumidores: el DOS (estrobo en el INS8154, $0E21) y el juego (puerto A del 6522, $0E0F).
// Una tecla se entrega a ambos a la vez; la siguiente espera a que el puerto del 6522 se libere (60 ms) y el estrobo
// del DOS se descarta si nadie lo lee en ~150 ms, para que el programa en marcha no bloquee la cola.
void System3Machine::pumpKeyboard(uint32_t cycles) {
    kbdGap_ = kbdGap_ > cycles ? kbdGap_ - cycles : 0;
    if (portKeyHold_) { portKeyHold_ = portKeyHold_ > cycles ? portKeyHold_ - cycles : 0; if (!portKeyHold_) portKey_ = 0xFF; }
    if (kbdStrobe_) { strobeAge_ += cycles; if (strobeAge_ > 150000) kbdStrobe_ = false; }
    if (!kbdStrobe_ && kbdGap_ == 0 && portKeyHold_ == 0 && kc_ > 0) {
        kbdData_ = key_[kh_];
        kh_ = (uint8_t)((kh_ + 1) % kKeyQueueSize);
        kc_--;
        kbdStrobe_ = true; strobeAge_ = 0;
        portKey_ = (uint8_t)(kbdData_ & 0x7F); portKeyHold_ = 60000;     // bit 7 = 0 mientras dura
    }
}

uint8_t System3Machine::rd(uint16_t a) {
    const uint8_t hi = (uint8_t)(a >> 8);
    if (hi < 0x08) return mem_[a];                       // RAM 0000-03FF y RAM de video 0400-07FF
    if (a >= kDosBase) return rom_[a - kDosBase];
    if (basicLoaded_ && a >= kBasicBase && a < (uint16_t)(kBasicBase + kRomSize)) return basicRom_[a - kBasicBase];
    if (ramMapped(a)) return ram_[ramIndex(a)];
    if (kExtras) {
        if (a >= 0x1000 && a < 0x1800) return vram80_[a - 0x1000];
        if (a == 0x1840 || a == 0x1841) return crtc80_.read((uint8_t)(a & 1));
    }
    if (hi >= 0x08 && hi < 0x10) {
        const uint8_t v = rdIo(a);
        if (trace_) trace_(false, a, v);
        return v;
    }
    return 0x00;                                         // sin decodificar: el bus flota a nivel bajo
}

uint8_t System3Machine::rdIo(uint16_t a) {
    switch (a >> 8) {
        case 0x08: return crtc_.read((uint8_t)(a & 1));
        case 0x0A: case 0x0B: return fdc_.read((uint8_t)(a & 7));
        case 0x0E: case 0x0F: {
            const uint8_t o = (uint8_t)a;
            if (kExtras && o < 0x20) return timer_.read((uint8_t)(o & 0x0F));
            if (o == 0x21) return readKbd();
            return ins_.read(o);
        }
        default: return 0x00;
    }
}

void System3Machine::wr(uint16_t a, uint8_t v) {
    const uint8_t hi = (uint8_t)(a >> 8);
    if (hi < 0x08) { mem_[a] = v; return; }
    if (a >= kDosBase) return;                           // ROM: escritura ignorada
    if (basicLoaded_ && a >= kBasicBase && a < (uint16_t)(kBasicBase + kRomSize)) return; // BASIC ROM: solo lectura
    if (ramMapped(a)) { ram_[ramIndex(a)] = v; return; }
    if (kExtras) {
        if (a >= 0x1000 && a < 0x1800) { vram80_[a - 0x1000] = v; return; }
        if (a == 0x1840 || a == 0x1841) { crtc80_.write((uint8_t)(a & 1), v); active80_ = true; return; }
    }
    if (hi >= 0x08 && hi < 0x10 && trace_) trace_(true, a, v);
    switch (hi) {
        case 0x08: crtc_.write((uint8_t)(a & 1), v); active80_ = false; break;   // el ultimo 6845 programado decide la pantalla
        case 0x0A: case 0x0B: fdc_.write((uint8_t)(a & 7), v); break;
        case 0x0E: case 0x0F:
            if (kExtras && (uint8_t)a < 0x20) timer_.write((uint8_t)(a & 0x0F), v); else ins_.write((uint8_t)a, v);
            break;
        default: break;
    }
}

void System3Machine::run(uint32_t instructions) {
    while (instructions) {
        const uint8_t chunk = instructions > 8 ? 8 : (uint8_t)instructions;
        const uint16_t done = cpu_.run(chunk);
        const uint32_t cyc = cpu_.cycles();
        const uint32_t d = cyc - lastCycles_;
        lastCycles_ = cyc;
        fdc_.tick(d);
        timer_.tick(d);
        pumpKeyboard(d);
        if (done == 0) break;
        instructions -= done;
    }
}

uint32_t System3Machine::runCycles(uint32_t cycles) {
    const uint32_t start = cpu_.cycles();
    uint32_t done = 0;
    while (done < cycles) {
        run(8);
        done = cpu_.cycles() - start;
    }
    return done;
}
