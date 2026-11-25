#pragma once

#include <stdint.h>

#include "FloppyDrive.h"

// Controladora de disquetes Intel 8271 (modo no-DMA, hasta dos unidades), avanzada por ciclos de CPU emulados mediante
// tick(). Sin temporizadores del anfitrion. Alineada con el emulador de referencia (elaverick/SystemEmulator) y con el
// comportamiento observado de la ROM del DOS.
//
// Registros (offset desde la base de la tarjeta, $0A00; el DOS usa +0/+1/+2 y el dato en +4):
//   lectura  +0 estado   +1 resultado   +4..+7 dato
//   escritura +0 orden   +1 parametro   +2 reset (bit 0)   +4..+7 dato
// Estado: b7 BUSY, b6 CMD FULL, b5 PARAM FULL, b4 RESULT FULL, b3 INT, b2 NDRQ (solo en modo no-DMA: registro de modo bit 0).
// Linea de interrupcion hacia la CPU = INT | NDRQ (en el System 3/4 va al NMI).
//
// Ordenes (bits 5:0; bits 7:6 = seleccion de unidad, 01 = unidad 0, 10 = unidad 1):
//   READ DATA     13 (3 par.)  12 (2 par., un registro)   +deleted 17 / 16
//   WRITE DATA    0B (3 par.)  0A (2 par.)                +deleted 0F / 0E
//   VERIFY        1F (3 par.)  1E (2 par.)
//   READ ID 1B (3)   FORMAT 23 (5)   SEEK 29 (1)   READ DRIVE STATUS 2C (0)   SPECIFY 35 (4)
//   WRITE SPECIAL 3A (2)   READ SPECIAL 3D (1)   SCAN 00 / 04 (5, no implementado: responde NOT FOUND)
// SEEK a la pista 0 es un RECALIBRADO (retrocede hasta el sensor de pista 0); el resto de movimientos usan la diferencia
// con el registro de pista. READ/WRITE/VERIFY/READ ID/FORMAT mueven la cabeza a la pista pedida (posicionamiento implicito).
//
// Registros especiales: 06 sector de SCAN, 10/11 pistas malas unidad 0, 12 pista actual unidad 0, 13/14 contador SCAN,
// 17 modo, 18/19 pistas malas unidad 1, 1A pista actual unidad 1, 22 estado de unidades (= READ DRIVE STATUS),
// 23 puerto de salida (bit 3 = cargar cabezal; bit 1 = pulso de paso, bit 2 = sentido).
// READ DRIVE STATUS: bit 2 RDY0, bit 6 RDY1, bit 1 pista 0, bit 3 protegida (o sin disco), bit 4 pulso de indice
// (cabezal cargado y 4 ms de cada vuelta).
class Intel8271 {
public:
    enum Result : uint8_t {
        RES_OK = 0x00, RES_LATE_DMA = 0x0A, RES_ID_CRC = 0x0C, RES_DATA_CRC = 0x0E,
        RES_NOT_READY = 0x10, RES_WRITE_PROTECT = 0x12, RES_TRACK0 = 0x14,
        RES_WRITE_FAULT = 0x16, RES_NOT_FOUND = 0x18
    };
    enum Status : uint8_t { ST_BUSY = 0x80, ST_CMDFULL = 0x40, ST_PARAMFULL = 0x20, ST_RESFULL = 0x10, ST_INT = 0x08, ST_NDRQ = 0x04 };

    // Temporizacion (1 MHz). 64 us por byte en FM; el resto son estimaciones.
    static const uint32_t kByteCycles = 64;
    static const uint32_t kParamAcceptCycles = 12;   // estimacion: tiempo hasta liberar PARAM FULL
    static const uint32_t kHeaderBytes = 19;         // ID + GAP2 + DAM antes del primer byte de datos (estimacion FM)

    class Host { public: virtual void line(bool level) = 0; };   // linea INT/NDRQ -> NMI

    // Hasta dos unidades; una unidad no conectada (puntero nulo) responde NOT READY.
    Intel8271(FloppyDrive& drive0, Host* host, FloppyDrive* drive1 = 0) : host_(host), now_(0) {
        drives_[0] = &drive0; drives_[1] = drive1; powerOn();
    }

    void powerOn();          // encendido: borra tambien los registros de pista, pistas malas, etc.
    void reset();            // RESET: conserva los registros de pista (como el emulador de referencia)
    uint8_t read(uint8_t reg);
    void write(uint8_t reg, uint8_t v);
    void tick(uint32_t cycles);

    uint8_t status() const;
    bool lineLevel() const { return (status_ & ST_INT) || ((status_ & ST_NDRQ) && (moder_ & 1)); }
    uint32_t unknownCommands() const { return unknown_; }
    uint8_t special(uint8_t r) const { return readSpecial(r); }
    bool headLoaded() const { return headLoaded_; }
    bool busy() const { return phase_ != P_IDLE; }

private:
    enum Phase : uint8_t {
        P_IDLE, P_PARAMS, P_FINISH_WAIT, P_HEADLOAD, P_SEEK, P_ISEEK, P_SEARCH, P_NOTFOUND, P_XFER_READ, P_XFER_WRITE,
        P_VERIFY, P_ID_SEARCH, P_ID_BYTES, P_FORMAT_WAIT, P_FORMAT_BYTES, P_FORMAT_END
    };
    enum Class : uint8_t { C_INVALID, C_READ, C_WRITE, C_VERIFY, C_READID, C_FORMAT, C_SEEK, C_DRVSTAT, C_SPECIFY, C_WSPECIAL, C_RSPECIAL, C_SCAN };

    void startCommand();
    void execCommand();                  // ejecuta la orden una vez cargado el cabezal
    bool implicitSeek();                 // posicionamiento implicito a 'track_' (true si hay que esperar)
    void startSearch();                  // READ/WRITE/VERIFY: seek implicito + localiza 'sector_'
    void locate();
    void beginReadId();
    void beginFormat();
    void searchDone();
    void sectorDone();
    void idNext();
    void finish(uint8_t result, bool interrupt);
    void setNdrq(bool v);
    void updateLine();
    void onTimeout();
    void acceptParam();
    static Class classify(uint8_t cmd, uint8_t* params);
    uint8_t driveStatus() const;
    uint8_t readSpecial(uint8_t r) const;
    void writeSpecial(uint8_t r, uint8_t v);

    FloppyDrive* drives_[2];
    FloppyDrive* cur_;                  // unidad de la orden en curso (o la ultima seleccionada)
    uint8_t unit_;                      // 0 o 1
    FloppyDrive& drv() { return *cur_; }
    uint8_t& trackReg() { return pcn_[unit_]; }   // registro de pista de la unidad en curso
    Host* host_;
    bool lastLine_;
    uint32_t now_, wait_, paramWait_;
    Phase phase_;
    Class cls_;
    uint8_t status_, result_, cmd_, params_[5], nparams_, need_;
    uint8_t data_, buf_[256];
    uint16_t idx_;
    uint8_t track_, sector_, remaining_, pending_;
    uint8_t idByte_, idCount_, fmtSectors_, fmtSeen_[16];
    // Registros especiales
    uint8_t pcn_[2], bad_[2][2], scanSector_, scanCount_[2], moder_, outPort_;
    // Temporizacion (SPECIFY): paso y asentamiento en unidades de 2 ms, carga de cabezal en 8 ms
    uint32_t stepCycles_, settleCycles_, headLoadCycles_, unloadRevs_, unloadWait_, unknown_;
    bool headLoaded_;
    bool paramPending_;
    uint8_t pendingParam_;
};
