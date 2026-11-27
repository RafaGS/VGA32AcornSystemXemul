// Acorn System 3 / System 4 (6502) sobre TTGO VGA32 con FabGL.
// La configuración física se selecciona en Model.h mediante el entorno de PlatformIO.
//
// Reparto: tarea de emulacion en el nucleo 0 (sincronizada con esp_timer a 1 MHz),
// Arduino loop() en el nucleo 1 (teclado, dibujo, menu). Un mutex protege el
// acceso a la maquina entre ambos.
#include <Arduino.h>
#include <dirent.h>
#include <esp_timer.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>

#include "fabgl.h"

#include "DiskImage.h"
#include "System3Machine.h"
#include "Teletext.h"
#include "embedded_roms.h"

static const char* kSdMount = "/SD";
static const char* kDiskDir = "/SD/system3";       // imagenes .ssd / .img (System 3: 102.400 B; System 4: 204.800 B max.)

static fabgl::VGA8Controller gDisplay;             // 8 colores, pines por defecto de FabGL (= TTGO VGA32)
static fabgl::PS2Controller gPs2;
static fabgl::Canvas* gCanvas = nullptr;

static System3Machine* gM = nullptr;
static SemaphoreHandle_t gLock;
static FileDiskImage gDisk[System3Machine::kDrives];            // una imagen por unidad (1 en el System 3, 2 en el System 4)
static bool gSdOk = false, gWriteProt[System3Machine::kDrives];
static bool gCaps = true;                          // mayusculas activas por defecto (el teclado Acorn es solo mayusculas)
static int gDosVariant = 0;                        // 0 = DOS 4.0a, 1 = DOS 4.0b
static std::vector<std::string> gImages;
static int gImgIdx[System3Machine::kDrives];                    // indice de la imagen insertada en cada unidad (-1: ninguna)
static std::string gOsd;
static uint32_t gOsdUntil = 0;

// Desplazamiento vertical de la imagen (px). Ajustable en vivo con F6 (subir) / F7 (bajar) si el monitor
// recorta la parte de abajo o de arriba; valor por defecto con -DS3_Y_OFFSET=n.
#ifndef S3_Y_OFFSET
#define S3_Y_OFFSET 10
#endif
static int gYOff = S3_Y_OFFSET;

static void osd(const std::string& s, uint32_t ms = 4000) { gOsd = s; gOsdUntil = millis() + ms; }

// ---- Maquina ----------------------------------------------------------------
static void loadRoms() {
    gM->loadRom(gDosVariant ? kRomDos40b : kRomDos40a, 4096);
#if S3_HAS_BASIC_ROM
    gM->loadBasicRom(kRomSystemBasic, 4096);                   // System 3 minimo: BASIC ROM en C000-CFFF
#endif
}

static void resetMachine(bool cold) {
    xSemaphoreTake(gLock, portMAX_DELAY);
    gM->reset(cold);
    xSemaphoreGive(gLock);
}

// ---- Discos -------------------------------------------------------------------
static bool hasExt(const std::string& n, const char* ext) {
    if (n.size() < strlen(ext)) return false;
    std::string e = n.substr(n.size() - strlen(ext));
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return e == ext;
}

static void scanImages() {
    gImages.clear();
    DIR* d = opendir(kDiskDir);
    if (!d) return;
    while (dirent* e = readdir(d)) {
        std::string n = e->d_name;
        if (hasExt(n, ".ssd") || hasExt(n, ".img")) gImages.push_back(n);
    }
    closedir(d);
    std::sort(gImages.begin(), gImages.end());
}

// Siguiente imagen para la unidad: nunca la que ya tiene la otra (dos escritores sobre el mismo fichero lo corromperian).
static int nextImage(unsigned unit) {
    const int n = (int)gImages.size();
    const int other = System3Machine::kDrives > 1 ? gImgIdx[1 - unit] : -1;
    int idx = gImgIdx[unit];
    for (int i = 0; i < n; i++) {
        idx = (idx + 1) % n;
        if (idx != other) return idx;
    }
    return gImgIdx[unit];                           // solo hay una imagen y la usa la otra unidad
}

static std::string unitTag(unsigned unit) { return System3Machine::kDrives > 1 ? "U" + std::to_string(unit) + ": " : ""; }

static void ejectDisk(unsigned unit) {
    xSemaphoreTake(gLock, portMAX_DELAY);
    gM->drive(unit).eject();                        // sincroniza antes de soltar
    gDisk[unit].close();
    xSemaphoreGive(gLock);
}

static void insertImage(unsigned unit, int idx) {
    ejectDisk(unit);
    if (idx < 0 || idx >= (int)gImages.size()) { gImgIdx[unit] = -1; osd(unitTag(unit) + "Sin disco"); return; }
    const std::string path = std::string(kDiskDir) + "/" + gImages[idx];
    xSemaphoreTake(gLock, portMAX_DELAY);
    bool ok = gDisk[unit].open(path.c_str()) && gM->drive(unit).insertAuto(&gDisk[unit]) == FloppyDrive::INSERT_OK;
    if (ok) gM->drive(unit).setWriteProtect(gWriteProt[unit]);
    else gDisk[unit].close();
    xSemaphoreGive(gLock);
    gImgIdx[unit] = ok ? idx : -1;
    osd(ok ? unitTag(unit) + "Disco: " + gImages[idx] + (gM->drive(unit).writeProtected() ? " [WP]" : "")
           : unitTag(unit) + "Imagen invalida: " + gImages[idx]);
}

// ---- Emulacion (nucleo 0) -----------------------------------------------------
// 1 ciclo de CPU = 1 us. Si se acumula retraso (p. ej. por accesos a la SD) se
// resincroniza en lugar de intentar recuperarlo a toda velocidad.
static void emuTask(void*) {
    const int64_t t0 = esp_timer_get_time();
    int64_t emulated = 0;
    for (;;) {
        const int64_t target = esp_timer_get_time() - t0;
        if (target > emulated + 100000) emulated = target - 20000;
        if (emulated < target) {
            uint32_t want = (uint32_t)(target - emulated);
            if (want > 20000) want = 20000;
            xSemaphoreTake(gLock, portMAX_DELAY);
            const uint32_t done = gM->runCycles(want);
            if (!gM->fdc().busy() && gM->drive().dirty()) gM->drive().sync();   // fsync con la 8271 libre
            xSemaphoreGive(gLock);
            emulated += done;
        } else {
            vTaskDelay(1);
        }
    }
}

// ---- Video ----------------------------------------------------------------------
static fabgl::RGB888 colour(uint8_t c) { return fabgl::RGB888((c & 1) ? 255 : 0, (c & 2) ? 255 : 0, (c & 4) ? 255 : 0); }

static TeletextCell gShadow[Teletext::kCols * Teletext::kRows];
static TeletextCell gCells[Teletext::kCols * Teletext::kRows];
static uint8_t gShadow80[2000];
static bool gShadowValid = false, gLastFlash = true, gMode80 = false, gOsdDrawn = false;
static int gCurDrawn = -1;

static void fillRect(fabgl::Canvas& cv, fabgl::RGB888 c, int x1, int y1, int x2, int y2) {
    cv.setBrushColor(c);
    cv.fillRectangle(x1, y1, x2, y2);
}

// Cursor del 6845 (R10 bits 6-5: 00 fijo, 01 oculto, 10 parpadeo 1/16, 11 parpadeo 1/32 de campo a 50 Hz).
// Devuelve el indice de celda del cursor visible (o -1) y sus lineas de barrido en pixeles de la celda.
static int cursorCell(const Crtc6845& crtc, uint16_t start, int cells, uint32_t now, int& cy0, int& cy1) {
    const uint8_t mode = (crtc.reg(10) >> 5) & 3;
    const uint16_t pos = (uint16_t)(crtc.cursorAddress() - start) & 0x3FFF;
    bool on = mode != 1 && pos < cells && now >= gOsdUntil;
    if (mode == 2) on = on && ((now / 320) & 1) == 0;
    if (mode == 3) on = on && ((now / 640) & 1) == 0;
    cy0 = (crtc.reg(10) & 0x1F) * Teletext::kCellH / 10;
    cy1 = ((crtc.reg(11) & 0x1F) + 1) * Teletext::kCellH / 10 - 1;
    if (cy1 >= Teletext::kCellH) cy1 = Teletext::kCellH - 1;
    if (cy0 > cy1) cy0 = cy1;
    return on ? pos : -1;
}

// ---- 40 columnas: teletexto SAA5050 ----
static void drawCell(int idx, bool flashVisible, bool cursor, int cy0, int cy1) {
    const int col = idx % Teletext::kCols, row = idx / Teletext::kCols;
    const int x = col * Teletext::kCellW, y = gYOff + row * Teletext::kCellH;
    const TeletextCell& c = gCells[idx];
    fabgl::Canvas& cv = *gCanvas;
    fillRect(cv, colour(c.bg), x, y, x + Teletext::kCellW - 1, y + Teletext::kCellH - 1);
    TeletextRect r[Teletext::kMaxRects];
    const uint8_t n = Teletext::cellRects(c, flashVisible, kRomSaa5050, r);
    if (n) {
        cv.setBrushColor(colour(c.fg));
        for (uint8_t i = 0; i < n; i++)
            cv.fillRectangle(x + r[i].x, y + r[i].y, x + r[i].x + r[i].w - 1, y + r[i].y + r[i].h - 1);
    }
    if (cursor) fillRect(cv, colour(7), x, y + cy0, x + Teletext::kCellW - 1, y + cy1);
}

static void drawFrame40(uint32_t now) {
    const Crtc6845& crtc = gM->crtc();                 // lectura sin bloqueo: carreras inocuas (solo se dibuja)
    const uint16_t start = crtc.startAddress();
    Teletext::decodeScreen(gM->videoRam(), 0x3FF, start, Teletext::kCols, Teletext::kRows, gCells);

    if (now < gOsdUntil) {                             // mensaje en la ultima fila
        for (int i = 0; i < Teletext::kCols; i++) {
            const char ch = i < (int)gOsd.size() ? gOsd[i] : ' ';
            gCells[(Teletext::kRows - 1) * Teletext::kCols + i] = Teletext::textCell(ch, 7, 4);
        }
    }
    int cy0, cy1;
    const int curIdx = cursorCell(crtc, start, Teletext::kCols * Teletext::kRows, now, cy0, cy1);
    const bool flashVisible = (now % 1200) < 900;
    const bool flashChanged = flashVisible != gLastFlash;

    for (int i = 0; i < Teletext::kCols * Teletext::kRows; i++) {
        const bool cursorHere = i == curIdx;
        const bool dirty = !gShadowValid || gCells[i] != gShadow[i] || (flashChanged && (gCells[i].flags & TC_FLASH)) ||
                           cursorHere != (i == gCurDrawn);
        if (dirty) { drawCell(i, flashVisible, cursorHere, cy0, cy1); gShadow[i] = gCells[i]; }
    }
    gCurDrawn = curIdx;
    gLastFlash = flashVisible;
}

// ---- 80 columnas: tarjeta VDU de 80 columnas del System 5 (solo con -DS3_SYSTEM5_EXTRAS; celdas de 8 x kCellH) ----
// Fuente: la ROM de caracteres real de la tarjeta (roms/vdu80.chr). El bit 7 del codigo se ignora (no hay video inverso).
static void drawCell80(int idx, uint8_t ch, bool cursor, int cy0, int cy1) {
    const int x = (idx % 80) * 8, y = gYOff + (idx / 80) * Teletext::kCellH;
    fabgl::Canvas& cv = *gCanvas;
    fillRect(cv, colour(0), x, y, x + 7, y + Teletext::kCellH - 1);
    TeletextRect r[Teletext::kMaxRects];
    const uint8_t n = Teletext::vduRects(ch, kFontVdu80, 8, Teletext::kCellH, r);
    if (n) {
        cv.setBrushColor(colour(7));
        for (uint8_t i = 0; i < n; i++)
            cv.fillRectangle(x + r[i].x, y + r[i].y, x + r[i].x + r[i].w - 1, y + r[i].y + r[i].h - 1);
    }
    if (cursor) fillRect(cv, colour(7), x, y + cy0, x + 7, y + cy1);
}

static void drawFrame80(uint32_t now) {
    const Crtc6845& crtc = gM->crtc80();
    const uint16_t start = crtc.startAddress();
    const uint8_t* vram = gM->videoRam80();
    int cy0, cy1;
    const int curIdx = cursorCell(crtc, start, 2000, now, cy0, cy1);

    for (int i = 0; i < 2000; i++) {
        const uint8_t ch = vram[(uint16_t)(start + i) & 0x7FF];
        const bool cursorHere = i == curIdx;
        if (!gShadowValid || ch != gShadow80[i] || cursorHere != (i == gCurDrawn)) {
            drawCell80(i, ch, cursorHere, cy0, cy1);
            gShadow80[i] = ch;
        }
    }
    gCurDrawn = curIdx;

    if (now < gOsdUntil) {                             // OSD sobre la ultima fila (blanco sobre azul)
        const int y = gYOff + 24 * Teletext::kCellH;
        fillRect(*gCanvas, colour(4), 0, y, 639, y + Teletext::kCellH - 1);
        for (int i = 0; i < 80 && i < (int)gOsd.size(); i++) {
            TeletextRect r[Teletext::kMaxRects];
            const uint8_t n = Teletext::vduRects((uint8_t)gOsd[i], kFontVdu80, 8, Teletext::kCellH, r);
            gCanvas->setBrushColor(colour(7));
            for (uint8_t k = 0; k < n; k++)
                gCanvas->fillRectangle(i * 8 + r[k].x, y + r[k].y, i * 8 + r[k].x + r[k].w - 1, y + r[k].y + r[k].h - 1);
        }
        gOsdDrawn = true;
    } else if (gOsdDrawn) {
        gOsdDrawn = false;
        gShadowValid = false;                          // al caducar el mensaje se repinta todo
    }
}

static void drawFrame(uint32_t now) {
    const bool m80 = gM->display80Active();
    if (m80 != gMode80) { gMode80 = m80; gShadowValid = false; gCurDrawn = -1; }
    if (!gShadowValid) { gCanvas->setBrushColor(fabgl::RGB888(0, 0, 0)); gCanvas->clear(); }
    if (m80) drawFrame80(now); else drawFrame40(now);
    gLastFlash = (now % 1200) < 900;
    gShadowValid = true;
}

// ---- Teclado (nucleo 1) -----------------------------------------------------------
// Acorn System: teclado ASCII de 7 bits; BACKSPACE = DELETE ($7F).
// Bloq Mayus (activo al arrancar): letras en mayuscula; con Mayus pulsado dan minuscula. F6/F7 suben/bajan la imagen 2 px. F8 cambia DOS 4.0a/b y reinicia, F9 siguiente imagen, F10 expulsar, F11 proteccion
// contra escritura, F12 RESET (Mayus+F12: reinicio en frio, borra la RAM).
static void handleKeys() {
    fabgl::Keyboard* kbd = gPs2.keyboard();
    fabgl::VirtualKeyItem it;
    while (kbd->getNextVirtualKey(&it, 0)) {
        if (!it.down) continue;
        switch (it.vk) {
            case fabgl::VK_CAPSLOCK:
                gCaps = !gCaps;
                kbd->setLEDs(false, gCaps, false);
                osd(gCaps ? "Mayusculas ON" : "Mayusculas OFF", 1500);
                continue;
            case fabgl::VK_F6: case fabgl::VK_F7: {
                gYOff += (it.vk == fabgl::VK_F7) ? 2 : -2;
                if (gYOff < 0) gYOff = 0;
                if (gYOff > 480 - Teletext::kRows * Teletext::kCellH) gYOff = 480 - Teletext::kRows * Teletext::kCellH;
                gShadowValid = false;                              // repinta todo
                osd("Desplazamiento vertical: " + std::to_string(gYOff) + " px (-DS3_Y_OFFSET)");
                continue;
            }
            case fabgl::VK_F8:
                gDosVariant ^= 1;
                xSemaphoreTake(gLock, portMAX_DELAY); loadRoms(); gM->reset(true); xSemaphoreGive(gLock);
                osd(gDosVariant ? "DOS 4.0b" : "DOS 4.0a");
                continue;
            case fabgl::VK_F9: case fabgl::VK_F10: case fabgl::VK_F11: {
                // F9 siguiente imagen, F10 expulsar, F11 proteccion de escritura. Unidad 0; con Ctrl, la unidad 1 (System 4).
                const unsigned unit = it.CTRL ? 1 : 0;
                if (unit >= System3Machine::kDrives) { osd("Este modelo solo tiene una unidad"); continue; }
                if (it.vk == fabgl::VK_F9) {
                    if (!gSdOk) { osd("SD no disponible"); continue; }
                    scanImages();
                    if (gImages.empty()) { osd("No hay imagenes en /system3"); continue; }
                    const int next = nextImage(unit);
                    if (next == gImgIdx[unit] && next >= 0) { osd(unitTag(unit) + "No hay otra imagen libre"); continue; }
                    insertImage(unit, next);
                } else if (it.vk == fabgl::VK_F10) {
                    ejectDisk(unit); gImgIdx[unit] = -1; osd(unitTag(unit) + "Disco expulsado");
                } else {
                    gWriteProt[unit] = !gWriteProt[unit];
                    xSemaphoreTake(gLock, portMAX_DELAY); gM->drive(unit).setWriteProtect(gWriteProt[unit]); xSemaphoreGive(gLock);
                    osd(unitTag(unit) + (gWriteProt[unit] ? "Proteccion de escritura ON" : "Proteccion de escritura OFF"));
                }
                continue;
            }
            case fabgl::VK_F12: resetMachine(it.SHIFT); osd("RESET"); continue;
            default: break;
        }
        uint8_t code = it.ASCII;
        if (it.vk == fabgl::VK_BACKSPACE) code = 0x7F;
        if ((code >= 'a' && code <= 'z') || (code >= 'A' && code <= 'Z')) {
            // Caps Lock logico propio (no depende del estado interno de FabGL): con Caps activo, Mayus + letra da minuscula.
            const bool upper = gCaps ? !it.SHIFT : it.SHIFT;
            code = (uint8_t)(upper ? (code & ~0x20) : (code | 0x20));
        }
        if (!code) continue;
        xSemaphoreTake(gLock, portMAX_DELAY);
        gM->queueKey(code);                              // si la cola esta llena se descarta la tecla
        xSemaphoreGive(gLock);
    }
}

// ---- Arduino ----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    gLock = xSemaphoreCreateMutex();

    gPs2.begin(fabgl::PS2Preset::KeyboardPort0);       // teclado PS/2 en el puerto 0 de la TTGO
#ifdef S3_KBD_LAYOUT
    gPs2.keyboard()->setLayout(&fabgl::S3_KBD_LAYOUT);
#endif
    gPs2.keyboard()->setLEDs(false, gCaps, false);      // enciende el LED de Bloq Mayus

    gDisplay.begin();                                   // pines por defecto de FabGL (= TTGO VGA32)
    gDisplay.setResolution(VGA_640x480_60Hz);
    gCanvas = new fabgl::Canvas(&gDisplay);

    // SD de la TTGO VGA32: MISO 2, MOSI 12, CLK 14, CS 13.
    gSdOk = fabgl::FileBrowser::mountSDCard(false, kSdMount, 4, 16 * 1024, 2, 12, 14, 13);

    gM = new System3Machine(S3_RAM_KB);
    loadRoms();
    gM->reset(true);

    for (unsigned u = 0; u < System3Machine::kDrives; u++) { gImgIdx[u] = -1; gWriteProt[u] = false; }
    if (gSdOk) {
        scanImages();
        // unidad 0 = primera imagen; unidad 1 (System 4) = la segunda, si existe
        for (unsigned u = 0; u < System3Machine::kDrives && u < gImages.size(); u++) insertImage(u, (int)u);
    }
    if (gImgIdx[0] < 0) osd(std::string(System3Machine::modelName()) + (gSdOk ? ": sin imagenes, F9 tras copiar .ssd a /system3" : ": sin SD, el DOS espera disco"));
    else osd(std::string(System3Machine::modelName()) + " - " + unitTag(0) + gImages[gImgIdx[0]] + (System3Machine::kDrives > 1 && gImgIdx[1] >= 0 ? "  " + unitTag(1) + gImages[gImgIdx[1]] : ""), 5000);

    xTaskCreatePinnedToCore(emuTask, "emu", 8192, nullptr, 2, nullptr, 0);
}

void loop() {
    static uint32_t last = 0;
    handleKeys();
    const uint32_t now = millis();
    if (now - last >= 33) { last = now; drawFrame(now); }
    delay(1);
}
