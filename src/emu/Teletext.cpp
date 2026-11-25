#include "Teletext.h"

#include <string.h>

// ---- Decodificacion de codigos de control --------------------------------
// "Set-at" actuan en la propia celda de control; "set-after" desde la siguiente.
//   01-07 color alfa (after)   08 parpadeo (after)   09 fijo (at)
//   0C altura normal (at)      0D doble altura (after)
//   11-17 color mosaico (after)   18 ocultar (at)   19 contiguo (at)   1A separado (at)
//   1C fondo negro (at)   1D nuevo fondo (at)   1E retener mosaico (at)   1F soltar (after)
// La celda de control se muestra como espacio, o como el ultimo mosaico si
// "retener" esta activo en modo grafico.
void Teletext::decodeRow(const uint8_t* codes, uint8_t cols, bool lowerHalf, TeletextCell* out, bool* hasDouble) {
    uint8_t fg = 7, bg = 0, held = 0;
    bool gfx = false, sep = false, conceal = false, hold = false, flash = false, dh = false, any = false;

    for (uint8_t i = 0; i < cols; i++) {
        const uint8_t code = codes[i] & 0x7F;
        TeletextCell& c = out[i];
        c.code = 0x20; c.flags = 0;

        if (code < 0x20) {
            switch (code) {
                case 0x09: flash = false; break;
                case 0x0C: dh = false; break;
                case 0x18: conceal = true; break;
                case 0x19: sep = false; break;
                case 0x1A: sep = true; break;
                case 0x1C: bg = 0; break;
                case 0x1D: bg = fg; break;
                case 0x1E: hold = true; break;
                default: break;
            }
            c.fg = fg; c.bg = bg;
            if (hold && gfx && !conceal) { c.code = held; c.flags |= TC_MOSAIC | (sep ? TC_SEPARATED : 0); }
            if (flash) c.flags |= TC_FLASH;                          // con el estado previo a los "set-after"
            if (dh) c.flags |= lowerHalf ? TC_DH_BOTTOM : TC_DH_TOP;
            if (code >= 0x01 && code <= 0x07) { gfx = false; fg = code; conceal = false; held = 0; }
            else if (code == 0x08) flash = true;
            else if (code == 0x0D) { dh = true; any = true; }
            else if (code >= 0x11 && code <= 0x17) { gfx = true; fg = (uint8_t)(code & 7); conceal = false; }
            else if (code == 0x1F) hold = false;
        } else {
            c.fg = fg; c.bg = bg;
            if (conceal) {
                c.code = 0x20;
            } else if (gfx && (code & 0x20)) {            // en modo mosaico solo 0x40-0x5F siguen siendo letras
                held = (uint8_t)((code & 0x1F) | ((code & 0x40) ? 0x20 : 0));
                c.code = held; c.flags |= TC_MOSAIC | (sep ? TC_SEPARATED : 0);
            } else {
                c.code = code;
            }
            if (flash) c.flags |= TC_FLASH;
            if (dh) c.flags |= lowerHalf ? TC_DH_BOTTOM : TC_DH_TOP;
        }
    }
    if (hasDouble) *hasDouble = any;
}

void Teletext::decodeScreen(const uint8_t* vram, uint16_t vmask, uint16_t start, uint8_t cols, uint8_t rows, TeletextCell* out) {
    uint8_t line[64];
    if (cols > 64) cols = 64;
    bool prevDouble = false, prevLower = false;
    for (uint8_t r = 0; r < rows; r++) {
        for (uint8_t c = 0; c < cols; c++) line[c] = vram[(uint16_t)(start + (uint16_t)r * cols + c) & vmask];
        const bool lower = prevDouble && !prevLower;     // segunda fila de una pareja de doble altura
        bool dbl = false;
        decodeRow(line, cols, lower, out + (uint16_t)r * cols, &dbl);
        prevLower = lower;
        prevDouble = dbl;
    }
}

// ---- Rectangulos -----------------------------------------------------------
// Escalado de la celda 6x10 del SAA5050 a kCellW x kCellH (16x18 por defecto): columnas con
// anchos 3,3,2,3,3,2; las filas se reparten por proporcion. En doble altura se estiran 5 filas fuente por mitad.
static uint16_t alphaRowMask(const uint8_t* rom, uint8_t code, uint8_t srcRow) {
    if (code < 0x20 || code > 0x7F) return 0;
    const uint8_t bits = rom[(uint16_t)(code - 0x20) * 10 + srcRow] & 0x1F;   // bit 4 = columna 1 de la celda
    uint16_t m = 0;
    for (uint8_t x = 0; x < Teletext::kCellW; x++) {
        const uint8_t sc = (uint8_t)(x * 6 / 16);                              // columna 0..5 de la celda 6x10
        if (sc >= 1 && (bits >> (5 - sc)) & 1) m |= (uint16_t)(0x8000 >> x);
    }
    return m;
}

static uint16_t spanMask(uint8_t x0, uint8_t x1) {    // columnas [x0,x1)
    uint16_t m = 0;
    for (uint8_t x = x0; x < x1; x++) m |= (uint16_t)(0x8000 >> x);
    return m;
}

uint8_t Teletext::cellRects(const TeletextCell& c, bool flashVisible, const uint8_t* rom, TeletextRect* out) {
    if ((c.flags & TC_FLASH) && !flashVisible) return 0;
    uint16_t rows[kCellH];

    if (c.flags & TC_MOSAIC) {
        // Tercios del mosaico (3,4,3 lineas de 10) escalados a kCellH; separado = hueco de 2 px por abajo y por la derecha.
        const uint8_t t1 = (uint8_t)((kCellH * 3 + 5) / 10), t2 = (uint8_t)((kCellH * 7 + 5) / 10);
        const uint8_t yb[2][3][2] = { { {0, t1}, {t1, t2}, {t2, kCellH} },
                                      { {0, (uint8_t)(t1 - 2)}, {t1, (uint8_t)(t2 - 2)}, {t2, (uint8_t)(kCellH - 2)} } };
        static const uint8_t xb[2][2][2] = { { {0, 8}, {8, 16} }, { {0, 6}, {8, 14} } };
        const uint8_t s = (c.flags & TC_SEPARATED) ? 1 : 0;
        memset(rows, 0, sizeof rows);
        for (uint8_t t = 0; t < 3; t++) {
            uint16_t m = 0;
            if (c.code & (1 << (t * 2))) m |= spanMask(xb[s][0][0], xb[s][0][1]);
            if (c.code & (2 << (t * 2))) m |= spanMask(xb[s][1][0], xb[s][1][1]);
            for (uint8_t y = yb[s][t][0]; y < yb[s][t][1]; y++) rows[y] = m;
        }
        // Doble altura en mosaicos: cada mitad ocupa la mitad del patron vertical (aproximacion: se estira).
        if (c.flags & (TC_DH_TOP | TC_DH_BOTTOM)) {
            uint16_t tmp[kCellH];
            // se construye la imagen doble (38 filas) y se toma la mitad que toca
            uint16_t full[kCellH * 2];
            for (uint8_t y = 0; y < kCellH * 2; y++) full[y] = rows[y / 2];
            for (uint8_t y = 0; y < kCellH; y++) tmp[y] = full[(c.flags & TC_DH_BOTTOM) ? (kCellH + y) : y];
            memcpy(rows, tmp, sizeof rows);
        }
    } else {
        for (uint8_t y = 0; y < kCellH; y++) {
            uint8_t src;
            if (c.flags & TC_DH_TOP) src = (uint8_t)(y * 5 / kCellH);
            else if (c.flags & TC_DH_BOTTOM) src = (uint8_t)(5 + y * 5 / kCellH);
            else src = (uint8_t)(y * 10 / kCellH);
            rows[y] = alphaRowMask(rom, c.code, src);
        }
    }

    // Une filas iguales consecutivas y convierte cada mascara en tramos horizontales.
    uint8_t n = 0;
    uint8_t y = 0;
    while (y < kCellH) {
        uint8_t y2 = (uint8_t)(y + 1);
        while (y2 < kCellH && rows[y2] == rows[y]) y2++;
        const uint16_t m = rows[y];
        uint8_t x = 0;
        while (m && x < kCellW) {
            if (m & (0x8000 >> x)) {
                uint8_t x2 = x;
                while (x2 < kCellW && (m & (0x8000 >> x2))) x2++;
                if (n < kMaxRects) { out[n].x = x; out[n].y = y; out[n].w = (uint8_t)(x2 - x); out[n].h = (uint8_t)(y2 - y); n++; }
                x = x2;
            } else x++;
        }
        y = y2;
    }
    return n;
}

// Une filas iguales consecutivas y convierte cada mascara de w puntos (bit 15 = columna 0) en tramos horizontales.
static uint8_t rowsToRects(const uint16_t* rows, uint8_t w, uint8_t h, TeletextRect* out) {
    uint8_t n = 0, y = 0;
    while (y < h) {
        uint8_t y2 = (uint8_t)(y + 1);
        while (y2 < h && rows[y2] == rows[y]) y2++;
        const uint16_t m = rows[y];
        uint8_t x = 0;
        while (m && x < w) {
            if (m & (0x8000 >> x)) {
                uint8_t x2 = x;
                while (x2 < w && (m & (0x8000 >> x2))) x2++;
                if (n < Teletext::kMaxRects) { out[n].x = x; out[n].y = y; out[n].w = (uint8_t)(x2 - x); out[n].h = (uint8_t)(y2 - y); n++; }
                x = x2;
            } else x++;
        }
        y = y2;
    }
    return n;
}

uint8_t Teletext::vduRects(uint8_t code, const uint8_t* font, uint8_t w, uint8_t h, TeletextRect* out) {
    if (w == 0 || w > 16 || h == 0 || h > 64) return 0;
    uint16_t rows[64];
    const uint16_t base = (uint16_t)((code & 0x7F) << 4);
    for (uint8_t y = 0; y < h; y++) {
        const uint8_t bits = font[base + (uint8_t)(y * 10 / h)];          // 10 lineas por celda
        uint16_t m = 0;
        for (uint8_t x = 0; x < w; x++) if ((bits >> (7 - x * 6 / w)) & 1) m |= (uint16_t)(0x8000 >> x);
        rows[y] = m;
    }
    return rowsToRects(rows, w, h, out);
}
