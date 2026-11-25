#pragma once

#include <stdint.h>

// Decodificador de teletexto SAA5050 (version UK), independiente de la pantalla.
// Convierte la RAM de video en celdas con atributos y las celdas en rectangulos
// de primer plano, de modo que el dibujo en FabGL solo necesita fillRectangle().
// Se prueba en el host (tests/test_teletext.cpp).
//
// Glifos: se leen de la ROM real (roms/saa5050.bin, 96 caracteres x 10 filas,
// 5 bits por fila en los bits 4..0, celda de 6x10). No se emula el suavizado
// de diagonales ("character rounding") del chip.
struct TeletextCell {
    uint8_t code;     // alfanumerico: 0x20-0x7F; mosaico (TC_MOSAIC): patron de 6 bits
    uint8_t fg, bg;   // 0-7: bit0 rojo, bit1 verde, bit2 azul
    uint8_t flags;
    bool operator==(const TeletextCell& o) const {
        return code == o.code && fg == o.fg && bg == o.bg && flags == o.flags;
    }
    bool operator!=(const TeletextCell& o) const { return !(*this == o); }
};

enum TeletextFlags : uint8_t {
    TC_MOSAIC = 1,      // 'code' es un patron de mosaico
    TC_SEPARATED = 2,   // mosaico separado
    TC_FLASH = 4,       // parpadeo
    TC_DH_TOP = 8,      // mitad superior de caracter de doble altura
    TC_DH_BOTTOM = 16   // mitad inferior
};

struct TeletextRect { uint8_t x, y, w, h; };

// Altura de celda en pantalla. 18 deja 30 px libres en 640x480 (margen contra el
// sobrescaneo del monitor); 19 es el maximo que cabe (475 px).
#ifndef S3_CELL_H
#define S3_CELL_H 18
#endif

class Teletext {
public:
    static const uint8_t kCols = 40, kRows = 25;
    static const uint8_t kCellW = 16, kCellH = S3_CELL_H;   // celda en pantalla
    static const uint16_t kCharRomSize = 960;
    static const uint8_t kMaxRects = 160;

    // Decodifica una fila. 'lowerHalf': la fila anterior tenia doble altura, esta
    // muestra las mitades inferiores. 'hasDouble' (opcional) = la fila contiene 0x0D.
    static void decodeRow(const uint8_t* codes, uint8_t cols, bool lowerHalf, TeletextCell* out, bool* hasDouble);

    // Decodifica la pantalla desde la RAM de video: la celda (r,c) se lee de
    // vram[(start + r*cols + c) & vmask].
    static void decodeScreen(const uint8_t* vram, uint16_t vmask, uint16_t start, uint8_t cols, uint8_t rows, TeletextCell* out);

    // Rectangulos de primer plano de la celda (coordenadas dentro de kCellW x kCellH).
    // Devuelve el numero de rectangulos; 0 si es solo fondo (espacio, oculto por parpadeo...).
    static uint8_t cellRects(const TeletextCell& c, bool flashVisible, const uint8_t* charRom, TeletextRect* out);

    // Texto de la tarjeta VDU de 80 columnas: glifo de su ROM de caracteres (roms/vdu80.chr: 2 KB = 128 caracteres x 16 filas,
    // 6 puntos en los bits 7..2; el bit 7 del codigo se ignora) escalado a w x h puntos (cuerpo de 10 lineas). Los codigos
    // $01-$1F son simbolos graficos reales de esa ROM (barras superior, central e inferior, etc.).
    static uint8_t vduRects(uint8_t code, const uint8_t* font, uint8_t w, uint8_t h, TeletextRect* out);
    static const uint16_t kVduFontSize = 2048;

    // Celda de texto plano (para mensajes en pantalla): blanco sobre 'bg'.
    static TeletextCell textCell(char ch, uint8_t fg, uint8_t bg) {
        TeletextCell c; const uint8_t u = (uint8_t)ch; c.code = (u >= 0x20 && u < 0x80) ? u : 0x20; c.fg = fg; c.bg = bg; c.flags = 0; return c;
    }
};
