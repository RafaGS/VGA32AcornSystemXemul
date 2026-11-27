#pragma once

// Seleccion de modelo EN COMPILACION (platformio.ini: build_flags):
//   -DS3_MODEL_SYSTEM3   Acorn System 3 minimo: 8 KB RAM + BASIC 4 KB ROM y una unidad de 100 KB
//   -DS3_MODEL_SYSTEM4   Perfil System 4 anterior: 16 KB RAM y dos unidades
//
// El System 3 minimo solicitado utiliza la RAM 2000-3FFF y la ROM BASIC C000-CFFF de la tarjeta 200,003.
// La ROM DOS permanece en F000-FFFF. El System 4 conserva el perfil de 16 KB con BASIC cargado desde disco a C000.
#if defined(S3_MODEL_SYSTEM3) && defined(S3_MODEL_SYSTEM4)
#error "Define solo uno de S3_MODEL_SYSTEM3 / S3_MODEL_SYSTEM4"
#endif

#if defined(S3_MODEL_SYSTEM4)
#define S3_MODEL_NAME "Acorn System 4"
#define S3_DRIVES 2
#define S3_RAM_KB 16
#define S3_HAS_BASIC_ROM 0
#define S3_DRIVE_TRACKS 80
#else
#define S3_MODEL_NAME "Acorn System 3"
#define S3_DRIVES 1
#define S3_RAM_KB 8
#define S3_HAS_BASIC_ROM 1
#define S3_DRIVE_TRACKS 40
#endif

// Tarjetas del System 5 (NO autenticas en un System 3/4): VIA 6522 en $0E00 con el teclado en su puerto A, VDU de 80
// columnas (RAM en $1000, 6845 en $1840) y 48 KB de DRAM contiguos en $2000-$DFFF (el juego usa variables en $5000). Hacen falta para ejecutar el juego Space Invaders del emulador de referencia, que
// esta escrito para el System 5. Se activan con -DS3_SYSTEM5_EXTRAS (entorno system4_invaders de platformio.ini).
#if defined(S3_SYSTEM5_EXTRAS)
#define S3_EXTRAS 1
#else
#define S3_EXTRAS 0
#endif
