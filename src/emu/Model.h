#pragma once

// Seleccion de modelo EN COMPILACION (platformio.ini: build_flags):
//   -DS3_MODEL_SYSTEM3   Acorn System 3: una unidad de disquete (valor por defecto)
//   -DS3_MODEL_SYSTEM4   Acorn System 4: dos unidades de disquete
// Fuente: la definicion de sistemas del emulador de referencia (elaverick/SystemEmulator) y la descripcion publica de
// Wikipedia: el System 4 es como el System 3 en un bastidor de doble altura (14 ranuras) con DOS unidades de disquete. Mismas
// tarjetas (CPU 200,000, RAM 8K x 2, VDU 40x25, controladora 8271) y la misma ROM del DOS.
#if defined(S3_MODEL_SYSTEM3) && defined(S3_MODEL_SYSTEM4)
#error "Define solo uno de S3_MODEL_SYSTEM3 / S3_MODEL_SYSTEM4"
#endif

#if defined(S3_MODEL_SYSTEM4)
#define S3_MODEL_NAME "Acorn System 4"
#define S3_DRIVES 2
#else
#define S3_MODEL_NAME "Acorn System 3"
#define S3_DRIVES 1
#endif

#define S3_RAM_KB 16        // RAM de serie en ambos modelos: 8 KB en $2000 + 8 KB en $C000 (con 8: solo la de $2000)

// Tarjetas del System 5 (NO autenticas en un System 3/4): VIA 6522 en $0E00 con el teclado en su puerto A, VDU de 80
// columnas (RAM en $1000, 6845 en $1840) y 48 KB de DRAM contiguos en $2000-$DFFF (el juego usa variables en $5000). Hacen falta para ejecutar el juego Space Invaders del emulador de referencia, que
// esta escrito para el System 5. Se activan con -DS3_SYSTEM5_EXTRAS (entorno system4_invaders de platformio.ini).
#if defined(S3_SYSTEM5_EXTRAS)
#define S3_EXTRAS 1
#else
#define S3_EXTRAS 0
#endif
