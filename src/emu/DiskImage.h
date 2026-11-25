#pragma once

// Almacenamiento de imagenes de disco del System 3.
//
// FloppyDrive habla con la interfaz abstracta DiskImage. En la TTGO VGA32 se usa FileDiskImage sobre la
// tarjeta SD (FabGL monta la FAT en /SD y se accede con fopen/fwrite); en las pruebas del PC tambien hay
// MemoryDiskImage. Las escrituras se vuelcan al fichero (fflush) y se fuerzan a la SD con flush()
// cuando la controladora queda inactiva.
#include <stdint.h>
#include <stdio.h>
#include <vector>

class DiskImage {
public:
    virtual ~DiskImage() {}
    virtual uint32_t size() const = 0;
    virtual bool read(uint32_t offset, uint8_t* dst, uint16_t n) = 0;
    virtual bool write(uint32_t offset, const uint8_t* src, uint16_t n) = 0;
    virtual bool readOnly() const = 0;   // proteccion del propio soporte (fichero de solo lectura)
    virtual void flush() {}              // fuerza los datos al medio (SD): se llama cuando la unidad queda inactiva
    // Amplia la imagen hasta 'newSize' bytes rellenando con 'fill' (FORMAT de pistas posteriores al final, como el emulador de
    // referencia). Devuelve false si el soporte no puede crecer.
    virtual bool grow(uint32_t newSize, uint8_t fill) { (void)newSize; (void)fill; return false; }
};

// Imagen en memoria (tests). flush() no hace nada.
class MemoryDiskImage : public DiskImage {
public:
    explicit MemoryDiskImage(uint32_t size, uint8_t fill = 0xE5) : data_(size, fill), ro_(false) {}
    MemoryDiskImage(const uint8_t* d, uint32_t n) : data_(d, d + n), ro_(false) {}
    uint32_t size() const override { return (uint32_t)data_.size(); }
    bool read(uint32_t off, uint8_t* dst, uint16_t n) override;
    bool write(uint32_t off, const uint8_t* src, uint16_t n) override;
    bool readOnly() const override { return ro_; }
    bool grow(uint32_t newSize, uint8_t fill) override { if (ro_ || newSize < data_.size()) return false; data_.resize(newSize, fill); return true; }
    void setReadOnly(bool r) { ro_ = r; }
    std::vector<uint8_t>& bytes() { return data_; }
private:
    std::vector<uint8_t> data_;
    bool ro_;
};

// Imagen plana en un fichero. Las escrituras se hacen directamente sobre el
// fichero (con fflush), de modo que los cambios persisten entre ejecuciones.
class FileDiskImage : public DiskImage {
public:
    FileDiskImage() : f_(0), size_(0), ro_(false) {}
    ~FileDiskImage() override { close(); }
    bool open(const char* path);          // intenta lectura/escritura; si no, solo lectura
    void close();
    uint32_t size() const override { return size_; }
    bool read(uint32_t off, uint8_t* dst, uint16_t n) override;
    bool write(uint32_t off, const uint8_t* src, uint16_t n) override;
    bool readOnly() const override { return ro_; }
    void flush() override;                // fflush + fsync
    bool grow(uint32_t newSize, uint8_t fill) override;
private:
    FILE* f_;
    uint32_t size_;
    bool ro_;
};
