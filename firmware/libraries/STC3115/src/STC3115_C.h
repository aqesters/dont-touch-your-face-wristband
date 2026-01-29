#ifndef STC3115_H
#define STC3115_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <Arduino.h>

#define DEBUG 0

// Battery gauge registers (little-endian format)
#define REG_MODE            0
#define REG_CTRL            1
#define REG_SOC             2
#define REG_COUNTER         4
#define REG_CURRENT         6
#define REG_VOLTAGE         8
#define REG_OCV             13
#define REG_CC_CNF          15
#define REG_VM_CNF          17
#define REG_CURRENT_THRES   21
#define REG_ID              24
#define REG_RAM             32

// Mapping for RAM storage (device registers 32-47)
#define RAM_TEST            32
#define RAM_SOC             33
#define RAM_OCV             35
#define RAM_CC_CNF          37
#define RAM_VM_CNF          39
#define RAM_CHECKSUM        41
#define RAM_SIZE            (RAM_CHECKSUM - REG_RAM)

// STC3115 device struct
typedef struct {
    uint8_t _address;
    uint16_t _capacity;
    uint8_t _rsense;
    uint16_t _resInt;
    uint8_t _ram[RAM_SIZE];
} STC3115;

// Public interface
void STC3115_init(STC3115* dev, uint16_t capacity, uint8_t rsense, uint16_t resInt);
void STC3115_startup(STC3115* dev);
void STC3115_initialize(STC3115* dev);
void STC3115_restore(STC3115* dev);
void STC3115_setParameters(STC3115* dev);
void STC3115_updateRAM(STC3115* dev);
void STC3115_run(STC3115* dev);
void STC3115_stop(STC3115* dev);
void STC3115_check(STC3115* dev);
float STC3115_readVoltage(STC3115* dev);
float STC3115_readOCV(STC3115* dev);
float STC3115_readCurrent(STC3115* dev);
float STC3115_readSOC(STC3115* dev);

// Low-level I2C read/write helpers (implement separately)
uint8_t read_u8(uint8_t reg, uint8_t len);
uint16_t read_u16(uint8_t reg, uint8_t len);
int16_t read_i16(uint8_t reg, uint8_t len);
void write_u8(uint8_t value, uint8_t reg, uint8_t len);
void write_u16(uint16_t value, uint8_t reg, uint8_t len);

#endif // STC3115_H
