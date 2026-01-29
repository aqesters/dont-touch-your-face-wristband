// STC3115.c - C version of STC3115 C++ driver

/*
Need to add read and write functions for I2C communication.
*/

#include "STC3115_C.h"
#include <Arduino.h>

#if DEBUG
  #define DEBUG_PRINT(x) do { Serial.println(F(x)); Serial.flush(); delay(100); } while (0)
#else
  #define DEBUG_PRINT(x)
#endif

void STC3115_init(STC3115* dev, uint16_t capacity, uint8_t rsense, uint16_t resInt) {
    dev->_address = 0b1110000;
    dev->_capacity = capacity;
    dev->_rsense = rsense;
    dev->_resInt = resInt;
    memset(dev->_ram, 0, sizeof(dev->_ram));
}

void STC3115_startup(STC3115* dev) {
    DEBUG_PRINT("Commencing device startup...");
    uint8_t partID = read_u8(REG_ID, 1);
    if (partID == 0x14) {
        bool needsInit = false;
        for (int i = 0; i < RAM_SIZE; i++) {
            if (read_u8(REG_RAM + i, 1) != dev->_ram[i]) {
                needsInit = true;
                break;
            }
        }
        if (needsInit) {
            DEBUG_PRINT("Cannot restore from RAM. Resetting RAM...");
            for (int i = 0; i < RAM_SIZE; i++) {
                write_u8(0, REG_RAM + i, 1);
            }
            STC3115_initialize(dev);
            STC3115_run(dev);
        } else {
            DEBUG_PRINT("RAM is OK.");
            uint8_t status = read_u8(REG_CTRL, 1);
            bool batfail = (status >> 3) & 1;
            bool pordet = (status >> 4) & 1;
            if (batfail || pordet) {
                DEBUG_PRINT("Component lost power in prior operation.");
                write_u8(0b11, REG_CTRL, 1);
                STC3115_initialize(dev);
            } else {
                STC3115_restore(dev);
            }
            STC3115_run(dev);
        }
    }
}

void STC3115_initialize(STC3115* dev) {
    DEBUG_PRINT("Initializing...");
    uint16_t tempOCV = read_u16(REG_OCV, 2);
    STC3115_setParameters(dev);
    write_u16(tempOCV, REG_OCV, 2);
    delay(200);
    DEBUG_PRINT("Initialization complete.");
}

void STC3115_restore(STC3115* dev) {
    DEBUG_PRINT("Restoring to last known battery state...");
    uint16_t tempSOC = read_u16(RAM_SOC, 2);
    STC3115_setParameters(dev);
    write_u16(tempSOC, REG_SOC, 2);
    DEBUG_PRINT("Restoration complete.");
}

void STC3115_setParameters(STC3115* dev) {
    DEBUG_PRINT("Setting parameters...");
    uint8_t tempMode = read_u8(REG_MODE, 1);
    tempMode &= ~(1 << 4);
    write_u8(tempMode, REG_MODE, 1);
    if (dev->_rsense && dev->_capacity) {
        DEBUG_PRINT("Overwriting settings for current sensing and control...");
        uint16_t regCC = dev->_rsense * dev->_capacity / 49.556;
        write_u16(regCC, REG_CC_CNF, 2);
        uint8_t relaxCurrent = dev->_capacity / 10;
        uint8_t regRelax = (uint8_t)((relaxCurrent * dev->_rsense) / 47.04);
        write_u8(regRelax, REG_CURRENT_THRES, 1);
    }
    if (dev->_resInt && dev->_capacity) {
        DEBUG_PRINT("Overwriting settings for voltage estimation algorithm...");
        uint16_t regVM = dev->_resInt * dev->_capacity / 977.78;
        write_u16(regVM, REG_VM_CNF, 2);
    }
    STC3115_updateRAM(dev);
}

void STC3115_run(STC3115* dev) {
    DEBUG_PRINT("Switching to \"Run\" mode...");
    uint8_t tempCtrl = read_u8(REG_CTRL, 1);
    tempCtrl &= ~(0b11 << 5);
    write_u8(tempCtrl, REG_CTRL, 1);
    write_u8(1 << 4, REG_MODE, 1);
}

void STC3115_stop(STC3115* dev) {
    STC3115_updateRAM(dev);
    DEBUG_PRINT("Switching to \"Standby\" mode...");
    uint8_t tempMode = read_u8(REG_MODE, 1);
    tempMode &= ~(1 << 4);
    write_u8(tempMode, REG_MODE, 1);
}

void STC3115_check(STC3115* dev) {
    uint8_t tempCtrl = read_u8(REG_CTRL, 1);
    uint8_t issues = (tempCtrl >> 3) & 0b11;
    if (issues) {
        DEBUG_PRINT("Power-on issue detected. Restarting...");
        STC3115_startup(dev);
    } else {
        STC3115_run(dev);
    }
}
