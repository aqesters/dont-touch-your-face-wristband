#ifndef VCNL4020_H
#define VCNL4020_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// Registers in VCNL4020 proximity device
#define DEV_ADDRESS 0x13  // I2C address for VCNL4020
#define REG_COMMAND 0X80    // Command register 
#define REG_ID 0x81    // Product ID and Revision register
#define REG_PROX_RATE 0x82    // Proximity measurement rate register
#define REG_CURRENT 0x83    // IR LED current register
//#define REG_LIGHT_RATE 0x84    // Ambient light measurement rate register
//#define REG_LIGHT_RESULT 0x85    // Ambient light results upper byte
#define REG_PROX_RESULT 0x87    // Proximity results upper byte
#define REG_INT_CTRL 0x89    // Interrupt control
//#define REG_THRESH_L 0x8A    // Low threshold upper byte
//#define REG_THRESH_H 0x8C    // High threshold upper byte
#define REG_INT_STATUS 0x8E    // Read and clear interrupts (to clear all, write "0b1111")

// Status codes for functions
typedef uint8_t VCNL4020_Status;
enum {
    STATUS_OK = 0,
    STATUS_CONNECTION_ERROR,
    STATUS_INVALID_PARAM,
    STATUS_UNKNOWN_ERROR
};

/*
@brief VCNL4020 modes
@details
- DISABLED: Sensor is idle, no measurements taken
- MANUAL: Self-timed mode, measurements taken by explicit command
- AUTO_PROX: Enables periodic proximity measurements
- AUTO_ALS: Enables periodic ambient light sensing measurements
- FULL_AUTO: Periodically measure proximity and ambient light 
@note - If MANUAL mode is set, then readProximity() and readAmbientLight() perform a single measurement.
@note - If FULL_AUTO mode is set, then readProximity() and readAmbientLight() return the last measured value.
*/
typedef uint8_t VCNL4020_Mode;
#define MANUAL     ((VCNL4020_Mode) 0b000)
#define AUTO_PROX  ((VCNL4020_Mode) 0b011)
#define AUTO_ALS   ((VCNL4020_Mode) 0b101)
#define FULL_AUTO  ((VCNL4020_Mode) 0b111)

// Struct for VCNL4020 device configuration
typedef struct 
{
    uint8_t address; // I2C address of the VCNL4020 sensor
    uint8_t current; // IR LED current setting
    VCNL4020_Mode mode;
    uint8_t prox_rate;
    bool connected;
} VCNL4020;

// Functions for sensor 
VCNL4020_Status VCNL4020_Init(VCNL4020 *pDevice);
bool VCNL4020_IsConnected(VCNL4020 *pDevice);
VCNL4020_Status VCNL4020_SetCurrent(VCNL4020 *pDevice, uint8_t current);
VCNL4020_Status VCNL4020_SetMode(VCNL4020 *pDevice, VCNL4020_Mode mode);
VCNL4020_Status VCNL4020_SetProxMeasurementRate(VCNL4020 *pDevice, uint8_t rate);
uint16_t VCNL4020_ReadProximity(VCNL4020 *pDevice);
VCNL4020_Status VCNL4020_DisableInterrupts(VCNL4020 *pDevice);
VCNL4020_Status VCNL4020_ClearInterrupts(VCNL4020 *pDevice);
//uint16_t VCNL4020_ReadAmbientLight(VCNL4020 *pDevice);
//uint8_t VCNL4020_SetProxMeasurementRate(VCNL4020 *pDevice, uint8_t rate_factor);
//uint8_t VCNL4020_SetAmbientMeasurementRate(VCNL4020 *pDevice, uint8_t rate_factor);
//uint8_t VCNL4020_SetInterrupts(VCNL4020 *pDevice, ...);
//uint8_t VCNL4020_InterruptStatus(VCNL4020 *pDevice);

// Helper functions
uint16_t I2C_ReadRegister(uint8_t dev_address, uint8_t reg_address, uint8_t length);
void I2C_WriteRegister(uint8_t dev_address, uint8_t reg_address, uint8_t length, uint16_t data);
void I2C_SetBit(uint8_t dev_address, uint8_t reg_address, uint8_t bit_pos, bool set_clear);

/* Coming soon: The above functions that are commented out! */

#ifdef __cplusplus
}      // extern "C"
#endif

#endif // VCNL4020_H