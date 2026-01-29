/*
C driver for VCNL4020 Proximity and Ambient Light Sensor
This driver provides functions to initialize the sensor, set the IR LED current,
set the sensor mode, and read proximity and ambient light values.
It is designed to be used with Arduino and compatible platforms.

Author: ARI ESTERS
Date: 24 August 2025
*/

#include "VCNL4020.h"
#include <stdint.h>
#include <Wire.h>      // For Arduino and AtTiny I2C communication

// Initialize sensor in idle state
VCNL4020_Status VCNL4020_Init(VCNL4020 *pDevice) 
{
    // Initial delay to allow sensor to power up
    delay(10);

    // Set the sensor's initial configuration
    pDevice->address = 0x13;    // Default I2C address
    pDevice->current = 0; 
    pDevice->mode = MANUAL;
    pDevice->prox_rate = 0;

    // Check if sensor is connected via I2C
    pDevice->connected = VCNL4020_IsConnected(pDevice);
    if (!pDevice->connected) {
        return STATUS_CONNECTION_ERROR; 
    }

   return STATUS_OK;
}

bool VCNL4020_IsConnected(VCNL4020 *pDevice) 
{
    uint8_t id = 0;

    // Product ID/revision register should return 0x21
    id = (uint8_t) I2C_ReadRegister(pDevice->address, REG_ID, 1);
    return (id == 0x21); 
}

// Set IR LED current for proximity measurements (does not turn on the LED)
VCNL4020_Status VCNL4020_SetCurrent(VCNL4020 *pDevice, uint8_t led_current) 
{
    uint8_t reg_value = 0;

    // Check for connection
    pDevice->connected = VCNL4020_IsConnected(pDevice);
    if (!pDevice->connected) {
        return STATUS_CONNECTION_ERROR; 
    }

    // Check for valid current values: 0, 10, 20, ..., 200 mA
    if (led_current > 200 || (led_current % 10) != 0) {
        return STATUS_INVALID_PARAM;     // Invalid current value
    }
    
    // Write to register via I2C
    reg_value = led_current / 10;
    I2C_WriteRegister(pDevice->address, REG_CURRENT, 1, reg_value);
    pDevice->current = led_current;
    
    return STATUS_OK;
}

VCNL4020_Status VCNL4020_SetMode(VCNL4020 *pDevice, VCNL4020_Mode mode) 
{
    uint8_t reg_value = 0;

    // Check for connection
    pDevice->connected = VCNL4020_IsConnected(pDevice);
    if (!pDevice->connected) {
        return STATUS_CONNECTION_ERROR; 
    }

    // Mode should be three bits (0-7)
    if (mode > 0x07) {
        return STATUS_INVALID_PARAM; // Invalid mode
    }

    // Update bits 0-2 based on mode
    I2C_WriteRegister(pDevice->address, REG_COMMAND, 1, mode);
    delay(10);

    // Update struct
    pDevice->mode = mode;

    return STATUS_OK;
}

/*
@brief VCNL4020 proximity measurement rates
@details 
Max rate is 250 measurements/second.
Each rate setting divides the max rate by a factor of 2^n:
@note Value of `rate` ranges from 0 to 7.
@note - 0: 1.953125 measurements/second
@note - 1: 3.90625 measurements/second
@note - 2: 7.8125 measurements/second
@note - 3: 15.625 measurements/second
@note - 4: 31.25 measurements/second
@note - 5: 62.5 measurements/second
@note - 6: 125 measurements/second
@note - 7: 250 measurements/second
*/
VCNL4020_Status VCNL4020_SetProxMeasurementRate(VCNL4020 *pDevice, uint8_t rate) 
{
    // Check for connection
    pDevice->connected = VCNL4020_IsConnected(pDevice);
    if (!pDevice->connected) {
        return STATUS_CONNECTION_ERROR; 
    }

    // Check for valid rate values
    if (rate > 7) {
        return STATUS_INVALID_PARAM; // Invalid rate factor
    }

    // Update proximity rate
    I2C_WriteRegister(pDevice->address, REG_PROX_RATE, 1, rate);

    return STATUS_OK;
}

uint16_t VCNL4020_ReadProximity(VCNL4020 *pDevice) 
{
    /*
    If mode is MANUAL: collects single measurement and read 
    If mode is AUTO: reads the most recent measurement
    */
    uint16_t reg_value = 0;
    VCNL4020_Status status;

    // Check for connection
    pDevice->connected = VCNL4020_IsConnected(pDevice);
    if (!pDevice->connected) {
        return (uint16_t) STATUS_CONNECTION_ERROR; 
    }

    // In MANUAL mode, trigger a single measurement
    if (pDevice->mode == MANUAL) {
        I2C_SetBit(pDevice->address, REG_COMMAND, 3, true);  // Set bit 3
        delay(10);
    }
    else {
        // In AUTO modes, wait for data ready
        uint8_t status_reg = 0;
        do {
            status_reg = (uint8_t) I2C_ReadRegister(pDevice->address, REG_COMMAND, 1);
        } while ((status_reg & (1 << 5)) == 0); // Wait until bit 6 is set
    }

    // Read proximity data from the sensor
    reg_value = I2C_ReadRegister(pDevice->address, REG_PROX_RESULT, 2);

    // Clear interrupt flags
    status = VCNL4020_ClearInterrupts(pDevice);
    if (status != STATUS_OK) {
        return (uint16_t) status; 
    }

    return reg_value; 
}

VCNL4020_Status VCNL4020_DisableInterrupts(VCNL4020 *pDevice) 
{
    // Check for connection
    pDevice->connected = VCNL4020_IsConnected(pDevice);
    if (!pDevice->connected) {
        return STATUS_CONNECTION_ERROR; 
    }

    // Write 0 to disable all interrupts
    I2C_WriteRegister(pDevice->address, REG_INT_CTRL, 1, 0x00);

    return STATUS_OK;
}

VCNL4020_Status VCNL4020_ClearInterrupts(VCNL4020 *pDevice) 
{
    // Check for connection
    pDevice->connected = VCNL4020_IsConnected(pDevice);
    if (!pDevice->connected) {
        return STATUS_CONNECTION_ERROR; 
    }

    // Write 0b1111 to clear all interrupt flags
    I2C_WriteRegister(pDevice->address, REG_INT_STATUS, 1, 0x0F);

    return STATUS_OK;
}

/*
I2C helper functions: 
All I2C functions read and write values in big-endian format.
*/
uint16_t I2C_ReadRegister(uint8_t dev_address, uint8_t reg_address, uint8_t length)
{ 
  uint16_t reg_value = 0;   // max 2 bytes
  uint8_t n_bytes_read;     // number of bytes actually read from device
  uint8_t this_byte;
  uint8_t n_bitshifts;

  // start I2C comms with device
  Wire.beginTransmission(dev_address);  

  // set target register for reading
  Wire.write(reg_address);
  Wire.endTransmission();

  // read contents from registers in big-endian order
  n_bytes_read = Wire.requestFrom(dev_address, length); 
  if (n_bytes_read < length) return 0xFFFF;

  for (int i = 0; i < length; i++) {    
    this_byte = Wire.read();
    n_bitshifts = 8 * (length - 1 - i);
    reg_value |= uint16_t (this_byte << n_bitshifts);
    // device auto-increments register after each read
  }
  return reg_value;
}

void I2C_WriteRegister(uint8_t dev_address, uint8_t reg_address, uint8_t length, uint16_t data) 
{
    uint8_t this_byte = 0;
    uint8_t n_bitshifts = 0;

    // start I2C comms with device
    Wire.beginTransmission(dev_address);  

    // set target register for writing
    Wire.write(reg_address);

    // write data to registers in big-endian order
    for (int i = 0; i < length; i++) {
        n_bitshifts = 8 * (length - 1 - i);      
        this_byte = (data >> n_bitshifts) & 0xFF; // extract byte to send  
        Wire.write(this_byte);
        // device auto-increments register after each write
    }
    Wire.endTransmission();
}

void I2C_SetBit(uint8_t dev_address, uint8_t reg_address, uint8_t bit_pos, bool set_clear)
{
    uint8_t reg_value = 0;

    // Read current register value
    reg_value = I2C_ReadRegister(dev_address, reg_address, 1);

    // Modify bit
    if (set_clear)  // Set 
        reg_value |= (1 << bit_pos);   
    else            // Clear 
        reg_value &= ~(1 << bit_pos);  

    // Update register value 
    I2C_WriteRegister(dev_address, reg_address, 1, reg_value);
}