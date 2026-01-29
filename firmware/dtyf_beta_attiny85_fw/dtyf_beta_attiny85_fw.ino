#include <Wire.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
//#include <STC3115.h>  // battery gauge - NOT USED IN THIS VERSION
#include <VCNL4020.h>

// Sensor inputs and thresholds
const uint8_t period = 200;                   // sampling period while awake, in ms
const float accel_threshold = 6.9;            // m/s^2, corresponds to 45 deg from vertical
const uint8_t led_current = 150;              // set between 0 and 200 mA (in increments of 10)
uint16_t prox_threshold = 0;                  // post-calibration threshold for proximity sensor

// For VCNL4020 calibration
bool do_calib = false;                        // determines whether proximity calibration is done
const uint16_t pthresh_with_calib = 1000;     // if do_calib == true
const uint16_t pthresh_no_calib = 4000;       // if do_calib == false

// Initialize variables
float accel = 0.0;
uint16_t prox = 0;
uint8_t n_calib_samples = 10;   // for proximity sensor calibration
unsigned long sum = 0;          // for proximity sensor calibration
uint16_t background = 0;        // for proximity sensor calibration
volatile bool int_triggered = false;

// Define pins
const int INT_PIN = 3;        // Triggered by accelerometer (PB3 / Attiny Pin 2)
const int ALERT_PIN = 4;      // LED indicating alert to user (PB4 / Attiny Pin 3)

// Device I2C addresses
#define ADDR_ACCEL 0X19         // LSM303AGR
#define ADDR_MAG 0x1E           // LSM303AGR

// Magnetometer registers
#define CFG_REG_A_M 0x60    // Magnetometer config

// Accelerometer registers
#define CTRL_REG1_A 0x20    // Options: enable XYZ measurements, data sampling rate, low-power mode
#define CTRL_REG4_A 0x23    // Options: full-scale, self-test
#define CTRL_REG6_A 0x25    // Interrupt pin settings
#define INT2_CFG_A 0x34     // Interrupt 2 (INT2) setup
#define INT2_SRC_A 0x35     // Check INT2 status (unlatches when read if latch is enabled)
#define INT2_THS_A 0x36     // Set threshold for INT2 function
#define ACCEL_OUT_X 0x28    // X accelerometer output
#define ACCEL_OUT_Y 0x2A    // Y accelerometer output
#define ACCEL_OUT_Z 0x2C    // Z accelerometer output

// I2C read operation (max read: 2 bytes)
template <typename T>
T i2cRead(uint8_t deviceAddress, uint8_t startRegister, uint8_t numBytes, bool bigEndian = false) 
{ 
  uint16_t rawValue = 0;   // max: 2 bytes worth  

  // start I2C comms with device
  Wire.beginTransmission(deviceAddress);  

  // set target register for reading
  Wire.write(startRegister);
  Wire.endTransmission();

  // read contents from consecutive registers
  Wire.requestFrom(deviceAddress, numBytes); 
  for (int i = 0; i < numBytes; i++) {    
    uint8_t currentByte = Wire.read();
    if (bigEndian) {
      rawValue |= uint16_t (currentByte << 8*(numBytes - 1 - i));
    }
    else {
      rawValue |= uint16_t (currentByte << 8*i);
    }
    // device auto-increments register after each read
  }

  // return in specified format
  return static_cast<T>(rawValue);
}

// I2C write operation
void i2cWrite(uint8_t command, uint8_t deviceAddress, uint8_t startRegister, uint8_t numBytes, bool bigEndian = false) 
{ 
  // start I2C comms with device
  Wire.beginTransmission(deviceAddress);

  // write one byte to each consecutive register
  Wire.write(startRegister);      
  for (int i = 0; i < numBytes; i++) { 
    uint8_t currentByte;

    // start at MSB (big endian) or LSB (little endian)
    if (bigEndian) {
      currentByte = 0xFF & (command >> 8*(numBytes - 1 - i));
    }
    else {
      currentByte = 0xFF & (command >> 8*i);
    }
    Wire.write(currentByte);
    // device auto-increments register after each write
  }
  Wire.endTransmission();
}

// Define sensor objects
//STC3115 gauge(110, 50, 100);    // capacity, sense resistor, internal resistance
VCNL4020 prox_sensor;

void setup() {
  /// BOARD SETUP ///
  pinMode(ALERT_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
  digitalWrite(ALERT_PIN, LOW);
  Wire.begin();  // initialize I2C
  delay(50);

  /// CONFIGURE ALL DEVICES ///
  setupSensors();     // accelerometer and IR sensor
  //gauge.startup();    // battery fuel gauge (USE IN FUTURE VERSIONS)

  /// SENSOR CALIBRATION ///
  if (do_calib) {
    background = proxCalibration(&prox_sensor);
    prox_threshold = pthresh_with_calib;
  } 
  else {
    background = 0;
    prox_threshold = pthresh_no_calib;
  }

  /// SET UP SLEEP/INTERRUPT LOGIC ///
  cli(); // Disable interrupts temporarily

  // Enable Pin Change Interrupt on PB3 (Pin 2 on ATtiny85)
  GIMSK |= (1 << PCIE);    // Enable Pin Change Interrupts
  PCMSK |= (1 << PCINT3); // Enable Pin Change Interrupt on PB3
  sleep_bod_disable(); // Disable brown-out detector

  sei(); // Re-enable interrupts
}

void loop() 
{
  // go back to sleep if interrupt is HIGH
  if (digitalRead(INT_PIN) == HIGH) { 
    //i2cWrite(1, ADDR_PROX, PROX_REG_00, 1);   // stop prox measurements
    digitalWrite(ALERT_PIN, LOW);
    goToSleep();
  }

  // check accel reading if interrupt is LOW
  else if (digitalRead(INT_PIN) == LOW) {
    accel = readAccelY();

    // arm is down -> not the signal we want 
    if (accel < 0) {  
      goToSleep();    
    }
    // arm is up -> get proximity measurements
    else {   
      // check if prox reading > threshold
      prox = VCNL4020_ReadProximity(&prox_sensor);
      prox = prox - background;
      if (prox > prox_threshold) {  
        digitalWrite(ALERT_PIN, HIGH);  // activate buzzer
      } else {
        digitalWrite(ALERT_PIN, LOW);   // deactivate buzzer
      }
      VCNL4020_ClearInterrupts(&prox_sensor);
      delay(period);
    }
  }
}

// Interrupt Service Routine for Pin Change Interrupt on Port D
ISR(PCINT0_vect) {
  int_triggered = true; 
}

/// FUNCTION LIST ///

// Configure sensors
void setupSensors() 
{
  // Disable magnetometer
  i2cWrite(0b11, ADDR_MAG, CFG_REG_A_M, 1);

  // Set up accelerometer
  i2cWrite(0b00100010, ADDR_ACCEL, CTRL_REG1_A, 1);   // normal mode, 10 Hz, enable Y-axis
  i2cWrite(0, ADDR_ACCEL, CTRL_REG4_A, 1);            // full scale = +/-2 g
  i2cWrite(0b00100010, ADDR_ACCEL, CTRL_REG6_A, 1);   // enable interrupt, active-low on INT2
  i2cWrite(1 << 3, ADDR_ACCEL, INT2_CFG_A, 1);        // INT2 activates when abs(y) > threshold
  i2cWrite(static_cast<uint8_t>(accel_threshold/9.8*1000/(2000./128)), ADDR_ACCEL, INT2_THS_A, 1);  // set threshold for interrupt
  // NOTE: ensure accelerometer runs 10 Hz max to allow start-up time during MCU wake-up

  // Set up proximity sensor
  VCNL4020_Init(&prox_sensor);
  VCNL4020_SetCurrent(&prox_sensor, led_current);   // set IR LED current
  VCNL4020_SetMode(&prox_sensor, MANUAL);           // get measurements on demand 
  delay(10);   
}

uint16_t proxCalibration(VCNL4020* pSensor)
{
  // Flush initial readings
  for (int i = 0; i < 3; i++) {
    prox = VCNL4020_ReadProximity(pSensor);
    delay(period);
  }

  // Perform calibration readings
  for (int i = 0; i < n_calib_samples; i++) {
    prox = VCNL4020_ReadProximity(pSensor);
    sum += prox;
    delay(period);
  }

  // Take average as background
  background = sum / n_calib_samples;
  return background;
}

// Read acceleration along X axis
float readAccelX() 
{
  // get acceleration readings (must set MSB to 1 to read consecutive registers)
  int16_t rawValue = i2cRead<int16_t>(ADDR_ACCEL, ACCEL_OUT_X | (1 << 7), 2); 
  rawValue = rawValue >> 6;  // remove lower 6 bits

  // convert to signed int, then convert to m/s^2 (normal mode = 3.9 milli-g per bit)
  return 3.9 * 9.8 / 1000 * rawValue;
}

// Read acceleration along Y axis
float readAccelY() 
{
  // get acceleration readings (must set MSB to 1 to read consecutive registers)
  int16_t rawValue = i2cRead<int16_t>(ADDR_ACCEL, ACCEL_OUT_Y | (1 << 7), 2); 
  rawValue = rawValue >> 6;  // remove lower 6 bits

  // convert to signed int, then convert to m/s^2 (normal mode = 3.9 milli-g per bit)
  return 3.9 * 9.8 / 1000 * rawValue;
}

// Place MCU in power-down mode
void goToSleep() 
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Use power-down mode
  sleep_enable();
  sleep_mode();    // Go to sleep
  sleep_disable(); // Resume here after wake-up
}