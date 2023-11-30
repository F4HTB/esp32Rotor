#include "lsm.h"
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define DLHC_DLM_DLH_MAG_ADDRESS 0b0011110
#define DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS 0b0011001


// Constructors ////////////////////////////////////////////////////////////////

LSM303::LSM303(void) {
  /*
  These values lead to an assumed magnetometer bias of 0.
  Use the Calibrate example program to determine appropriate values
  for your particular unit. The Heading example demonstrates how to
  adjust these values in your own sketch.
  */
  cal.m_min = (LSM303::vector<int16_t>){ -32767, -32767, -32767 };
  cal.m_max = (LSM303::vector<int16_t>){ +32767, +32767, +32767 };

  io_timeout = 500;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readMag(), or read() since the last call to timeoutOccurred()?
bool LSM303::timeoutOccurred() {
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

bool LSM303::init() {
  acc_address = DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS;  // DLHC doesn't have configurable SA0 but uses same acc address as DLM/DLH with SA0 high
  mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
  translated_regs[-OUT_X_H_M] = DLHC_OUT_X_H_M;
  translated_regs[-OUT_X_L_M] = DLHC_OUT_X_L_M;
  translated_regs[-OUT_Y_H_M] = DLHC_OUT_Y_H_M;
  translated_regs[-OUT_Y_L_M] = DLHC_OUT_Y_L_M;
  translated_regs[-OUT_Z_H_M] = DLHC_OUT_Z_H_M;
  translated_regs[-OUT_Z_L_M] = DLHC_OUT_Z_L_M;
  return true;
}

/*
Enables the LSM303's accelerometer and magnetometer. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and +/- 1.3 gauss for magnetometer
  (+/- 4 gauss on LSM303D).
- Selects 50 Hz ODR (output data rate) for accelerometer and 7.5 Hz
  ODR for magnetometer (6.25 Hz on LSM303D). (These are the ODR
  settings for which the electrical characteristics are specified in
  the datasheets.)
- Enables high resolution modes (if available).
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM303::enableDefault(void) {
  // Accelerometer
  // 0x08 = 0b00001000
  // FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
  writeAccReg(CTRL_REG4_A, 0x08);
  // 0x47 = 0b01000111
  // ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
  writeAccReg(CTRL_REG1_A, 0x47);

  // // Magnetometer
  // // 0x0C = 0b00001100
  // // DO = 011 (7.5 Hz ODR)
  // writeMagReg(CRA_REG_M, 0x0C);
  // // 0x20 = 0b00100000
  // // GN = 001 (+/- 1.3 gauss full scale)
  // writeMagReg(CRB_REG_M, 0x20);
  // // 0x00 = 0b00000000
  // // MD = 00 (continuous-conversion mode)
  // writeMagReg(MR_REG_M, 0x00);

  // Magnetometer
  // 0x18 = 0b00011000
  // DO = 110 ( 75Hz ODR)
  writeMagReg(CRA_REG_M, 0x18);
  // 0x28 = 0b00101000
  // GN = 001 (+/- 1.3 gauss full scale)
  writeMagReg(CRB_REG_M, 0x28);
  // 0x00 = 0b00000000
  // MD = 00 (continuous-conversion mode)
  writeMagReg(MR_REG_M, 0x00);
}

// Writes an accelerometer register
void LSM303::writeAccReg(byte reg, byte value) {
  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Reads an accelerometer register
byte LSM303::readAccReg(byte reg) {
  byte value;

  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Writes a magnetometer register
void LSM303::writeMagReg(byte reg, byte value) {
  Wire.beginTransmission(mag_address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Reads a magnetometer register
byte LSM303::readMagReg(int reg) {
  byte value;

  // if dummy register address (magnetometer Y/Z), look up actual translated address (based on device type)
  if (reg < 0) {
    reg = translated_regs[-reg];
  }

  Wire.beginTransmission(mag_address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(mag_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void LSM303::writeReg(byte reg, byte value) {
  // mag address == acc_address for LSM303D, so it doesn't really matter which one we use.
  if (reg < CTRL_REG1_A) {
    writeMagReg(reg, value);
  } else {
    writeAccReg(reg, value);
  }
}

// Note that this function will not work for reading TEMP_OUT_H_M and TEMP_OUT_L_M on the DLHC.
// To read those two registers, use readMagReg() instead.
byte LSM303::readReg(int reg) {
  // mag address == acc_address for LSM303D, so it doesn't really matter which one we use.
  // Use readMagReg so it can translate OUT_[XYZ]_[HL]_M
  if (reg < CTRL_REG1_A) {
    return readMagReg(reg);
  } else {
    return readAccReg(reg);
  }
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM303::readAcc(void) {
  Wire.beginTransmission(acc_address);
  Wire.write(OUT_X_L_A | (1 << 7));
  last_status = Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6) {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout) {
      did_timeout = true;
      return;
    }
  }

  // byte yla = Wire.read();
  // byte yha = Wire.read();
  // byte xla = Wire.read();
  // byte xha = Wire.read();
  // byte zla = Wire.read();
  // byte zha = Wire.read();

  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);
}

// Reads the 3 magnetometer channels and stores them in vector m
void LSM303::readMag(void) {
  Wire.beginTransmission(mag_address);
  Wire.write(translated_regs[-OUT_X_H_M]);
  last_status = Wire.endTransmission();
  Wire.requestFrom(mag_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6) {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout) {
      did_timeout = true;
      return;
    }
  }

  byte xlm, xhm, ylm, yhm, zlm, zhm;

  xhm = Wire.read();
  xlm = Wire.read();
  zhm = Wire.read();
  zlm = Wire.read();
  yhm = Wire.read();
  ylm = Wire.read();

  m.x = (int16_t)(xhm << 8 | xlm);
  m.z = (int16_t)(zhm << 8 | zlm);
  m.y = (int16_t)(yhm << 8 | ylm);
}

// Reads all 6 channels of the LSM303 and stores them in the object variables
void LSM303::read(void) {
  readAcc();
  readMag();
}

void LSM303::readMagSmooth(void) {
  for (int t = 0; t < 10; t++) {
    read();



    a.x = (0.05 * a.x) + aLast.x * (0.95);
    a.y = (0.05 * a.y) + aLast.y * (0.95);
    a.z = (0.05 * a.z) + aLast.z * (0.95);

    m.x = (0.05 * m.x) + mLast.x * (0.95);
    m.y = (0.05 * m.y) + mLast.y * (0.95);
    m.z = (0.05 * m.z) + mLast.z * (0.95);

    aLast.x = a.x;
    aLast.y = a.y;
    aLast.z = a.z;

    mLast.x = m.x;
    mLast.y = m.y;
    mLast.z = m.z;
  }
}

/*
Returns the angular difference in the horizontal plane between a
default vector and north, in degrees.

The default vector here is chosen to point along the surface of the
PCB, in the direction of the top of the text on the silkscreen.
This is the +X axis on the Pololu LSM303D carrier and the -Y axis on
the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
*/
void LSM303::heading(void) {
  headingAZ();
}

void LSM303::headingAZ(void) {
  vector<int> X = { 1, 0, 0 };
  vector<int> Y = { 0, 1, 0 };
  vector<int> Z = { 0, 0, 1 };
  vector<int32_t> temp_m = { m.x, m.y, m.z };

  // subtract offset (average of min and max) from magnetometer readings
  temp_m.x -= cal.moffset.x;
  temp_m.y -= cal.moffset.y;
  temp_m.z -= cal.moffset.z;

  vector<float> E;
  vector<float> N;
  vector<float> U;
  vector_copy(&a, &U);
  vector_normalize(&U);
  vector_cross(&temp_m, &a, &E);
  vector_normalize(&E);
  vector_cross(&a, &E, &N);
  vector_normalize(&N);


  // compute heading
  el = (atan2(vector_dot(&X, &U), vector_dot(&Z, &U)) * 180 / PI) + elError;
  az = atan2(vector_dot(&E, &X), vector_dot(&N, &X)) * 180 / PI + azError;
  if (az < 0) az += 360;
}

void LSM303::vector_normalize(vector<float> *a) {
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}