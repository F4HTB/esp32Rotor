#ifndef LSM_h
#define LSM_h

#include <Arduino.h>  // for byte data type

class LSM303 {
public:
  template<typename T> struct vector {
    T x, y, z;
  };

  // register addresses
  enum regAddr {
    CTRL_REG1_A = 0x20,   // DLH, DLM, DLHC
    CTRL_REG2_A = 0x21,   // DLH, DLM, DLHC
    CTRL_REG3_A = 0x22,   // DLH, DLM, DLHC
    CTRL_REG4_A = 0x23,   // DLH, DLM, DLHC
    CTRL_REG5_A = 0x24,   // DLH, DLM, DLHC
    CTRL_REG6_A = 0x25,   // DLHC
    REFERENCE_A = 0x26,   // DLH, DLM, DLHC
    STATUS_REG_A = 0x27,  // DLH, DLM, DLHC

    OUT_X_L_A = 0x28,
    OUT_X_H_A = 0x29,
    OUT_Y_L_A = 0x2A,
    OUT_Y_H_A = 0x2B,
    OUT_Z_L_A = 0x2C,
    OUT_Z_H_A = 0x2D,

    FIFO_CTRL_REG_A = 0x2E,  // DLHC
    FIFO_SRC_REG_A = 0x2F,   // DLHC

    INT1_CFG_A = 0x30,       // DLH, DLM, DLHC
    INT1_SRC_A = 0x31,       // DLH, DLM, DLHC
    INT1_THS_A = 0x32,       // DLH, DLM, DLHC
    INT1_DURATION_A = 0x33,  // DLH, DLM, DLHC
    INT2_CFG_A = 0x34,       // DLH, DLM, DLHC
    INT2_SRC_A = 0x35,       // DLH, DLM, DLHC
    INT2_THS_A = 0x36,       // DLH, DLM, DLHC
    INT2_DURATION_A = 0x37,  // DLH, DLM, DLHC

    CLICK_CFG_A = 0x38,     // DLHC
    CLICK_SRC_A = 0x39,     // DLHC
    CLICK_THS_A = 0x3A,     // DLHC
    TIME_LIMIT_A = 0x3B,    // DLHC
    TIME_LATENCY_A = 0x3C,  // DLHC
    TIME_WINDOW_A = 0x3D,   // DLHC

    CRA_REG_M = 0x00,  // DLH, DLM, DLHC
    CRB_REG_M = 0x01,  // DLH, DLM, DLHC
    MR_REG_M = 0x02,   // DLH, DLM, DLHC

    SR_REG_M = 0x09,   // DLH, DLM, DLHC
    IRA_REG_M = 0x0A,  // DLH, DLM, DLHC
    IRB_REG_M = 0x0B,  // DLH, DLM, DLHC
    IRC_REG_M = 0x0C,  // DLH, DLM, DLHC

    TEMP_OUT_H_M = 0x31,  // DLHC
    TEMP_OUT_L_M = 0x32,  // DLHC


    // dummy addresses for registers in different locations on different devices;
    // the library translates these based on device type
    // value with sign flipped is used as index into translated_regs array

    OUT_X_H_M = -1,
    OUT_X_L_M = -2,
    OUT_Y_H_M = -3,
    OUT_Y_L_M = -4,
    OUT_Z_H_M = -5,
    OUT_Z_L_M = -6,
    // update dummy_reg_count if registers are added here!

    // device-specific register addresses
    DLHC_OUT_X_H_M = 0x03,
    DLHC_OUT_X_L_M = 0x04,
    DLHC_OUT_Z_H_M = 0x05,
    DLHC_OUT_Z_L_M = 0x06,
    DLHC_OUT_Y_H_M = 0x07,
    DLHC_OUT_Y_L_M = 0x08,
  };

  vector<int16_t> a;      // accelerometer readings
  vector<int16_t> m;      // magnetometer readings
  vector<int16_t> aLast;  // accelerometer readings
  vector<int16_t> mLast;  // magnetometer readings

  struct {
    vector<int16_t> m_max;    // maximum magnetometer values, used for calibration
    vector<int16_t> m_min;    // minimum magnetometer values, used for calibration
    vector<int32_t> moffset;  // magnetometer offset, used for calculation
    vector<int16_t> a_max;    // maximum magnetometer values, used for calibration
    vector<int16_t> a_min;    // minimum magnetometer values, used for calibration
  } cal;

  byte last_status;  // status of last I2C transmission

  LSM303(void);

  bool init();

  void enableDefault(void);

  void writeAccReg(byte reg, byte value);
  byte readAccReg(byte reg);
  void writeMagReg(byte reg, byte value);
  byte readMagReg(int reg);

  void writeReg(byte reg, byte value);
  byte readReg(int reg);

  void readAcc(void);
  void readMag(void);
  void read(void);
  void readMagSmooth(void);

  bool timeoutOccurred(void);

  void heading(void);
  void headingAZ(void);

  // vector functions
  template<typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
  template<typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
  template<typename Ta, typename To> static void vector_neg(const vector<Ta> *a, vector<To> *out);
  template<typename Ta, typename To> static void vector_copy(const vector<Ta> *a, vector<To> *out);
  static void vector_normalize(vector<float> *a);

  float az;  //Antenna azimuth
  float el;  //Antenna elevation
  float azError = 0;  //Antenna azimuth Error
  float elError = 0;  //Antenna elevation Error


private:
  byte acc_address;
  byte mag_address;

  static const int dummy_reg_count = 6;
  regAddr translated_regs[dummy_reg_count + 1];  // index 0 not used

  unsigned int io_timeout;
  bool did_timeout;
};

/*
Returns the angular difference in the horizontal plane between the
"from" vector and north, in degrees.

Description of heading algorithm:
Shift and scale the magnetic reading based on calibration data to find
the North vector. Use the acceleration readings to determine the Up
vector (gravity is measured as an upward acceleration). The cross
product of North and Up vectors is East. The vectors East and North
form a basis for the horizontal plane. The From vector is projected
into the horizontal plane and the angle between the projected vector
and horizontal north is returned.
*/
template<typename Ta, typename To> void LSM303::vector_neg(const vector<Ta> *a, vector<To> *out) {
  out->x = -(a->x);
  out->y = -(a->y);
  out->z = -(a->z);
}

template<typename Ta, typename Tb, typename To> void LSM303::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out) {
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template<typename Ta, typename Tb> float LSM303::vector_dot(const vector<Ta> *a, const vector<Tb> *b) {
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

template<typename Ta, typename To> void LSM303::vector_copy(const vector<Ta> *a, vector<To> *out) {
  out->x = a->x;
  out->y = a->y;
  out->z = a->z;
}

#endif
