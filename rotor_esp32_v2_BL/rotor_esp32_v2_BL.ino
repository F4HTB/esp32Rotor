#include "TB6612FNG.h"
#include <Wire.h>
#include "lsm.h"
#include <Preferences.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "BluetoothSerial.h"

////////Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
////////!Bluetooth

////////Buzz
#define Buzz_pin 17
////////!Buzz

////////Motors
// To reverse forward motor direction, switch the AIN1 and AIN2 or BIN1 and BIN2 pin numbers.
#define TB_Standby_pin 27  // IO27 GPIO27
#define TB_AIN1_pin 14     // TMS  GPIO14
#define TB_AIN2_pin 12     // TDI  GPIO12
#define TB_PWMA_pin 13     // TCK  GPIO13
#define TB_BIN1_pin 26     // IO26 GPIO26
#define TB_BIN2_pin 25     // IO25 GPIO25
#define TB_PWMB_pin 33     // IO33 GPIO33
Tb6612fng motors(TB_Standby_pin, TB_AIN1_pin, TB_AIN2_pin, TB_PWMA_pin, TB_BIN1_pin, TB_BIN2_pin, TB_PWMB_pin);
#define azHall_pin 18
int azHallS = 1;
int azHallN = 0;
int azHall360 = 12736;

float cubic_function_spline[] = { 0.00000, 0.000298, 0.001184, 0.002646, 0.004672, 0.00725, 0.010368, 0.014014, 0.018176, 0.022842, 0.028000, 0.033638, 0.039744, 0.046306, 0.053312, 0.060750, 0.068608, 0.076874, 0.085536, 0.094582, 0.104000, 0.113778, 0.123904, 0.134366, 0.145152, 0.156250, 0.167648, 0.179334, 0.191296, 0.203522, 0.216000, 0.228718, 0.241664, 0.254826, 0.268192, 0.281750, 0.295488, 0.309394, 0.323456, 0.337662, 0.352000, 0.366458, 0.381024, 0.395686, 0.410432, 0.425250, 0.440128, 0.455054, 0.470016, 0.485002, 0.500000, 0.514998, 0.529984, 0.544946, 0.559872, 0.574750, 0.589568, 0.604314, 0.618976, 0.633542, 0.648000, 0.662338, 0.676544, 0.690606, 0.704512, 0.718250, 0.731808, 0.745174, 0.758336, 0.771282, 0.784000, 0.796478, 0.808704, 0.820666, 0.832352, 0.843750, 0.854848, 0.865634, 0.876096, 0.886222, 0.896000, 0.905418, 0.914464, 0.923126, 0.931392, 0.939250, 0.946688, 0.953694, 0.960256, 0.966362, 0.972000, 0.977158, 0.981824, 0.985986, 0.989632, 0.992750, 0.995328, 0.997354, 0.998816, 0.999702, 1.00000 };

////////!Motors

////////Compas
// 3.3V - VCC 3.3v pin
// GND  - GND 0v pin
// IO21 - SCL pin
// IO22 - SDA pin
LSM303 compass;
float azSet = 0;  //Antenna azimuth set point
float elSet = 0;  //Antenna elevation set point
float azVT = 0.4;
float elVT = 0.2;
////////!Compas

////////Battery
#define BAT_MIN 1551  //(2500mV / 2) * 4095 / 3300 = 1551
#define BAT_MAX 1054  //(4200mV / 2) * 4095 / 3300 = 2605, 2605 - BAT_MIN =
#define bat_ADC 34    //
int bat_value = 0;
////////!Battery

////////Global
String line;    //Command line
String BTline;  //Bluetooth Command line
bool smonitor = 0;
bool calMagR = 0;
bool calAccR = 0;
bool initSYS = false;
////////

////////
Preferences preferences;
////////

//////////

void beep(int time, int repeat_num = 1) {
  if (repeat_num > 1) { time = time / 2; }
  while (repeat_num > 0) {
    digitalWrite(Buzz_pin, HIGH);
    delay(time);
    digitalWrite(Buzz_pin, LOW);
    if (repeat_num > 1) delay(time);
    repeat_num--;
  }
}

void printCal(void) {
  //Print the calibration data

  Serial.printf("Calibration: %.1f<=%.1f , %.1f<=%.1f , %.1f<=%.1f , %.1f<=%.1f , %.1f<=%.1f , %.1f<=%.1f\n",
                (float)compass.cal.m_min.x,
                (float)compass.m.x,
                (float)compass.cal.m_min.y,
                (float)compass.m.y,
                (float)compass.cal.m_min.z,
                (float)compass.m.z,
                (float)compass.cal.m_max.x,
                (float)compass.m.x,
                (float)compass.cal.m_max.y,
                (float)compass.m.y,
                (float)compass.cal.m_max.z,
                (float)compass.m.z);
}

void save() {
  //Save the calibration data to EEPROM
  preferences.putShort("m_min_x", compass.cal.m_min.x);
  preferences.putShort("m_min_y", compass.cal.m_min.y);
  preferences.putShort("m_min_z", compass.cal.m_min.z);
  preferences.putShort("m_max_x", compass.cal.m_max.x);
  preferences.putShort("m_max_y", compass.cal.m_max.y);
  preferences.putShort("m_max_z", compass.cal.m_max.z);
  preferences.putFloat("azError", compass.azError);
  preferences.putFloat("elError", compass.elError);
  beep(100, 2);
}

void restore() {
  //Restore the calibration data from EEPROM
  compass.cal.m_min.x = preferences.getShort("m_min_x", -32767);
  compass.cal.m_min.y = preferences.getShort("m_min_y", -32767);
  compass.cal.m_min.z = preferences.getShort("m_min_z", -32767);
  compass.cal.m_max.x = preferences.getShort("m_max_x", +32767);
  compass.cal.m_max.y = preferences.getShort("m_max_y", +32767);
  compass.cal.m_max.z = preferences.getShort("m_max_z", +32767);
  compass.cal.moffset.x = ((int32_t)compass.cal.m_min.x + compass.cal.m_max.x) / 2;
  compass.cal.moffset.y = ((int32_t)compass.cal.m_min.y + compass.cal.m_max.y) / 2;
  compass.cal.moffset.z = ((int32_t)compass.cal.m_min.z + compass.cal.m_max.z) / 2;
  compass.azError = preferences.getFloat("azError", 0);
  compass.elError = preferences.getFloat("elError", 0);
  printCal();
}

void calcompassMag() {
  unsigned long lasttime = millis();
  bool change = false;
  LSM303::vector<int16_t> lastmin;
  LSM303::vector<int16_t> lastmax;
  compass.cal.m_min = (LSM303::vector<int16_t>){ +32767, +32767, +32767 };
  compass.cal.m_max = (LSM303::vector<int16_t>){ -32767, -32767, -32767 };
  while (lasttime > (millis() - 10000)) {
    change = false;
    compass.readMag();
    compass.cal.m_min.x = min(compass.cal.m_min.x, compass.m.x);
    compass.cal.m_min.y = min(compass.cal.m_min.y, compass.m.y);
    compass.cal.m_min.z = min(compass.cal.m_min.z, compass.m.z);
    compass.cal.m_max.x = max(compass.cal.m_max.x, compass.m.x);
    compass.cal.m_max.y = max(compass.cal.m_max.y, compass.m.y);
    compass.cal.m_max.z = max(compass.cal.m_max.z, compass.m.z);

    if (lastmin.x != compass.cal.m_min.x) {
      lastmin.x = compass.cal.m_min.x;
      change = true;
    }
    if (lastmin.y != compass.cal.m_min.y) {
      lastmin.y = compass.cal.m_min.y;
      change = true;
    }
    if (lastmin.z != compass.cal.m_min.z) {
      lastmin.z = compass.cal.m_min.z;
      change = true;
    }
    if (lastmax.x != compass.cal.m_max.x) {
      lastmax.x = compass.cal.m_max.x;
      change = true;
    }
    if (lastmax.y != compass.cal.m_max.y) {
      lastmax.y = compass.cal.m_max.y;
      change = true;
    }
    if (lastmax.z != compass.cal.m_max.z) {
      lastmax.z = compass.cal.m_max.z;
      change = true;
    }

    if (change) {
      beep(50, 2);
      printCal();
      lasttime = millis();
    }
  }
  compass.cal.moffset.x = (compass.cal.m_min.x + compass.cal.m_max.x) / 2;
  compass.cal.moffset.y = (compass.cal.m_min.y + compass.cal.m_max.y) / 2;
  compass.cal.moffset.z = (compass.cal.m_min.z + compass.cal.m_max.z) / 2;
  save();
  beep(1000, 3);
}

void printAzEl() {
  //Print the rotator feedback data in Easycomm II format
  compass.readMagSmooth();
  compass.heading();
  float AZAngle = 0;
  (initSYS) ? AZAngle = getazH() : AZAngle = compass.az;
  Serial.printf("AZ%.1f EL%.1f\n", AZAngle, compass.el);
}

void getminazelVT() {
  compass.readMagSmooth();
  compass.heading();
  float initAZangle = getazH();
  float initELangle = compass.el;
  float minazVT = 0.01;
  float minelVT = 0.01;
  while ((abs(diffAngle(getazH(), initAZangle)) < 6) && (minazVT < 1)) {
    minazVT += 0.02;
    motors.drive(minazVT, 0, 0, 0);
    beep(250, 2);
  }
  motors.drive(0, 0, 0, 0);
  while ((abs(diffAngle(compass.el, initELangle)) < 2) && (minelVT < 1)) {
    compass.readMagSmooth();
    compass.heading();
    minelVT += 0.02;
    motors.drive(0, minelVT, 0, 0);
    beep(250, 2);
  }
  motors.drive(0, 0, 0, 0);
  azVT = minazVT;
  if (azVT < 0.1) azVT = 0.1;
  elVT = minelVT;
  if (elVT < 0.1) elVT = 0.1;
}


int posaccAZ = 0;
int posaccEL = 0;
float cubicaccel[] = { 0, 0.028, 0.104, 0.216, 0.352, 0.5, 0.648, 0.784, 0.896, 0.972, 1 };
bool drivemotortoAzEl(float sAz, float sEl, float speedAZ = 0.1, float speedEL = 0.1, float margin = 2) {
  float vAz = 0;
  float vEl = 0;
  compass.readMagSmooth();
  compass.heading();

  sAz = constrain(sAz, 0, 360);
  sEl = constrain(sEl, 0, 180);

  float AZdiffAngle = 0;
  if (initSYS) {
    AZdiffAngle = diffAngle(sAz, getazH());
  } else {
    AZdiffAngle = diffAngle(sAz, compass.az);
  }

  if (abs(AZdiffAngle) < margin) {
    if (posaccAZ > 0) posaccAZ--;
    if (posaccAZ > 0) posaccAZ--;
  } else {
    if (posaccAZ < 10) posaccAZ++;
  }

  if (AZdiffAngle > 0) {
    vAz = speedAZ * cubicaccel[posaccAZ];
    azHallS = 1;
  } else {
    vAz = -speedAZ * cubicaccel[posaccAZ];
    azHallS = -1;
  }

  float ELdiffAngle = diffAngle(sEl, compass.el);
  if (abs(ELdiffAngle) < margin) {
    if (posaccEL > 0) posaccEL--;
    if (posaccEL > 0) posaccEL--;
  } else {
    if (posaccEL < 10) posaccEL++;
  }

  if (ELdiffAngle > 0) {
    vEl = speedEL * cubicaccel[posaccEL];
  } else {
    vEl = -speedEL * cubicaccel[posaccEL];
  }

  if(abs(vAz) || abs(vEl)){beep(1);motors.enable(true);}else{motors.enable(false);}
  motors.drive(vAz, vEl, 0, 0);
  
  bool ret = false;
  ((abs(compass.az - sAz) < margin) && (abs(compass.el - sEl) < margin)) ? ret = true : ret = false;
  return ret;
}

float param2float(String line) {
  String param;
  int firstSpace;
  firstSpace = line.indexOf(' ');
  param = line.substring(5, firstSpace);
  Serial.printf("%.1f\n", param.toFloat());
  return param.toFloat();
}

void processUserCommands(String line) {
  if (line.startsWith("Bat")) {  //Query command received
    bat_value = ((analogRead(bat_ADC) - BAT_MIN) * 100) / BAT_MAX;
    if (Serial) {
      Serial.printf("%d%\n", bat_value);
    }
  } else if (line.startsWith("mon")) {   //Query command received
    smonitor = !smonitor;                //Send the current Azimuth and Elevation
  } else if (line.startsWith("calm")) {  //Query command received
    calcompassMag();
  } else if (line.startsWith("restore")) {
    restore();
  } else if (line.startsWith("save")) {
    save();
  } else if (line.startsWith("hlevel")) {
    getplane();
  } else if (line.startsWith("Erraz")) {
    compass.azError = param2float(line);
    initSYS = false;
    while (!drivemotortoAzEl(0, 0, azVT, elVT, 1)) { beep(100); }
    azHallN = 0;
    initSYS = true;
  } else if (line.startsWith("Errel")) {
    compass.elError = param2float(line);
  } else if (line.startsWith("AZspd")) {
    azVT = param2float(line);
  } else if (line.startsWith("ELspd")) {
    elVT = param2float(line);
  } else if (line.startsWith("reset")) {
    ESP.restart();
  } else if (line.startsWith("minVT")) {
    getminazelVT();
    Serial.printf("azVT:%.2f elVT:%.2f\n", azVT, elVT);
  }
}


void processEasycommCommands(String line) {
  //Process Easycomm II rotator commands
  //Easycomm II position command: AZnn.n ELnn.n UP000 XXX DN000 XXX\n
  //Easycomm II query command: AZ EL \n
  String param;                    //Parameter value
  int firstSpace;                  //Position of the first space in the command line
  int secondSpace;                 //Position of the second space in the command line
  if (line.startsWith("AZ EL")) {  //Query command received
    printAzEl();                   //Send the current Azimuth and Elevation
  } else {
    if (line.startsWith("AZ")) {                            //Position command received: Parse the line.
      firstSpace = line.indexOf(' ');                       //Get the position of the first space
      secondSpace = line.indexOf(' ', firstSpace + 1);      //Get the position of the second space
      param = line.substring(2, firstSpace);                //Get the first parameter
      azSet = param.toFloat();                              //Set the azSet value
      param = line.substring(firstSpace + 3, secondSpace);  //Get the second parameter
      elSet = param.toFloat();                              //Set the elSet value
    }
  }
}

void processCommands(void) {
  //Process incoming data from the control computer
  //User commands are entered by the user and are terminated with a carriage return
  //Easycomm commands are generated by a tracking program and are terminated with a line feed
  while (Serial.available() > 0) {
    char ch = Serial.read();  //Read a single character from the serial buffer
    switch (ch) {
      case 13:                      //Carriage return received
        processUserCommands(line);  //Process user commands
        line = "";                  //Command processed: Clear the command line
        break;
      case 10:                          //Line feed received
        processEasycommCommands(line);  //Process Easycomm commands
        line = "";                      //Command processed: Clear the command line
        break;
      default:       //Any other character received
        line += ch;  //Add this character to the command line
        break;
    }
  }
  while (SerialBT.available() > 0) {
    char ch = SerialBT.read();
    if (ch == 10) {
      Serial.printf("BT %S\n", BTline.c_str());
      processEasycommCommands(BTline);
      BTline = "";
    } else {
      BTline += ch;
    }
  }
}

float diffAngle(float a, float b) {
  //Calculate the acute angle between two angles in -180..180 degree format
  float diff = a - b;
  if (diff < -180) diff += 360;
  if (diff > 180) diff -= 360;
  return diff;
}

void getplane(void) {
  short int i = 100;
  int16_t maxS = 0;
  while (i > 0) {
    compass.readMagSmooth();
    digitalWrite(Buzz_pin, HIGH);
    delay(100);
    i--;
    int16_t z = abs(compass.a.z);
    int16_t diff = abs(maxS - z);
    Serial.println(z);
    diff = constrain(diff, 10, 500) / 2;
    delay(diff);
    digitalWrite(Buzz_pin, LOW);
    delay(diff);
    maxS = max(maxS, z);
  }
}

void IRAM_ATTR isrF() {
  azHallN += azHallS;
}

float getazH(void) {
  float azH = (((float)azHallN * 360) / azHall360);
  if (azH < 0) azH += 360;
  return azH;
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  switch (event) {
    case ESP_SPP_SRV_OPEN_EVT:
      Serial.println("BT Connected");
      beep(40, 5);
      break;
    case ESP_SPP_CLOSE_EVT:
      Serial.println("BT Disconnected");
      beep(400, 2);
      break;
    default:
      break;
  }
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.print("\n");
}

/////////
#define I2C_SDA 21
#define I2C_SCL 22
void setup() {
  Serial.begin(38400);
  SerialBT.register_callback(callback);
  SerialBT.begin("F4HTB_ESP32_Rotor");
  Serial.print("\nYou can pair bluetooth on:");
  printDeviceAddress();
  motors.begin();
  preferences.begin("eeprom", false);
  Wire.begin(I2C_SDA, I2C_SCL,400000);
  compass.init();
  compass.enableDefault();
  restore();
  pinMode(azHall_pin, INPUT);
  attachInterrupt(azHall_pin, isrF, CHANGE);
  pinMode(Buzz_pin, OUTPUT);
  beep(1000, 2);
  getminazelVT();
  compass.readMagSmooth();
  compass.readMagSmooth();
  compass.heading();
  beep(1000);
  while (!drivemotortoAzEl(0, 0, azVT, elVT, 1)) {
    beep(100);
    //Serial.printf("Compass AZ%.1f EL%.1f Hall AZ %d %.1f\n", compass.az, compass.el, azHallN, getazH());
  }
  azHallN = 0;
  initSYS = true;
}

void loop() {

  if (smonitor) {
    compass.readMagSmooth();
    compass.heading();
    Serial.printf("Compass AZ%.1f EL%.1f Hall AZ %d %.1f\n", compass.az, compass.el, azHallN, getazH());
  }
  processCommands();
  drivemotortoAzEl(azSet, elSet, azVT, elVT, 1);
  delay(100);
}