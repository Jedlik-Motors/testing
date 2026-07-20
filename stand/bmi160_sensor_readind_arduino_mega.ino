//====================================================================
//  BMI160 test - Arduino Mega 2560   (sensor only, no motor)
//
//  Wiring:  SDA -> D20 , SCL -> D21 , VCC -> 3V3 , GND -> GND
//           SDO -> GND  (I2C address 0x68)
//
//  Open Serial Monitor at 9600 baud. Tilt the board and watch.
//====================================================================
#include <Wire.h>

#define BMI160_ADDR      0x68     // 0x69 if SDO is tied to VDD
#define REG_CHIP_ID      0x00     // should read 0xD1
#define REG_CMD          0x7E
#define REG_GYR_DATA     0x0C     // GYR_X_L, 6 bytes
#define REG_ACC_DATA     0x12     // ACC_X_L, 6 bytes

// sensitivities for the DEFAULT ranges after power-up
const float ACC_LSB_PER_G   = 16384.0f;   // +/- 2 g
const float GYR_LSB_PER_DPS = 16.4f;      // +/- 2000 deg/s

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// read 6 bytes starting at reg -> signed little-endian x,y,z
bool read3(uint8_t reg, int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 6);
  if (Wire.available() < 6) return false;

  // read low-then-high explicitly (order matters, don't one-line it)
  uint8_t xl = Wire.read(), xh = Wire.read();
  uint8_t yl = Wire.read(), yh = Wire.read();
  uint8_t zl = Wire.read(), zh = Wire.read();
  x = (int16_t)(xl | (xh << 8));
  y = (int16_t)(yl | (yh << 8));
  z = (int16_t)(zl | (zh << 8));
  return true;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();                     // Mega picks SDA=20, SCL=21 automatically

  // --- verify the sensor is actually talking ---
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_CHIP_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 1);
  uint8_t id = Wire.available() ? Wire.read() : 0x00;
  Serial.print(F("CHIP_ID = 0x"));
  Serial.print(id, HEX);
  Serial.println(id == 0xD1 ? F("  (OK)") : F("  (WRONG - check wiring/address)"));

  // --- power up both sensors ---
  writeReg(REG_CMD, 0x11);  delay(100);   // accel -> normal mode
  writeReg(REG_CMD, 0x15);  delay(100);   // gyro  -> normal mode

  Serial.println(F("BMI160 ready\n"));
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  bool okA = read3(REG_ACC_DATA, ax, ay, az);
  bool okG = read3(REG_GYR_DATA, gx, gy, gz);

  if (okA && okG) {
    float axg = ax / ACC_LSB_PER_G;       // g
    float ayg = ay / ACC_LSB_PER_G;
    float azg = az / ACC_LSB_PER_G;

    float gxd = gx / GYR_LSB_PER_DPS;      // deg/s
    float gyd = gy / GYR_LSB_PER_DPS;
    float gzd = gz / GYR_LSB_PER_DPS;

    float tilt = atan2(axg, azg) * 57.29578f;   // deg, from accel

    Serial.print(F("A[g] "));
    Serial.print(axg, 2); Serial.print(' ');
    Serial.print(ayg, 2); Serial.print(' ');
    Serial.print(azg, 2);
    Serial.print(F("   G[dps] "));
    Serial.print(gxd, 1); Serial.print(' ');
    Serial.print(gyd, 1); Serial.print(' ');
    Serial.print(gzd, 1);
    Serial.print(F("   tilt = "));
    Serial.print(tilt, 1);
    Serial.println(F(" deg"));
  } else {
    Serial.println(F("read fail - check wiring / address"));
  }

  delay(100);
}