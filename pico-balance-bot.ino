#include <Wire.h>
#include <PID_v1.h>

// ---------------- IMU registers ----------------
#define MPU_ADDR      0x68   // AD0 low. Use 0x69 if the I2C scan shows that.
#define REG_PWR_MGMT1 0x6B
#define REG_WHO_AM_I  0x75
#define REG_CONFIG    0x1A
#define REG_GYRO_CFG  0x1B
#define REG_ACCEL_CFG 0x1C
#define REG_DATA      0x3B   // ACCEL_XOUT_H, 14 bytes: ax ay az temp gx gy gz

// ---------------- Pins ----------------
// IMU: VCC -> 3V3(OUT) pin 36, GND -> GND
const int PIN_SDA = 4;       // GP4 (physical pin 6)
const int PIN_SCL = 5;       // GP5 (physical pin 7)

// L298N (remove the ENA/ENB jumper caps; keep the 5V-EN jumper on)
const int ENA = 21;          // PWM, left motor speed
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 20;          // PWM, right motor speed

const int MIN_PWM   = 30;    // smallest PWM that actually turns the motors
const int COAST_PWM = 15;    // outputs below this coast instead of kicking

// ---------------- PID — tune these ----------------
// Order: raise Kp until it oscillates, add Kd to damp, then a little Ki
// only if it stands but slowly drifts.
double setpoint = 1.5;       // measured pitch (deg) at the true balance point
double Kp = 40.0;
double Ki = 0.0;
double Kd = 5.0;

double input, output;
// REVERSE because of how the IMU and motors are oriented on this chassis.
// If wheels drive the wrong way in BOTH fall directions, flip to DIRECT.
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

// ---------------- Filter state ----------------
float pitch = 0.0f;          // filtered tilt angle, degrees
float gyroOffsetY = 0.0f;    // gyro bias measured at startup
unsigned long lastMicros = 0;
unsigned long lastPrint = 0;

// ---------------- I2C helpers ----------------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Uses a normal STOP between the register write and the read (no repeated
// start) — many clone MPU boards return 0xFF with endTransmission(false).
bool readIMU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_DATA);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();              // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

void configureIMU() {
  writeReg(REG_PWR_MGMT1, 0x00);  // wake up
  delay(100);
  writeReg(REG_CONFIG,    0x03);  // DLPF ~41-44 Hz, smooths vibration
  writeReg(REG_GYRO_CFG,  0x08);  // gyro +/-500 dps -> 65.5 LSB per dps
  writeReg(REG_ACCEL_CFG, 0x00);  // accel +/-2 g    -> 16384 LSB per g
}

// ---------------- Motors ----------------
void motorsStop() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void drive(double out) {
  int pwm = (int)constrain(fabs(out), 0, 255);
  if (pwm < COAST_PWM) { motorsStop(); return; }  // tiny outputs -> coast
  if (pwm < MIN_PWM) pwm = MIN_PWM;               // overcome motor stiction

  if (out > 0) {          // wheels forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {                // wheels backward
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }   // wait for monitor, max 3 s

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  analogWriteFreq(20000);                  // 20 kHz PWM: no audible motor whine
  motorsStop();

  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  Wire.setClock(100000);                   // 100 kHz: reliable on clone boards

  // Identify the chip: 0x71 = MPU-9250, 0x70 = MPU-6500, 0x68 = MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 1);
  uint8_t who = Wire.read();
  Serial.print("WHO_AM_I = 0x"); Serial.println(who, HEX);

  configureIMU();

  // Gyro bias calibration — keep the bot STILL for ~2 seconds
  Serial.println("Calibrating gyro, don't move...");
  long sum = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < 500; i++) {
    readIMU(ax, ay, az, gx, gy, gz);
    sum += gy;
    delay(3);
  }
  gyroOffsetY = sum / 500.0f;
  Serial.print("Gyro Y offset: "); Serial.println(gyroOffsetY);

  // Seed the filter from the accelerometer (same formula as in loop)
  readIMU(ax, ay, az, gx, gy, gz);
  pitch = atan2f((float)ax, -(float)az) * 180.0f / PI;

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(5);                    // 200 Hz
  pid.SetOutputLimits(-255, 255);

  lastMicros = micros();
}

// ---------------- Loop ----------------
void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  // Tolerate brief I2C glitches; stop motors and re-init if the IMU is lost
  static int failCount = 0;
  if (!readIMU(ax, ay, az, gx, gy, gz)) {
    failCount++;
    if (failCount == 3) motorsStop();      // blind -> stop driving
    if (failCount >= 20) {
      Serial.println("IMU lost - reinitializing...");
      Wire.end();
      delay(5);
      Wire.begin();
      Wire.setClock(100000);
      configureIMU();
      failCount = 0;
    }
    return;
  }
  failCount = 0;

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  lastMicros = now;
  if (dt <= 0 || dt > 0.05f) return;       // guard against timing glitches

  // --- Complementary filter ---
  // IMU is mounted face-down (Z toward the floor), so az is negated to keep
  // the balance angle near 0 deg, and the gyro rate is negated to match.
  float accPitch = atan2f((float)ax, -(float)az) * 180.0f / PI;
  float gyroRate = -((float)gy - gyroOffsetY) / 65.5f;   // deg/s
  pitch = 0.98f * (pitch + gyroRate * dt) + 0.02f * accPitch;

  input = pitch;
  pid.Compute();

  // Fallen over: stop instead of spinning the wheels
  if (fabs(pitch - setpoint) > 45.0f) {
    motorsStop();
  } else {
    drive(output);
  }

  // Debug output at 20 Hz
  if (now - lastPrint > 50000) {
    lastPrint = now;
    Serial.print(pitch); Serial.print(" => "); Serial.println(output);
  }
}
