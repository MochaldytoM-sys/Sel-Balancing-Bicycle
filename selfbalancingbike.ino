```cpp
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <math.h>

// ==================================================
// OBJEK
// ==================================================
Adafruit_MPU6050 mpu;
Servo steeringServo;

// ==================================================
// PIN MPU6050
// ==================================================
#define SDA_PIN 21
#define SCL_PIN 22

// ==================================================
// MOTOR A: RODA PENGGERAK
// ==================================================
const int WHEEL_IN1 = 14;
const int WHEEL_IN2 = 27;
const int WHEEL_EN  = 13;

// ==================================================
// MOTOR B: PIRINGAN BALANCING
// ==================================================
const int BALANCE_IN1 = 26;
const int BALANCE_IN2 = 25;
const int BALANCE_EN  = 33;

// ==================================================
// SERVO KEMUDI
// ==================================================
const int SERVO_PIN = 18;

const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 60;
const int SERVO_RIGHT  = 120;

// ==================================================
// PWM ESP32 - ARDUINO CORE 3.x
// ==================================================
const int PWM_FREQ = 15000;
const int PWM_RES  = 8;

// ==================================================
// PARAMETER BALANCING
// NILAI AWAL UNTUK PENGUJIAN
// ==================================================
float Kp = 4.0;
float Kd = 0.8;

const float TARGET_ANGLE = 0.0;
const float DEAD_BAND = 1.0;
const float MAX_SAFE_ANGLE = 15.0;

const int MAX_BALANCE_PWM = 150;
const int MIN_BALANCE_PWM = 0;

// Batas perubahan PWM setiap siklus
const int PWM_STEP = 5;

// ==================================================
// VARIABEL SENSOR
// ==================================================
float rollAngle = 0.0;
float gyroRate = 0.0;
float angleOffset = 0.0;

float pidOutput = 0.0;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;

const unsigned long CONTROL_INTERVAL_US = 5000;
const unsigned long PRINT_INTERVAL_MS = 100;

// ==================================================
// STATUS SISTEM
// ==================================================
bool balanceEnabled = false;
bool wheelEnabled = false;
bool sensorOK = false;
bool safetyTrip = false;

int wheelCommand = 0;
int steeringAngle = SERVO_CENTER;
int currentBalancePWM = 0;

// ==================================================
// MOTOR CONTROL
// ==================================================
void setMotor(
  int in1,
  int in2,
  int en,
  int speedValue
) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (speedValue < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(en, abs(speedValue));
}

void stopWheel() {
  setMotor(
    WHEEL_IN1,
    WHEEL_IN2,
    WHEEL_EN,
    0
  );
}

void stopBalanceMotor() {
  setMotor(
    BALANCE_IN1,
    BALANCE_IN2,
    BALANCE_EN,
    0
  );

  currentBalancePWM = 0;
}

// ==================================================
// PWM SMOOTHING MOTOR BALANCING
// ==================================================
void setBalanceMotorSmooth(int targetPWM) {
  targetPWM = constrain(
    targetPWM,
    -MAX_BALANCE_PWM,
    MAX_BALANCE_PWM
  );

  // Naikkan atau turunkan PWM secara bertahap.
  // Jika arah berbalik, turunkan PWM ke nol dahulu.
  if ((currentBalancePWM > 0 && targetPWM < 0) ||
      (currentBalancePWM < 0 && targetPWM > 0)) {

    if (currentBalancePWM > 0) {
      currentBalancePWM =
        max(0, currentBalancePWM - PWM_STEP);
    }
    else {
      currentBalancePWM =
        min(0, currentBalancePWM + PWM_STEP);
    }
  }
  else if (targetPWM > currentBalancePWM) {
    currentBalancePWM += min(
      PWM_STEP,
      targetPWM - currentBalancePWM
    );
  }
  else if (targetPWM < currentBalancePWM) {
    currentBalancePWM -= min(
      PWM_STEP,
      currentBalancePWM - targetPWM
    );
  }

  setMotor(
    BALANCE_IN1,
    BALANCE_IN2,
    BALANCE_EN,
    currentBalancePWM
  );
}

// ==================================================
// SERVO
// ==================================================
void setSteering(int angle) {
  steeringAngle = constrain(angle, 45, 135);
  steeringServo.write(steeringAngle);
}

// ==================================================
// KALIBRASI MPU6050
// Sepeda harus tegak dan diam.
// ==================================================
bool calibrateMPU() {
  const int samples = 400;
  float sumAngle = 0.0;

  Serial.println();
  Serial.println("KALIBRASI MPU6050");
  Serial.println("Tegakkan sepeda dan jangan digerakkan.");

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    float angle = atan2(
      a.acceleration.y,
      a.acceleration.z
    ) * 180.0 / PI;

    sumAngle += angle;

    delay(5);
  }

  angleOffset = sumAngle / samples;

  Serial.print("Offset sudut: ");
  Serial.println(angleOffset, 2);

  return true;
}

// ==================================================
// BACA MPU6050
// Asumsi sumbu X gyro mengukur roll.
// Orientasi sensor wajib diperiksa.
// ==================================================
void updateSensor(float dt) {
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  float accelAngle = atan2(
    a.acceleration.y,
    a.acceleration.z
  ) * 180.0 / PI;

  accelAngle -= angleOffset;

  // rad/s menjadi derajat/s
  gyroRate = g.gyro.x * 180.0 / PI;

  // Complementary filter
  rollAngle =
    0.98 * (rollAngle + gyroRate * dt)
    + 0.02 * accelAngle;
}

// ==================================================
// KONTROL BALANCING
// ==================================================
void updateBalance() {
  if (!balanceEnabled || !sensorOK || safetyTrip) {
    setBalanceMotorSmooth(0);
    return;
  }

  // Sudut terlalu besar: hentikan sistem
  if (fabs(rollAngle) > MAX_SAFE_ANGLE) {
    safetyTrip = true;
    balanceEnabled = false;

    stopBalanceMotor();

    Serial.println();
    Serial.println("SAFETY STOP!");
    Serial.println("Periksa posisi dan arah koreksi motor.");
    return;
  }

  float error = TARGET_ANGLE - rollAngle;

  // PD Controller
  float output = Kp * error - Kd * gyroRate;

  // Motor berhenti di sekitar titik tegak
  if (fabs(error) < DEAD_BAND &&
      fabs(gyroRate) < 3.0) {
    output = 0.0;
  }

  pidOutput = output;

  int targetPWM = (int)output;

  targetPWM = constrain(
    targetPWM,
    -MAX_BALANCE_PWM,
    MAX_BALANCE_PWM
  );

  setBalanceMotorSmooth(targetPWM);
}

// ==================================================
// MOTOR RODA
// ==================================================
void updateWheel() {
  if (!wheelEnabled || wheelCommand == 0) {
    stopWheel();
    return;
  }

  const int WHEEL_PWM = 100;

  if (wheelCommand > 0) {
    setMotor(
      WHEEL_IN1,
      WHEEL_IN2,
      WHEEL_EN,
      WHEEL_PWM
    );
  }
  else {
    setMotor(
      WHEEL_IN1,
      WHEEL_IN2,
      WHEEL_EN,
      -WHEEL_PWM
    );
  }
}

// ==================================================
// PERINTAH SERIAL
// ==================================================
void processCommand(char cmd) {
  switch (cmd) {

    case 'W':
    case 'w':
      wheelCommand = 1;
      wheelEnabled = true;
      Serial.println("RODA MAJU");
      break;

    case 'S':
    case 's':
      wheelCommand = -1;
      wheelEnabled = true;
      Serial.println("RODA MUNDUR");
      break;

    case 'X':
    case 'x':
      wheelCommand = 0;
      wheelEnabled = false;
      stopWheel();
      Serial.println("RODA STOP");
      break;

    case 'A':
    case 'a':
      setSteering(SERVO_LEFT);
      Serial.println("BELOK KIRI");
      break;

    case 'D':
    case 'd':
      setSteering(SERVO_RIGHT);
      Serial.println("BELOK KANAN");
      break;

    case 'C':
    case 'c':
      setSteering(SERVO_CENTER);
      Serial.println("SERVO TENGAH");
      break;

    case 'B':
    case 'b':
      if (sensorOK && !safetyTrip) {
        balanceEnabled = true;
        Serial.println("BALANCING ON");
      }
      break;

    case 'N':
    case 'n':
      balanceEnabled = false;
      stopBalanceMotor();
      Serial.println("BALANCING OFF");
      break;

    case 'R':
    case 'r':
      balanceEnabled = false;
      wheelEnabled = false;
      wheelCommand = 0;

      stopWheel();
      stopBalanceMotor();
      setSteering(SERVO_CENTER);

      Serial.println("SEMUA MOTOR STOP");
      break;

    case 'T':
    case 't':
      // Reset hanya saat motor berhenti.
      if (!balanceEnabled && !wheelEnabled) {
        safetyTrip = false;
        rollAngle = 0.0;

        Serial.println(
          "SAFETY RESET - periksa sepeda sebelum mengaktifkan."
        );
      }
      break;
  }
}

// ==================================================
// SETUP
// ==================================================
void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(WHEEL_IN1, OUTPUT);
  pinMode(WHEEL_IN2, OUTPUT);

  pinMode(BALANCE_IN1, OUTPUT);
  pinMode(BALANCE_IN2, OUTPUT);

  // ESP32 Arduino Core 3.x
  bool pwmA = ledcAttach(
    WHEEL_EN,
    PWM_FREQ,
    PWM_RES
  );

  bool pwmB = ledcAttach(
    BALANCE_EN,
    PWM_FREQ,
    PWM_RES
  );

  if (!pwmA || !pwmB) {
    Serial.println("ERROR: PWM gagal diinisialisasi.");

    while (true) {
      delay(1000);
    }
  }

  stopWheel();
  stopBalanceMotor();

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  setSteering(SERVO_CENTER);

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 tidak ditemukan!");

    while (true) {
      stopWheel();
      stopBalanceMotor();
      delay(100);
    }
  }

  mpu.setAccelerometerRange(
    MPU6050_RANGE_4_G
  );

  mpu.setGyroRange(
    MPU6050_RANGE_500_DEG
  );

  mpu.setFilterBandwidth(
    MPU6050_BAND_44_HZ
  );

  calibrateMPU();

  rollAngle = 0.0;
  sensorOK = true;

  lastControlTime = micros();
  lastPrintTime = millis();

  Serial.println();
  Serial.println("MINI BALANCING BIKE READY");
  Serial.println("-------------------------");
  Serial.println("W = Maju");
  Serial.println("S = Mundur");
  Serial.println("X = Stop roda");
  Serial.println("A = Belok kiri");
  Serial.println("D = Belok kanan");
  Serial.println("C = Servo tengah");
  Serial.println("B = Balancing ON");
  Serial.println("N = Balancing OFF");
  Serial.println("R = Semua motor STOP");
  Serial.println("T = Reset safety trip");
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd != '\n' && cmd != '\r') {
      processCommand(cmd);
    }
  }

  unsigned long now = micros();

  if (now - lastControlTime >= CONTROL_INTERVAL_US) {
    float dt =
      (now - lastControlTime) / 1000000.0;

    lastControlTime = now;

    updateSensor(dt);
    updateBalance();
    updateWheel();
  }

  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = millis();

    Serial.print("Roll: ");
    Serial.print(rollAngle, 2);

    Serial.print(" | Gyro: ");
    Serial.print(gyroRate, 2);

    Serial.print(" | PD: ");
    Serial.print(pidOutput, 2);

    Serial.print(" | PWM: ");
    Serial.print(currentBalancePWM);

    Serial.print(" | Servo: ");
    Serial.print(steeringAngle);

    Serial.print(" | Balancing: ");
    Serial.println(
      balanceEnabled ? "ON" : "OFF"
    );
  }
}
```