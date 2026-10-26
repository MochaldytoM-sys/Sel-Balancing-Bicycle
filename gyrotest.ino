#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float pitch, roll, yaw;  // Variable to store yaw, pitch, roll values
unsigned long last_time;
float gyroX_rate, gyroY_rate, gyroZ_rate;  // Gyroscope rate for integration

// Definisikan pin untuk motor driver L298N
const int motor1IN1 = 14;  // IN1 (arah motor 1 maju)
const int motor1IN2 = 27;  // IN2 (arah motor 1 mundur)
const int motor1PWM = 13;  // ENA (PWM kecepatan motor 1)

// Batas toleransi roll untuk menghentikan motor
const float rollToleranceMin = -0.50;
const float rollToleranceMax = 0.50;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Atur pin motor driver sebagai output
  pinMode(motor1IN1, OUTPUT);
  pinMode(motor1IN2, OUTPUT);
  pinMode(motor1PWM, OUTPUT);

  Serial.println("");
  delay(100);
  last_time = millis();
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate Pitch and Roll from Accelerometer
  roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  // Calculate Yaw from Gyroscope (Integration of gyroZ over time)
  unsigned long current_time = millis();
  float delta_time = (current_time - last_time) / 1000.0;  // Convert to seconds

  gyroZ_rate = g.gyro.z;  // Gyroscope Z-axis rotation rate
  yaw += gyroZ_rate * delta_time;  // Integrate to get yaw

  // Display the results
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("\tRoll: ");
  Serial.print(roll);
  Serial.print("\tYaw: ");
  Serial.println(yaw);

  // Logika kontrol motor berdasarkan nilai Roll
  if (roll > rollToleranceMin && roll < rollToleranceMax) {
    // Jika roll berada dalam rentang toleransi, matikan motor
    digitalWrite(motor1IN1, LOW);
    digitalWrite(motor1IN2, LOW);
    analogWrite(motor1PWM, 0);
  } else if (roll > 0) {
    // Jika roll positif, motor bergerak maju
    digitalWrite(motor1IN1, HIGH);
    digitalWrite(motor1IN2, LOW);
    analogWrite(motor1PWM, map(roll, 0.5, 90, 0, 255));  // Kecepatan sesuai roll
  } else if (roll < 0) {
    // Jika roll negatif, motor bergerak mundur
    digitalWrite(motor1IN1, LOW);
    digitalWrite(motor1IN2, HIGH);
    analogWrite(motor1PWM, map(abs(roll), 0.5, 90, 0, 255));  // Kecepatan sesuai roll
  }

  Serial.println("");
  last_time = current_time;
  delay(100);  // Update rate of sensor readings
}
