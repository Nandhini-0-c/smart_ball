#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h>
#include <math.h>

// Sensor and Bluetooth objects
Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;

// Kalman filter variables
float velocity = 0;
float P = 1, Q = 0.005, R = 1, K;
float filtered_accel_x = 0, last_velocity = 0;
unsigned long last_time = 0;

// Calibration bias
float accel_bias_x = 0, gyro_bias_z = 0;

// Spin variables
float angular_velocity_z = 0;
float spin_rate = 0;
float total_rotation = 0;

// Orientation variables
float pitch = 0, roll = 0;
float alpha_orientation = 0.98; // Complementary filter weight

// Control flags
bool capture = false;
bool released = false;

// Stationary and release detection
const float STATIONARY_THRESHOLD = 0.15;
const float RELEASE_DROP_THRESHOLD = 6;
static float prev_accel_mag = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("SmartBall");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);
  calibrateMPU();
  last_time = millis();

  SerialBT.println("Send 'start' to begin tracking or 'stop' to end tracking.");
}

void loop() {
  // Check Bluetooth command
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd == "start") {
      capture = true;
      released = false;
      velocity = 0;
      last_velocity = 0;
      total_rotation = 0;
      last_time = millis();
      SerialBT.println("Started tracking...");
    } else if (cmd == "stop") {
      capture = false;
      velocity = 0;
      last_velocity = 0;
      total_rotation = 0;
      SerialBT.println("Stopped tracking and reset state.");
    }
  }

  if (!capture) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Bias-corrected raw values
  float accel_x = a.acceleration.x - accel_bias_x;
  angular_velocity_z = g.gyro.z - gyro_bias_z;

  // Low-pass filter acceleration
  float alpha = 0.3;
  filtered_accel_x = alpha * accel_x + (1 - alpha) * filtered_accel_x;

  // Time step
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  // Kalman prediction step
  float predicted_velocity = last_velocity + filtered_accel_x * dt;
  P += Q;
  K = P / (P + R);
  velocity = predicted_velocity;
  P *= (1 - K);
  last_velocity = velocity;

  // Spin computation
  total_rotation += angular_velocity_z * dt;
  spin_rate = fabs(angular_velocity_z * 9.5493);  // rad/s to RPM

  // Complementary filter for pitch and roll
  float pitch_acc = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  float roll_acc  = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;
  pitch = alpha_orientation * (pitch + g.gyro.x * dt * 180 / PI) + (1 - alpha_orientation) * pitch_acc;
  roll  = alpha_orientation * (roll  + g.gyro.y * dt * 180 / PI) + (1 - alpha_orientation) * roll_acc;

  // Release detection (based on total acceleration drop)
  float acc_mag = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
  if (!released && prev_accel_mag > 12 && acc_mag < RELEASE_DROP_THRESHOLD) {
    released = true;
    SerialBT.println("Ball Released!");
    Serial.println("Ball Released!");
  }
  prev_accel_mag = acc_mag;

  // Stationary reset
  if (fabs(filtered_accel_x) < STATIONARY_THRESHOLD && fabs(angular_velocity_z) < 0.05) {
    velocity = 0;
    last_velocity = 0;
    total_rotation = 0;
  }

  // Send data via Bluetooth and Serial
  String output = "Speed: " + String(velocity, 2) + " m/s\n";
  output += "Spin Rate: " + String(spin_rate, 2) + " RPM\n";
  output += "Pitch: " + String(pitch, 1) + "°\n";
  output += "Roll: " + String(roll, 1) + "°\n";
  output += "Accel X: " + String(filtered_accel_x, 2) + " m/s²\n";
  output += "Gyro Z: " + String(angular_velocity_z, 2) + " rad/s\n";
  output += "-------------------------\n";

  Serial.print(output);
  SerialBT.print(output);

  delay(50); // Smoothing
}

void calibrateMPU() {
  Serial.println("Calibrating...");
  float sum_ax = 0, sum_gz = 0;
  const int samples = 200;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum_ax += a.acceleration.x;
    sum_gz += g.gyro.z;
    delay(10);
  }

  accel_bias_x = sum_ax / samples;
  gyro_bias_z = sum_gz / samples;
  Serial.println("Calibration Done.");
}


