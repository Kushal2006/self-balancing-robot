#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// ================= PIN DEFINITIONS =================
const int MOTOR_L_PWM = 25;
const int MOTOR_L_DIR = 26;
const int MOTOR_R_PWM = 27;
const int MOTOR_R_DIR = 14;

const int SDA_PIN = 21;
const int SCL_PIN = 22;
// ==================================================

// ================= PID PARAMETERS =================
float Kp = 25.0;
float Ki = 1.2;
float Kd = 0.8;
// ==================================================

// ================= GLOBAL VARIABLES ================
float setPoint = 0.0;      // Upright angle
float angle = 0.0;

float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;

unsigned long prevTime = 0;
float dt = 0;

// IMU raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;
// ==================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  prevTime = millis();
  Serial.println("Self Balancing Robot Initialized");
}

void loop() {
  // Calculate time difference
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Read MPU6050 data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Accelerometer angle (pitch)
  float accAngle = atan2(ay, az) * 180 / PI;

  // Gyroscope rate (deg/sec)
  float gyroRate = gx / 131.0;

  // Complementary filter
  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;

  // PID calculations
  error = setPoint - angle;
  integral += error * dt;
  derivative = (error - prevError) / dt;

  float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prevError = error;

  // Drive motors
  driveMotors(pidOutput);

  // Debug output
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" | PID Output: ");
  Serial.println(pidOutput);
}

void driveMotors(float pidOutput) {
  int pwmValue = constrain(abs(pidOutput), 0, 255);

  if (pidOutput > 0) {
    digitalWrite(MOTOR_L_DIR, HIGH);
    digitalWrite(MOTOR_R_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_L_DIR, LOW);
    digitalWrite(MOTOR_R_DIR, LOW);
  }

  analogWrite(MOTOR_L_PWM, pwmValue);
  analogWrite(MOTOR_R_PWM, pwmValue);
}
