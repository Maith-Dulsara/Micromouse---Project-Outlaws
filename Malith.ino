#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

// IR sensor pins
const int leftIR = 0;
const int middleIR = 19;
const int rightIR = 16;

#define leftIR_out 32
#define middleIR_out 39
#define rightIR_out 35

// Motor control pins
const int leftMotorFwdPin = 25;
const int leftMotorRevPin = 33;
const int rightMotorFwdPin = 4;
const int rightMotorRevPin = 2;

int IRenable[3] = {leftIR, middleIR, rightIR};
int IRoutput[3] = {leftIR_out, middleIR_out, rightIR_out};

int a[3]; int b[3]; int c[3];

// PID constants
double target = 0;
double angle = 0;
double kp = 10;
double ki = 6;
double kd = 11.0;
int lstval = 0;
long total_err = 0;
double lastInput = 0;
double integral = 0;

// Motor speed
int Speed = 120;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(leftMotorFwdPin, OUTPUT);
  pinMode(leftMotorRevPin, OUTPUT);
  pinMode(rightMotorFwdPin, OUTPUT);
  pinMode(rightMotorRevPin, OUTPUT);

  // IR sensor pins
  pinMode(leftIR, INPUT);
  pinMode(middleIR, INPUT);
  pinMode(rightIR, INPUT);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  while (status != 0) { }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);

  mpu.calcOffsets();
  Serial.println("Done!\n");
}

void loop() {
  gyro();
  readIRSensor();

  // Check for obstacles and perform obstacle avoidance
  if (c[1] < 1500) {
    movePID(angle);
  } else {
    if (c[0] > 1500 && c[2] > 700) {
      // Obstacle detected on the left, turn right
      stopMotors();
      delay(500);  // Adjust delay for desired turn duration
      turnRight();
    }
    if (c[2] > 1500 && c[0] > 700) {
      // Obstacle detected on the right, turn left
      stopMotors();
      delay(500);  // Adjust delay for desired turn duration
      turnLeft();
    }
  }
}

void movePID(int targetAngle) {
  int error = targetAngle - angle;
  double Pterm = kp * error;
  double Dterm = kd * (error - lastInput);
  integral += error * ki;
  int diff = Pterm + Dterm + integral;

  lastInput = error;
  motorControl(Speed + diff, Speed - diff);
}

void motorControl(int leftSpeed, int rightSpeed) {
  // Control motor speed and direction
  if (leftSpeed > 180) {
    leftSpeed = 180;
  }
  if (leftSpeed < 0) {
    leftSpeed = 0;
  }
  if (rightSpeed > 180) {
    rightSpeed = 180;
  }
  if (rightSpeed < 0) {
    rightSpeed = 0;
  }

  // Control left motor
  analogWrite(leftMotorFwdPin, leftSpeed);
  digitalWrite(leftMotorRevPin, LOW);

  // Control right motor
  analogWrite(rightMotorFwdPin, rightSpeed);
  digitalWrite(rightMotorRevPin, LOW);
}

void stopMotors() {
  // Stop both motors
  analogWrite(leftMotorFwdPin, 0);
  analogWrite(rightMotorFwdPin, 0);
}

void turnLeft() {
  // Turn left by reversing the left motor
  analogWrite(leftMotorRevPin, Speed);
  digitalWrite(rightMotorRevPin, LOW);
}

void turnRight() {
  // Turn right by reversing the right motor
  analogWrite(rightMotorRevPin, Speed);
  digitalWrite(leftMotorRevPin, LOW);
}

void readIRSensor() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(IRenable[i], HIGH);
    delay(10);
    a[i] = analogRead(IRoutput[i]);
    digitalWrite(IRenable[i], LOW);
    delay(10);
    b[i] = analogRead(IRoutput[i]);

    c[i] = a[i] - b[i];

    Serial.print(c[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}

void gyro() {
  mpu.update();
  if ((millis() - timer) > 10) {
    timer = millis();
  }
  angle = mpu.getAngleZ();
}
