#include <Adafruit_PWMServoDriver.h>
#include <HCSR04.h>
#include "HUSKYLENS.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define USMIN  500
#define USMAX  1500
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

const float L = 80; // 63mm
const float l = 95; // 63mm
const int initialAngles[8] = {90, 0, 90, 0, 90, 0, 90, 0};

// Ultrasonic sensor
const int triggerPin = 9;  
const int echoPin = 10; 

// Huskylens
const int frameCenterX = 160;
const int frameCenterY = 120;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
int angles[8];
float x[4], y[4];

HUSKYLENS huskylens;
int ID1 = 1;

float height[4] = {120, 120, 120, 120}; // Initial height for each legs : 100mm
int pitch = -10; //-10;
int roll = 0;
int speedRatio = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  HCSR04.begin(triggerPin, echoPin);

  Wire.begin();
  huskylens.begin(Wire);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

  x[0] = x[1] = x[2] = x[3] = 0;
  y[0] = y[1] = y[2] = y[3] = 0;

  for(int i = 0 ; i < 8 ; i++) {
    pwm.writeMicroseconds(i, map(initialAngles[i], 0, 180, USMIN, USMAX));
  }

  IK();
  updatePositions();

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //move();
  //pitchMove();
  //rollMove();
  //pushUp();
  //tracking();
  test();
}

void tracking() {
  if (!huskylens.request(ID1)) {
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  } else if(!huskylens.isLearned()) {
    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  } else if(!huskylens.available()) {
    Serial.println(F("No block or arrow appears on the screen!"));
  } else {
      HUSKYLENSResult result = huskylens.read();
      int errorPitch = result.yCenter - frameCenterY;
      int errorRoll = result.xCenter - frameCenterX;
      pitch = map(errorPitch, -120, 120, 10, -10);
      roll = map(errorRoll, -160, 160, 10, -10);
      IK();
      updatePositions();
  }
}

void move() {
  gait(60);
}

void gait(int stride) {
  float speed = stride / speedRatio;

  for(x[0] = 0 , x[3] = 0 ; x[0] <= stride && x[3] <= stride ; x[0] += speed, x[3] += speed) {
    y[0] = sqrt(pow(((float)stride / 2), 2) - pow((x[0] - ((float)stride / 2)), 2));
    y[3] = sqrt(pow(((float)stride / 2), 2) - pow((x[3] - ((float)stride / 2)), 2));
    IK();
    updatePositions();
    delay(5);
  }

  for(x[0] = stride, x[3] = stride ; x[0] >= 0 && x[3] >= 0 ; x[0] -= speed, x[3] -= speed) {
    IK();
    updatePositions();
    delay(5);
  }

  for(x[1] = 0 , x[2] = 0 ; x[1] <= stride && x[2] <= stride ; x[1] += speed, x[2] += speed) {
    y[1] = sqrt(pow(((float)stride / 2), 2) - pow((x[1] - ((float)stride / 2)), 2));
    y[2] = sqrt(pow(((float)stride / 2), 2) - pow((x[2] - ((float)stride / 2)), 2));
    IK();
    updatePositions();
    delay(5);
  }

  for(x[1] = stride, x[2] = stride ; x[1] >= 0 && x[2] >= 0 ; x[1] -= speed, x[2] -= speed) {
    IK();
    updatePositions();
    delay(5);
  }
}

void test() {
  int steps = 6;
  for(y[0] = 0, y[3] = 0 ; y[0] <= 50 && y[3] <= 50 ; y[0] += steps , y[3] += steps) {
    IK();
    updatePositions();
  }
  for(x[0] = 0, x[3] = 0 ; x[0] <= 60 && x[3] <= 60 ; x[0] += steps, x[3] += steps) {
    IK();
    updatePositions();
  }
  for(y[0] = 50, y[3] = 50 ; y[0] >= 0 && y[3] >= 0 ; y[0] -= steps, y[3] -= steps) {
    IK();
    updatePositions();
  }
  for(x[0] = 60, x[3] = 60 ; x[0] >= 0 && x[3] >= 0 ; x[0] -= steps, x[3] -= steps) {
    IK();
    updatePositions();
  }
  for(y[1] = 0, y[2] = 0 ; y[1] <= 50 && y[2] <= 50 ; y[1] += steps, y[2] += steps) {
    IK();
    updatePositions();
  }
  for(x[1] = 0, x[2] = 0 ; x[1] <= 60 && x[2] <= 60 ; x[1] += steps, x[2] += steps) {
    IK();
    updatePositions();
  }
  for(y[1] = 50, y[2] = 50 ; y[1] >= 0 && y[2] >= 0 ; y[1] -= steps, y[2] -= steps) {
    IK();
    updatePositions();
  }
  for(x[1] = 60, x[2] = 60 ; x[1] >= 0 && x[2] >= 0 ; x[1] -= steps, x[2] -= steps) {
    IK();
    updatePositions();
  }
}

void rollMove() {
  for(roll = 0; roll <= 7 ; roll++) {
    IK();
    updatePositions();
    delay(20);
  }
  for(roll = 7; roll >= 0 ; roll--) {
    IK();
    updatePositions();
    delay(20);
  }
  for(roll = 0; roll >= -7 ; roll--) {
    IK();
    updatePositions();
    delay(20);
  }
  for(roll = -7; roll <= 0 ; roll++) {
    IK();
    updatePositions();
    delay(20);
  }
}

void pitchMove() {
  for(pitch = 0 ; pitch <= 10 ; pitch++) {
    IK();
    updatePositions();
    delay(20);
  }
  for(pitch = 10 ; pitch >= 0 ; pitch--) {
    IK();
    updatePositions();
    delay(20);
  }
  for(pitch = 0 ; pitch >= -20 ; pitch--) {
    IK();
    updatePositions();
    delay(20);
  }
  for(pitch = -20 ; pitch <= 0 ; pitch++) {
    IK();
    updatePositions();
    delay(20);
  }
}

void pushUp() {
  for(int h = 120 ; h >= 80 ; h--) {
    height[0] = height[1] = height[2] = height[3]= h;
    IK();
    updatePositions();
    delay(1);
  }
  for(int h = 80 ; h <= 120 ; h++) {
    height[0] = height[1] = height[2] = height[3]= h;
    IK();
    updatePositions();
    delay(1);
  }
}

void updatePositions() {
  for(int i = 0 ; i < 8 ; i++) {
    int angle = angles[i];
    int microseconds = map(angle, 0, 180, USMIN, USMAX);
    pwm.writeMicroseconds(i, microseconds);
  }
}

void IK() {

  float a[4], b[4], theta[4], h[4];

  for(int i = 0 ; i < 4 ; i++) {

    if(i == 0) {
      theta[i] = atan((x[i] + 63.5 * (1 - cos((pitch * PI) / 180))) / height[i]);
      h[i] =  ((height[i] - y[i]) + 63.5 * sin((pitch * PI) / 180) + 87 * sin((roll * PI) / 180)) / cos(theta[i]);

    } else if(i == 1) {
      theta[i] = atan((x[i] + 63.5 * (1 - cos((pitch * PI) / 180))) / height[i]);
      h[i] =  ((height[i] - y[i]) + 63.5 * sin((pitch * PI) / 180) - 87 * sin((roll * PI) / 180)) / cos(theta[i]);

    } else if(i == 2) {
      theta[i] = atan((x[i] - 63.5 * (1 - cos((pitch * PI) / 180))) / height[i]);
      h[i] =  ((height[i] - y[i]) - 63.5 * sin((pitch * PI) / 180) + 87 * sin((roll * PI) / 180)) / cos(theta[i]);

    } else {
      theta[i] = atan((x[i] - 63.5 * (1 - cos((pitch * PI) / 180))) / height[i]);
      h[i] =  ((height[i] - y[i]) - 63.5 * sin((pitch * PI) / 180) - 87 * sin((roll * PI) / 180)) / cos(theta[i]);

    }

    a[i] = acos((pow(L, 2) + pow(h[i], 2) - pow(l, 2)) / (2 * L * h[i]));
    b[i] = acos((pow(L, 2) + pow(l, 2) - pow(h[i], 2)) / (2 * L * l));

  }

  // front right leg --> index = 0
  angles[0] = initialAngles[0] - abs(a[0] * 180 / PI) + (theta[0] * 180 / PI);
  angles[1] = initialAngles[1] + 180 - abs(b[0] * 180 / PI);

  // back left leg --> index = 3
  angles[6] = initialAngles[6] - abs(a[3] * 180 / PI) + (theta[3] * 180 / PI);
  angles[7] = initialAngles[7] + 180 - abs(b[3] * 180 / PI);

  // Opposite side :
  // front left leg --> index = 1
  angles[2] = initialAngles[2] - abs(a[1] * 180 / PI) + (theta[1] * 180 / PI);
  angles[3] = initialAngles[3] + 180 - abs(b[1] * 180 / PI);

  // back right leg --> index = 2
  angles[4] = initialAngles[4] - abs(a[2] * 180 / PI) + (theta[2] * 180 / PI);
  angles[5] = initialAngles[5] + 180 - abs(b[2] * 180 / PI);
}
