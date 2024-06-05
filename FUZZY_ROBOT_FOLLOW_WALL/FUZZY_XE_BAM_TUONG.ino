/*-------------------------------Include library--------------------------*/
#include <Wire.h>
#include <VL53L0X.h>
/*-------------------------------Define sensor--------------------------*/
const uint8_t sensorCount = 3;
const uint8_t xshutPins[sensorCount] = { 4, 2, 15 };  // xshutPins - EN sensors
VL53L0X sensors[sensorCount];
/*-------------------------------PWM--------------------------*/
const int ledPin1 = 25;  //chân xung pwm 1
const int ledPin2 = 13;  //chân xung pwm 2
// configuration of PWM channels
const int freq = 5000;      // tần số xung
const int ledChannel1 = 0;  // kênh PWM
const int ledChannel2 = 1;  // kênh PWM
const int resolution = 8;   // độ phân giải 8bit
/*-------------------------------MOTOR DIRECTION--------------------------*/
#define DIR1_1 27
#define DIR1_2 26
#define DIR2_1 12
#define DIR2_2 14
/*-------------------------------GLOBAL VARIABLES--------------------------*/
bool running_focus_left = false;
bool running_focus_right = false;
int sensorLeft, sensorMid, sensorRight;  // sensors value
unsigned long previousMillis = 0;
int previousSensorLeft, previousSensorRight;
int missLeftWall, missRightWall;
/*-------------------------------PID Define--------------------------*/
float baseFwd, turn;
float sumErrorLeft, previousErrorLeft;
float sumErrorRight, previousErrorRight;
int leftDuty, rightDuty;
/*-------------------------------check Define--------------------------*/
int cnt;
/*-------------------------------Define Fuzzy--------------------------*/
float u_dot = 0;
float u = 0;
float C1 = 0.22;
float C2 = 0.25;
float C3 = 0.31;
float C4 = 0.21;
float C5 = 0.27;
float k1 = 0.5 / 40.0;
float k2 = 0.5 / 150.0;
float ku = 20;
/*-------------------------------ROBOT PARAMETERS--------------------------*/
float cal_trueMF(float e, float L, float c1, float c2, float R) {
  float y = 0;
  if (e < L) {
    y = 0;
  } else if (e < c1) {
    y = (e - L) / (c1 - L);
  } else if (e < c2) {
    y = 1;
  } else if (e < R) {
    y = (R - e) / (R - c2);
  } else {
    y = 0;
  }
  return y;
}
float Calibrate_Fuzzy(float e, float e_dot) {
  float e_NB, e_NS, e_ZE, e_PS, e_PB;
  float de_NE, de_ZE, de_PO;
  float y_NB, y_NM, y_NS, y_ZE, y_PS, y_PM, y_PB;
  float beta1, beta2, beta3, beta4, beta5, beta6, beta7, beta8, beta9, beta10, beta11, beta12, beta13, beta14, beta15;
  float y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12, y13, y14, y15;

  e = constrain(e, -1, 1);
  e_dot = constrain(e_dot, -1, 1);

  Serial.print("\te:");
  Serial.print(e);
  Serial.print("\te_dot:");
  Serial.print(e_dot);
  Serial.print(" ");

  e_NB = cal_trueMF(e, -10, -1, -C2, -C1);
  e_NS = cal_trueMF(e, -C2, -C1, -C1, 0);
  e_ZE = cal_trueMF(e, -C1, 0, 0, C1);
  e_PS = cal_trueMF(e, 0, C1, C1, C2);
  e_PB = cal_trueMF(e, C1, C2, 1, 10);

  de_NE = cal_trueMF(e_dot, -10, -1, -C3, 0);
  de_ZE = cal_trueMF(e_dot, -C3, 0, 0, C3);
  de_PO = cal_trueMF(e_dot, 0, C3, 1, 10);
  //OUTPUT
  y_NB = -1;
  y_NM = -C5;
  y_NS = -C4;
  y_ZE = 0;
  y_PS = C4;
  y_PM = C5;
  y_PB = 1;
  //Fuzzy Rules
  beta1 = e_NB * de_NE;
  y1 = y_PB;
  beta2 = e_NB * de_ZE;
  y2 = y_PM;
  beta3 = e_NB * de_PO;
  y3 = y_PS;

  beta4 = e_NS * de_NE;
  y4 = y_PM;
  beta5 = e_NS * de_ZE;
  y5 = y_PS;
  beta6 = e_NS * de_PO;
  y6 = y_ZE;

  beta7 = e_ZE * de_NE;
  y7 = y_PS;
  beta8 = e_ZE * de_ZE;
  y8 = y_ZE;
  beta9 = e_ZE * de_PO;
  y9 = y_NS;

  beta10 = e_PS * de_NE;
  y10 = y_ZE;
  beta11 = e_PS * de_ZE;
  y11 = y_NS;
  beta12 = e_PS * de_PO;
  y12 = y_NM;

  beta13 = e_PB * de_NE;
  y13 = y_NS;
  beta14 = e_PB * de_ZE;
  y14 = y_NM;
  beta15 = e_PB * de_PO;
  y15 = y_NB;
  //Defuzzification: Weighted Average
  u_dot = beta1 * y1 + beta2 * y2 + beta3 * y3 + beta4 * y4 + beta5 * y5
      + beta6 * y6 + beta7 * y7 + beta8 * y8 + beta9 * y9 + beta10 * y10
      + beta11 * y11 + beta12 * y12 + beta13 * y13 + beta14 * y14 + beta15 * y15;
  return u += u_dot;
}
float Fuzzy_controlLeft(float sensorLeft, float desiredSensorLeft, float Ts) {
  float errorLeft = desiredSensorLeft - (float)sensorLeft;
  float errorLeft_dot = (errorLeft - previousErrorLeft) / Ts;
  float turn_ = constrain(Calibrate_Fuzzy(errorLeft * k1, errorLeft_dot * k2) * ku, -50, 50);
  Serial.print("\tturn_:");
  Serial.print(turn_);
  Serial.print("\tu:");
  Serial.print(u);
  Serial.print("\terrorLeft:");
  Serial.print(errorLeft);
  Serial.print("\terrorLeft_dot:");
  Serial.print(errorLeft_dot);
  Serial.print(" ");
  previousErrorLeft = errorLeft;
  return turn_;
}
void setupsensor() {
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(100);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1) {}
    }
    sensors[i].setAddress(0x50 + i);  //change adress sensors
    // sensors[i].setMeasurementTimingBudget(20000);
    sensors[i].startContinuous();
  }
}
void setup_pwm() {
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcAttachPin(ledPin1, ledChannel1);
  ledcAttachPin(ledPin2, ledChannel2);

  pinMode(DIR1_1, OUTPUT);
  pinMode(DIR1_2, OUTPUT);
  pinMode(DIR2_1, OUTPUT);
  pinMode(DIR2_2, OUTPUT);
}
void print_value() {
  Serial.print("L:");
  Serial.print(sensorLeft);
  Serial.print('\t');
  Serial.print("M: ");
  Serial.print(sensorMid);
  Serial.print('\t');
  Serial.print("R: ");
  Serial.print(sensorRight);
  Serial.print('\t');
}
void read_value_sensors() {
  //read value
  sensorLeft = sensors[0].readRangeContinuousMillimeters() - 30;
  sensorMid = sensors[1].readRangeContinuousMillimeters();
  sensorRight = sensors[2].readRangeContinuousMillimeters() - 60;
  // giới hạn giá trị
  sensorLeft = constrain(sensorLeft, 30, 250);  //range from 30 to 250
  sensorMid = constrain(sensorMid, 30, 250);
  sensorRight = constrain(sensorRight, 30, 250);
}
int getWallMidState(int sensorMid, int threshold) {
  if (sensorMid > threshold) return 0;
  else return 1;
}
bool getWallLeft() {
  if ((sensorLeft) >= 130) return true;
  else return false;
}
bool getWallRight() {
  if ((sensorRight - previousSensorRight) >= 100) return true;
  else return false;
}
float PID_LeftWall(int sensorLeft, float desiredSensorLeft, float Kp, float Ki, float Kd, float Ts) {
  float errorLeft = desiredSensorLeft - (float)sensorLeft;
  sumErrorLeft += errorLeft;
  float turn_ = Kp * errorLeft + Ki * sumErrorLeft * Ts + Kd * (errorLeft - previousErrorLeft) / Ts;
  previousErrorLeft = errorLeft;
  return turn_;
}
float PID_RightWall(int sensorRight, float desiredSensorRight, float Kp, float Ki, float Kd, float Ts) {
  float errorRight = (float)sensorRight - desiredSensorRight;
  sumErrorRight += errorRight;
  float turn_ = Kp * errorRight + Ki * sumErrorRight * Ts + Kd * (errorRight - previousErrorRight) / Ts;
  previousErrorRight = errorRight;
  return turn_;
}
void Left_Motor_PWM(float PWM_) {
  if (PWM_ >= 0) {
    ledcWrite(ledChannel1, PWM_);
    digitalWrite(DIR1_1, LOW);
    digitalWrite(DIR1_2, HIGH);
  } else {
    ledcWrite(ledChannel1, abs(PWM_));
    digitalWrite(DIR1_1, HIGH);
    digitalWrite(DIR1_2, LOW);
  }
}
void Right_Motor_PWM(float PWM_) {
  if (PWM_ >= 0) {
    ledcWrite(ledChannel2, PWM_);
    digitalWrite(DIR2_1, HIGH);
    digitalWrite(DIR2_2, LOW);
  } else {
    ledcWrite(ledChannel2, abs(PWM_));
    digitalWrite(DIR2_1, LOW);
    digitalWrite(DIR2_2, HIGH);
  }
}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C
  setupsensor();
  delay(1000);
  setup_pwm();
  Serial.println("Focus left or Focus right ?");
  while (1) {
    read_value_sensors();
    // print_value();
    // Serial.println();
    if (sensorLeft < 40) {
      Serial.println("LEFT!");
      running_focus_left = true;  // focus left
      break;
    }
    // if (sensorRight < 40) {
    //   Serial.println("RIGHT!");
    //   running_focus_right = true;  // focus right
    //   break;
    // }
  }
}

void loop() {
  unsigned long nowMillis = millis();
  if (nowMillis - previousMillis >= 100) {
    read_value_sensors();
    print_value();
    // trạng thái tường trước với giá trị đặt là 75mm
    int state = getWallMidState(sensorMid, 90);
    if (running_focus_left) {
      Serial.print("state ");
      Serial.print(state);
      Serial.print('\t');
      Serial.print("missLeftWall ");
      Serial.print(getWallLeft());
      Serial.print('\t');
      // if ((sensorLeft - previousSensorLeft) >= 100) missLeftWall = true;
      if (!getWallLeft()) {
        turn = Fuzzy_controlLeft(sensorLeft, 70, 0.1);
        Serial.print(" FUZZY LEFT ");
        baseFwd = 50;  // 250
      }
      Serial.println();
      // previousSensorLeft = sensorLeft;
      previousMillis = nowMillis;
    }
    if (running_focus_right) {
      Serial.print("state ");
      Serial.print(state);
      Serial.print('\t');
      Serial.print("missRightWall ");
      Serial.print(getWallRight());
      Serial.print('\t');
      if (!getWallRight()) {
        cnt = 0;
        if (state == 0) {
          turn = PID_RightWall(sensorRight, 70.0, 2.5, 0.0, 0.1, 0.05);
          Serial.print(" PID RIGHT ");
          baseFwd = 250;
        } else {
          sumErrorRight = 0;
          baseFwd = 0;
          Serial.print(" TURN RIGHT ");
          turn = -250;
        }
      } else {
        sumErrorRight = 0;
        cnt += 1;
        if (cnt <= 4) {  // tuning  number 10 for other condition // timming
          turn = 0;
          baseFwd = 250;
          Serial.print(" Straight");
        } else if (cnt <= 7) {  // tuning  number 15 for other condition /timming
          baseFwd = 0;
          turn = 100;
          Serial.print(" Turn Right with holding position ");
        } else {
          baseFwd = 150;
          turn = 90;
          Serial.print(" Turn Right");
        }
        if (!getWallRight()) {
          cnt = 0;
          baseFwd = 0;
          turn = 0;
        }
      }
      //Serial.println();
      //previousSensorLeft = sensorLeft;
      previousMillis = nowMillis;
    }
    // leftDuty = constrain(baseFwd + turn, -250.0, 250.0);
    // rightDuty = constrain(baseFwd - turn, -250.0, 250.0);
    leftDuty = constrain(baseFwd - turn, -250.0, 250.0);
    rightDuty = constrain(baseFwd + turn, -250.0, 250.0);
    Serial.print('\t');
    Serial.print(leftDuty);
    Serial.print('\t');
    Serial.print(rightDuty);
    Serial.print('\t');
    Serial.println();
    Left_Motor_PWM((int)leftDuty);
    Right_Motor_PWM((int)rightDuty);
    // Left_Motor_PWM((int)100);
    // Right_Motor_PWM((int)-100);
    // previousSensorLeft = sensorLeft;
    previousMillis = nowMillis;
  }
}