//Preparation
  //Library
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <Wire.h>

#define TCAADDR 0x70

//Object
  //MPU
MPU6050 MPU1(Wire);
MPU6050 MPU2(Wire);
MPU6050 MPU3(Wire);
MPU6050 MPU4(Wire);
MPU6050 MPU5(Wire);

  //Servo
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;

//Variable
  //IMU
float pitch1, roll1, yaw1;
float pitch2, roll2, yaw2;
float pitch3, roll3, yaw3;
float pitch4, roll4, yaw4;
float pitch5, roll5, yaw5;

int theta1_i_IMU;
int theta1_f_IMU;
int theta2_i_IMU;
int theta2_f_IMU;
int theta3_i_IMU;
int theta3_f_IMU;
int theta4_i_IMU;
int theta4_f_IMU;
int theta5_i_IMU;
int theta5_f_IMU;

  //Ultrsaonic Sensor
int trig = 11;
int echo = 12;
float duration;
float distance_mm;

  //Vibration Sensor
int vib_sensor = 13;
float vib_val;

  //Servo
int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;
int pos5 = 0;

  //Push Button
int PB1Pin = 5;
int PB2Pin = 6;
int PB3Pin = 7;
int PB4Pin = 8;
int PB5Pin = 9;

int PB1State = 0;
int LastPB1State = 0;
int PB2State = 0;
int LastPB2State = 0;
int PB3State = 0;
int LastPB3State = 0;
int PB4State = 0;
int LastPB4State = 0;
int PB5State = 0;
int LastPB5State = 0;

bool PBPress = false;

  //End Effector
int EE = 10;

  //Link Length
const int d1 = 700;
const int d2 = 2400;
const int d3 = 2100;
const int d4 = 650;

  //Joint Angle
int theta1_i;
int theta1_f;
int theta2_i_1;
int theta2_i_2;
int theta2_f_1;
int theta2_f_2;
int theta3_i_1;
int theta3_i_2;
int theta3_f_1;
int theta3_f_2;
int theta4_i_1;
int theta4_i_2;
int theta4_f_1;
int theta4_f_2;
int theta5_i;
int theta5_f;

int StopAngle1;
int StopAngle2;
int StopAngle3;
int StopAngle4;
int StopAngle5;

  //Distance
float p_i;
float p_f;
float pz_i_1;
float pz_i_2;
float pz_f_1;
float pz_f_2;
 
//Main Code
  //MUX
void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

  //Setup
void setup() {
    //MUX and IMU
  tcaselect(0);
  MPU1.begin();
  tcaselect(1);
  MPU2.begin();
  tcaselect(2);
  MPU3.begin();
  tcaselect(3);
  MPU4.begin();
  tcaselect(4);
  MPU5.begin();
  
    //Ultrasonic Sensor
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

    //Vibration Sensor
  pinMode(vib_sensor, INPUT);
  
    //Servo
  Servo1.attach(0);
  Servo2.attach(1);
  Servo3.attach(2);
  Servo4.attach(3);
  Servo5.attach(4);
  
  Servo1.write(pos1);
  Servo2.write(pos2);
  Servo3.write(pos3);
  Servo4.write(pos4);
  Servo5.write(pos5);
  
    //Push Button
  pinMode(PB1Pin, INPUT_PULLUP);
  pinMode(PB2Pin, INPUT_PULLUP);
  pinMode(PB3Pin, INPUT_PULLUP);
  pinMode(PB4Pin, INPUT_PULLUP);
  pinMode(PB5Pin, INPUT_PULLUP);

    //I2C Communcation
  Wire.begin();

    //Serial Monitor
  Serial.begin(9600);
}

//Loop
void loop() {
  //Get Data
  GetIMUData();
  GetUSData();
  GetVibData();
  if (vib_val > 10000)  {
    EmergencyStop();
  }

  //Display Data
  DisplayData();

  //Choose Handling Mode
  PB1State = digitalRead(PB1Pin);
  PB2State = digitalRead(PB2Pin);
  PB3State = digitalRead(PB3Pin);
  PB4State = digitalRead(PB4Pin);
  PB5State = digitalRead(PB5Pin);

  //Check Push Button Press
  CheckPB1Press();
  CheckPB2Press();
  CheckPB3Press();
  CheckPB4Press();
  CheckPB5Press();
}

//Get IMU Data
void GetIMUData() {
  tcaselect(0);
  MPU1.update();
  pitch1 = MPU1.getAngleX();
  roll1 = MPU1.getAngleY();
  yaw1 = MPU1.getAngleZ();
  
  tcaselect(1);
  MPU2.update();
  pitch2 = MPU2.getAngleX();
  roll2 = MPU2.getAngleY();
  yaw2 = MPU2.getAngleZ();
  
  tcaselect(2);
  MPU3.update();
  pitch3 = MPU3.getAngleX();
  roll3 = MPU3.getAngleY();
  yaw3 = MPU3.getAngleZ();
  
  tcaselect(3);
  MPU4.update();
  pitch4 = MPU4.getAngleX();
  roll4 = MPU4.getAngleY();
  yaw4 = MPU4.getAngleZ();
  
  tcaselect(4);
  MPU5.update();
  pitch5 = MPU5.getAngleX();
  roll5 = MPU5.getAngleY();
  yaw5 = MPU5.getAngleZ();
}

//Get Ultrasonic Sensor Data
void GetUSData() {
  digitalWrite(trig, HIGH);
  delay(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance_mm = duration/5.84;
  delay(100);
}

//Get Vibration Sensor Data
void GetVibData() {
  vib_val = pulseIn(vib_sensor, HIGH);
  return vib_val;
  delay(10);
}

//Emergency Stop
void EmergencyStop()  {
  GetIMUData();
  
  StopAngle1 = round(yaw1);
  StopAngle2 = round(roll2);
  StopAngle3 = round(roll3);
  StopAngle4 = round(roll4);
  StopAngle5 = round(yaw5);
  
  Servo1.write(StopAngle1);
  Servo2.write(StopAngle2);
  Servo3.write(StopAngle3);
  Servo4.write(StopAngle4);
  Servo5.write(StopAngle5);
}

//Display Data
void DisplayData() {
  Serial.print("Pitch1: ");
  Serial.println(pitch1);
  
  Serial.print("Roll1: ");
  Serial.println(roll1);
  
  Serial.print("Yaw1: ");
  Serial.println(yaw1);
  
  Serial.println();
  
  Serial.print("Pitch2: ");
  Serial.println(pitch2);
  
  Serial.print("Roll2: ");
  Serial.println(roll2);
  
  Serial.print("Yaw2: ");
  Serial.println(yaw2);
  
  Serial.println();
  
  Serial.print("Pitch3: ");
  Serial.println(pitch3);
  
  Serial.print("Roll3: ");
  Serial.println(roll3);
  
  Serial.print("Yaw3: ");
  Serial.println(yaw3);
  
  Serial.println();
  
  Serial.print("Pitch4: ");
  Serial.println(pitch4);
  
  Serial.print("Roll4: ");
  Serial.println(roll4);
  
  Serial.print("Yaw4: ");
  Serial.println(yaw4);
  
  Serial.println();
  
  Serial.print("Pitch5: ");
  Serial.println(pitch5);
  
  Serial.print("Roll5: ");
  Serial.println(roll5);
  
  Serial.print("Yaw5: ");
  Serial.println(yaw5);
  
  Serial.println();

  Serial.println(distance_mm);
  Serial.println();

  Serial.println(vib_measurement);
  Serial.println();
}

//Handling Mode
  //Handling Mode#1
void CheckPB1Press() {
  if (PB1State != LastPB1State) {
    if (PB1State == LOW) {
      PBPress = true;
      //Attachment point
      theta1_i = 0;
      theta5_i = -23;
      Servo1.write(theta1_i);
      Servo5.write(theta5_i);
      GetIMUData();
      theta1_i_IMU = round(yaw1);
      theta5_i_IMU = round(yaw5);
      if (0.95*theta1_i < theta1_i_IMU < 1.05*theta1_i && 0.95*theta5_i < theta5_i_IMU < 1.05*theta5_i) {
        p_i = 3707;
        pz_i_1 = 1833;
        theta2_i_1 = 48;
        theta3_i_1 = -theta2_i_1;
        theta4_i_1 = 0;
        Servo2.write(theta2_i_1);
        Servo3.write(theta3_i_1);
        Servo4.write(theta4_i_1);
        GetIMUData();
        theta2_i_IMU = round(roll2);
        theta3_i_IMU = round(roll3);
        theta4_i_IMU = round(roll4);                                                                                
        if (0.95*theta2_i_1 < theta2_i_IMU < 1.05*theta2_i_1 && 0.95*theta3_i_1 < theta3_i_IMU < 1.05*theta3_i_1 && 0.95*theta4_i_1 < theta4_i_IMU < 1.05*theta4_i_1) {
          GetUSData();                                                                                              
          pz _i_2 = pz_i_1 - distance_mm;                                                                           
          theta3_i_2 = asin((-p_i^2-pz_i_2^2-d4^2+d3^2+d2^2-d1^2-2*pz_i_2*d4+2*pz_i_2*d1+2*d4*d1)/(2*d3*d2));
          theta2_i_2 = atan((p_i)/(pz_i_2+d4-d1))-atan((d3*cos(theta3_i_2))/(-d3*sin(theta3_i_2)+d2));
          theta4_i_2 = theta2_i_2 + theta3_i_2;
          Servo3.write(theta3_i_2);
          Servo2.write(theta2_i_2);
          Servo4.write(theta4_i_2);
          GetIMUData();
          theta3_i_IMU = round(roll3);
          theta2_i_IMU = round(roll2);
          theta4_i_IMU = round(roll4);
          if (0.95*theta3_i_2 < theta3_i_IMU < 1.05*theta3_i_2 && 0.95*theta2_i_2 < theta2_i_IMU < 1.05*theta2_i_2 && 0.95*theta4_i_2 < theta4_i_IMU < 1.05*theta4_i_2) {
            digitalWrite(EE, HIGH);
          }
        }
      }
      delay(500);
      Servo3.write(theta3_i_1);
      Servo4.write(theta4_i_1);
      
      //Detachment point
      theta1_f = 64;
      theta5_f = -50;
      Servo1.write(theta1_f);
      Servo5.write(theta5_f);
      GetIMUData();
      theta1_f_IMU = round(yaw1);
      theta5_f_IMU = round(yaw5);
      if (0.95*theta1_f < theta1_f_IMU < 1.05*theta1_f && 0.95*theta5_f < theta5_f_IMU < 1.05*theta5_f) {
        p_f = 4024;
        pz_f_1 = 1485;
        theta2_f_1 = 37;
        theta3_f_1 = -theta2_f_1;
        theta4_f_1 = 0;
        Servo2.write(theta2_f_1);
        Servo3.write(theta3_f_1);
        Servo4.write(theta4_f_1);
        GetIMUData();
        theta2_f_IMU = round(roll2);
        theta3_f_IMU = round(roll3);
        theta4_f_IMU = round(roll4);
        if (0.95*theta2_f_1 < theta2_f_IMU < 1.05*theta2_f_1 && 0.95*theta3_f_1 < theta3_f_IMU < 1.05*theta3_f_1 && 0.95*theta4_f_1 < theta4_f_IMU < 1.05*theta4_f_1) {
          GetUSData();
          pz_f_2 = pz_f_1 - distance_mm;
          theta3_f_2 = asin((-p_i^2-pz_f_2^2-d4^2+d3^2+d2^2-d1^2-2*pz_i_2*d4+2*pz_f_2*d1+2*d4*d1)/(2*d3*d2));
          theta2_f_2 = atan((p_i)/(pz_f_2+d4-d1))-atan((d3*cos(theta3_f_2))/(-d3*sin(theta3_f_2)+d2));
          theta4_f_2 = theta2_f_2 + theta3_f_2;
          Servo3.write(theta3_f_2);
          Servo2.write(theta2_f_2);
          Servo4.write(theta4_f_2);
          GetIMUData();
          theta3_f_IMU = round(roll3);
          theta2_f_IMU = round(roll2);
          theta4_f_IMU = round(roll4);
          if (0.95*theta3_f_2 < theta3_f_IMU < 1.05*theta3_f_2 && 0.95*theta2_f_2 < theta2_f_IMU < 1.05*theta2_f_2 && 0.95*theta4_f_2 < theta4_f_IMU < 1.05*theta4_f_2) {
            digitalWrite(EE, LOW);
          }
        }
      }
      delay(500);
      Servo3.write(theta3_f_1);
      Servo4.write(theta4_f_1);
    }
  LastPB1State = PB1State;
  }
}

  //Handling Mode#2
void CheckPB2Press() {
  if (PB2State != LastPB2State) {
    if (PB2State == LOW) {
      PBPress = true;
      //Attachment point
      theta1_i = 64;
      theta5_i = -50;
      Servo1.write(theta1_i);
      Servo5.write(theta5_i);
      GetIMUData();
      theta1_i_IMU = round(yaw1);
      theta5_i_IMU = round(yaw5);
      if (0.95*theta1_i < theta1_i_IMU < 1.05*theta1_i && 0.95*theta5_i < theta5_i_IMU < 1.05*theta5_i) {
        p_i = 4024;
        pz_i_1 = 1485;
        theta2_i_1 = 38;
        theta3_i_1 = -theta2_i_1;
        theta4_i_1 = 0;
        Servo2.write(theta2_i_1);
        Servo3.write(theta3_i_1);
        Servo4.write(theta4_i_1);
        GetIMUData();
        theta2_i_IMU = round(roll2);
        theta3_i_IMU = round(roll3);
        theta4_i_IMU = round(roll4);
        if (theta2_i_IMU == theta2_i_1 && theta3_i_IMU == theta3_i_1 && theta4_i_IMU == theta4_i_1) {
        GetUSData();
          pz_i_2 = pz_i_1 - distance_mm;
          theta3_i_2 = asin((-p_i^2-pz_i_2^2-d4^2+d3^2+d2^2-d1^2-2*pz_i_2*d4+2*pz_i_2*d1+2*d4*d1)/(2*d3*d2));
          theta2_i_2 = atan((p_i)/(pz_i_2+d4-d1))-atan((d3*cos(theta3_i_2))/(-d3*sin(theta3_i_2)+d2));
          theta4_i_2 = theta2_i_2 + theta3_i_2;
          Servo3.write(theta3_i_2);
          Servo2.write(theta2_i_2);
          Servo4.write(theta4_i_2);
          GetIMUData();
          theta3_i_IMU = round(roll3);
          theta2_i_IMU = round(roll2);
          theta4_i_IMU = round(roll4);
          if (0.95*theta3_i_2 < theta3_i_IMU < 1.05*theta3_i_2 && 0.95*theta2_i_2 < theta2_i_IMU < 1.05*theta2_i_2 && 0.95*theta4_i_2 < theta4_i_IMU < 1.05*theta4_i_2) {
              digitalWrite(EE, HIGH);
          }
        }
      }
      delay(500);
      Servo3.write(theta3_i_1);
      Servo4.write(theta4_i_1);
      
      //Detachment point
      theta1_f = 90;
      theta5_f = -23;
      Servo1.write(theta1_f);
      Servo5.write(theta5_f);
      GetIMUData();
      theta1_f_IMU = round(yaw1);
      theta5_f_IMU = round(yaw5);
      if (0.95*theta1_f < theta1_f_IMU < 1.05*theta1_f && 0.95*theta5_f < theta5_f_IMU < 1.05*theta5_f) {
        p_f = 2812;
        theta2_f_1 = 73;
        theta3_f_1 = -theta2_f_1;
        theta4_f_1 = 0;
        Servo2.write(theta2_f_1);
        Servo3.write(theta3_f_1);
        Servo4.write(theta4_f_1);
        GetIMUData();
        theta2_f_IMU = round(roll2);
        theta3_f_IMU = round(roll3);
        theta4_i_IMU = round(roll4);
        if (0.95*theta2_f_1 < theta2_f_IMU < 1.05*theta2_f_1 && 0.95*theta3_f_1 < theta3_f_IMU < 1.05*theta3_f_1 && 0.95*theta4_f_1 < theta4_f_IMU < 1.05*theta4_f_1) {
          GetUSData();
          pz_f_2 = pz_f_1 - distance_mm;
          theta3_f_2 = asin((-p_i^2-pz_f_2^2-d4^2+d3^2+d2^2-d1^2-2*pz_i_2*d4+2*pz_f_2*d1+2*d4*d1)/(2*d3*d2));
          theta2_f_2 = atan((p_i)/(pz_f_2+d4-d1))-atan((d3*cos(theta3_f_2))/(-d3*sin(theta3_f_2)+d2));
          theta4_f_2 = theta2_f_2 + theta3_f_2;
          Servo3.write(theta3_f_2);
          Servo2.write(theta2_f_2);
          Servo4.write(theta4_f_2);
          GetIMUData();
          theta3_f_IMU = round(roll3);
          theta2_f_IMU = round(roll2);
          theta4_f_IMU = round(roll4);
          if (theta3_f_IMU == theta3_f_2 && theta2_f_IMU == theta2_f_2 && theta4_f_IMU == theta4_f_2) {
            digitalWrite(EE, LOW);
          }
        }
      }
      delay(500);
      Servo3.write(theta3_f_1);
      Servo4.write(theta4_f_1);
    }
  LastPB2State = PB2State;
  }
}

//Handling Mode#3
void CheckPB3Press() {
  if (PB3State != LastPB3State) {
    if (PB3State == LOW) {
      PBPress = true;
      //Attachment point
      theta1_i = 90;
      theta5_i = -23;
      Servo1.write(theta1_i);
      Servo5.write(theta5_i);
      GetIMUData();
      theta1_i_IMU = round(yaw1);
      theta5_i_IMU = round(yaw5);
      if (0.95*theta1_i < theta1_i_IMU < 1.05*theta1_i && 0.95*theta5_i < theta5_i_IMU < 1.05*theta5_i) {
        p_i = 2812;
        pz_i_1 = 2342;
        theta2_i_1 = 73;
        theta3_i_1 = -theta2_i_1;
        theta4_i_1 = 0;
        Servo2.write(theta2_i_1);
        Servo3.write(theta3_i_1);
        Servo4.write(theta4_i_1);
        GetIMUData();
        theta2_i_IMU = round(roll2);
        theta3_i_IMU = round(roll3);
        theta4_i_IMU = round(roll4);
        if (0.95*theta2_i_1 < theta2_i_IMU < 1.05*theta2_i_1 && 0.95*theta3_i_1 < theta3_i_IMU < 1.05*theta3_i_1 && 0.95*theta4_i_1 < theta4_i_IMU < 1.05*theta4_i_1) {
          GetUSData();
          pz_i_2 = pz_i_1 - distance_mm;
          theta3_i_2 = asin((-p_i^2-pz_i_2^2-d4^2+d3^2+d2^2-d1^2-2*pz_i_2*d4+2*pz_i_2*d1+2*d4*d1)/(2*d3*d2));
          theta2_i_2 = atan((p_i)/(pz_i_2+d4-d1))-atan((d3*cos(theta3_i_2))/(-d3*sin(theta3_i_2)+d2));
          theta4_i_2 = theta2_i_2 + theta3_i_2;
          Servo3.write(theta3_i_2);
          Servo2.write(theta2_i_2);
          Servo4.write(theta4_i_2);
          GetIMUData();
          theta3_i_IMU = round(roll3);
          theta2_i_IMU = round(roll2);
          theta4_i_IMU = round(roll4);
          if (0.95*theta3_i_2 < theta3_i_IMU < 1.05*theta3_i_2 && 0.95*theta2_i_2 < theta2_i_IMU < 1.05*theta2_i_2 && 0.95*theta4_i_2 < theta4_i_IMU < 1.05*theta4_i_2) {
            digitalWrite(EE, HIGH);
          }
        }
      }
      delay(500);
      Servo3.write(theta3_i_1);
      Servo4.write(theta4_i_1);
      
      //Detachment point
      theta1_f = 131;
      theta5_f = 18;
      Servo1.write(theta1_f);
      Servo5.write(theta5_f);
      GetIMUData();
      theta1_f_IMU = round(yaw1);
      theta5_f_IMU = round(yaw5);
      if (0.95*theta1_f < theta1_f_IMU < 1.05*theta1_f && 0.95*theta5_f < theta5_f_IMU < 1.05*theta5_f) {
        p_f = 2695;
        pz_f_1 = 2375;
        theta2_f_1 = 76;
        theta3_f_1 = -theta2_f_1;
        theta4_f_1 = 0;
        Servo2.write(theta2_f_1);
        Servo3.write(theta3_f_1);
        Servo4.write(theta4_f_1);
        GetIMUData();
        theta2_f_IMU = round(roll2);
        theta3_f_IMU = round(roll3);
        theta4_f_IMU = round(roll4);
        if (0.95*theta2_f_1 < theta2_f_IMU < 1.05*theta2_f_1 && 0.95*theta3_f_1 < theta3_f_IMU < 1.05*theta3_f_1 && 0.95*theta4_f_1 < theta4_f_IMU < 1.05*theta4_f_1) {
          GetUSData();
          pz_f_2 = pz_f_1 - distance_mm;
          theta3_f_2 = asin((-p_i^2-pz_f_2^2-d4^2+d3^2+d2^2-d1^2-2*pz_i_2*d4+2*pz_f_2*d1+2*d4*d1)/(2*d3*d2));
          theta2_f_2 = atan((p_i)/(pz_f_2+d4-d1))-atan((d3*cos(theta3_f_2))/(-d3*sin(theta3_f_2)+d2));
          theta4_f_2 = theta2_f_2 + theta3_f_2;
          Servo3.write(theta3_f_2);
          Servo2.write(theta2_f_2);
          Servo4.write(theta4_f_2);
          GetIMUData();
          theta3_f_IMU = round(roll3);
          theta2_f_IMU = round(roll2);
          theta4_f_IMU = round(roll4);
          if (0.95*theta3_f_2 < theta3_f_IMU < 1.05*theta3_f_2 && 0.95*theta2_f_2 < theta2_f_IMU < 1.05*theta2_f_2 && 0.95*theta4_f_2 < theta4_f_IMU < 1.05*theta4_f_2) {
            digitalWrite(EE, LOW);
          }
        }
      }
      delay(500);
      Servo3.write(theta3_f_1);
      Servo4.write(theta4_f_1);
    }
  LastPB3State = PB3State;
  }
}

  //Handling Mode#4
void CheckPB1Press() {
  if (PB1State != LastPB1State) {
    if (PB1State == LOW) {
      PBPress = true;
      //Attachment point
      theta1_i = 131;
      theta5_i = 18;
      Servo1.write(theta1_i);
      Servo5.write(theta5_i);
      GetIMUData();
      theta1_i_IMU = round(yaw1);
      theta5_i_IMU = round(yaw5);
      if (0.95*theta1_i < theta1_i_IMU < 1.05*theta1_i && 0.95*theta5_i < theta5_i_IMU < 1.05*theta5_i) {
        p_i = 2695;
        pz_i_1 = 2375;
        theta2_i_1 = 76;
        theta3_i_1 = -theta2_i_1;
        theta4_i_1 = 0;
        Servo2.write(theta2_i_1);
        Servo3.write(theta3_i_1);
        Servo4.write(theta4_i_1);
        GetIMUData();
        theta2_i_IMU = round(roll2);
        theta3_i_IMU = round(roll3);
        theta4_i_IMU = round(roll4);
        if (0.95*theta2_i_1 < theta2_i_IMU < 1.05*theta2_i_1 && 0.95*theta3_i_1 < theta3_i_IMU < 1.05*theta3_i_1 && 0.95*theta4_i_1 < theta4_i_IMU < 1.05*theta4_i_1) {
          GetUSData();
          pz_i_2 = pz_i_1 - distance_mm;
          theta3_i_2 = asin((-p_i^2-pz_i_2^2-d4^2+d3^2+d2^2-d1^2-2*pz_i_2*d4+2*pz_i_2*d1+2*d4*d1)/(2*d3*d2));
          theta2_i_2 = atan((p_i)/(pz_i_2+d4-d1))-atan((d3*cos(theta3_i_2))/(-d3*sin(theta3_i_2)+d2));
          theta4_i_2 = theta2_i_2 + theta3_i_2;
          Servo3.write(theta3_i_2);
          Servo2.write(theta2_i_2);
          Servo4.write(theta4_i_2);
          GetIMUData();
          theta3_i_IMU = round(roll3);
          theta2_i_IMU = round(roll2);
          theta4_i_IMU = round(roll4);
          if (0.95*theta3_i_2 < theta3_i_IMU < 1.05*theta3_i_2 && 0.95*theta2_i_2 < theta2_i_IMU < 1.05*theta2_i_2 && 0.95*theta4_i_2 < theta4_i_IMU < 1.05*theta4_i_2) {
            digitalWrite(EE, HIGH);
          }
        }
      }
      delay(500);
      Servo3.write(theta3_i_1);
      Servo4.write(theta4_i_1);
      
      //Detachment point
      theta1_f = 158;
      theta5_f = 45;
      Servo1.write(theta1_f);
      Servo5.write(theta5_f);
      GetIMUData();
      theta1_f_IMU = round(yaw1);
      theta5_f_IMU = round(yaw5);
      if (0.95*theta1_f < theta1_f_IMU < 1.05*theta1_f && 0.95*theta5_f < theta5_f_IMU < 1.05*theta5_f) {
        p_f = 3639;
        pz_f_1 = 1892;
        theta2_f_1 = 50;
        theta3_f_1 = -theta2_f_1;
        theta4_f_1 = 0;
        Servo2.write(theta2_f_1);
        Servo3.write(theta3_f_1);
        Servo4.write(theta4_f_1);
        GetIMUData();
        theta2_f_IMU = round(roll2);
        theta3_f_IMU = round(roll3);
        theta4_f_IMU = round(roll4);
        if (0.95*theta2_f_1 < theta2_f_IMU < 1.05*theta2_f_1 && 0.95*theta3_f_1 < theta3_f_IMU < 1.05*theta3_f_1 && 0.95*theta4_f_1 < theta4_f_IMU < 1.05*theta4_f_1) {
          GetUSData();
          pz_f_2 = pz_f_1 - distance_mm;
          theta3_f_2 = asin((-p_i^2-pz_f_2^2-d4^2+d3^2+d2^2-d1^2-2*pz_i_2*d4+2*pz_f_2*d1+2*d4*d1)/(2*d3*d2));
          theta2_f_2 = atan((p_i)/(pz_f_2+d4-d1))-atan((d3*cos(theta3_f_2))/(-d3*sin(theta3_f_2)+d2));
          theta4_f_2 = theta2_f_2 + theta3_f_2;
          Servo3.write(theta3_f_2);
          Servo2.write(theta2_f_2);
          Servo4.write(theta4_f_2);
          GetIMUData();
          theta3_f_IMU = round(roll3);
          theta2_f_IMU = round(roll2);
          theta4_f_IMU = round(roll4);
          if (0.95*theta3_f_2 < theta3_f_IMU < 1.05*theta3_f_2 && 0.95*theta2_f_2 < theta2_f_IMU < 1.05*theta2_f_2 && 0.95*theta4_f_2 < theta4_f_IMU < 1.05*theta4_f_2) {
            digitalWrite(EE, LOW);
          }
        }
      }
      delay(500);
      Servo3.write(theta3_f_1);
      Servo4.write(theta4_f_1);
    }
  LastPB4State = PB4State;
  }
}

//Turn the robot off
void CheckPB5Press() {
  if (PB5State != LastPB5State) {
    if (PB5State == LOW) {
      PBPress = true;
      Servo1.write(0);
      Servo2.write(0);
      Servo3.write(0);
      Servo4.write(0);
      Servo5.write(0);
      digitalWrite(EE, LOW);
    }
    LastPB5State = PB5State;
  }
}
