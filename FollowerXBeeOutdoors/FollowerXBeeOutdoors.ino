#include <SabertoothSimplified.h>
#include "Wire.h"
#include <SoftwareSerial.h>
#include "sensorbar.h"
#include <I2C.h>

enum STATES {IDLE, MOVING_RIGHT, MOVING_LEFT, VERIFICATION_RIGHT, VERIFICATION_LEFT, RETURN, VERIFICATION_RETURN};
STATES current_state = IDLE;

SoftwareSerial Motor_Serial12(NOT_A_PIN, 3);
SoftwareSerial Motor_Serial34(NOT_A_PIN, 5);
SabertoothSimplified motor_12(Motor_Serial12);
SabertoothSimplified motor_34(Motor_Serial34);

const uint8_t SX1509_ADDRESS = 0x3E;
SensorBar mySensorBar(SX1509_ADDRESS);

int photo_switches[6] = {};
int photo_switch = 13;
const int TRIG_PIN = A0;
const int ECHO_PIN = A1;

double distance_left_max;
double distance_left_min;
double distance_right_max;
double distance_right_min;
double distance_centre_max;
double distance_centre_min;

#define LIDARLite_ADDRESS 0x62
#define RegisterMeasure   0x00
#define MeasureValue      0x04
#define RegisterHighLowB  0x8f

void setup() {
  Serial.begin(1200);
  Motor_Serial12.begin(9600);
  Motor_Serial34.begin(9600);

  I2c.begin();
  delay(100);
  I2c.timeOut(50);

  for (int i = 0; i < 6; i++) {
    photo_switches[i] = i + 6;
    pinMode(photo_switches[i], INPUT);
  }
  pinMode(photo_switch, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  UltraSonic_Distance();
  LIDAR_Distance();

  mySensorBar.setBarStrobe();
  mySensorBar.setInvertBits();
  uint8_t returnStatus = mySensorBar.begin();

  Assign_Distance();

  Serial.println("Follower Block Ready");
}

void loop() {
  switch (current_state) {
    case IDLE: {
        if (Serial.available()) {
          int Command = Serial.read();

          if (Command == 0xF5) {
            current_state = MOVING_RIGHT;
            Serial.println("Moving Right");
          }

          else if (Command == 0xF6) {
            current_state = MOVING_LEFT;
            Serial.println("Moving Left");
          }

          else if (Command == 0xF7) {
            current_state = RETURN;
            Serial.println("Returning");
          }
          Serial_Flush();
        }
        break;
      }

    case MOVING_RIGHT: {
        Serial.println("Now Right");

        int Distance = LIDAR_Distance();
        bool Moving = false;

        while (Distance > distance_right_min) {
          Motor_Right();

          int Position = mySensorBar.getPosition();
          if ((Position == 0) && (Distance > distance_right_min && Distance < distance_right_max)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print(".....");
            Serial.println(Position);

            Moving = true;
            break;
          }
          Distance = LIDAR_Distance();
        }

        if (!Moving)
          while (Distance < distance_right_max) {
            Motor_Left();

            int Position = mySensorBar.getPosition();
            if ((Position == 0) && (Distance > distance_right_min && Distance < distance_right_max)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print(".....");
              Serial.println(Position);

              break;
            }
            Distance = LIDAR_Distance();
          }
        Motor_Stop();
        current_state = VERIFICATION_RIGHT;
        break;
      }

    case VERIFICATION_RIGHT: {
        Serial.println("Verifying Right");

        int Distance = LIDAR_Distance();
        bool Verified = false;

        while (Distance > distance_right_min) {
          Motor_Right();

          Distance = LIDAR_Distance();

          int Position = mySensorBar.getPosition();
          if ((Position == 0) && (Distance < distance_right_max && Distance > distance_right_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print(".....");
            Serial.println(Position);

            Verified = true;
            break;
          }
        }

        if (!Verified)
          while (Distance < distance_right_max) {
            Motor_Left();

            Distance = LIDAR_Distance();

            int Position = mySensorBar.getPosition();
            if ((Position == 0) && (Distance < distance_right_max && Distance > distance_right_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print(".....");
              Serial.println(Position);

              break;
            }
          }
        Motor_Stop();

        AlignSideways();
        AlignLengthways();

        for (int i = 0; i <= 1000 ; i++)
          Serial.write(0xE5);

        current_state = IDLE;
        Serial_Flush();
        break;
      }

    case MOVING_LEFT: {
        Serial.println("Now Left");

        int Distance = LIDAR_Distance();
        bool Moving = false;

        while (Distance < distance_left_max) {
          Motor_Left();

          int Position = mySensorBar.getPosition();
          if ((Position == 0) && (Distance < distance_left_max && Distance > distance_left_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print(".....");
            Serial.println(Position);

            Moving = true;
            break;
          }
          Distance = LIDAR_Distance();
        }

        if (!Moving)
          while (Distance > distance_left_min) {
            Motor_Right();

            int Position = mySensorBar.getPosition();
            if ((Position == 0) && (Distance < distance_left_max && Distance > distance_left_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print(".....");
              Serial.println(Position);

              break;
            }
            Distance = LIDAR_Distance();
          }
        Motor_Stop();
        current_state = VERIFICATION_LEFT;
        break;
      }

    case VERIFICATION_LEFT: {
        Serial.println("Verifying Left");

        int Distance = LIDAR_Distance();
        bool Verified = false;

        while (Distance < distance_left_max) {
          Motor_Left();

          Distance = LIDAR_Distance();

          int Position = mySensorBar.getPosition();
          if ((Position == 0) && (Distance < distance_left_max && Distance > distance_left_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print(".....");
            Serial.println(Position);

            Verified = true;
            break;
          }
        }

        if (!Verified)
          while (Distance > distance_left_min) {
            Motor_Right();

            Distance = LIDAR_Distance();

            int Position = mySensorBar.getPosition();
            if ((Position == 0) && (Distance < distance_left_max && Distance > distance_left_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print(".....");
              Serial.println(Position);

              break;
            }
          }
        Motor_Stop();

        AlignSideways();
        AlignLengthways();

        for (int i = 0; i <= 1000 ; i++)
          Serial.write(0xE6);

        current_state = IDLE;
        Serial_Flush();
        break;
      }

    case RETURN: {
        Serial.println("Now Return");

        int Distance = LIDAR_Distance();
        bool Moving = false;

        while (Distance < distance_centre_max) {
          Motor_Left();

          int Position = mySensorBar.getPosition();
          if ((Position == 0) & (Distance < distance_centre_max && Distance > distance_centre_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print(".....");
            Serial.println(Position);

            Moving = true;
            break;
          }
          Distance = LIDAR_Distance();
        }

        if (!Moving)
          while (Distance > distance_centre_min) {
            Motor_Right();

            int Position = mySensorBar.getPosition();
            if ((Position == 0) && (Distance < distance_centre_max && Distance > distance_centre_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print(".....");
              Serial.println(Position);

              break;
            }
            Distance = LIDAR_Distance();
          }
        Motor_Stop();
        current_state = VERIFICATION_RETURN;
        break;
      }

    case VERIFICATION_RETURN: {
        Serial.println("Verifying Return");

        int Distance = LIDAR_Distance();
        bool Verified = false;

        while (Distance > distance_centre_max) {
          Motor_Right();

          Distance = LIDAR_Distance();

          int Position = mySensorBar.getPosition();
          if ((Position == 0) && (Distance < distance_centre_max && Distance > distance_centre_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print(".....");
            Serial.println(Position);

            Verified = true;
            break;
          }
        }

        if (!Verified)
          while (Distance < distance_centre_min) {
            Motor_Left();

            Distance = LIDAR_Distance();

            int Position = mySensorBar.getPosition();
            if ((Position == 0) && (Distance < distance_centre_max && Distance > distance_centre_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print(".....");
              Serial.println(Position);

              break;
            }
          }
        Motor_Stop();

        AlignSideways();
        AlignLengthways();

        for (int i = 0; i <= 1000 ; i++)
          Serial.write(0xE7);

        current_state = IDLE;
        Serial_Flush();
        break;
      }
  }
}











//Functions.........................................











int LIDAR_Distance() {
  uint8_t nackack = 100;
  while (nackack != 0) {
    nackack = I2c.write(LIDARLite_ADDRESS, RegisterMeasure, MeasureValue);
    delay(1);
  }
  byte distanceArray[2];

  nackack = 100;
  while (nackack != 0) {
    nackack = I2c.read(LIDARLite_ADDRESS, RegisterHighLowB, 2, distanceArray);
    delay(1);
  }
  int distance = (distanceArray[0] << 8) + distanceArray[1];
  return distance;
}

double UltraSonic_Distance() {
  double duration, distanceCm;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);

  distanceCm = duration / 29.1 / 2 ;

  return distanceCm;
}

void Motor_Right() {
  motor_12.motor(1, -32);
  motor_12.motor(2, 32);
  motor_34.motor(1, -32);
  motor_34.motor(2, 32);
}

void Motor_Right_Slow() {
  motor_12.motor(1, -10);
  motor_12.motor(2, 10);
  motor_34.motor(1, -10);
  motor_34.motor(2, 10);
}

void Motor_Left() {
  motor_12.motor(1, 32);
  motor_12.motor(2, -32);
  motor_34.motor(1, 32);
  motor_34.motor(2, -32);
}

void Motor_Left_Slow() {
  motor_12.motor(1, 10);
  motor_12.motor(2, -10);
  motor_34.motor(1, 10);
  motor_34.motor(2, -10);
}

void Motor_Forward() {
  motor_12.motor(1, 10);
  motor_12.motor(2, 10);
  motor_34.motor(1, 10);
  motor_34.motor(2, 10);
}

void Motor_Backward() {
  motor_12.motor(1, -10);
  motor_12.motor(2, -10);
  motor_34.motor(1, -10);
  motor_34.motor(2, -10);
}

void Motor_Stop() {
  motor_12.motor(1, 0);
  motor_12.motor(2, 0);
  motor_34.motor(1, 0);
  motor_34.motor(2, 0);
}

void Serial_Flush() {
  while (Serial.available() > 0) {
    char X = Serial.read();
  }
}

void AlignSideways() {
  Serial.println("Checking alignment....");
  digitalWrite(photo_switch, HIGH);
  delay(1000);

  bool  A = digitalRead(photo_switches[0]);
  bool  B = digitalRead(photo_switches[1]);
  bool  C = digitalRead(photo_switches[2]);
  bool  D = digitalRead(photo_switches[3]);
  bool  E = digitalRead(photo_switches[4]);
  bool  F = digitalRead(photo_switches[5]);

  bool Align = false;

  while ((!A && F) || (!F && !A && !B)) {
    Motor_Right_Slow();

    A = digitalRead(photo_switches[0]);
    B = digitalRead(photo_switches[1]);
    C = digitalRead(photo_switches[2]);
    D = digitalRead(photo_switches[3]);
    E = digitalRead(photo_switches[4]);
    F = digitalRead(photo_switches[5]);

    Align = true;
  }

  if (!Align)
    while ((A && !F) || (!A && !E && !F)) {
      Motor_Left_Slow();

      A = digitalRead(photo_switches[0]);
      B = digitalRead(photo_switches[1]);
      C = digitalRead(photo_switches[2]);
      D = digitalRead(photo_switches[3]);
      E = digitalRead(photo_switches[4]);
      F = digitalRead(photo_switches[5]);
    }
  Motor_Stop();
  digitalWrite(photo_switch, LOW);
}

void AlignLengthways() {
  double Distance = UltraSonic_Distance();
  bool Aligning = false;

  while (Distance > 14.0) {
    Motor_Forward();

    if (Distance > 13.0 && Distance < 14.0) {
      Motor_Stop();

      Aligning = true;
      break;
    }
    Distance = UltraSonic_Distance();
  }
  Motor_Stop();

  if (!Aligning)
    while (Distance < 13.0) {
      Motor_Backward();

      if (Distance > 13.0 && Distance < 14.0) {
        Motor_Stop();

        break;
      }
      Distance = UltraSonic_Distance();
    }
  Motor_Stop();
}

void Assign_Distance() {
  distance_left_max = (((LIDAR_Distance() + 20) * 1.5) - 20) + 3;
  distance_left_min = (((LIDAR_Distance() + 20) * 1.5) - 20) - 3;
  distance_right_max = (((LIDAR_Distance() + 20) * 0.5) - 20) + 3;
  distance_right_min = (((LIDAR_Distance() + 20) * 0.5) - 20) - 3;
  distance_centre_max = LIDAR_Distance() + 3;
  distance_centre_min = LIDAR_Distance() - 3;
}
