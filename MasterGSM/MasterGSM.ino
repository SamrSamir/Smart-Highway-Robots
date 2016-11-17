#include <SabertoothSimplified.h>
#include "Wire.h"
#include <I2C.h>
#include "sensorbar.h"
#include <GSM.h>

enum STATES {IDLING, MOVING_RIGHT, MOVING_LEFT, VERIFICATION_RIGHT, VERIFICATION_LEFT, COMMAND_RIGHT, COMMAND_LEFT, RETURN, VERIFICATION_RETURN, COMMAND_RETURN};
STATES current_state = IDLING;

SabertoothSimplified motor_12(Serial2);
SabertoothSimplified motor_34(Serial3);

const uint8_t SX1509_ADDRESS = 0x3E;
SensorBar mySensorBar(SX1509_ADDRESS);

int photo_switch = 4;

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

GSM gsmAccess;
GSM_SMS sms;

void setup() {
  Serial.begin(1200);
  Serial2.begin(9600);
  Serial3.begin(9600);

  Serial.println("Smart Traffic Barrier");
  delay(1000);
  Serial.println("Calibrating sensors");
  delay(2000);

  I2c.begin();
  delay(100);
  I2c.timeOut(50);

  pinMode(photo_switch, OUTPUT);

  LIDAR_Distance();

  mySensorBar.setBarStrobe();
  mySensorBar.clearInvertBits();
  uint8_t returnStatus = mySensorBar.begin();

  Assign_Distance();

  Serial.println("Initializing communication");

  boolean notConnected = true;

  while (notConnected) {
    if (gsmAccess.begin() == GSM_READY) {
      notConnected = false;
    }
    else {
      Serial.println("Connection couldn't be established");
      Serial.println("Contact our support center");
      delay(1000);
    }
  }

  Serial.println("Communications array initialized");
  delay(1000);
  Serial.println("Calibration completed");
  delay(1000);
  Serial.println("Master Block Ready");
}

void loop() {
  switch (current_state) {
    case IDLING: {
        digitalWrite(photo_switch, LOW);

        if (sms.available()) {
          char Order = sms.read();

          if (Order == '4') {
            current_state = MOVING_LEFT;
            Serial.println("Moving Left");
          }

          else if (Order == '6') {
            current_state = MOVING_RIGHT;
            Serial.println("Moving Right");
          }

          else if (Order == '5') {
            current_state = RETURN;
            Serial.println("Returning");
          }
          sms.flush();
        }
        break;
      }

    case MOVING_RIGHT: {
        Serial.println("Now Right");

        int Distance = LIDAR_Distance();
        bool Moving = false;

        while (Distance > distance_right_min) {
          Motor_Right();

          int density = mySensorBar.getDensity();
          if ((density == 8) && (Distance > distance_right_min && Distance < distance_right_max)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print("................................");
            Serial.println(density);

            Moving = true;
            break;
          }
          Distance = LIDAR_Distance();
        }

        if (!Moving)
          while (Distance < distance_right_max) {
            Motor_Left();

            int density = mySensorBar.getDensity();
            if ((density == 8) && (Distance > distance_right_min && Distance < distance_right_max)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print("................................");
              Serial.println(density);

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

          int density = mySensorBar.getDensity();
          if ((density == 8) && (Distance < distance_right_max && Distance > distance_right_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print("................................");
            Serial.println(density);

            Verified = true;
            break;
          }
        }

        if (!Verified)
          while (Distance < distance_right_max) {
            Motor_Left();

            Distance = LIDAR_Distance();

            int density = mySensorBar.getDensity();
            if ((density == 8) && (Distance < distance_right_max && Distance > distance_right_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print("................................");
              Serial.println(density);

              break;
            }
          }
        Motor_Stop();
        digitalWrite(photo_switch, HIGH);
        current_state = COMMAND_RIGHT;
        break;
      }

    case COMMAND_RIGHT: {
        int confirm = 0;
        for (int i = 0; i <= 1000; i++)
          Serial.write(0xF5);

        while (confirm != 0xE5) {
          confirm =  Serial.read();
        }

        Lane_Change_Completed();
        current_state = IDLING;
        break;
      }

    case MOVING_LEFT: {
        Serial.println("Now Left");

        int Distance = LIDAR_Distance();
        bool Moving = false;

        while (Distance < distance_left_max) {
          Motor_Left();

          int density = mySensorBar.getDensity();
          if ((density == 8) && (Distance < distance_left_max && Distance > distance_left_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print("................................");
            Serial.println(density);

            Moving = true;
            break;
          }
          Distance = LIDAR_Distance();
        }

        if (!Moving)
          while (Distance > distance_left_min) {
            Motor_Right();

            int density = mySensorBar.getDensity();
            if ((density == 8) && (Distance < distance_left_max && Distance > distance_left_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print("................................");
              Serial.println(density);

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

          int density = mySensorBar.getDensity();
          if ((density == 8) && (Distance < distance_left_max && Distance > distance_left_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print("................................");
            Serial.println(density);

            Verified = true;
            break;
          }
        }

        if (!Verified)
          while (Distance > distance_left_min) {
            Motor_Right();

            Distance = LIDAR_Distance();

            int density = mySensorBar.getDensity();
            if ((density == 8) && (Distance < distance_left_max && Distance > distance_left_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print("................................");
              Serial.println(density);

              break;
            }
          }
        Motor_Stop();
        digitalWrite(photo_switch, HIGH);
        current_state = COMMAND_LEFT;
        break;
      }

    case COMMAND_LEFT: {
        int confirm = 0;
        for (int i = 0; i <= 1000; i++)
          Serial.write(0xF6);

        while (confirm != 0xE6) {
          confirm =  Serial.read();
        }

        Lane_Change_Completed();
        current_state = IDLING;
        break;
      }

    case RETURN: {
        Serial.println("Now Return");

        int Distance = LIDAR_Distance();
        bool Moving = false;

        while (Distance < distance_centre_max) {
          Motor_Left();

          int density = mySensorBar.getDensity();
          if ((density == 8) & (Distance < distance_centre_max && Distance > distance_centre_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print("................................");
            Serial.println(density);

            Moving = true;
            break;
          }
          Distance = LIDAR_Distance();
        }

        if (!Moving)
          while (Distance > distance_centre_min) {
            Motor_Right();

            int density = mySensorBar.getDensity();
            if ((density == 8) && (Distance < distance_centre_max && Distance > distance_centre_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print("................................");
              Serial.println(density);

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

          int density = mySensorBar.getDensity();
          if ((density == 8) && (Distance < distance_centre_max && Distance > distance_centre_min)) {
            Motor_Stop();

            Serial.print("Stopping at.....");
            Serial.print(Distance);
            Serial.print("................................");
            Serial.println(density);

            Verified = true;
            break;
          }
        }

        if (!Verified)
          while (Distance < distance_centre_min) {
            Motor_Left();

            Distance = LIDAR_Distance();

            int density = mySensorBar.getDensity();
            if ((density == 8) && (Distance < distance_centre_max && Distance > distance_centre_min)) {
              Motor_Stop();

              Serial.print("Stopping at.....");
              Serial.print(Distance);
              Serial.print("................................");
              Serial.println(density);

              break;
            }
          }
        Motor_Stop();
        digitalWrite(photo_switch, HIGH);
        current_state = COMMAND_RETURN;
        break;
      }

    case COMMAND_RETURN: {
        int confirm = 0;
        for (int i = 0; i <= 1000; i++)
          Serial.write(0xF7);

        while (confirm != 0xE7) {
          confirm =  Serial.read();
        }

        Lane_Change_Completed();
        current_state = IDLING;
        break;
      }
  }
}









//Functions...........................................................................









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

void Motor_Right() {
  motor_12.motor(1, 32);
  motor_12.motor(2, 32);
  motor_34.motor(1, 32);
  motor_34.motor(2, 32);
}

void Motor_Left() {
  motor_12.motor(1, -32);
  motor_12.motor(2, -32);
  motor_34.motor(1, -32);
  motor_34.motor(2, -32);
}

void Motor_Stop() {
  motor_12.motor(1, 0);
  motor_12.motor(2, 0);
  motor_34.motor(1, 0);
  motor_34.motor(2, 0);
}

void Assign_Distance() {
  distance_left_max = (((LIDAR_Distance() + 20) * 1.5) - 20) + 3;
  distance_left_min = (((LIDAR_Distance() + 20) * 1.5) - 20) - 3;
  distance_right_max = (((LIDAR_Distance() + 20) * 0.5) - 20) + 3;
  distance_right_min = (((LIDAR_Distance() + 20) * 0.5) - 20) - 3;
  distance_centre_max = LIDAR_Distance() + 3;
  distance_centre_min = LIDAR_Distance() - 3;
}

void Lane_Change_Completed() {
  char remoteNum[] = "00971505232586";
  char txtMsg[] = "Lane Change Completed";
  sms.beginSMS(remoteNum);
  sms.print(txtMsg);
  sms.endSMS();
}
