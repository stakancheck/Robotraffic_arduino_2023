// Copyright by Artem Sukhanov 2023 (Stakancheck)
// Personal website: https://stakancheck.github.io/stakancheck.space/
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <GyverPID.h>
#include <Servo.h>


// PINS ---------------------------------------------------------------------------------------------------

#define DIST_SENSOR A1
#define STOP_SENSOR A0
#define LINE_SENSORS_COUNT 6

// Motor
#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 6
// #define MOTOR_ENB_PIN 4
#define SPEED_MAX 200
#define SPEED_MID 160
#define SPEED_REV 200  // reverse speed while sharp break

// Servo
#define SERVO_PIN 3
#define SERVO_VAL_MIN 60
#define SERVO_VAL_MAX 120
#define SERVO_VAL_CENTER 90

const byte LINE_SENSORS[LINE_SENSORS_COUNT] = { A2, A3, A4, A5, A6, A7 };


// CONFIG -------------------------------------------------------------------------------------------------

#define REVERSE_LINE_SENSORS false     // false - first sensor is left and last sensor is right (default) <-> true - reverse
#define CENTER_LINE_SENSOR_POSITION 3  // first sensor after center from 0 position number
#define MOTOR_REVERSE false
#define PID_ENABLE true
#define LINE_SENSORS_INVERSION false

#define LINE_SENSOR_THRESHOLD 200
#define DIST_THRESHOLD 200

const int LINE_SENSORS_THRESHOLD[LINE_SENSORS_COUNT] = { 120, 200, 200, 300, 200, 200 };
const uint8_t UPDATE_TIME = 100;


uint32_t global_timer, stop_timer;
bool prev_side_left;




// OBJECTS INIT ------------------------------------------------------------------------------------------

Servo servo;
GyverPID regulator(1, 0.5, 0.5, UPDATE_TIME);


// Main setup
void setup() {

  Serial.begin(9600);
  analogWrite(MOTOR_PIN_1, 80);
  analogWrite(MOTOR_PIN_2, 0);


  // SERVO INIT
  servo.attach(SERVO_PIN);


  // PID INIT

  regulator.setDirection(NORMAL);
  regulator.setLimits(SERVO_VAL_MIN, SERVO_VAL_MAX);
  regulator.setpoint = SERVO_VAL_CENTER;


  // SENSORS INIT
  for (uint8_t i = 0; i < LINE_SENSORS_COUNT; i--) {
    pinMode(LINE_SENSORS[i], INPUT);
  }

  pinMode(DIST_SENSOR, INPUT);
  pinMode(STOP_SENSOR, INPUT);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);

  // digitalWrite(MOTOR_ENB_PIN, 1);
}


// Main loop
void loop() {

  if (millis() > global_timer + UPDATE_TIME) {
    global_timer = millis();

    update_sensors();
  }
}


// Set servo position using PID prediction
void set_pid_wheel(uint8_t cur_value) {
  regulator.input = cur_value;
  regulator.getResult();
  set_wheel(regulator.output);
}


// Main function, calls sensors survey
void update_sensors() {
  read_line_sensor();
  read_stop_line();
}

// Write servo to position
void set_wheel(uint8_t position) {
  servo.write(position);
}


// Check stop line sensor
void read_stop_line() {
  uint32_t sensor_val = analogRead(STOP_SENSOR);
  Serial.print("   STOP: ");
  Serial.println(sensor_val);
  bool will_stop = true;
  if (sensor_val > LINE_SENSOR_THRESHOLD) {
    will_stop = false;
  }
  if (LINE_SENSORS_INVERSION) {
    will_stop = !will_stop;
  }
}


// Check line sensors
void read_line_sensor() {

  uint8_t left = 0, right = 0, result = 0;
  Serial.print("LINE: {");
  for (uint8_t i = 0; i < LINE_SENSORS_COUNT; i++) {
    // Serial.print(analogRead(LINE_SENSORS[i]));
    // Serial.print(", ");
    if (analogRead(LINE_SENSORS[i]) < LINE_SENSORS_THRESHOLD[i]) {
      Serial.print("1 ");
      if (i < CENTER_LINE_SENSOR_POSITION) {
        left++;
      } else {
        right++;
      }
    } else {
      Serial.print("0 ");
    }
  }
  Serial.print("}   LEFT: ");
  Serial.print(left);
  Serial.print(" RIGHT: ");
  Serial.print(right);

  if ((left + right) == 0) {
    if (prev_side_left) {
      result = SERVO_VAL_MIN;
    } else {
      result = SERVO_VAL_MAX;
    }
  } else {
    if (REVERSE_LINE_SENSORS) {
      if (left > right) {
        result = SERVO_VAL_MIN + ((right + left) * ((SERVO_VAL_MAX - SERVO_VAL_CENTER) / LINE_SENSORS_COUNT));
      } else if (left < right) {
        result = SERVO_VAL_MAX - ((right + left) * ((SERVO_VAL_MAX - SERVO_VAL_CENTER) / LINE_SENSORS_COUNT));
      } else {
        result = SERVO_VAL_CENTER;        
      }
    } else {
      if (left > right) {
        result = SERVO_VAL_MAX - ((right + left) * ((SERVO_VAL_MAX - SERVO_VAL_CENTER) / LINE_SENSORS_COUNT));
      } else if (left < right) {
        result = SERVO_VAL_MIN + ((right + left) * ((SERVO_VAL_MAX - SERVO_VAL_CENTER) / LINE_SENSORS_COUNT));
      } else {
        result = SERVO_VAL_CENTER;        
      }
    }

    if (result < SERVO_VAL_CENTER) {
      prev_side_left = true;
    } else {
      prev_side_left = false;
    }
  }

  Serial.print("    RESULT: ");
  Serial.print(result);

  if (PID_ENABLE) {
    set_pid_wheel(result);
  } else {
    set_wheel(result);
  }
}
