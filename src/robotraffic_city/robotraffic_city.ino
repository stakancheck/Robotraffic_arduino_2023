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

#include <Arduino.h>
#include <GyverPID.h>
#include <Servo.h>


// PINS ---------------------------------------------------------------------------------------------------

#define DIST_SENSOR A1        // Distance sensor
#define STOP_SENSOR A0        // Stop Line sensor
#define LINE_SENSORS_COUNT 6  // Number of line sensors

// Motor
#define MOTOR_PIN_F 5  // Motor control forward
#define MOTOR_PIN_B 6  // Motor control back
#define SPEED_MAX 160  // Maximum permissible speed
#define SPEED_MIN 100  // Minimum permissible speed
#define SPEED_REV 200  // Reverse speed during a sudden stop

// Servo
#define SERVO_PIN 3          // Servo pin
#define SERVO_VAL_MIN 60     // Minimum angle of rotation of the servo (right)
#define SERVO_VAL_MAX 120    // Maximum angle of rotation of the servo (left)
#define SERVO_VAL_CENTER 90  // Central angle of rotation of the servo (center)

const byte LINE_SENSORS[LINE_SENSORS_COUNT] = { A2, A3, A4, A5, A6, A7 };  // Pins of line sensors
const uint8_t SERVO_RATIO = (SERVO_VAL_MAX - SERVO_VAL_CENTER) / LINE_SENSORS_COUNT;


// CONFIG -------------------------------------------------------------------------------------------------

#define INE_SENSORS_INVERSION false   // Change line sensors operation mode
#define MOTOR_REVERSE false           // Change the direction of the motor
#define PID_ENABLE false              // Enable the operation of the PID controller
#define LINE_SENSORS_INVERSION false  // Invert the operation of the line sensors

#define SHARP_BRAKE true       // Sharp brake with a short reverse movement
#define SHARP_BRAKE_DELAY 300  // Duration of reverse movement

#define START_DELAY 300  // Delay before program operation
#define STOP_DELAY 5000  // Duration of a stop at a traffic light

#define LINE_SENSOR_THRESHOLD 800  // Threshold of operation of the stop sensor
#define DIST_THRESHOLD 30          // Threshold values of the distance sensor (cm)

const int LINE_SENSORS_THRESHOLD[LINE_SENSORS_COUNT] = { 800, 800, 800, 800, 800, 800 };
const uint8_t UPDATE_TIME = 100;


// VARIABLES ---------------------------------------------------------------------------------------------

#define DEBUG true

uint32_t global_timer, stop_timer;
uint8_t current_speed, prev_speed;
bool prev_side_left, ignore_stop, stop_status, barrier, start_state = true;


// OBJECTS INIT ------------------------------------------------------------------------------------------

Servo servo;                                     // Init Servo as servo
GyverPID regulator(0.8, 0.5, 0.5, UPDATE_TIME);  // Init PID as regulator (KP, KI, KD, UPDATE_TIME)


// MAIN SETUP --------------------------------------------------------------------------------------------

void setup() {

  if (DEBUG) {
    Serial.begin(9600);
  }

  // SERVO CONF
  servo.attach(SERVO_PIN);


  // PID CONF
  regulator.setDirection(NORMAL);
  regulator.setLimits(SERVO_VAL_MIN, SERVO_VAL_MAX);
  regulator.setpoint = SERVO_VAL_CENTER;


  // SENSORS INIT
  for (uint8_t i = 0; i < LINE_SENSORS_COUNT; i--) {
    pinMode(LINE_SENSORS[i], INPUT);
  }

  pinMode(DIST_SENSOR, INPUT);
  pinMode(STOP_SENSOR, INPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(MOTOR_PIN_F, OUTPUT);

  delay(START_DELAY);
}


// Main loop
void loop() {

  if (millis() > global_timer + UPDATE_TIME) {
    global_timer = millis();

    update_sensors();
    update_speed();
  }


  if (stop_status) {
    if (millis() > stop_timer + STOP_DELAY) {
      update_stop_status();
    }
  }
}


// Main function, calls sensors survey
void update_sensors() {
  read_line_sensor();
  read_stop_line();
  read_dist();
}


// Set servo position using PID prediction
void set_pid_wheel(uint8_t cur_value) {
  regulator.input = cur_value;
  regulator.getResult();
  current_speed = count_speed(regulator.output);  // Update speed
  set_wheel(regulator.output);
}


// Write servo to position
void set_wheel(uint8_t position) {
  current_speed = count_speed(position);  // Update speed
  servo.write(position);
}


// Check stop line sensor
void read_stop_line() {
  uint32_t sensor_val = analogRead(STOP_SENSOR);

  if (sensor_val > LINE_SENSOR_THRESHOLD) {
    if (!ignore_stop) {
      stop();
    }
  } else {
    ignore_stop = false;
  }
}



// Check line sensors
void read_line_sensor() {

  uint8_t left, right, result;

  for (uint8_t i = 0; i < LINE_SENSORS_COUNT; i++) {
    if (((analogRead(LINE_SENSORS[i]) < LINE_SENSORS_THRESHOLD[i]) && !LINE_SENSORS_INVERSION) || ((analogRead(LINE_SENSORS[i]) > LINE_SENSORS_THRESHOLD[i]) && LINE_SENSORS_INVERSION)) {
      if (i < (LINE_SENSORS_COUNT / 2)) {
        left++;
      } else {
        right++;
      }
    }
  }

  // Print debug info
  if (DEBUG) {
    Serial.print("}   LEFT: ");
    Serial.print(left);
    Serial.print(" RIGHT: ");
    Serial.print(right);
  }


  if ((left + right) == 0) {
    if (prev_side_left) {
      result = SERVO_VAL_MIN;
    } else {
      result = SERVO_VAL_MAX;
    }
  } else {
    if (left > right) {
      result = SERVO_VAL_MIN + ((right + left) * SERVO_RATIO);
    } else if (left < right) {
      result = SERVO_VAL_MAX - ((right + left) * SERVO_RATIO);
    } else {
      result = SERVO_VAL_CENTER;
    }
  }

  if (result < SERVO_VAL_CENTER) {
    prev_side_left = true;
  } else {
    prev_side_left = false;
  }

  if (DEBUG) {
    Serial.print("    RESULT ANGLE: ");
    Serial.print(result);
    Serial.print("   SPEED: ");
    Serial.print(current_speed);
  }

  if (PID_ENABLE) {
    set_pid_wheel(result);
  } else {
    set_wheel(result);
  }
}


uint8_t count_speed(uint8_t angle) {
  return map(angle, SERVO_VAL_MIN, SERVO_VAL_MAX, SPEED_MIN, SPEED_MAX);
}


// Update main motor speed
void update_speed() {
  if (stop_status || barrier) {
    digitalWrite(MOTOR_PIN_F, LOW);
    digitalWrite(MOTOR_PIN_B, LOW);
  } else {
    if (prev_speed != current_speed) {
      prev_speed = current_speed;
      digitalWrite(MOTOR_PIN_B, LOW);
      analogWrite(MOTOR_PIN_F, current_speed);
    }
  }
}


// Check barrier
void read_dist() {
  float distance = 27.726 * pow(analogRead(DIST_SENSOR), -1.2045);
  if (distance < DIST_THRESHOLD) {
    if (!barrier) {
      if (start_state) {
        start_state = false;
      } else {
        sharp_brake_check();
      }
      barrier = true;
    }
  } else {
    barrier = false;
  }
}


// Restore stop status after after the timer
void update_stop_status() {
  if (stop_status) {
    stop_status = false;
    ignore_stop = true;
  }
}


// Update stop delay timer
void clear_stop_time() {
  stop_timer = millis();
}


// Make sharp brake if necessary
void sharp_brake_check() {
  if (SHARP_BRAKE) {
    analogWrite(MOTOR_PIN_B, MOTOR_REVERSE);
    delay(SHARP_BRAKE_DELAY);
  }
}


// Stop motor
void stop() {
  if (!stop_status) {
    sharp_brake_check();
    stop_status = true;
    clear_stop_time();
  }
}
