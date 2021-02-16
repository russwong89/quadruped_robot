// Copyright (c) 2021 Russell Wong
//
// “Commons Clause” License Condition v1.0
//
// The Software is provided to you by the Licensor under the License, as defined
// below, subject to the following condition.
//
// Without limiting other conditions in the License, the grant of rights under
// the License will not include, and the License does not grant to you, the right
// to Sell the Software.
//
// For purposes of the foregoing, “Sell” means practicing any or all of the rights
// granted to you under the License to provide to third parties, for a fee or other
// consideration (including without limitation fees for hosting or consulting/
// support services related to the Software), a product or service whose value derives,
// entirely or substantially, from the functionality of the Software. Any license
// notice or attribution required by the License must also include this Commons Clause
// License Condition notice.
//
// Software: https://github.com/russwong89/quadruped_robot
//
// License: MIT License with Commons Clause License Condition
// (https://github.com/russwong89/quadruped_robot/blob/main/LICENSE.md)
//
// Licensor: Russell Wong (russwong89@gmail.com)

#include <Servo.h>
#include <SoftwareSerial.h>
#include "servo_util.h"

const int RX_PIN = 10;
const int TX_PIN = 11;

Servo servos[servo_util::NUM_SERVOS];
SoftwareSerial HC05(RX_PIN, TX_PIN); // Arduino(RX, TX) || HC-05(TX, RX)

const int BTN_MOVE_FORWARD = 1;
const int BTN_MOVE_BACKWARD = 2;
const int BTN_ROTATE_CW = 3;
const int BTN_ROTATE_CCW = 4;
const int BTN_OPEN_LEGS = 5;
const int BTN_CLOSE_LEGS = 6;

bool legs_open = false;

void write_angle(const int &servo_num, int angle) {
  // Safer servo write function which checks for angle limits
  if (angle > servo_util::MAX_ANGLE) angle = servo_util::MAX_ANGLE;
  if (angle < servo_util::MIN_ANGLE) angle = servo_util::MIN_ANGLE;
  servos[servo_num].write(angle);
}

void close_legs() {
  // First set all shoulder servos to their home positions
  write_angle(0, servo_util::CLOSED_INITIAL_ANGLES[0]);
  write_angle(3, 70); // Compensate for faulty servo
  write_angle(6, servo_util::CLOSED_INITIAL_ANGLES[6]);
  write_angle(9, servo_util::CLOSED_INITIAL_ANGLES[9]);
  delay(1000);

  // Next set all thigh servos to their initial positions
  write_angle(1, servo_util::CLOSED_INITIAL_ANGLES[1]);
  servos[4].write(145); // Compensate for faulty servo
  write_angle(7, servo_util::CLOSED_INITIAL_ANGLES[7]);
  write_angle(10, servo_util::CLOSED_INITIAL_ANGLES[10]);
  delay(1000);

  // Compensate for faulty servo
  write_angle(3, servo_util::CLOSED_INITIAL_ANGLES[3]);
  delay(1000);

  // Next set all calf servos to their initial positions
  write_angle(2, servo_util::CLOSED_INITIAL_ANGLES[2]);
  write_angle(4, servo_util::CLOSED_INITIAL_ANGLES[4]);
  write_angle(5, servo_util::CLOSED_INITIAL_ANGLES[5]);
  write_angle(8, servo_util::CLOSED_INITIAL_ANGLES[8]);
  write_angle(11, servo_util::CLOSED_INITIAL_ANGLES[11]);
  delay(1000);
}

void open_legs() {
  // First set all shoulder servos to their initial positions
  write_angle(0, servo_util::OPEN_INITIAL_ANGLES[0]);
  write_angle(3, servo_util::OPEN_INITIAL_ANGLES[3]);
  write_angle(6, servo_util::OPEN_INITIAL_ANGLES[6]);
  write_angle(9, servo_util::OPEN_INITIAL_ANGLES[9]);
  delay(1000);

  // Next set all calf servos to their initial positions
  write_angle(2, servo_util::OPEN_INITIAL_ANGLES[2]);
  write_angle(5, servo_util::OPEN_INITIAL_ANGLES[5]);
  write_angle(8, servo_util::OPEN_INITIAL_ANGLES[8]);
  write_angle(11, servo_util::OPEN_INITIAL_ANGLES[11]);
  delay(1000);

  // Next set all thigh servos to their initial positions
  write_angle(1, servo_util::OPEN_INITIAL_ANGLES[1]);
  write_angle(4, servo_util::OPEN_INITIAL_ANGLES[4]);
  write_angle(7, servo_util::OPEN_INITIAL_ANGLES[7]);
  write_angle(10, servo_util::OPEN_INITIAL_ANGLES[10]);
  delay(1000);
}

void actuate_servos(int (&angles)[servo_util::ANGLES_PER_MOTION][servo_util::NUM_SERVOS]) {
  for (int i = 0; i < servo_util::ANGLES_PER_MOTION; ++i) {
    for (int j = 0; j < servo_util::NUM_SERVOS; ++j) {
      write_angle(j, angles[i][j]);
    }
    delay(30);
  }
}

void setup_servo(const int &servo_num) {
  if (servo_num >= servo_util::NUM_SERVOS) return;
  servos[servo_num].attach(servo_util::FIRST_SERVO_PIN + servo_num, servo_util::PULSE_RANGES[servo_num][0], servo_util::PULSE_RANGES[servo_num][1]);
  write_angle(servo_num, servo_util::CLOSED_INITIAL_ANGLES[servo_num]);
  delay(15);
}

void setup() {
  Serial.begin(9600);
  HC05.begin(9600);

  // First setup shoulder servos
  setup_servo(0);
  setup_servo(3);
  setup_servo(6);
  setup_servo(9);
  delay(1000);

  // Next setup thigh servos
  setup_servo(1);
  setup_servo(4);
  setup_servo(7);
  setup_servo(10);
  delay(1000);

  // Next setup calf servos
  setup_servo(2);
  setup_servo(5);
  setup_servo(8);
  setup_servo(11);
  delay(1000);

  Serial.println("Setup complete");
}

void loop() {
  while (!HC05.available()) {
    delay(5);
  }
  int button_cmd = HC05.read()-'0';
  Serial.println(button_cmd);
  switch (button_cmd) {
    case BTN_OPEN_LEGS: {
      if (!legs_open) {
        open_legs();
        legs_open = true;
      }
      break;
    }
    case BTN_CLOSE_LEGS: {
      if (legs_open) {
        close_legs();
        legs_open = false;
      }
      break;
    }
    case BTN_MOVE_FORWARD: {
      if (legs_open) {
        actuate_servos(servo_util::MOVE_FORWARD_ANGLES);
      }
      break;
    }
    case BTN_MOVE_BACKWARD: {
      if (legs_open) {
        actuate_servos(servo_util::MOVE_BACKWARD_ANGLES);
      }
      break;
    }
    case BTN_ROTATE_CW: {
      if (legs_open) {
        actuate_servos(servo_util::ROTATE_CW_ANGLES);
      }
      break;
    }
    case BTN_ROTATE_CCW: {
      if (legs_open) {
        actuate_servos(servo_util::ROTATE_CCW_ANGLES);
      }
      break;
    }
    default: {
      Serial.println("Unrecognized button press");
      break;
    }
  }

  // Clear serial buffer
  while (HC05.available()) {
    HC05.read();
  }
}
