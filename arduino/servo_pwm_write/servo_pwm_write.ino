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

// Test script for calibrating servos
#include <Servo.h>

const int SERVO_PIN = 30;
Servo servo;

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  servo.write(90);
}

void loop() {
  while (Serial.available() > 0) {
    int ms = Serial.parseInt();
    if (ms > 2400 || ms < 600) {
      Serial.println("Error: out of bounds");
    } else {
      servo.writeMicroseconds(ms);
      Serial.print("Wrote "); Serial.print(ms); Serial.println(" ms");
    }
    do {
      Serial.read();
    }  while (Serial.available() > 0);
    delay(15);
  }
}
