#include <Servo.h>
#define SERVO_PIN 3
#define SERVO_VAL_MIN 60
#define SERVO_VAL_MAX 120
#define SERVO_VAL_CENTER 90
Servo servo;

void setup() {
  Serial.begin(9600);

  // SERVO INIT
  servo.attach(SERVO_PIN);
}


void loop() {

  servo.write(SERVO_VAL_CENTER);
  delay(1000);
  servo.write(SERVO_VAL_MAX);
  delay(1000);
  servo.write(SERVO_VAL_CENTER);
  delay(1000);
  servo.write(SERVO_VAL_MIN);
  delay(1000);

}
