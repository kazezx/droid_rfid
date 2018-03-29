//MOTOR BLOCK
#include <AFMotor.h>
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
int ims = 250;

//SONAR BLOCK
#include <NewPing.h>
NewPing front_sonar(A13, A12, 300);
NewPing leftback_sonar(A11, A10, 300);
NewPing rightback_sonar(A7, A8, 300);

//SERVO BLOCK
#include <Servo.h>
Servo servo;
int servoPin = 10;
int servoAngle = 0;


void setup() {
  Serial.begin(9600);
  motor1.setSpeed(ims);
  motor2.setSpeed(ims);
  motor3.setSpeed(ims);
  motor4.setSpeed(ims);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  servo.attach(servoPin);
  servo.write(68);
  delay(1000);
}

void loop() {
  servo.write(68);
  int distance = front_sonar.ping_cm();
  if (distance < 25 ) {
    path_choose();
  }
  servo.write(68);
  Serial.println("forward");
  forward();
  delay(20);
}

void forward() {
  //motor1.setSpeed(ims);
  //motor2.setSpeed(ims);
  //motor3.setSpeed(ims);
  //motor4.setSpeed(ims);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(100);
}

void backward() {
  //motor1.setSpeed(ims);
  //motor2.setSpeed(ims);
  //motor3.setSpeed(ims);
  //motor4.setSpeed(ims);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(100);
  motion_stop();
}

void rotate() {
  motor3.setSpeed(ims - 20);
  motor4.setSpeed(ims - 20);
  motor1.setSpeed(ims);
  motor2.setSpeed(ims);
  motor3.run(FORWARD);      // turn motor 1 forward
  motor4.run(FORWARD);      // turn motor 2 forward
  motor1.run(BACKWARD);    // turn motor 3 backward
  motor2.run(BACKWARD);    // turn motor 4 backward
  delay(1200); // run motors this way for 1500
  motion_stop();
}

void path_choose() {
  motion_stop();
  look_left();
  int left_path = front_sonar.ping_cm();
  delay(1500);
  look_right();
  int right_path = front_sonar.ping_cm();
  delay(1500);
  servo.write(68);
  if (left_path > right_path) {
    Serial.println("left rotating");
    turn_left();
  }
  else if (right_path > left_path) {
    Serial.println("right rotating");
    turn_right();
  }
  else {
    Serial.println("backward");
    backward();
  }
}

void motion_stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  motor1.setSpeed(ims);
  motor2.setSpeed(ims);
  motor3.setSpeed(ims);
  motor4.setSpeed(ims);
  delay(100);
}

void look_left() {
  for (servoAngle = 30; servoAngle < 105; servoAngle++) //0 to 135
  {
    servo.write(servoAngle);
    delay(10);
  }
}

void look_right() {
  for (servoAngle = 105; servoAngle > 30; servoAngle--) //135 to 0
  {
    servo.write(servoAngle);
    delay(10);
  }
}

void turn_left() {
  motor3.setSpeed(ims - 20);
  motor4.setSpeed(ims - 20);
  motor1.setSpeed(ims);
  motor2.setSpeed(ims);
  motor3.run(FORWARD);      // turn motor 1 forward
  motor4.run(FORWARD);      // turn motor 2 forward
  motor1.run(BACKWARD);    // turn motor 3 backward
  motor2.run(BACKWARD);    // turn motor 4 backward
  delay(1500); // run motors this way for 1500
  motion_stop();
}

void turn_right() {
  motor1.setSpeed(ims - 20);
  motor2.setSpeed(ims - 20);
  motor3.setSpeed(ims);
  motor4.setSpeed(ims);
  motor1.run(FORWARD);      // turn motor 1 forward
  motor2.run(FORWARD);      // turn motor 2 forward
  motor3.run(BACKWARD);    // turn motor 3 backward
  motor4.run(BACKWARD);    // turn motor 4 backward
  delay(1500); // run motors this way for 1500
  motion_stop();
}
