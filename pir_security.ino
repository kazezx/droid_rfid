#include <NewPing.h>
#include <Servo.h>
Servo servo;
NewPing front_sonar(A13, A12, 300);
int servoPin = 10;
int servoAngle = 0;;
int left_sensor = 46;
int right_sensor = 47;
int state_left = LOW;
int val_left = 0;
int state_right = LOW;
int val_right = 0;
int ledleft = 53;
int ledright = 24;
int d = 0;
void setup() {
  pinMode(left_sensor, INPUT);
  pinMode(right_sensor, INPUT);
  pinMode(ledleft, OUTPUT);
  pinMode(ledright, OUTPUT);
  Serial.begin(115200);
  servo.attach(servoPin);
  servo.write(68);
}

void loop() {
  motionscanner();
  delay(50);
}
void motionscanner() {
  servo.write(68);
  d = 0;
  val_right = digitalRead(right_sensor);
  val_left = digitalRead(left_sensor);
  Serial.println(val_left);
  Serial.println(val_right);

  if ( val_right == HIGH) {
    servo.write(0);
    d = front_sonar.ping_cm();
  }
  //servo.write(68);
  if ( val_left == HIGH) {
    servo.write(135);
    d = front_sonar.ping_cm();
  }
  //servo.write(68);
  if (val_left == HIGH && val_right == HIGH) {
    servo.write(68);
    d = front_sonar.ping_cm();
  }

  Serial.print("Distance detected : ");
  Serial.println(d);

}
