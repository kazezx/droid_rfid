#include <AFMotor.h>
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

float Kp = 1, Ki = 0, Kd = 0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[5] = {0, 0, 0, 0, 0};
int ims = 230;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

void setup()
{
  Serial.begin(9600);
  motor1.setSpeed(ims);
  motor2.setSpeed(ims);
  motor3.setSpeed(ims);
  motor4.setSpeed(ims);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  ;
  delay(1000);
}

void loop()
{
  read_sensor_values();
  calculate_pid();
  motor_control();
  delay(3000);
  Serial.println();
}

void read_sensor_values()
{
  sensor[0] = digitalRead(15);
  sensor[1] = digitalRead(16);
  sensor[2] = digitalRead(17);
  sensor[3] = digitalRead(18);
  sensor[4] = digitalRead(19);
  for (int i = 0; i < 5; i++) {
    Serial.println(sensor[i]);
  }
  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[4] == 0) && (sensor[4] == 1))
    error = 4;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[4] == 1) && (sensor[4] == 1))
    error = 3;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[4] == 1) && (sensor[4] == 0))
    error = 2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[4] == 1) && (sensor[4] == 0))
    error = 1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[4] == 0) && (sensor[4] == 0))
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[4] == 0) && (sensor[4] == 0))
    error = -1;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[4] == 0) && (sensor[4] == 0))
    error = -2;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[4] == 0) && (sensor[4] == 0))
    error = -3;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[4] == 0) && (sensor[4] == 0))
    error = -4;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[4] == 0) && (sensor[4] == 0))
    if (error == -4) error = -5;
    else error = 5;
  Serial.print("error :: ");
  Serial.println(error);

}

void calculate_pid()
{
  P = error;
  Serial.print("P : ");
  Serial.println(P);
  I = I + previous_I;
  Serial.print("I : ");
  Serial.println(I);
  D = error - previous_error;
  Serial.print("d : ");
  Serial.println(D);
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  Serial.print("PID value : ");
  Serial.println(PID_value);
  previous_I = I;
  Serial.print("Previous I : ");
  Serial.println(previous_I);
  previous_error = error;
  Serial.print("Previous error : ");
  Serial.println(previous_error);

}

void motor_control()
{
  // Calculating the effective motor speed:
  int lms = ims - PID_value * 10;
  int rms = ims + PID_value * 10 ;

  Serial.print("Right = ");
  Serial.println(rms);

  Serial.print("Left = ");
  Serial.println(lms);
  // The motor speed should not exceed the max PWM value
  constrain(lms, 0, 255);
  constrain(rms, 0, 255);

  motor1.setSpeed(lms); //Left Motor Speed
  motor2.setSpeed(lms); //Left Motor Speed
  motor3.setSpeed(rms); //Right Motor Speed
  motor4.setSpeed(rms); //Right Motor Speed
  //following lines of code are to make the bot move forward
  /*The pin numbers and high, low values might be different
    depending on your connections */
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor2.run(FORWARD);
  motor4.run(FORWARD);
}
