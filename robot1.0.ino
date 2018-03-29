/*
   RFID
   MOTOR(obsctacle+move
   send to nodemcu(current_pos)
   pir_security
*/

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

//RFID BLOCK
#include <SPI.h>
#include <MFRC522.h>

#define SS1_PIN 33
#define SS2_PIN 36
#define SS3_PIN 40
#define RST_PIN 30
MFRC522 mfrc522_1(SS1_PIN, RST_PIN);
MFRC522 mfrc522_2(SS2_PIN, RST_PIN);
MFRC522 mfrc522_3(SS3_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

#define NTAG_PAGELENGTH 4
#define NTAG_BLOCKLENGTH 4*NTAG_PAGELENGTH

byte block_X1 [16] ;
byte block_Y1 [16] ;

byte block_X2 [16] ;
byte block_Y2 [16] ;

byte block_X3 [16] ;
byte block_Y3 [16] ;

int count  = 0 ;
int count1 = 0 ;
int count2 = 0 ;
int count3 = 0 ;

int x_pos = 0;
int y_pos = 0;

int pX = 0 ;
int pY = 0 ;

//I2C BLOCK
#include <Wire.h>
String goal_packet = "";
String packet_coor = "";
char posBuffer[32];
String goal_coor = "";
String coor_track = "";

//PIR BLOCK
int left_sensor = 46;
int right_sensor = 47;
int state_left = LOW;
int val_left = 0;
int state_right = LOW;
int val_right = 0;
int ledleft = 53;
int ledright = 24;
int d = 0;
String postn = "";

//PID BLOCK
float Kp = 1, Ki = 0, Kd = 0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[5] = {0, 0, 0, 0, 0};

void setup() {
  //Serial.begin(9600);
  SPI.begin();
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  mfrc522_1.PCD_Init();
  mfrc522_2.PCD_Init();
  mfrc522_3.PCD_Init();
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  motor1.setSpeed(ims);
  motor2.setSpeed(ims);
  motor3.setSpeed(ims);
  motor4.setSpeed(ims);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  pinMode(left_sensor, INPUT);
  pinMode(right_sensor, INPUT);
  pinMode(ledleft, OUTPUT);
  pinMode(ledright, OUTPUT);
  servo.attach(servoPin);
  servo.write(68);
  delay(1000);
}

void loop() {
  forward();
  delay(500);
  //rfid
  count = 0;
  x_pos = 0;
  y_pos = 0;
  scanner1();
  scanner2();
  scanner3();
  count = count1 + count2 + count3;
  if (count != 0) {
    //Serial.println(count);
    int x = x_pos / count;
    int y = y_pos / count;
    if (x > 0 ) {
      pX = x ;
    }
    if (y > 0) {
      pY = y ;
    }
  }
  //data exchange
  String coordinates = "C:" + String (pX) + ',' + String (pY) ;
  packet_coor = coordinates;
  //movement
  d = 0;
  motionscanner();
  //Serial.print("d after detection :");
  //Serial.println(d);
  if (postn.equals("mid")) {
    //Serial.println("mid detection");
    int my = d + pY ;
    int mx = pX;
    coor_track = "M:" + String (mx) + ',' + String (my) ;
    packet_coor = coor_track;
    //Serial.println(coor_track);
    delay(20);
  }
  else if (postn.equals("left")) {
    //Serial.println("left detection");
    float alpha = 23.0 / 180.0;
    double qx = sin(alpha);
    double qy = cos(alpha);
    float lt_x = d * qx ;
    float lt_y = d * qy ;
    int lpX = pX - lt_x ;
    int lpY = pY + lt_y ;
    coor_track = "M:" + String (lpX) + ',' + String (lpY) ;
    packet_coor = coor_track;
    //Serial.println(coor_track);
    delay(20);
  }
  else if (postn.equals("right")) {
    //Serial.println("right detection");
    float alpha = 22.0 / 180.0;
    float ri_y = d * (sin(alpha));
    float ri_x = d * (cos(alpha));
    int rpX = pX + ri_x ;
    int rpY = pY + ri_y ;
    coor_track = "M:" + String (rpX) + ',' + String (rpY) ;
    packet_coor = coor_track;
    //Serial.println(coor_track);
    delay(20);
  }

  else {
    int distance = front_sonar.ping_cm();
    if (distance < 25 ) {
      pY = pY + 25 ;
      String coor_obstacle = "O:" + String (pX) + ',' + String (pY) ;
      packet_coor = coor_obstacle;
      path_choose();
    }
    //Serial.println("forward!!!");
    forward();
  }
  d = 0;
  //pX = 0;
  //pY = 0;
  delay(25);
  //postn = 0 ;
}

//MOTION ROUTINES
void forward() {
  int lms = ims - PID_value * 10;
  int rms = ims + PID_value * 10 ;

  constrain(lms, 0, 255);
  constrain(rms, 0, 255);

  motor1.setSpeed(lms);
  motor2.setSpeed(lms);
  motor3.setSpeed(rms);
  motor4.setSpeed(rms);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(100);
}

void backward() {
  int lms = ims - PID_value * 10;
  int rms = ims + PID_value * 10 ;

  constrain(lms, 0, 255);
  constrain(rms, 0, 255);

  motor1.setSpeed(lms);
  motor2.setSpeed(lms);
  motor3.setSpeed(rms);
  motor4.setSpeed(rms);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(1000);
  motion_stop();
}

void rotate() {
  motor3.setSpeed(ims - 20);
  motor4.setSpeed(ims - 20);
  motor1.setSpeed(ims);
  motor2.setSpeed(ims);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  delay(1200);
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
    turn_left();
  }
  else if (right_path > left_path) {
    turn_right();
  }
  else {
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
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  delay(1200);
  motion_stop();
}

void turn_right() {
  motor1.setSpeed(ims - 20);
  motor2.setSpeed(ims - 20);
  motor3.setSpeed(ims);
  motor4.setSpeed(ims);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(1200);
  motion_stop();
}

//RFID ROUTINES
void scanner1() {
  if ( ! mfrc522_1.PICC_IsNewCardPresent()) {
    count1 = 0 ;
    return;
  }
  if ( ! mfrc522_1.PICC_ReadCardSerial()) {
    count1 = 0 ;
    return;
  }
  count1 = 1 ;
  int y1 = NTAGReadBlock_1(7, block_Y1);
  int x1 = NTAGReadBlock_1(6, block_X1);
  delay(100);
  String x1_coor = "" ;
  String y1_coor = "" ;

  for (int j = 0; j < 4 ; j++) {
    x1_coor += String(block_X1[j]);
  }
  for (int i = 0; i < 4 ; i++) {
    y1_coor += String(block_Y1[i]);
  }
  x_pos += x1_coor.toInt();
  y_pos += y1_coor.toInt();
}

void scanner2() {
  if ( ! mfrc522_2.PICC_IsNewCardPresent()) {
    count2 = 0 ;
    return;
  }
  if ( ! mfrc522_2.PICC_ReadCardSerial()) {
    count2 = 0 ;
    return;
  }
  count2 = 1 ;
  int y2 = NTAGReadBlock_2(7, block_Y2);
  int x2 = NTAGReadBlock_2(6, block_X2);
  delay(100);
  String x2_coor = "" ;
  String y2_coor = "" ;
  for (int j = 0; j < 4 ; j++) {
    x2_coor += String(block_X2[j]);
  }
  for (int i = 0; i < 4 ; i++) {
    y2_coor += String(block_Y2[i]);
  }
  x_pos += x2_coor.toInt();
  y_pos += y2_coor.toInt();
}

void scanner3() {
  if ( ! mfrc522_3.PICC_IsNewCardPresent()) {
    count3 = 0 ;
    return;
  }
  if ( ! mfrc522_3.PICC_ReadCardSerial()) {
    count3 = 0 ;
    return;
  }
  count3 = 1 ;
  int y3 = NTAGReadBlock_3(7, block_Y3);
  int x3 = NTAGReadBlock_3(6, block_X3);
  delay(100);
  String x3_coor = "" ;
  String y3_coor = "" ;
  for (int j = 0; j < 4 ; j++) {
    x3_coor += String(block_X3[j]);
  }
  for (int i = 0; i < 4 ; i++) {
    y3_coor += String(block_Y3[i]);
  }
  x_pos += x3_coor.toInt();
  y_pos += y3_coor.toInt();
}

int NTAGReadBlock_1(byte blockAddr, uint8_t *data) {
  byte length = NTAG_BLOCKLENGTH + 2;
  byte buffer[NTAG_BLOCKLENGTH + 2] = { 0 }; // this holds CRC as well
  MFRC522::StatusCode status;
  if ((status = mfrc522_1.MIFARE_Read(blockAddr, buffer, &length)) != mfrc522_1.STATUS_OK) {
    return -1;
  }
  if (length != NTAG_BLOCKLENGTH + 2) {
    return -2;
  }
  memcpy(data, buffer, NTAG_BLOCKLENGTH);
  return 0;
}

int NTAGReadBlock_2(byte blockAddr, uint8_t *data) {
  byte length = NTAG_BLOCKLENGTH + 2;
  byte buffer[NTAG_BLOCKLENGTH + 2] = { 0 }; // this holds CRC as well
  MFRC522::StatusCode status;
  if ((status = mfrc522_2.MIFARE_Read(blockAddr, buffer, &length)) != mfrc522_2.STATUS_OK) {
    return -1;
  }
  if (length != NTAG_BLOCKLENGTH + 2) {
    return -2;
  }
  memcpy(data, buffer, NTAG_BLOCKLENGTH);
  return 0;
}

int NTAGReadBlock_3(byte blockAddr, uint8_t *data) {
  byte length = NTAG_BLOCKLENGTH + 2;
  byte buffer[NTAG_BLOCKLENGTH + 2] = { 0 }; // this holds CRC as well
  MFRC522::StatusCode status;
  if ((status = mfrc522_3.MIFARE_Read(blockAddr, buffer, &length)) != mfrc522_3.STATUS_OK) {
    return -1;
  }
  if (length != NTAG_BLOCKLENGTH + 2) {
    return -2;
  }
  memcpy(data, buffer, NTAG_BLOCKLENGTH);
  return 0;
}

//DATA EXCHANGE
void receiveEvent(int howMany) {
  goal_packet = "";
  while (0 < Wire.available()) {
    char c = Wire.read();
    goal_packet += c ;
  }
  goal_coor = goal_packet;
  Serial.println(goal_coor);
}

void requestEvent() {
  if (count <= 0) {
    packet_coor.toCharArray(posBuffer, (packet_coor.length() + 1));
    Wire.write(posBuffer);
  }
  else {
    return 0;
  }
}

//PIR DETECTION
void motionscanner() {
  d = 0;
  val_right = digitalRead(right_sensor);
  val_left = digitalRead(left_sensor);
  //Serial.println(val_left);
  //Serial.println(val_right);
  if (val_left == LOW && val_right == HIGH) {
    sense_right();
  }
  else if (val_right == LOW && val_left == HIGH) {
    sense_left();
  }
  else if (val_right == HIGH && val_left == HIGH) {
    sense_mid();
  }
  else if (val_right == LOW && val_left == LOW) {
    servo.write(68);
    postn = "" ;
    digitalWrite(ledright, LOW);
    digitalWrite(ledleft, LOW);
  }
  //Serial.print("Distance detected : ");
  //Serial.println(d);
}

void sense_right() {
  val_right = digitalRead(right_sensor);
  if (val_right == HIGH) {
    digitalWrite(ledright, HIGH);
    servo.write(0);
    d = front_sonar.ping_cm();
    postn = "right";
    delay(10);
    if (state_right == LOW) {
      state_right = HIGH;
    }
  }
  else {
    digitalWrite(ledright, LOW);
    servo.write(68);
    d = 0;
    postn = "";
    delay(10);
    if (state_right == HIGH) {
      state_right = LOW;
    }
  }
}

void sense_left() {
  val_left = digitalRead(left_sensor);
  if (val_left == HIGH) {
    digitalWrite(ledleft, HIGH);
    servo.write(135);
    d = front_sonar.ping_cm();
    postn = "left";
    delay(10);
    if (state_left == LOW) {
      state_left = HIGH;
    }
  }
  else {
    digitalWrite(ledleft, LOW); // turn LED OFF
    servo.write(68);
    d = 0;
    postn = "";
    delay(10);
    if (state_left == HIGH) {
      state_left = LOW;
    }
  }
}

void sense_mid() {
  val_left = digitalRead(left_sensor);
  val_right = digitalRead(right_sensor);
  if (val_left == HIGH && val_right == HIGH) {
    digitalWrite(ledleft, HIGH);
    digitalWrite(ledright, HIGH);
    servo.write(68);
    d = front_sonar.ping_cm();
    postn = "mid";
    delay(10);
    if (state_left == LOW) {
      state_left = HIGH;
    }
  }
  else {
    digitalWrite(ledleft, LOW);
    servo.write(68);
    d = 0;
    postn = "";
    delay(10);
    if (state_left == HIGH) {
      state_left = LOW;
    }
  }
}

void read_sensor_values()
{
  sensor[0] = digitalRead(15);
  sensor[1] = digitalRead(16);
  sensor[2] = digitalRead(17);
  sensor[3] = digitalRead(18);
  sensor[4] = digitalRead(19);

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

}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_I = I;
  previous_error = error;
}
