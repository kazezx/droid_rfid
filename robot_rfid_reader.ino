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

int count1 = 0 ;
int count2 = 0 ;
int count3 = 0 ;

int x_pos = 0;
int y_pos = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  mfrc522_1.PCD_Init();
  mfrc522_2.PCD_Init();
  mfrc522_3.PCD_Init();
  Serial.println("Scan a MIFARE Classic card");
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

}

void loop() {
  int  c = 0;
  x_pos = 0;
  y_pos = 0;
  scanner1();
  scanner2();
  scanner3();
  //delay(200);
  c = count1 + count2 + count3;
  Serial.print("Counter:");
  Serial.println(c);
  Serial.print("X position:");
  Serial.println(x_pos / c);
  Serial.print("Y position:");
  Serial.println(y_pos / c);
  Serial.println();
  //delay(1000);
}

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
  Serial.print("X1:");
  Serial.println(x1_coor.toInt());
  Serial.print("Y1:");
  Serial.println(y1_coor.toInt());
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
  Serial.print("X2:");
  Serial.println(x2_coor.toInt());
  Serial.print("Y2:");
  Serial.println(y2_coor.toInt());
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
  Serial.print("X3:");
  Serial.println(x3_coor.toInt());
  Serial.print("Y3:");
  Serial.println(y3_coor.toInt());
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
