#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "StringSplitter.h"

//OLED SETUP
Adafruit_SSD1306 display(-1);

//WIFI SETUP
const char* ssid = "Zuhayr" ;
const char* password = "Zuhayr007" ;
WiFiUDP DroidPort;
unsigned int mcuport = 2807 ;
char packetBuffer[255];

boolean checker = false ;
String zx = "";
String goal = "";

const unsigned char myBitmap [] PROGMEM = {
  // 'Untitled-1, 128x64px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xfd, 0x17, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x08, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x0c, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x40, 0x00, 0x00, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x40, 0x00, 0x00, 0x18, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x40, 0x02, 0x00, 0x38, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x60, 0x07, 0x80, 0x0c, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x10, 0x07, 0x80, 0x10, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x50, 0x07, 0x80, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x10, 0x1e, 0xc0, 0xc8, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x48, 0x0e, 0x60, 0x20, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x48, 0x1c, 0x10, 0x80, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x24, 0x3c, 0x11, 0x88, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x20, 0x38, 0x01, 0x48, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x78, 0x09, 0x04, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x20, 0x30, 0x06, 0x88, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x20, 0x30, 0x02, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x20, 0xa0, 0x03, 0x84, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x30, 0x60, 0x01, 0x08, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x20, 0x40, 0x03, 0x84, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x30, 0x40, 0x01, 0xc4, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x60, 0x06, 0x6c, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x20, 0x0e, 0x24, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x19, 0x10, 0x0e, 0x34, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x18, 0x98, 0x0e, 0x1c, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x18, 0x08, 0x0e, 0x0c, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x18, 0x0c, 0x1c, 0x0c, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x1c, 0x04, 0x1c, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x08, 0x06, 0x3c, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x02, 0x38, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x02, 0x38, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x03, 0x70, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x01, 0x70, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x01, 0xf0, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0xe0, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0xe0, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0xc0, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Wire.begin(D2, D1);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  Serial.begin(115200);
  display.drawBitmap(0, 0, myBitmap, 128, 64, WHITE);
  display.display();
  delay(2500);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("SSID : ");
  display.print(ssid);
  WiFi.begin(ssid, password);
  Serial.println("Connected");
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    Serial.print(".");
  }
  display.setCursor(0, 10);
  display.print("IP : ");
  //display.setCursor(0, 20);
  display.print(WiFi.localIP());
  Serial.println(WiFi.localIP());
  DroidPort.begin(mcuport);
  display.setCursor(0, 20);
  display.print("Port : ");
  display.println(mcuport);
  display.display();
  delay(1500);
}

void loop() {
  delay(1000);
  zx = loc_receiver();
  char loc [zx.length() + 1];
  if (zx != "") {
    zx.toCharArray(loc, (zx.length() + 1));
    Wire.beginTransmission(8);
    Wire.write(loc);
    Wire.endTransmission();
    goal = zx;
  }
  checker = false ;
  String r_position = "";
  Wire.requestFrom(8, 8);
  while (Wire.available()) {
    char x = Wire.read();
    r_position += x ;
  }
  Serial.println(r_position);
  display.clearDisplay();;
  display.setCursor(0, 10);
  display.print("Pos. : ");
  String model = mode_split(r_position);
  String coordinates = data_split(r_position);
  display.print(coordinates);
  display.setCursor(0, 20);
  display.print("Type :");
  if (model.equals("C")) {
    map_sender(coordinates);
    display.setCursor(0, 20);
    display.print("Type : Robot ");
  }
  else if (model.equals("O")) {
    obstacle_sender(coordinates);
    display.setCursor(0, 20);
    display.print("Type : Obstacle");
  }
  else if (model.equals("M")) {
    track_sender(coordinates);
    display.setCursor(0, 20);
    display.print("Type : Movement");
  }
  display.setCursor(0, 30);
  display.print("Goal : ");
  display.print(goal);
  display.display();
}

String loc_receiver() {
  String myData = "";
  int packetSize = DroidPort.parsePacket();
  if (packetSize) {
    int len = DroidPort.read(packetBuffer, 64);
    for (int i = 0; i < packetSize; i++) {
      myData += (char)packetBuffer[i];
    }
  }
  return myData;
}

void obstacle_sender(String obs_code) {
  char obs_packet[(obs_code.length() + 1)];
  if (obs_code != "") {
    obs_code.toCharArray(obs_packet, (obs_code.length() + 1));
    DroidPort.beginPacket("192.168.100.2", 55982);
    DroidPort.write(obs_packet);
    DroidPort.endPacket();
    delay(100);
  }
}

void map_sender(String map_code) {
  char map_packet[(map_code.length() + 1)];
  if (map_code != "") {
    map_code.toCharArray(map_packet, (map_code.length() + 1));
    DroidPort.beginPacket("192.168.100.2", 55982);
    DroidPort.write(map_packet);
    DroidPort.endPacket();
    delay(100);
  }
}

void track_sender(String map_code) {
  char track_packet[(map_code.length() + 1)];
  if (map_code != "") {
    map_code.toCharArray(track_packet, (map_code.length() + 1));
    DroidPort.beginPacket("192.168.100.2", 55982);
    DroidPort.write(track_packet);
    DroidPort.endPacket();
    delay(100);
  }
}

String mode_split(String text) {
  String codex = "" ;
  StringSplitter *splitter = new StringSplitter(text, ':', 2);
  int itemCount = splitter->getItemCount();
  for (int i = 0; i < 1; i++) {
    codex = splitter->getItemAtIndex(i);
  }
  return codex;
}

String data_split(String text) {
  String coor = "" ;
  StringSplitter *splitter = new StringSplitter(text, ':', 2);
  int itemCount = splitter->getItemCount();
  for (int i = 1; i < 2; i++) {
    coor = splitter->getItemAtIndex(i);
  }
  return coor;
}
