#include <Wire.h>
String packet = "";
String packet_coor = "";
char posBuffer[32];
void setup() {
  Wire.begin(8);
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent); 
  Serial.begin(9600);          
}

void loop() {
  packet_coor = packet;
  delay(10);
}

void receiveEvent(int howMany) {
  while (0 < Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println();
}

void requestEvent() {
  packet_coor.toCharArray(posBuffer, (packet_coor.length() + 1));
  Wire.write(posBuffer);
}
