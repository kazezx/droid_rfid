//** NODEMCU UDP SERVER FOR DROID **
// BY KAZE

//libraries to be used
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//variables definitions
const char* ssid = "---" ;   //SSID of Network.
const char* password = "---" ;   //Password of SSID.
WiFiUDP DroidPort;
unsigned int mcuport = --- ;  //Port of NodeMCU.
char packetBuffer[255];

//void setup method for nodemcu
void setup() {
  Serial.begin(115200);   //baud rate for debugging.
  delay(100);
  Serial.print("Connecting to : ");
  Serial.print(ssid);
  WiFi.begin(ssid, password); //requesting connection to network.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Network Accessed @ IP : ");
  Serial.println(WiFi.localIP());
  DroidPort.begin(mcuport);   //open UDP listener at mcuport.
  Serial.print("Port Set : ");
  Serial.println(mcuport);
}

//Methods to be executed by nodemcu till on.
void loop ()
{
  sender();
  receiver();
}

//Sender method for udp packet.
void sender(){
  DroidPort.beginPacket("---", ---);  //create udp packet with destination (ip,port) from currently received packet.
  DroidPort.write("X=300,Y=500");  //write string to udp receiver.
  DroidPort.endPacket();   //ends port packet sending.
  delay(50000);
}

//Receiver method for udp packet.
void receiver(){
  int packetSize = DroidPort.parsePacket();  //reads packet size received.
  if (packetSize) {
    int len = DroidPort.read(packetBuffer, 255);  //reads packet as char[].
    String myData = ""; //string holder
    for (int i = 0; i < packetSize; i++) {
      myData += (char)packetBuffer[i];//read packets as a character looper.
    }
    Serial.println(myData);
  }
}


