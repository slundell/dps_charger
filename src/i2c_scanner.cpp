

#include "Arduino.h"
#ifdef ESP32
#include <WiFi.h>
#endif
#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif


#include <ArduinoOTA.h>
#include <Wire.h>
#include "wifi-config.h"

WiFiServer TelnetServer(23);
WiFiClient Telnet;

void print(String p){
  Serial.print(p);
  Telnet.print(p);
  Serial.flush();
}

void print(unsigned char b, unsigned char m){
  Serial.print(b, m);
  Telnet.print(b, m);
  Serial.flush();
}

void println(String p){
  Serial.println(p);
  Telnet.println(p);
}

void println(){
  Serial.println();
  Telnet.println();
}


void handleTelnet() {
    if (TelnetServer.hasClient()) {
      if (!Telnet || !Telnet.connected()) {
        if (Telnet) Telnet.stop();
        Telnet = TelnetServer.available();
      } else {
        TelnetServer.available().stop();
      }
    }
}

void setup() {


  Serial.begin(115200);
  Wire.begin();
  WiFi.mode(WIFI_STA);
  WiFi.begin(LOCAL_SSID, LOCAL_PSK);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(5000);
    ESP.restart();
  }
 
  ArduinoOTA.begin(); 
 
  TelnetServer.begin();
  TelnetServer.setNoDelay(true);

  println("Ready");
  //println("IP address: ");
  //println(WiFi.localIP());

}



void print_byte(uint8_t b){
  print("0x");
  if (b < 16)
    print("0");
  print(b, HEX);
}

void scan_for_device(uint8_t from, uint8_t to) {
  println("\n\n\nScanning");
  byte address;
  for (address = from ; address <= to; address++ )  {
    if (!(address % 4)) {
      print(" ");
    }
    if (!(address % 16)) {
      println();
    }
    
    
    Wire.beginTransmission(address);
    uint8_t ack = Wire.endTransmission();
    if (ack == 0){
      print_byte(address);
      Serial.flush();
    } else {
      print("....");
    }

  handleTelnet();    
    
  }
  println("\nDone");
  
}
void loop()
{
  scan_for_device(0x00, 0x7F);
  handleTelnet();
  ArduinoOTA.handle();
  delay(3000);
}
