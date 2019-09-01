

#include "Arduino.h"
#ifdef ESP32
#include <WiFi.h>
#endif
#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif


#include <ArduinoOTA.h>
#include "wifi-config.h"

WiFiServer TelnetServer(23);
WiFiClient Telnet;

#include <PrintEx.h>
StreamEx debugI = Telnet;

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
  WiFi.mode(WIFI_STA);
  WiFi.begin(LOCAL_SSID, LOCAL_PSK);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    debugI.printf("Connecting to: %s\r\n", LOCAL_SSID);
    delay(5000);
    ESP.restart();
  }
 
  ArduinoOTA.begin(); 
 
  TelnetServer.begin();
  TelnetServer.setNoDelay(true);
  debugI.printf("Start %d\n", 0);
}


uint i = 0;
void loop()
{
  i++;
  debugI.printf("loop %d %d\n", i, millis());
  handleTelnet();
  ArduinoOTA.handle();

}
