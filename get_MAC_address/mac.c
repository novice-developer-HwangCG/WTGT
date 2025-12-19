#include <WiFi.h>
void setup(){ Serial.begin(115200); WiFi.mode(WIFI_STA); delay(100); Serial.println(WiFi.macAddress()); }
void loop(){}

// EC:E3:34:1B:61:D8