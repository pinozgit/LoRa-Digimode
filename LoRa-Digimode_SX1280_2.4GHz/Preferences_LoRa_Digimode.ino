/*
 * Program to write the EEPROM with the configuration settings
 * for LoRa Digimode Programs
 * 
 * 73 de Pino ZP4KFX
 */
#include <Preferences.h>

Preferences preferences;

const char* ssid = "YOUR_SSID";       // put here your credentials
const char* password = "YOUR_PWD";


String MyCall = "N0CALL";          // change it !
String MyLocator = "AA00aa" ;
String BTName = "LoRaDigimode";
String MyIP = "192.168.1.200";
long frequency = 2445.0E6 ; //433.7400E6;   // Proposed frequency
boolean beacon = true;
int BeaconInterval = 60 ; // seconds
int TXDelay = 50 ; // wait for PTT relay if used
boolean flood = false; 


void setup() {
  Serial.begin(115200);
  Serial.println();

  preferences.begin("setup", false);
  preferences.putString("ssid", ssid); 
  preferences.putString("password", password);
  preferences.putString("MyCall", MyCall);
  preferences.putString("MyLocator", MyLocator);
  preferences.putString("BTName", BTName);
  preferences.putString("MyIP", MyIP);
  preferences.putLong("Frequency", frequency);
  preferences.putInt("Interval", BeaconInterval);
  preferences.putInt("TXDelay", TXDelay);
  preferences.putBool("Beacon", beacon);
  preferences.putBool("Flood", flood);

  Serial.println("Network Credentials Saved using Preferences");

  preferences.end();
}
void loop(){
  
}
