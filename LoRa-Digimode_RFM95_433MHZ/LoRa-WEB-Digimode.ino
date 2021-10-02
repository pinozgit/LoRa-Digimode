/*
   This program allows QSO's between 2 or more stations in the style of RTTY or PSK31 digimodes
   using LoRa modules.

   The interface can be the serial monitor of the Arduino IDE or the WEB interface at
   http://192.168.1.241/webserial  (for example)
   Change the IP address in accord to your needs.
   Change also in the code  the content of MyCall
   If you are testing on the same router, make sure to have different IP's in the 2 stations....
   and maybe, different calls.

   commands:  /list    lists the congiguration
              /edit    lets change the configuration and store it into EEPROM 
                       (Beacon ON/OFF and Beacon Intervall are not stored into EEPROM) 
              /ping    asks other stations for a control

   While in /edit you have the following commands: /cal  /loc /fre /ipa  /bea /int.
   /ipa not allowed in web mode
   /end terminates the edit mode
   
   73 de Pino ZP4KFX

   

*/
//  WEMOS D1 MINI ESP32   at 115200
//  ESP32 Dev Module




#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include "WebSerial.h"

#include <Ticker.h>  //Ticker Library for beacon
Ticker TXB;

#include <Preferences.h>  // For settings in EEPROM 
Preferences preferences;
  
//----------------------------------------------

#include <WiFi.h>
#include <AsyncTCP.h>

#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);


//#include "credentials-2.h"
//const char* ssid = mySSID;
//const char* password = myPASSWORD;

 String ssid ;
 String password ;


//-----------------------------------------------------------------
String MyCall = "" ; //   = "ZP4KFX";  //"US3MW" ;  // change it !
String MyLocator = ""; // = "GG14hq" ;
 
int TXDelay = 50 ; // wait for PTT relay if used

String Beacon ;  // = "LoRa Beacon " + MyCall + " " + MyLocator ;

String d = "";

// MISO  19
// MOSI  23
// SCK   18




#define ss 5       // NSS
#define rst 14     //  Reset
#define dio0 2     //  DIO0
//#define dio1 12    //  not used by now NOT use 12 is for boot

#define ledrx 36   //   RX     LED green
#define ledtx 39   //   TX     LED red



int spreadingFactor = 10;//
long  signalBandwidth = 250E3;//
int codingRateDenominator = 5;//
long  preambleLength = 8;
int syncWord = 18;
int gain = 0 ;

long frequency ; //  = 433.0E6    868.74E6 ;
const int pwr = 17;            // LoRa output power 2-17


String MyIP ;
boolean EditMode = false;
boolean EditModeWEB = false;

boolean beacon = true;
int BeaconInterval = 60 ; // seconds

//------------------------------------------------------------------

int getIpBlock(int index, String str) {
    char separator = '.';
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = str.length()-1;
  
    for(int i=0; i<=maxIndex && found<=index; i++){
      if(str.charAt(i)==separator || i==maxIndex){
          found++;
          strIndex[0] = strIndex[1]+1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
    }
    
    return found>index ? str.substring(strIndex[0], strIndex[1]).toInt() : 0;
}
//------------------------------------------------------------------
IPAddress str2IP(String str) {

    IPAddress ret( getIpBlock(0,str),getIpBlock(1,str),getIpBlock(2,str),getIpBlock(3,str) );
    return ret;

}

//-------------------------------------------------------------------
void TXBeacon()  {
   if (beacon) {
    Serial.println(Beacon);
    WebSerial.println(Beacon);
    digitalWrite(ledtx, HIGH);
    //delay(TXDelay); // if used for PTT 
    LoRa.beginPacket();
    LoRa.print(Beacon);
    LoRa.endPacket();
    digitalWrite(ledtx, LOW); 
   }
}

//-------------------------------------------------------------------
void List(){
     Serial.print("Call            "); Serial.println(MyCall);
     Serial.print("Locator         "); Serial.println(MyLocator);
     Serial.print("Frequency       "); Serial.println(frequency);
     Serial.print("IP Address      "); Serial.println(MyIP);
     Serial.print("Beacon          "); 
          if (beacon) {
     Serial.println("ON");}
     else {
     Serial.println("OFF"); 
     }
     Serial.print("Beacon Interval "); Serial.println(BeaconInterval);
     Serial.println("Type /end to exit Edit Mode ");
     Serial.println("Commands: /cal  /loc /fre /ipa  /bea /int");

     WebSerial.print("Call            "); WebSerial.println(MyCall);
     WebSerial.print("Locator         "); WebSerial.println(MyLocator);
     WebSerial.print("Frequency       "); WebSerial.println(String(frequency));
     WebSerial.print("IP Address      "); WebSerial.println(MyIP);
     WebSerial.print("Beacon          "); 
          if (beacon) {
     WebSerial.println("ON");}
     else {
     WebSerial.println("OFF"); 
     }
     WebSerial.print("Beacon Interval "); WebSerial.println(BeaconInterval);
     WebSerial.println("Type /end to exit Edit Mode ");
     WebSerial.println("Commands: /cal /loc /fre /ipa  /bea /int");
}

//------------------------------------------------------------------
void recvMsg(uint8_t *data, size_t len) {  // read input from WEB interface
 
  //String d = "";
  for (int i = 0; i < len; i++) {
    d += char(data[i]);
  } // d received
}


//-------------------------------------------------------------------
void Analyze_d(){
 // Analyze d for a valid command
      if (d !=""){
      int ind1 = d.indexOf(' ');
      String Command = d.substring(0, ind1);
      //int ind2 = readString.indexOf(' ', ind1+1 );
      String Value = d.substring(ind1+1);
      Serial.print(Command); Serial.print("  "); Serial.println(Value);
      WebSerial.print(Command); WebSerial.print("  "); WebSerial.println(Value);
      
       if (Command == "/cal") {  MyCall = Value; 
                                 preferences.putString("MyCall", MyCall); }
             
       else if (Command == "/loc") { MyLocator = Value;  
                                     preferences.putString("MyCall", MyCall);}

       else if (Command == "/fre") { frequency = Value.toInt() ;
                                      preferences.putLong("Frequency", frequency);
                                      LoRa.setFrequency(frequency); }

       else if (Command == "/ipa") { MyIP = Value  ; 
                                     preferences.putString("IP_add", MyIP);} // to be done: update the Bluetooth

       else if (Command == "/bea") { 
               if (Value == "on"){ beacon = true ; }
               else if (Value== "off"){ beacon = false ;}
               else { Serial.print("Valid values are on and off");}
               }

       else if (Command == "/int") { BeaconInterval = Value.toInt() ;  } 

       else  {   Serial.println("What ?"); 
                 WebSerial.println("What ?"); }
       Beacon = "LoRa Beacon " + MyCall + " " + MyLocator ;
       List();      
      }    
}

//-------------------------------------------------------------------
void EditWEB(){   // personalization from WEB interface
   TXB.detach(); // disable Beacon
   do { 
     if (d != "") { 
          if ( d == "/end"){ 
             EditModeWEB = false;
             WebSerial.println("EDIT MODE ended  ");
             d = "";
             }
          else if ( (d.indexOf("/ipa") >= 0)){ 
              WebSerial.println("Not allowed in Web mode"); // do not change the IP address in web mode
              d = ""; }
          else {
                 Analyze_d(); d = "" ;
          } 
     } 
   } while (EditModeWEB == true );   
   d = "";
   TXB.attach(BeaconInterval, TXBeacon );  
   }

//-------------------------------------------------------------------
void Edit(){   // personalization from Serial USB
   TXB.detach(); // disable Beacon
   do {
    d = Serial.readStringUntil('\n');
    if ( d == "/end"){ 
       EditMode = false;
       Serial.println("EDIT MODE ended  ");
    }
    else { 
          Analyze_d();
    }
    d = ""; 
   } while (EditMode == true );
  // d = "";
   TXB.attach(BeaconInterval, TXBeacon );
}



//-------------------------------------------------------------------
void PONG(String rxdata, int rssi, int snr) {
  
  if (rxdata.indexOf("/ping") != -1){ // respond to ping
    String pong = "pong de " + MyCall + " [RSSI: " + String(rssi) + " SNR: " + String(snr) + "]: 73"  ;
    TXB.detach(); // disable Beacon
          long rd = random(10, 3000); // it generate random numbers from min to max to avoid collisions
          Serial.print(" Random time "); Serial.println(rd);
          delay(rd);
    digitalWrite(ledtx, HIGH); 
    delay(TXDelay); // if used for PTT
    LoRa.beginPacket();
    LoRa.print(pong);
    LoRa.endPacket();
    digitalWrite(ledtx, LOW); 
    TXB.attach(BeaconInterval, TXBeacon );   // Enable Beacon
  }
}
//-------------------------------------------------------------------
void LoRaTX() {  // TX packet 
    digitalWrite(ledtx, HIGH); 
    LoRa.beginPacket();
    LoRa.print(MyCall + " > " + d);
    LoRa.endPacket();
    d = "";
    digitalWrite(ledtx, LOW); 
}
//********************************************************************************
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Digimode WEB");

  randomSeed(analogRead(0)); // randomize using noise from analog pin 0

  preferences.begin("setup", false);

  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  MyCall = preferences.getString("MyCall", "N0CALL");
  MyLocator = preferences.getString("MyLocator", "AA00aa");
  MyIP = preferences.getString("IP_add", "192.168.1.221");
  frequency = preferences.getLong("Frequency", 433.7400E6);  // select one
  //frequency = preferences.getLong("Frequency", 868.7400E6);
  
  Beacon = "LoRa Beacon " + MyCall + " " + MyLocator ;
 
  pinMode(ledrx, OUTPUT);
  pinMode(ledtx, OUTPUT);

  digitalWrite(ledrx, HIGH);
  delay(1000);
  digitalWrite(ledrx, LOW);
  digitalWrite(ledtx, HIGH);
  delay(1000);
  digitalWrite(ledtx, LOW);

   
 //IPAddress local_IP(192, 168, 1, 223);   // Local IP
  IPAddress local_IP(str2IP(MyIP));   // Local IP
  IPAddress gateway(192, 168, 1, 1);       // gateway of my network
  IPAddress subnet(255, 255, 255, 0);       // subnet mask of my network
  IPAddress dns(8, 8, 8, 8); // Google DNS
  WiFi.disconnect();
  delay(1200);
  WiFi.mode(WIFI_STA); // switch off AP
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long start = millis();
  uint8_t connectionStatus;
  bool   AttemptConnection = true;
  while (AttemptConnection) {
    connectionStatus = WiFi.status();
    if (millis() > start + 15000) { // Wait 15-secs maximum
      AttemptConnection = false;
    }
    if (connectionStatus == WL_CONNECTED || connectionStatus == WL_CONNECT_FAILED) {
      AttemptConnection = false;
    }
    delay(50);
  }
  if (connectionStatus == WL_CONNECTED) {
    Serial.println("WiFi connected at: " + WiFi.localIP().toString());
    Serial.print("WebSerial is accessible at http://" );
    Serial.print(WiFi.localIP());Serial.println("/webserial");
  }
  else {
    Serial.println("Wi-Fi Connexion Failed");
  }
  //    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //        Serial.printf("WiFi Failed!\n");
  //        return;
  //    }

//  Serial.print("IP Address: ");
//  Serial.println(WiFi.localIP());
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  server.begin();
  
  //---------------------настройка лора -----------------------------------------------
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
  while (!LoRa.begin(frequency))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
 
  LoRa.setSpreadingFactor(spreadingFactor);
  LoRa.setSignalBandwidth(signalBandwidth); // non gli piace
  LoRa.setCodingRate4(codingRateDenominator);
  LoRa.setPreambleLength(preambleLength);
  LoRa.setSyncWord(syncWord);
  LoRa.setTxPower(pwr);//17 LoRa output power (2-17)
  //  LoRa.setGain(gain);//почему нет ????
  TXB.attach(BeaconInterval, TXBeacon );
  Serial.println("LoRa Initializing OK!");

  Serial.println("Type  /edit to enter edit mode ");
  Serial.println("Type  /list to list the configuration ");
  Serial.println("Type  /ping to ask reports to other stations ");
  List();
  
}

//********************************************************************************
void loop() { 
  if (EditMode) { Serial.println("Edit Mode Started "); // Edit via USB serial
                  Edit(); }
  else if (EditModeWEB) { WebSerial.println("Edit Mode Started "); 
                  EditWEB(); }
  else {
      int packetSize = LoRa.parsePacket();    // try to parse packet from RX
      if (packetSize)
      {
        digitalWrite(ledrx, HIGH);
    
        //Serial.print("Received packet '");
        Serial.print("[RSSI: ");         // print RSSI of packet
        int rssi = LoRa.packetRssi();
        Serial.print(rssi);
        Serial.print(" SNR: ");
        int snr = LoRa.packetSnr();
        Serial.print(snr);
        Serial.print("]>");
    
        WebSerial.print("[RSSI: ");
        WebSerial.print(rssi);
        WebSerial.print(" SNR: ");
        WebSerial.print(snr);
        WebSerial.print("]>");
    
        while (LoRa.available())              // read packet
        {
          String LoRaData = LoRa.readString();
          Serial.print(LoRaData);
          WebSerial.print(LoRaData);
          PONG(LoRaData,rssi,snr) ; // if /ping found do a pong
        }
        Serial.println();
        WebSerial.println();
      }
        digitalWrite(ledrx, LOW);

      // Input from web interface
       if (d != "") {
             if (d.indexOf("/edit") >= 0) { EditModeWEB = true;
             Serial.println("Edit Mode ON"); 
             WebSerial.println("Edit Mode ON");
             d = "";
             List();
             }
      else if (d.indexOf("/list") >= 0) { List(); d = "";}
      else {
            WebSerial.println(MyCall + " > " + d);
            Serial.println(MyCall + " > " + d);
            LoRaTX();
           }
         } 

       
      // Input from serial interface
      if (Serial.available()) {
        d = Serial.readStringUntil('\n');
        if (d == "/edit") { EditMode = true; 
           Serial.println("Setting Mode ON"); 
           d = "";
            List();
          }
        else if (d.indexOf("/list") >= 0) { List(); d ="";}
        else {
          WebSerial.println(MyCall + " > " + d);
          Serial.println(MyCall + " > " + d);
          LoRaTX();
          }    
      }
      
  }

} // loop
