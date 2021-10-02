/* This program allows QSO's between 2 or more stations in the style of RTTY or PSK31 digimodes
   using LoRa modules.

   The interface can be the serial monitor of the Arduino IDE, a Cellphone with USB serial app and
   cable or a Cellphone with Bluetooth serial app. 
   

   Tested with the app's: "Serial USB Terminal" and with "Serial Bluetooth Terminal"
   Setting: Newline: CR+LF    ...  
   Send Newline: Auto, Clear input on send, Local Echo OFF
   
   Feel free to improve this program !

   commands:  /list    lists the congiguration
              /edit    lets change the configuration and store it into EEPROM 
                       (Beacon ON/OFF and Beacon Intervall are not stored into EEPROM) 
              /ping    asks other stations for a control
              
   While in /edit you have the following commands: /cal  /loc /fre /blu  /bea /int.
   
   /end terminates the edit mode

   73 de Pino ZP4KFX

   
 */
//  WEMOS D1 MINI ESP32   at 115200 baud
//  ESP32 Dev Module
//  TTGO T7 V1.3 Mini32  at 115200 baud

#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

#include <Ticker.h>  //Ticker Library
Ticker TXB;

#include <Preferences.h>
Preferences preferences;
  
//----------------------------------------------
String MyCall = "" ; //   = "ZP4KFX";  //"US3MW" ;  // change it !
String MyLocator = ""; // = "GG14hq" ;
 
int TXDelay = 50 ; // wait for PTT relay if used

String Beacon ;  // = "LoRa Beacon " + MyCall + " " + MyLocator ;

String d = "";

// MISO  19
// MOSI  23
// SCK   18

#define ss   5     // NSS
#define rst 14     //  Reset
#define dio0 2     //  DIO0
//#define dio1 12    //  not used by now NOT use 12 is for boot

#define ledrx 36   //   RX     LED green or buzer ...  To be modified in accord to your board
#define ledtx 39   //   TX     LED red  

int spreadingFactor = 10;//
long  signalBandwidth = 250E3;//
int codingRateDenominator = 5;//
long  preambleLength = 8;
int syncWord = 18;
int gain = 0 ;

long frequency  ; // = 433.7400E6;   // Proposed frequency
const int pwr = 17;            // LoRa output power 2-17

String BTName ; // = "ZP4KFX-LoRa"; //Bluetooth device name that you like

boolean EditMode = false;
boolean EditModeBT = false;
boolean beacon = true;
int BeaconInterval = 60 ; // seconds

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//-------------------------------------------------------------------
void Analyze_d(){
 // Analyze d for a valid command
      if (d !=""){
      int ind1 = d.indexOf(' ');
      String Command = d.substring(0, ind1);
      //int ind2 = readString.indexOf(' ', ind1+1 );
      String Value = d.substring(ind1+1);
      Serial.print(Command); Serial.print("  "); Serial.println(Value);
      SerialBT.print(Command); SerialBT.print("  "); SerialBT.println(Value);
      
       if (Command == "/cal") {  MyCall = Value; 
                                 preferences.putString("MyCall", MyCall); }
             
       else if (Command == "/loc") { MyLocator = Value;  
                                     preferences.putString("MyCall", MyCall);}

       else if (Command == "/fre") { frequency = Value.toInt() ;
                                      preferences.putLong("Frequency", frequency);
                                      LoRa.setFrequency(frequency); }

       else if (Command == "/blu") { BTName = Value  ; 
                                     preferences.putString("BTName", BTName);} // to be done: update the Bluetooth

       else if (Command == "/bea") { 
               if (Value == "on"){ beacon = true ; }
               else if (Value== "off"){ beacon = false ;}
               else { Serial.print("Valid values are on and off");}
               }

       else if (Command == "/int") { BeaconInterval = Value.toInt() ;  } 

       else  {   Serial.println("What ?"); 
                 SerialBT.println("What ?"); }
       Beacon = "LoRa Beacon " + MyCall + " " + MyLocator ;
       List();      
      }    
}
//-------------------------------------------------------------------
void Edit(){   // personalization
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
     
   } while (EditMode == true );
   d = "";
   TXB.attach(BeaconInterval, TXBeacon );
}

//-------------------------------------------------------------------
void EditBT(){   // personalization
   TXB.detach(); // disable Beacon
   do {
    d = SerialBT.readStringUntil('\n');
    if ( d == "/end"){ 
       EditModeBT = false;
       SerialBT.println("EDIT MODE ended  ");
    }
    else {
           Analyze_d();
    } 
     
   } while (EditModeBT == true ); 
   d = "";
   TXB.attach(BeaconInterval, TXBeacon );  
}
//-------------------------------------------------------------------
void List(){
     Serial.print("Call            "); Serial.println(MyCall);
     Serial.print("Locator         "); Serial.println(MyLocator);
     Serial.print("Frequency       "); Serial.println(frequency);
     Serial.print("Bluetooth       "); Serial.println(BTName);
     Serial.print("Beacon          "); 
          if (beacon) {
     Serial.println("ON");}
     else {
     Serial.println("OFF"); 
     }
     Serial.print("Beacon Interval "); Serial.println(BeaconInterval);
     Serial.println("Type /end to exit Edit Mode ");
     Serial.println("Commands: /cal  /loc /fre /blu  /bea /int");

     SerialBT.print("Call            "); SerialBT.println(MyCall);
     SerialBT.print("Locator         "); SerialBT.println(MyLocator);
     SerialBT.print("Frequency       "); SerialBT.println(frequency);
     SerialBT.print("Bluetooth       "); SerialBT.println(BTName);
     SerialBT.print("Beacon          "); 
          if (beacon) {
     SerialBT.println("ON");}
     else {
     SerialBT.println("OFF"); 
     }
     SerialBT.print("Beacon Interval "); SerialBT.println(BeaconInterval);
     SerialBT.println("Type /end to exit Edit Mode ");
     SerialBT.println("Commands: /cal  /loc /fre /blu  /bea /int");
}

//-------------------------------------------------------------------
void TXBeacon()  {
   if (beacon) {
    Serial.println(Beacon);
    SerialBT.println(Beacon);
    digitalWrite(ledtx, HIGH);
    //delay(TXDelay); // if used for PTT 
    LoRa.beginPacket();
    LoRa.print(Beacon);
    LoRa.endPacket();
    digitalWrite(ledtx, LOW); 
   }
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

//********************************************************************************
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Digimode Bluetooth");

  randomSeed(analogRead(0)); // randomize using noise from analog pin 0

  preferences.begin("setup", false);
  MyCall = preferences.getString("MyCall", "N0CALL");
  MyLocator = preferences.getString("MyLocator", "AA00aa");
  BTName = preferences.getString("BTName", "MyBT");
  //frequency = preferences.getLong("Frequency", 433.7400E6); // select one
  frequency = preferences.getLong("Frequency", 868.7400E6);
  
  Beacon = "LoRa Beacon " + MyCall + " " + MyLocator ;

  List(); Serial.println(" /edit to enter edit mode "); 
  
  pinMode(ledrx, OUTPUT);
  pinMode(ledtx, OUTPUT);

  digitalWrite(ledrx, HIGH);
  delay(1000);
  digitalWrite(ledrx, LOW);
  digitalWrite(ledtx, HIGH);
  delay(1000);
  digitalWrite(ledtx, LOW);

  SerialBT.begin(BTName); //Bluetooth device name that you like
  Serial.println("The device started, now you can pair it with bluetooth!");

  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
  while (!LoRa.begin(frequency))     //  433.740E6 proposed LoRa Digimode frequency 
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

    List();
  Serial.println("Type  /edit to enter edit mode ");
  Serial.println("Type  /list to list the configuration ");
  Serial.println("Type  /ping to ask reports to other stations ");

}

//*********************************************************************************
void loop() {
    if (EditMode) { Serial.println("Edit Mode Started "); 
                    Edit(); }
    else if (EditModeBT) { SerialBT.println("Edit Mode Started "); 
                    EditBT(); }
    else {
     // RX
     int packetSize = LoRa.parsePacket();    // try to parse packet
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
        
        SerialBT.print("[RSSI: ");    // Print to BlueTooth
        SerialBT.print(LoRa.packetRssi());
        SerialBT.print(" SNR: ");
        SerialBT.print(LoRa.packetSnr());
        SerialBT.print("]>");
        
        while (LoRa.available())              // read packet
             {
              String LoRaData = LoRa.readString();
              Serial.print(LoRaData);
              SerialBT.print((LoRaData));
              PONG(LoRaData,rssi,snr) ; // if /ping found do a pong
            }
    Serial.println();
    SerialBT.println();

    digitalWrite(ledrx, LOW);
     }

   // TX   
  // Input from serial interface
  if (Serial.available()) {
    d = Serial.readStringUntil('\n');
    if (d == "/edit") { EditMode = true; 
    Serial.println("Setting Mode ON"); 
    d = "";
    List();
    }
    else {
    SerialBT.println(MyCall + " > " + d);
    Serial.println(MyCall + " > " + d);
     }
  }

  // Input fron Bluetooth
  if (SerialBT.available()) {
     d = SerialBT.readStringUntil('\n');
      //if (d == "/edit") { EditMode = true; 
    if (d.indexOf("/edit") >= 0) { EditModeBT = true; 
    Serial.println("Setting Mode ON"); 
    SerialBT.println("Setting Mode ON");
    d = "";
    List();
    }
    else {
     SerialBT.println(MyCall + " > " + d);
     Serial.println(MyCall + " > " + d);
    }
  } 
     
  // Tx to the other LoRa station
  if (d != "") {
    TXB.detach(); // disable Beacon
    digitalWrite(ledtx, HIGH); 
    delay(TXDelay); // if used for PTT
    LoRa.beginPacket();
    LoRa.print(MyCall + " > " + d);
    LoRa.endPacket();
    d = "";
    digitalWrite(ledtx, LOW); 
    TXB.attach(BeaconInterval, TXBeacon );   // Enable Beacon
    }
    
  
  delay(20);
    }
}
