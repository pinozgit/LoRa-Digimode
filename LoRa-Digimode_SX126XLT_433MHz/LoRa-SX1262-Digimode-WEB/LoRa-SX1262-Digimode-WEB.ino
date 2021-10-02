/*
 * This program allows QSO's between 2 or more stations in the style of RTTY or PSK31 digimodes
   using LoRa modules.
   This version is for the 2.4GHz modules.

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

   The configuration is stored into EEPROM. For the first time setting use the companion program "Preferences_LoRa_Digimode"
   The hardware settings are into Settings.h ... edit in accord to your setup.
   
   73 de Pino ZP4KFX

 * 
 *  The 2.4GHz module is controlled by the following library
 *   https://github.com/StuartsProjects/SX12XX-LoRa
 * 
 */

//  WEMOS D1 MINI ESP32   at 115200
//  ESP32 Dev Module


 
#define Program_Version "V1.0"

#include <Arduino.h>
#include <SPI.h>                                 //the lora device is SPI based so load the SPI library
#include <SX126XLT.h>                             //include the appropriate library   
#include "Settings.h"                            //include the setiings file, frequencies, LoRa settings etc   
 

SX126XLT LT;                                     //create a library class instance called LT

uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                 //create the buffer that received packets are copied into

uint8_t RXPacketL;                               //stores length of packet received
int8_t  PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio of received packet
int32_t FreqError;                               // stores Frequency Error

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

 String ssid ;
 String password ;

//-----------------------------------------------------------------
String MyCall = "" ; //   = "ZP4KFX";  //"US3MW" ;  // change it !
String MyLocator = ""; // = "GG14hq" ;
 
int TXDelay = 50 ; // wait for PTT relay if used

String Beacon ;  // = "LoRa Beacon " + MyCall + " " + MyLocator ;
boolean SendBeacon = false ;

String d = ""; 

//uint8_t TXbuff[] = ""; // no more used

uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;

String MyIP ;
boolean EditMode = false;
boolean EditModeWEB = false;

boolean beacon = true;
int BeaconInterval = 60 ; // seconds
//------------------------------------------------------------------------------------------------------------------------------
void printElapsedTime();
void  LoRaTX(String);
void TXBeacon();
//------------------------------------------------------------------------------------------------------------------------------
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
IPAddress str2IP(String str) {  // String to IP

    IPAddress ret( getIpBlock(0,str),getIpBlock(1,str),getIpBlock(2,str),getIpBlock(3,str) );
    return ret;
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

       else if (Command == "/fre") {   long  fre = strtoul( Value.c_str(), NULL, 10) ;  
                                       frequency = fre ;
                                       preferences.putLong("Frequency", frequency);
                                       LT.setRfFrequency(frequency, Offset);
                                      }

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
void TXBeacon()  { // Tick routine... TX removed from here and now in loop
   if (beacon) {
    SendBeacon = true; 
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
    LoRaTX(pong);
    digitalWrite(ledtx, LOW); 
    TXB.attach(BeaconInterval, TXBeacon );   // Enable Beacon
  }
}
//-------------------------------------------------------------------
void TXpacket_is_OK()
{
  //if here packet has been sent OK
//  uint16_t localCRC;
//
//  Serial.print(F("  BytesSent,"));
//  Serial.print(TXPacketL);                             //print transmitted packet length
// // localCRC = LT.CRCCCITT(TXbuff, TXPacketL, 0xFFFF);
//  Serial.print(F("  CRC,"));
//  Serial.print(localCRC, HEX);                              //print CRC of sent packet
//  Serial.print(F("  TransmitTime,"));
//  Serial.print(endmS - startmS);                       //print transmit time of packet
//  Serial.print(F("mS"));
//  Serial.print(F("  PacketsSent,"));
//  Serial.println(TXPacketCount);                         //print total of packets sent OK
}

//------------------------------------------------------------------------------------------------------------------------------
void TXpacket_is_Error()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                  //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                         //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                    //print IRQ status
  LT.printIrqStatus();
  Serial.println();
  //prints the text of which IRQs set
}

//-------------------------------------------------------------------
void LoRaTX(String Message ) {  // TX a string
    digitalWrite(ledtx, HIGH); 

        uint8_t dataArray[1 + Message.length()];  // convert the string into an array of uint8_t
        Message.getBytes(dataArray, 1 + Message.length());
     
    TXPacketL = sizeof(dataArray) - 1 ; // without -1 sends an extra ^@  mistery
  //  digitalWrite(LED1, HIGH);  // ???
    startmS =  millis();                                         //start transmit timer     
      if (LT.transmit(dataArray, TXPacketL, 10000, TXpower, WAIT_TX)) //will return packet length sent if OK, otherwise 0 if transmit, timeout 10 seconds
      {
        endmS = millis();                                          //packet sent, note end time
        TXPacketCount++;
        TXpacket_is_OK();
      }
      else
      {
        TXpacket_is_Error();                                 //transmit packet returned 0, there was an error
      }

 // digitalWrite(LED1, LOW); // ???

    d = "";
    digitalWrite(ledtx, LOW); 
}
//------------------------------------------------------------------------------------------------------------------------------
void RXpacket_is_OK()
{
//  uint16_t IRQStatus, localCRC;
//
//  IRQStatus = LT.readIrqStatus();                  //read the LoRa device IRQ status register
//
//  RXpacketCount++;

//  printElapsedTime();                              //print elapsed time to Serial Monitor
//  Serial.print(F("  "));
//  LT.printASCIIPacket(RXBUFFER, RXPacketL);        //print the packet as ASCII characters

//  localCRC = LT.CRCCCITT(RXBUFFER, RXPacketL, 0xFFFF);  //calculate the CRC, this is the external CRC calculation of the RXBUFFER
//  Serial.print(F(",CRC:"));                        //contents, not the LoRa device internal CRC
//  Serial.print(localCRC, HEX);
//  Serial.print(F(",RSSI:"));
//  Serial.print(PacketRSSI);
//  Serial.print(F("dBm,SNR:"));
//  Serial.print(PacketSNR);
//  Serial.print(F("dB,Length:"));
//  Serial.print(RXPacketL);
//  Serial.print(F(",Packets:"));
//  Serial.print(RXpacketCount);
//  Serial.print(F(",Errors:"));
//  Serial.print(errors);
//  Serial.print(F(",IRQreg:"));
//  Serial.print(IRQStatus, HEX);
//  Serial.print(F(" ,FErr:"));
//  Serial.print(FreqError);

digitalWrite(LED1, HIGH); 
   //Serial.print("Received packet '");
                  Serial.print("[RSSI: ");         // print RSSI of packet
                  Serial.print(PacketRSSI);
                  Serial.print(" SNR: ");
                  Serial.print(PacketSNR);
                  Serial.print("]>");
              
                  WebSerial.print("[RSSI: ");
                  WebSerial.print(PacketRSSI);
                  WebSerial.print(" SNR: ");
                  WebSerial.print(PacketSNR);
                  WebSerial.print("]>");
    //LT.printASCIIPacket(RXBUFFER, RXPacketL);        //print the packet as ASCII characters
    String str = (char*)RXBUFFER    ;  // received string
    Serial.print(str);
    WebSerial.print(str);
    PONG(str,PacketRSSI,PacketSNR) ; // if /ping found do a pong

   // azzerare il BUFFER !
   for (int i = 0; i <= RXBUFFER_SIZE; i++) { RXBUFFER[i] = (char)0; }
    
  digitalWrite(LED1, LOW); 

    Serial.println();
    WebSerial.println();
}


//------------------------------------------------------------------------------------------------------------------------------
void RXpacket_is_Error()
{
//  uint16_t IRQStatus;
//  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register
//
//  printElapsedTime();                               //print elapsed time to Serial Monitor
//
//  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
//  {
//    Serial.print(F(" RXTimeout")); 
//  }
//  else
//  {
//    errors++;
//    Serial.print(F(" PacketError"));
//    Serial.print(F(",RSSI,"));
//    Serial.print(PacketRSSI);
//    Serial.print(F("dBm,SNR,"));
//    Serial.print(PacketSNR);
//    Serial.print(F("dB,Length,"));
//    Serial.print(LT.readRXPacketL());               //get the real packet length
//    Serial.print(F(",Packets,"));
//    Serial.print(RXpacketCount);
//    Serial.print(F(",Errors,"));
//    Serial.print(errors);
//    Serial.print(F(",IRQreg,"));
//    Serial.print(IRQStatus, HEX);
//    LT.printIrqStatus();                            //print the names of the IRQ registers set
//
//    WebSerial.print(F(" Packet Error "));
//    WebSerial.print(F(",RSSI,"));
//    WebSerial.print(PacketRSSI);
//    WebSerial.print(F("dBm,SNR,"));
//    WebSerial.println(PacketSNR);
//  }
//
//  delay(250);                                       //gives a longer buzzer and LED flash for error 
//  
}

//------------------------------------------------------------------------------------------------------------------------------
void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}

//------------------------------------------------------------------------------------------------------------------------------
void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}

//****************************************************************************************************
void setup()
{
  pinMode(LED1, OUTPUT);                        //setup pin as output for indicator LED
  led_Flash(2, 125);                            //two quick LED flashes to indicate program start

    randomSeed(analogRead(0)); // randomize using noise from analog pin 0

  Serial.begin(115200);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();
  Serial.println(F("LoRa Digimode SX1262 starting"));
  Serial.println();

  if (BUZZER > 0)
  {
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, HIGH);
    delay(50);
    digitalWrite(BUZZER, LOW);
  }


  preferences.begin("setup", false);

  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  MyCall = preferences.getString("MyCall", "N0CALL");
  MyLocator = preferences.getString("MyLocator", "AA00aa");
  MyIP = preferences.getString("IP_add", "192.168.1.224");
  frequency = preferences.getLong("Frequency", 433.7400E6);  // select one
  //frequency = preferences.getLong("Frequency", 868.7400E6);
  //frequency = preferences.getLong("Frequency", 2445.0E6);
  
  Beacon = "LoRa Beacon " + MyCall + " " + MyLocator ;

  //IPAddress local_IP(192, 168, 1, 225);   // Local IP
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
  
//  Serial.print("Connecting to WiFi ..");
//  while (WiFi.status() != WL_CONNECTED) {
//    Serial.print('.');
//    delay(1000);
//  }

  unsigned long start = millis();
  uint8_t connectionStatus;
  bool   AttemptConnection = true;
  
  while (AttemptConnection) {
    connectionStatus = WiFi.status();
    if (millis() > start + 25000) { // Wait 15-secs maximum
      AttemptConnection = false;
    }
    if (connectionStatus == WL_CONNECTED || connectionStatus == WL_CONNECT_FAILED) {
      AttemptConnection = false;
    }
    delay(100);
  }
  
  //if (WL_CONNECTED) {   // connectionStatus == 
  if (connectionStatus == WL_CONNECTED) {   //  
    Serial.println("WiFi connected at: " + WiFi.localIP().toString());
    Serial.print("WebSerial is accessible at http://" );
    Serial.print(WiFi.localIP());Serial.println("/webserial");
  }
  else {
    Serial.println("Wi-Fi Connexion Failed");
  }

  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  server.begin();
  
  SPI.begin();

  //SPI beginTranscation is normally part of library routines, but if it is disabled in the library
  //a single instance is needed here, so uncomment the program line below
  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found
  ////if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, SW, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125); 
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                       //long fast speed LED flash indicates device error
    }
  }

  //The function call list below shows the complete setup for the LoRa device using the information defined in the
  //Settings.h file.
  //The 'Setup LoRa device' list below can be replaced with a single function call;
 //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  //***************************************************************************************************
  //Setup LoRa device
  //***************************************************************************************************
  LT.setMode(MODE_STDBY_RC);
  LT.setRegulatorMode(USE_DCDC);
  LT.setPaConfig(0x04, PAAUTO, LORA_DEVICE);
  LT.setDIO3AsTCXOCtrl(TCXO_CTRL_3_3V);
  LT.calibrateDevice(ALLDevices);                //is required after setting TCXO
  LT.calibrateImage(frequency);
  LT.setDIO2AsRfSwitchCtrl();
  LT.setPacketType(PACKET_TYPE_LORA);
  LT.setRfFrequency(frequency, Offset);
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, Optimisation);
  LT.setBufferBaseAddress(0, 0);
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  LT.setHighSensitivity();  //set for maximum gain
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
  //***************************************************************************************************


  Serial.println();
  LT.printModemSettings();                                     //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                                 //reads and prints the configured operting settings, useful check
  Serial.println();
  Serial.println();
 // LT.printRegisters(0x900, 0x9FF);                             //print contents of device registers
  Serial.println();
  Serial.println();

  Serial.print(F("Receiver ready - RXBUFFER_SIZE "));
  Serial.println(RXBUFFER_SIZE);
  Serial.println();

  TXB.attach(BeaconInterval, TXBeacon );
  
  Serial.println(F("Type  /edit to enter edit mode "));
  Serial.println(F("Type  /list to list the configuration "));
  Serial.println(F("Type  /ping to ask reports to other stations "));
  List();
}

//*************************************************************************************************************
void loop(){
   if (EditMode) { Serial.println("Edit Mode Started "); // Edit via USB serial
                  Edit(); }
  else if (EditModeWEB) { WebSerial.println("Edit Mode Started "); 
                  EditWEB(); }
  else {
        RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 600, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout
      
        PacketRSSI = LT.readPacketRSSI();              //read the recived RSSI value
        PacketSNR = LT.readPacketSNR();                //read the received SNR value
        FreqError = LT.getFrequencyErrorHz();
      
        if (RXPacketL == 0)                            //if the LT.receive() function detects an error, RXpacketL == 0
        {
          RXpacket_is_Error();
        }
        else
        {
              if (BUZZER > 0)                                //turn buzzer on
            {
              digitalWrite(BUZZER, HIGH);
            }
            
          RXpacket_is_OK();
          
              if (BUZZER > 0)
            {
              digitalWrite(BUZZER, LOW);                    //buzzer off
            }
        }
      

        if (SendBeacon) {
          Serial.println(Beacon);
          WebSerial.println(Beacon);
          digitalWrite(ledtx, HIGH); // LED and relay
          delay(TXDelay); // if used for PTT 
          LoRaTX(Beacon);
          digitalWrite(ledtx, LOW); 
          SendBeacon = false; 
        }
////////////////////////////////
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
            LoRaTX(MyCall + " > " + d);
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
          LoRaTX(MyCall + " > " + d);
          }    
      }
  } //else
}// loop
