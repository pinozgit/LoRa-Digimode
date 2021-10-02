/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 06/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup. Some pins such as DIO2,
//DIO3, BUZZER may not be in used by this sketch so they do not need to be
//connected and should be set to -1.
/*
// Alternativa connessioni ESP32
//#define NSS 5                                   //select pin on LoRa device
//#define SCK 18                                  //SCK on SPI3
//#define MISO 19                                 //MISO on SPI3 
//#define MOSI 23                                 //MOSI on SPI3 
//
//#define NRESET 27                               //reset pin on LoRa device
//#define RFBUSY 25                               //busy line
*/

// Connessioni usate
//  MOSI  23
//  MISO  19
//  SCK   18
//  NSS    5

//#ifdef ESP32
#define NSS 5     //10                      //select pin on LoRa device
#define NRESET 14 // 9                    //reset pin on LoRa device
#define RFBUSY 32 //7                    //SX128X busy pin 
#define DIO1 34   //3                      //DIO1 pin on LoRa device, used for RX and TX done 
#define ledrx 2    //   RX     LED green
#define ledtx 26   //  TX      LED red
//#elif defined(ESP8266)
//#define NSS 15       // NSS
//#define NRESET 2       //  Reset
//#define DIO1 4      //  DIO0
//#define ledrx 5//   RX     LED green
//#define ledtx 16//  TX     LED red
//#endif



#define DIO2 -1                     //DIO2 pin on LoRa device, normally not used so set to -1 
#define DIO3 -1                     //DIO3 pin on LoRa device, normally not used so set to -1

#define RX_EN -1                    //pin for RX enable, used on some SX128X devices, set to -1 if not used
#define TX_EN -1                    //pin for TX enable, used on some SX128X devices, set to -1 if not used
#define BUZZER -1               //pin for BUZZER, set to -1 if not used 
#define LED1 2    //8                      //on board LED, high for on
#define LORA_DEVICE DEVICE_SX1280   //this is the device we are using 2.4GHz 

//LoRa Modem Parameters
 uint32_t frequency = 2445000000;           //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes  
const uint8_t Bandwidth = LORA_BW_0400;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate

const uint8_t TXpower = 10;                      //Power for transmissions in dBm

const uint16_t packet_delay = 1000;              //mS delay between packets

#define RXBUFFER_SIZE 100                        //RX buffer size 
