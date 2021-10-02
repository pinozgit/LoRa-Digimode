/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 06/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

#define NSS 5                                   //select pin on LoRa device
#define SCK 18                                  //SCK on SPI3
#define MISO 19                                 //MISO on SPI3 
#define MOSI 23                                 //MOSI on SPI3 

#define NRESET 14                               //reset pin on LoRa device
#define RFBUSY 25                               //busy line

#define LED1 2                                  //on board LED, high for on
#define DIO1 35                                 //DIO1 pin on LoRa device, used for RX and TX done 
#define SW -1                                   //SW pin on Dorji devices is used to turn RF switch on\off, set to -1 if not used    
#define BUZZER -1                               //pin for buzzer, set to -1 if not used 
#define VCCPOWER -1  //14                             //pin controls power to external devices

#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using

#define ledrx 2    //   RX     LED green
#define ledtx 26   //  TX      LED red

//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
 uint32_t frequency = 433740000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_250;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF10;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

const int8_t TXpower = 10;                      //LoRa transmit power in dBm

const uint16_t packet_delay = 5000;             //mS delay between packets
#define RXBUFFER_SIZE 100                        //RX buffer size  
