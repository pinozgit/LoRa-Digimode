# LoRa-Digimode
HAM Radio programs to use LoRa modules for keyboard-to-keyboard communications 

Here are some programs for using LoRa as a new Digimode for HAM Radio QSO's.

Al programs run on a ESP32 connected to a LoRa module or on integrated boards.

All programs can be controlled via the USB serial interface, moreover WEB programs
have a WEB interface and BT programs have a Bluetooth interface.

WEB programs are useful when the board is on the tower directly connected to the antenna.
A Wi-Fi router is needed.

Bluetooth programs are useful using a cellphone and the "Serial Bluetooth Terminal" app.

Also the "Serial USB Terminal" app can be used with the right USB cable.

The programs for the RFM95 module use the LoRa lib and have been tested on 433MHz and 868MHz.

The programs for the SX126XLT has been tested with the 1W 433MHz module.

The programs for the SX1280 work for the 2.4GHz modules.

Configurations settings are stored into EEPROM. For the first time use the Preferences_LoRa_Digimode
program to write your call and other setting into the EEPROM.

Some of these values can be changed in a second time with the command /edit

The LoRa-Digimode programs send a beacon message, by default every minute.

The /ping command can be used to ask for reports to other stations.

Please report bugs to Pino ZP4KFX   pinozollo@gmail.com or  https://telegram.me/zp4kfx  
