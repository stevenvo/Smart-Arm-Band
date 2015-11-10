//
// Smart-Arm-Band
//
// Description of the project
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author 		Steven Vo
// 				Steven Vo
//
// Date			11/9/15 11:41 PM
// Version		<#version#>
//
// Copyright	© Steven Vo, 2015
// Licence		<#licence#>
//
// See         ReadMe.txt for references
//


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#   include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#   include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#   include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#   include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#   include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(ESP8266) // ESP8266 specific
#   include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not defined
#endif // end IDE

// Include application, user and local libraries
//#include <spi4teensy3.h>
#include "Adafruit_BLE_UART.h"
#include "Adafruit_ssd1306syp.h"

// Prototypes


// Define variables and constants
/* ============================== */
/* ===== Bluetooth BLE UART ===== */
// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);


/* ============================== */
/* ======== SSD-1306 OLED ======= */
#define SDA_PIN 18
#define SCL_PIN 19
Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);


void draw_header(boolean clear, char txt[]){
   if (clear){
      display.clear();
   }
   display.setTextSize(2);
   display.setTextColor(BLACK, WHITE); // 'inverted' text
   display.setCursor(0,0);
   display.println(txt);
   display.update();
}

void draw_text(char* txt){
   //  if (clear){
   //    display.clear();
   //  }
   //  display.clear();
   draw_header(true, "SMART BAND");
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0,17);
   display.println(txt);
   display.update();
}

/**************************************************************************/
/*!
 Constantly checks for new events on the nRF8001
 */
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

int check_ble_status(){
   // Ask what is our current status
   aci_evt_opcode_t status = BTLEserial.getState();
   // If the status changed....
   if (status != laststatus) {
      // print it out!
      if (status == ACI_EVT_DEVICE_STARTED) {
         Serial.println(F("* Advertising started"));
         draw_text("* Advertising started");
      }
      if (status == ACI_EVT_CONNECTED) {
         Serial.println(F("* Connected!"));
         draw_text("* Connected!");
      }
      if (status == ACI_EVT_DISCONNECTED) {
         Serial.println(F("* Disconnected or advertising timed out"));
         draw_text("* Disconnected or advertising timed out");
      }
      // OK set the last status change to this one
      laststatus = status;
   }
}

void process_ble(){
   // Lets see if there's any data for us!
   if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
   }
   // OK while we still have something to read, get a character and print it out
   while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
   }
   
   // Next up, see if we have any data to get from the Serial console
   
   if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();
      
      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());
      
      //      draw_text(sendbuffersize);
      
      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");
      
      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
   }
}


void setup() {
   
   Serial.begin(9600); // for debugging
   Serial.println("SETUP");
   
   display.initialize();
   Serial.println(F("- SSD-1306 is ready"));
   draw_text("- SSD-1306 is ready");
   
   
   
   while(!Serial); // Leonardo/Micro should wait for serial init
   BTLEserial.setDeviceName("armband"); /* 7 characters max! */
   BTLEserial.begin();
   Serial.println(F("- BLE nRF8001 is ready"));
   draw_text("- BLE nRF8001 is ready");
   
   
}




void loop() {
   // put your main code here, to run repeatedly:
   // Tell the nRF8001 to do whatever it should be working on.
   BTLEserial.pollACI();
   
   int status = check_ble_status();
   
   if (status == ACI_EVT_CONNECTED) {
      process_ble();
   }
   
   //  draw_something();
}