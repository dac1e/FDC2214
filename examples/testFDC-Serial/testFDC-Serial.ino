// This is example code to demonstrate the functionality of FDC2214 library
//
// There is no warranty to the code. There is no support, don't ask for it.
// Use or skip on your own responsibility.
// NOT ALL FEATURES ARE TESTED! 
//
// The code might get revisited at some point or it might not.
// The code does more than I need at the moment.
//
// Feel free to do whatever you want with it. No licensing BS. No limitations.
//
// Created by Harijs Zablockis, Intelitech, March 2018
//
// Supported chips: FDC2112, FDC2114, FDC2212, FDC2214
// Transmits data via serial - use SerialPlot to draw graphs
// 
// FDC2214 hardware configuration:
// Component value as in default circuit from datasheet. (18uH inductor and 33pF cap)
// 
// SD and ADDR pins tied to GND
// INTB pin not used
// 
// ARDUINO <--> FDC
// A4 <-------> SDA
// A5 <-------> SCL
// 
// !!!!!! Arduinos are mostly 5V. FDC chips are 3.3V, so either use 3.3V version of Arduino, like pro mini, or use level shifter on I2C bus.
//

// ### FDC
#include <Arduino.h>
#include <Wire.h>
#include "FDC2214.h"

FDC2214 capsense(FDC2214_I2C_ADDR_0, Wire); // Use FDC2214_I2C_ADDR_0

// ### Tell application the maximum of number of channels.
static constexpr size_t CHAN_COUNT_MAX = 4;


// ###
void setup() {
  
  // ### Start I2C 
  Wire.begin();
//  Wire.setClock(400000L);
  
  // ### Start serial
  Serial.begin(115200);
  Serial.println("\nFDC2214 test");
  
  // ### Start FDC

//  // Start FDC2214 with at max 2 channels init, external oscillator
//  const FDC2214_DEVICE device = capsense.begin(0x3, false, FDC2214_DEGLITCH_10Mhz, false, FDC2214_GAIN_1); //setup first two channels, don't stay in sleep mode, deglitch at 10MHz, external oscillator, gain=1

//  // Start FDC2214 with at max 4 channels init, internal oscillator
//  const FDC2214_DEVICE device = capsense.begin(0xF, false, FDC2214_DEGLITCH_10Mhz, true, FDC2214_GAIN_1); //setup all four channels, don't stay in sleep mode, deglitch at 10MHz, internal oscillator, gain=1

  // Start FDC2214 with at max 4 channels init, external oscillator
  const FDC2214_DEVICE device = capsense.begin(0xF, false, FDC2214_DEGLITCH_10Mhz, false, FDC2214_GAIN_1); //setup all four channels, don't stay in sleep mode, deglitch at 10MHz, external oscillator, gain=1

  if (device != FDC2214_DEVICE_INVALID) {
    Serial.println("Sensor OK");
  } else {
    Serial.println("Sensor Fail");
  }
}

// ### 
void loop() {
  unsigned long capa[CHAN_COUNT_MAX]; // variable to store data from FDC
  const size_t n = capsense.getChannelCount();
  for (int i = 0; i < n; i++){ // for each channel
    // ### read 28bit data
    capa[i]= capsense.getReading(i);//
    // ### Transmit data to serial in simple format readable by SerialPlot application.
    Serial.print(capa[i]);  
    if (i < n-1) Serial.print(", ");
    else Serial.println("");
  }
  // No point in sleeping
  //delay(100); 
}


