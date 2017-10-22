int exerciseNr;

// Variables for Exercise 2 (Balansboll)
#include "NAxisMotion.h"
#include <Wire.h>
#include <RBL_nRF8001.h>
#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
NAxisMotion mySensor;

float a;
float time_start;
float time_stop;
float total_time;
float startTime;

void software_Reset() { // Restarts program from beginning but does not reset the peripherals and registers
  asm volatile ("  jmp 0");
}


void setup() {
    // Set your BLE Shield name here, max. length 10
  ble_set_name("MaMBa-1");
  
  // Init. and start BLE library.
  ble_begin();
  startTime = millis();

  I2C.begin();
  mySensor.initSensor();
  //Can be configured to other operation modes as desired
  mySensor.setOperationMode(OPERATION_MODE_NDOF);
  //The default is AUTO
  //Changing to manual requires calling the relevant
  //update functions prior to calling the read functions
  mySensor.setUpdateMode(MANUAL);
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  Serial.begin(9600);
}


void ble_write_float(float x){
    union {
        float f;
        unsigned char c[4];
    } data;

    data.f = x;
    ble_write(data.c[0]);
    ble_write(data.c[1]);
    ble_write(data.c[2]);
    ble_write(data.c[3]);
}

void loop() {
  if (millis() > startTime + 600000) {
    software_Reset();
  }
  if(ble_available() ) {
    exerciseNr = ble_read();
    Serial.println(exerciseNr);
    
  }
  //exerciseNr = Serial.parseInt();
  
  
  if (exerciseNr == 50) {
    Serial.println("start");
    time_start = millis();
  
    while(true) {
      mySensor.updateAccel();
      mySensor.updateLinearAccel();      
      mySensor.updateGravAccel();     
      mySensor.updateCalibStatus();

      a = sqrt(mySensor.readAccelX()*mySensor.readAccelX()+mySensor.readAccelY()*mySensor.readAccelY()+mySensor.readAccelZ()*mySensor.readAccelZ());
      if (a > 32) {
        time_stop = millis();
        total_time = (time_stop - time_start)/1000;
        //Serial.println(total_time);
        break;
      }  
    }
    Serial.println(total_time);
    ble_write_float(total_time);
    
  }

  // Send total_time via bluetooth
  ble_do_events();
  exerciseNr = 0;
}

