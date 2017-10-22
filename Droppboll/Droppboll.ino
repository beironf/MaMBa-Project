// Common variables
#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
#include <RBL_nRF8001.h>

int exerciseNr;
int message;

// Pins
#define SENSORPIN_1 4
#define SENSORPIN_2 5
int SOLENOID_1 = 6;
int SOLENOID_2 = 7;

// Variables Droppboll:
int sensorState_1 = 0; 
int sensorState_2 = 0; 
float time_start = 0; 
float time_stop = 0; 
float total_time = 0;
bool exercise_started = false;
float starting_time;   // Is used if exercise has started but there is no ball 


void setup() {
  Serial.begin(9600);
  // Set your BLE Shield name here, max. length 10
  ble_set_name("MaMBa-2");
  
  // Init. and start BLE library.
  ble_begin();

  //Exercise 3
  // initialize the solenoid pin as an output:
  pinMode(SOLENOID_1, OUTPUT);   
  //pinMode(SOLENOID_2, OUTPUT);
  digitalWrite(SOLENOID_1, LOW);
  //digitalWrite(SOLENOID_2, LOW);
  // initialize the sensor pin as an input:
  pinMode(SENSORPIN_1, INPUT);    
  pinMode(SENSORPIN_2, INPUT); 
  digitalWrite(SENSORPIN_1, HIGH); // turn on the pullup
  digitalWrite(SENSORPIN_2, HIGH); // turn on the pullup
  Serial.println("Setup done");
}


void loop(){

  if (ble_available() ) {
    message = ble_read(); 
  }

  if (message == 50) {
    Serial.println("Solenoid should start");
    //digitalWrite(SOLENOID_1, HIGH);     
    delay(100);
    //digitalWrite(SOLENOID_1, LOW);
    //delay(300);
    //digitalWrite(SOLENOID_2, HIGH);
    //delay(100);
    //digitalWrite(SOLENOID_2, LOW);
    exercise_started = false;
    starting_time = millis();
    message = 51; // Message = 3 if exercise is running
  } 
  if (message == 51) {
    sensorState_1 = digitalRead(SENSORPIN_1);
    sensorState_2 = digitalRead(SENSORPIN_2);
    
    if (millis() - starting_time > 2000 && !exercise_started) { // Is used if there is no ball
      total_time = 0;
      message = 53; 
    }
        
    // check if the first sensor beam is broken
    if (sensorState_1 == LOW && !exercise_started) {     
      time_start = millis();  
      exercise_started = true;
      Serial.println("Started");
    }

    // Check if max time is reached
    if (exercise_started && millis() - time_start > 15000) { 
      total_time = 15;
      exercise_started = false;
      message = 53;
    }
        
     // check if the second sensor beam is broken
    if (sensorState_2 == LOW && exercise_started) {
        time_stop = millis();
        total_time = (time_stop - time_start)/1000;
        Serial.println(total_time);
          
        // Exercise has ended
        exercise_started = false;
        message = 53;
    }
  }
  if (message == 52) { // Exercise is stopped
    total_time = 15;
    exercise_started = false;
    message = 53;
  } 
  if (message == 53) {
    ble_write_float(total_time);
    message = 0;
  }
  ble_do_events();
}


 


// METHODS

// Send a float through BLE
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



