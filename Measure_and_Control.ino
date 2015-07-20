//Copyright 2015 Adam Koeppel

#include <Arduino.h>  
#include <OneWire.h>        //include the OneWire library
#include <DS2438.h>         //include the DS2438 library

// define the Arduino digital I/O pin to be used for the 1-Wire network here
const uint8_t ONE_WIRE_PIN = 3;                 //defined as pin 3

// define the 1-Wire addresses of the DS2438 battery monitors here (lsb first)
uint8_t DS2438_address1[] = { 0x26, 0x5B, 0xDD, 0xD6, 0x01, 0x00, 0x00, 0xA6 };       //device #1
uint8_t DS2438_address2[] = { 0x26, 0x69, 0xDD, 0xD6, 0x01, 0x00, 0x00, 0x25 };       //device #2
uint8_t DS2438_address3[] = { 0x26, 0x6D, 0xDD, 0xD6, 0x01, 0x00, 0x00, 0xF9 };       //device #3

// define the 1-Wire addresses of the DS2413 GPIOs here
uint8_t DS2413_address1[8] = { 0x3A, 0x2B, 0x26, 0x18, 0x00, 0x00, 0x00, 0x91 };       //device #1 (top)
uint8_t DS2413_address2[8] = { 0x3A, 0x29, 0x1E, 0x18, 0x00, 0x00, 0x00, 0x45 };       //device #2 (middle)
uint8_t DS2413_address3[8] = { 0x3A, 0xA7, 0x34, 0x18, 0x00, 0x00, 0x00, 0xF9 };       //device #3 (bottom)

//command shortcuts for DS2413
//#define DS2413_ACCESS_READ  0xF5
#define DS2413_ACCESS_WRITE 0x5A
//#define DS2413_ACK_SUCCESS  0xAA
//#define DS2413_ACK_ERROR    0xFF

//Channel B pulse closes valve
//Channel A pulse opens valve
//command shortcuts for DS2438
#define pulseClose 0x1
#define pulseOpen 0x2
#define pulseStandby 0x3

//variables to hold setpoints
uint8_t wet1 = 3;
uint8_t dry1 = 2;
uint8_t wet2 = 3;
uint8_t dry2 = 2;
uint8_t wet3 = 3;
uint8_t dry3 = 2;

//variables to hold voltages and temperatures
long v1;
long t1;
long v2;
long t2;
long v3;
long t3;

OneWire oneWire(ONE_WIRE_PIN);                     //create onewire instance
DS2438 ds2438u1(&oneWire, DS2438_address1 );       //create DS2438 insteance for device 1
DS2438 ds2438u2(&oneWire, DS2438_address2 );       //create DS2438 insteance for device 2
DS2438 ds2438u3(&oneWire, DS2438_address3 );       //create DS2438 insteance for device 3

void setup() 
{
  Serial.begin(9600);                     //begin serial communications
  ds2438u1.begin();                       //begin DS2438 1
  ds2438u2.begin();                       //begin DS2438 2
  ds2438u3.begin();                       //begin DS2438 3
  
  //close all valves to start
  write(pulseStandby, DS2413_address1);
  delay(500);
  write(pulseStandby, DS2413_address2);
  delay(500);
  write(pulseStandby, DS2413_address3);
  delay(500);
  
  write(pulseClose, DS2413_address1);
  delay(500);
  write(pulseStandby, DS2413_address1);
  delay(500);
  write(pulseClose, DS2413_address2);
  delay(500);
  write(pulseStandby, DS2413_address2);
  delay(500);
  write(pulseClose, DS2413_address3);
  delay(500);
  write(pulseStandby, DS2413_address3);
  delay(500);
  Serial.println("All Valves Closed");
  
  
//  close(DS2413_address1);                 //close valve 1
//  close(DS2413_address2);                 //close valve 2
//  close(DS2413_address3);                 //close valve 3
}

void loop()                             //main operational loop
{ 
  //unit 1 read and update
  ds2438u1.update();                                            //request update from DS2438 unit 1
  if (ds2438u1.isError()) 
  {
    Serial.println("Error reading from DS2438 device 1");       //error handling if device is disconnected
  } 
  else 
  {
    v1 = ds2438u1.getVoltage(DS2438_CHA);                       //set v1 to voltage
    t1 = ds2438u1.getTemperature();                             //set t1 to temperature
    Serial.println("Voltage1= ");
    Serial.println(v1);                                           //display voltage
    //make valve decision
    if(v1 > wet1)                                                  //need to close valve, soil is wet
    {
      write(pulseClose, DS2413_address1);  //call the write function to perform 0x0 to address
      delay(500);
      write(pulseStandby, DS2413_address1);   //call the write function to perform 0x1 to address
      delay(500);
//      close(DS2413_address1);                                   //close associated valve
      Serial.println("Valve1 shut");                            //display valve state
      Serial.println();
    }
    if(v1 < dry1)                                               //need to open valve, soil is dry
    {
      write(pulseOpen, DS2413_address1);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address1);   //call the write function to perform 0x2
      delay(500);
//      open(DS2413_address1);                                     //open associated valve
      Serial.println("Valve1 open");                            //display valve state
      Serial.println();
    }
//    Serial.println("Temperature1= ");
//    Serial.println(t1);                                           //display temperature
//    Serial.println();
//    Serial.println();        
  delay(1000);                                                   //brief delay
  }
    
  //unit 2 read and update
  ds2438u2.update();                                            //request update from DS2438 unit 2
  if (ds2438u2.isError())
  {
    Serial.println("Error reading from DS2438 device 2");       //error handling if device is disconnected
  } 
  else 
  {
    v2 = ds2438u2.getVoltage(DS2438_CHA);                       //set v2 to voltage
    t2 = ds2438u2.getTemperature();                             //set t2 to temperature
    Serial.println("Voltage2= ");
    Serial.println(v2);                                           //display voltage
    //make valve decision
    if(v2 > wet2)                                                  //need to close valve, soil is wet
    {
      write(pulseClose, DS2413_address2);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address2);   //call the write function to perform 0x1
      delay(500);
//      close(DS2413_address2);                                   //close associated valve
      Serial.println("Valve2 shut");                            //display valve state
      Serial.println();
    }
    if(v2 < dry2)                                               //need to open valve, soil is dry
    {
      write(pulseOpen, DS2413_address2);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address2);   //call the write function to perform 0x2
      delay(500);
//      open(DS2413_address2);                                    //open associated valve
      Serial.println("Valve2 open");                            //display valve state
      Serial.println();
    }
//    Serial.println("Temperature2= ");
//    Serial.println(t2);                                           //display temperature
//    Serial.println();
//    Serial.println();
  delay(1000);  
  }

  //unit 3 read and update
  ds2438u3.update();                                            //request update from DS2438 unit 3
  if (ds2438u3.isError())
  {
    Serial.println("Error reading from DS2438 device 3");       //error handling if device is disconnected
  } 
  else 
  {
    v3 = ds2438u3.getVoltage(DS2438_CHA);                       //set v3 to voltage
    t3 = ds2438u3.getTemperature();                             //set t3 to temperature
    Serial.println("Voltage3= ");
    Serial.println(v3);                                           //display voltage
    //make valve decision
    if(v3 > wet3)                                                  //need to close valve, soil is wet
    {
      write(pulseClose, DS2413_address3);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address3);   //call the write function to perform 0x1
      delay(500);
//      close(DS2413_address3);                                   //close associated valve
      Serial.println("Valve3 shut");                            //display valve state
      Serial.println();
    }
    if(v3 < dry3)                                               //need to open valve, soil is dry
    {
      write(pulseOpen, DS2413_address3);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address3);   //call the write function to perform 0x2
      delay(500);
//      open(DS2413_address3);                                    //open associated valve
      Serial.println("Valve3 open");                            //display valve state
      Serial.println();
    }
    //Serial.println("Temperature3= ");
    //Serial.println(t3);                                           //display temperature
    Serial.println();
    Serial.println();
  delay(1000);  
  }
    Serial.println("Temperature1= ");
    Serial.println(t1);                                           //display temperature
    Serial.println("Temperature2= ");
    Serial.println(t2);                                           //display temperature
    Serial.println("Temperature3= ");
    Serial.println(t3);                                           //display temperature
    Serial.println();
    Serial.println();
}

//function that writes a change to a DS2413
int write(uint8_t state, uint8_t address[8])    // pass in address of the DS2413 and the state the valve should be in
{
  oneWire.reset();
  oneWire.select(address);
  oneWire.write(DS2413_ACCESS_WRITE);
  oneWire.write(~state);
  oneWire.write(state);                         //Invert data and resend   WHY????????  WHY IS THIS NECESSARY???
  oneWire.reset();
}


/*
//function that closes a valve attached to a specific DS2413
int close(uint8_t address[8])              //just pass in address of the DS2413
{
  oneWire.reset();                         //reset to begin oneWire
  oneWire.select(address);                 //select the correct DS2413
  oneWire.write(DS2413_ACCESS_WRITE);      //access the write functionality
  //GPIO pin A will be the pin to control the closing of the valve. Turning off GPIO A pulls the h bridge high sending power to the valve
  oneWire.write(~0x0);                     //send the inverted data  WHY????????  WHY IS THIS NECESSARY???
  oneWire.write(0x0);                      //send the correct data  WHY????????  WHY IS THIS NECESSARY???
  delay(1000);                              //delay to allow the valve to close and latch
  //now write high to GPIO A to pull H bridge low and stop sending power to the valve
  oneWire.write(~0x1);                     //send the inverted data  WHY????????  WHY IS THIS NECESSARY???
  oneWire.write(0x1);                      //send the correct data  WHY????????  WHY IS THIS NECESSARY???
  oneWire.reset();                         //terminate the oneWire communication
}

//function that opens a valve attached to a specific DS2413
int open(uint8_t address[8])              //just pass in address of the DS2413
{
  oneWire.reset();                         //reset to begin oneWire
  oneWire.select(address);                 //select the correct DS2413
  oneWire.write(DS2413_ACCESS_WRITE);      //access the write functionality
  //GPIO pin B will be the pin to control the opening of the valve. Turning off GPIO B pulls the h bridge high sending power to the valve
  oneWire.write(~0x0);                     //send the inverted data  WHY????????  WHY IS THIS NECESSARY???
  oneWire.write(0x0);                      //send the correct data  WHY????????  WHY IS THIS NECESSARY???
  delay(1000);                              //delay to allow the valve to close and latch
  //now write high to GPIO A to pull H bridge low and stop sending power to the valve
  oneWire.write(~0x2);                     //send the inverted data  WHY????????  WHY IS THIS NECESSARY???
  oneWire.write(0x2);                      //send the correct data  WHY????????  WHY IS THIS NECESSARY???
  oneWire.reset();                         //terminate the oneWire communication
}
*/

