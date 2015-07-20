//Code sourced from Copyright 2013, bechter.com - All Rights Reserved
#include <Arduino.h>  
#include <OneWire.h>        //include the OneWire library
#include <DS2438.h>         //include the DS2438 library

// define the Arduino digital I/O pin to be used for the 1-Wire network here
const uint8_t ONE_WIRE_PIN = 3;                 //defined as pin 3

// define the 1-Wire addresses of the DS2438 battery monitors here (lsb first)
uint8_t DS2438_address1[] = { 0x26, 0x5B, 0xDD, 0xD6, 0x01, 0x00, 0x00, 0xA6 };       //device #1
uint8_t DS2438_address2[] = { 0x26, 0x69, 0xDD, 0xD6, 0x01, 0x00, 0x00, 0x25 };       //device #2
uint8_t DS2438_address3[] = { 0x26, 0x6D, 0xDD, 0xD6, 0x01, 0x00, 0x00, 0xF9 };       //device #3



//variables to hold setpoints
uint8_t wet1 = 3;
uint8_t dry1 = 2;
uint8_t wet2 = 3;
uint8_t dry2 = 2;
uint8_t wet3 = 3;
uint8_t dry3 = 2;

//variables to hold voltages and temperatures
uint8_t v1;
uint8_t t1;
uint8_t v2;
uint8_t t2;
uint8_t v3;
uint8_t t3;


OneWire ow(ONE_WIRE_PIN);                     //create onewire instance
DS2438 ds2438u1(&ow, DS2438_address1 );       //create DS2438 insteance for device 1
DS2438 ds2438u2(&ow, DS2438_address2 );       //create DS2438 insteance for device 2
DS2438 ds2438u3(&ow, DS2438_address3 );       //create DS2438 insteance for device 3

void setup() {
    Serial.begin(9600);                     //begin serial communications
    ds2438u1.begin();                       //begin unit 1
    ds2438u2.begin();                       //begin unit 2
    ds2438u3.begin();                       //begin unit 3
}

void loop() {                               //main operational loop
  
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
    Serial.print(v1);                                           //display voltage
    Serial.println("Temperature1= ");
    Serial.print(t1);                                           //display voltage        
    if(v1 > wet1)
    {
      //turn off associated valve
      Serial.println("Valve1 shut");                            //display valve state
    }
    if(v1 < dry1)
    {
      //turn on associated valve
      Serial.println("Valve1 open");                            //display valve state
    }
  delay(100);                                                   //brief delay
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
    Serial.print(v2);                                           //display voltage
    Serial.println("Temperature2= ");
    Serial.print(t2);                                           //display voltage        
    if(v2 > wet2)
    {
      //turn off associated valve
      Serial.println("Valve2 shut");                            //display valve state
    }
    if(v2 < dry2)
    {
      //turn on associated valve
      Serial.println("Valve2 open");                            //display valve state
    }
  delay(100);  
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
    Serial.print(v3);                                           //display voltage
    Serial.println("Temperature3= ");
    Serial.print(t3);                                           //display voltage        
    if(v3 > wet3)
    {
      //turn off associated valve
      Serial.println("Valve3 shut");                            //display valve state
    }
    if(v3 < dry3)
    {
      //turn on associated valve
      Serial.println("Valve3 open");                            //display valve state
    }
  delay(100);  
  }
}
