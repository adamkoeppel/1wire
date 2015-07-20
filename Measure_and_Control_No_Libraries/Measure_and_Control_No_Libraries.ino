//Copyright 2015 Adam Koeppel

//#include <Arduino.h>  
#include <OneWire.h>        //include the OneWire library
//#include <DS2438.h>         //include the DS2438 library

//Below is text from the DS2438 library, via portions @ bechter.com via 

//first library excerpt is the .h 
#ifndef DS2438_h
#define DS2438_h

#include <Arduino.h>
#include <OneWire.h>

#define DS2438_TEMPERATURE_CONVERSION_COMMAND 0x44
#define DS2438_VOLTAGE_CONVERSION_COMMAND 0xb4
#define DS2438_WRITE_SCRATCHPAD_COMMAND 0x4e
#define DS2438_COPY_SCRATCHPAD_COMMAND 0x48
#define DS2438_READ_SCRATCHPAD_COMMAND 0xbe
#define DS2438_RECALL_MEMORY_COMMAND 0xb8
#define DS2438_PAGE_0 0x00

#define DS2438_CHA 0
#define DS2438_CHB 1

#define DS2438_MODE_CHA 0x01
#define DS2438_MODE_CHB 0x02
#define DS2438_MODE_TEMPERATURE 0x04

#define DS2438_TEMPERATURE_DELAY 10
#define DS2438_VOLTAGE_CONVERSION_DELAY 8

class DS2438 {
    public:
        DS2438(OneWire *ow, uint8_t *address);
        void begin(uint8_t mode=(DS2438_MODE_CHA | DS2438_MODE_CHB | DS2438_MODE_TEMPERATURE));
        void update();
        double getTemperature();
        float getVoltage(int channel=DS2438_CHA);
        boolean isError();
        unsigned long getTimestamp();
    private:
        OneWire *_ow;
        uint8_t *_address;
        uint8_t _mode;
        double _temperature;
        float _voltageA;
        float _voltageB;
        unsigned long _timestamp;
        boolean _error;
        boolean startConversion(int channel, boolean doTemperature);
        boolean selectChannel(int channel);
        void writePageZero(uint8_t *data);
        boolean readPageZero(uint8_t *data);
};

#endif

//second library excerpt is the .cpp file
DS2438::DS2438(OneWire *ow, uint8_t *address) {
    _ow = ow;
    _address = address;
};

void DS2438::begin(uint8_t mode) {
    _mode = mode & (DS2438_MODE_CHA | DS2438_MODE_CHB | DS2438_MODE_TEMPERATURE);
    _temperature = 0;
    _voltageA = 0.0;
    _voltageB = 0.0;
    _error = true;
    _timestamp = 0;
}

void DS2438::update() {
    uint8_t data[9];

    _error = true;
    _timestamp = millis();

    if (_mode & DS2438_MODE_CHA || _mode == DS2438_MODE_TEMPERATURE) {
        boolean doTemperature = _mode & DS2438_MODE_TEMPERATURE;
        if (!startConversion(DS2438_CHA, doTemperature)) {
            return;
        }
        if (!readPageZero(data))
            return;
        if (doTemperature) {
            _temperature = (double)(((((int16_t)data[2]) << 8) | (data[1] & 0x0ff)) >> 3) * 0.03125;
        }
        if (_mode & DS2438_MODE_CHA) {
            _voltageA = (((data[4] << 8) & 0x00300) | (data[3] & 0x0ff)) / 100.0;
        }
    }
    if (_mode & DS2438_MODE_CHB) {
        boolean doTemperature = _mode & DS2438_MODE_TEMPERATURE & !(_mode & DS2438_MODE_CHA);
        if (!startConversion(DS2438_CHB, doTemperature)) {
            return;
        }
        if (!readPageZero(data))
            return;
        if (doTemperature) {
            _temperature = (double)(((((int16_t)data[2]) << 8) | (data[1] & 0x0ff)) >> 3) * 0.03125;
        }
        _voltageB = (((data[4] << 8) & 0x00300) | (data[3] & 0x0ff)) / 100.0;
    }
    _error = false;
}

double DS2438::getTemperature() {
    return _temperature;
}

float DS2438::getVoltage(int channel) {
    if (channel == DS2438_CHA) {
        return _voltageA;
    } else if (channel == DS2438_CHB) {
        return _voltageB;
    } else {
        return 0.0;
    }
}

boolean DS2438::isError() {
    return _error;
}

unsigned long DS2438::getTimestamp() {
    return _timestamp;
}

boolean DS2438::startConversion(int channel, boolean doTemperature) {
    if (!selectChannel(channel))
        return false;
    _ow->reset();
    _ow->select(_address);
    if (doTemperature) {
        _ow->write(DS2438_TEMPERATURE_CONVERSION_COMMAND, 0);
        delay(DS2438_TEMPERATURE_DELAY);
        _ow->reset();
        _ow->select(_address);
    }
    _ow->write(DS2438_VOLTAGE_CONVERSION_COMMAND, 0);
    delay(DS2438_VOLTAGE_CONVERSION_DELAY);
    return true;
}

boolean DS2438::selectChannel(int channel) {
    uint8_t data[9];
    if (readPageZero(data)) {
        if (channel == DS2438_CHB)
            data[0] = data[0] | 0x08;
        else
            data[0] = data[0] & 0xf7;
        writePageZero(data);
        return true;
    }
    return false;
}

void DS2438::writePageZero(uint8_t *data) {
    _ow->reset();
    _ow->select(_address);
    _ow->write(DS2438_WRITE_SCRATCHPAD_COMMAND, 0);
    _ow->write(DS2438_PAGE_0, 0);
    for (int i = 0; i < 8; i++)
        _ow->write(data[i], 0);
    _ow->reset();
    _ow->select(_address);
    _ow->write(DS2438_COPY_SCRATCHPAD_COMMAND, 0);
    _ow->write(DS2438_PAGE_0, 0);
}

boolean DS2438::readPageZero(uint8_t *data) {
    _ow->reset();
    _ow->select(_address);
    _ow->write(DS2438_RECALL_MEMORY_COMMAND, 0);
    _ow->write(DS2438_PAGE_0, 0);
    _ow->reset();
    _ow->select(_address);
    _ow->write(DS2438_READ_SCRATCHPAD_COMMAND, 0);
    _ow->write(DS2438_PAGE_0, 0);
    for (int i = 0; i < 9; i++)
        data[i] = _ow->read();
    return _ow->crc8(data, 8) == data[8];
}
//end library code embded in program

// define the Arduino digital I/O pin to be used for the 1-Wire network here
const uint8_t ONE_WIRE_PIN = 13;                 //defined as pin 13

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
float v1;
float t1;
float v2;
float t2;
float v3;
float t3;

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
  Serial.println("Closing Valves");
  write(pulseStandby, DS2413_address1);
  delay(500);
  write(pulseStandby, DS2413_address2);
  delay(500);
  write(pulseStandby, DS2413_address3);
  delay(500);
  Serial.println("All Valves Standby");
  
  write(pulseClose, DS2413_address1);
  delay(500);
  write(pulseStandby, DS2413_address1);
  delay(500);
  Serial.println("Valve 1 Closed");
  write(pulseClose, DS2413_address2);
  delay(500);
  write(pulseStandby, DS2413_address2);
  delay(500);
  Serial.println("Valve 2 Closed");
  write(pulseClose, DS2413_address3);
  delay(500);
  write(pulseStandby, DS2413_address3);
  delay(500);
  Serial.println("Valve 3 Closed");
//  Serial.println("All Valves Closed");
  Serial.println();
  
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
    
    if(v1 > wet1)                                                  //need to close valve, soil is wet
    {
      write(pulseClose, DS2413_address1);  //call the write function to perform 0x0 to address
      delay(500);
      write(pulseStandby, DS2413_address1);   //call the write function to perform 0x1 to address
      delay(500);
//      close(DS2413_address1);                                   //close associated valve
      Serial.println("Valve1 shut");                            //display valve state
    }
    if(v1 < dry1)                                               //need to open valve, soil is dry
    {
      write(pulseOpen, DS2413_address1);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address1);   //call the write function to perform 0x2
      delay(500);
//      open(DS2413_address1);                                     //open associated valve
      Serial.println("Valve1 open");                            //display valve state
    }
  delay(1500);                                                   //brief delay
  Serial.println();
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
    
    if(v2 > wet2)                                                  //need to close valve, soil is wet
    {
      write(pulseClose, DS2413_address2);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address2);   //call the write function to perform 0x1
      delay(500);
//      close(DS2413_address2);                                   //close associated valve
      Serial.println("Valve2 shut");                            //display valve state
    }
    if(v2 < dry2)                                               //need to open valve, soil is dry
    {
      write(pulseOpen, DS2413_address2);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address2);   //call the write function to perform 0x2
      delay(500);
//      open(DS2413_address2);                                    //open associated valve
      Serial.println("Valve2 open");                            //display valve state
    }
  delay(1500);  
  Serial.println();
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
    
    if(v3 > wet3)                                                  //need to close valve, soil is wet
    {
      write(pulseClose, DS2413_address3);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address3);   //call the write function to perform 0x1
      delay(500);
//      close(DS2413_address3);                                   //close associated valve
      Serial.println("Valve3 shut");                            //display valve state
    }
    if(v3 < dry3)                                               //need to open valve, soil is dry
    {
      write(pulseOpen, DS2413_address3);  //call the write function to perform 0x0
      delay(500);
      write(pulseStandby, DS2413_address3);   //call the write function to perform 0x2
      delay(500);
//      open(DS2413_address3);                                    //open associated valve
      Serial.println("Valve3 open");                            //display valve state
    }
  delay(1500);  
  Serial.println();
  }
  Serial.println();
  Serial.println();
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





