#ifndef _MULTIPLEX_SELECTOR_H
#define _MULTIPLEX_SELECTOR_H
#include "multiplexSelector.h"
#endif
#ifndef _MULTIPLEX_BUS_H
#define _MULTIPLEX_BUS_H
#include "multiplexBus.h"
#endif 

#define SMOOTH_READ_FACTOR 10

//configure inputs and CCs
#define USE_NRPN true
#define MIDI_CHANNEL 1
#define NUMBER_OF_POTS 20
#define NUMBER_OF_SWITCHES 13
#define NUMBER_OF_EXTERNAL_SWITCHES 6
#define NUMBER_OF_ROTARY_SWITCHES 6
#define NUMBER_OF_ROTARY_SWITCH_PINS 5
const int analogPins[3] = {A0, A1, A2};    // select the input pin for the potentiometer
const int potCCs[NUMBER_OF_POTS] = {90, 5, 23, 12, 13, 75, 76, 77, 79, 78, 74, 20, 73, 71, 21, 72, 70, 22, 15, 7};
const int potNRPNs[NUMBER_OF_POTS] = {50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69};
const int digitalPins[NUMBER_OF_SWITCHES] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
const int externalSwitchPins[NUMBER_OF_EXTERNAL_SWITCHES] = {8, 9, 10, 11, 12, 13 };
const int externalSwitchCCs[NUMBER_OF_EXTERNAL_SWITCHES] = { 25, 84, 69, 65, 24, 24 };
const int switchCCs[NUMBER_OF_SWITCHES] = {85, 89, 80, 83, 81, 26, 82, 86, 87, 88, 67, 102, 103};
const int rotarySwitchCCs[NUMBER_OF_ROTARY_SWITCHES] = { 92, 93, 95, 16, 17, 18 };
const int rotarySwitchPins[NUMBER_OF_ROTARY_SWITCH_PINS] = { 46, 45, 44, 43, 42 };

const double NRPN_SCALING_FACTOR = (double)16383 / (double)1023;

//pot data arrays
int lastPotValues[NUMBER_OF_POTS];
unsigned long potTotals[NUMBER_OF_POTS];
int potReadCount[NUMBER_OF_POTS];
unsigned long potReadMillis[NUMBER_OF_POTS];

//switch data arrays
int lastSwitchValues[NUMBER_OF_SWITCHES];
int lastExternalSwitchValues[NUMBER_OF_EXTERNAL_SWITCHES];
int lastRotarySwitchValues[NUMBER_OF_ROTARY_SWITCHES * 6];

void setup() {  
  //  Set MIDI baud rate:
  Serial.begin(31250);
  //Serial.begin(115200);
  
  //Init switch pins
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    pinMode(digitalPins[i], INPUT);  
  }
  
  for (int i = 0; i < NUMBER_OF_EXTERNAL_SWITCHES; i++)
  {
    pinMode(externalSwitchPins[i], INPUT_PULLUP);
  }
  
  for (int i = 0; i < NUMBER_OF_ROTARY_SWITCH_PINS; i++)
  {
    pinMode(rotarySwitchPins[i], INPUT_PULLUP);
  }
  
  pinMode(50, OUTPUT);    // pots s0
  pinMode(51, OUTPUT);    // pots s1
  pinMode(52, OUTPUT);    // pots s2

  pinMode(47, OUTPUT);    // rotary switches s0
  pinMode(48, OUTPUT);    // rotary switches s1
  pinMode(49, OUTPUT);    // rotary switches s2  
  
  //Init arrays
  for (int i = 0; i < NUMBER_OF_POTS; i++)
  {  
    lastPotValues[i] = -1;
    potTotals[i] = 0;
    potReadCount[i] = 0;
    potReadMillis[i] = 0;
  }
  
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    lastSwitchValues[i] = -1;  
  }
  
  for (int i = 0; i < NUMBER_OF_EXTERNAL_SWITCHES; i++)
  {
    lastExternalSwitchValues[i] = -1;  
  }
  
  //Init rotary switch data array so that values are sent at startup
  for (int i = 0; i < NUMBER_OF_ROTARY_SWITCHES * 6; i++)
  {
    lastRotarySwitchValues[i] = -1;
  }
}

void loop() 
{   
  readPots();
  readSwitches();
  readRotarySwitches();
}

void readRotarySwitches()
{
  multiplexBus pins = {47, 48, 49};
  
  for (int i = 0; i < NUMBER_OF_ROTARY_SWITCHES * 6; i++)
  {    
    int pin = i / 8;    
    int multiplexIndex = i % 8;
    multiplexSelect(multiplexIndex, pins);
    int sensorValue = digitalRead(rotarySwitchPins[pin]);
    if (sensorValue != lastRotarySwitchValues[i])
    {
      lastRotarySwitchValues[i] = sensorValue;

      if (sensorValue == LOW)
      {     
        int rotarySwitch = i / 6;
        int pos = i % 6;
        int value = pos * 127 / 5;      
        MIDI_TX(0xB0 + MIDI_CHANNEL - 1, rotarySwitchCCs[rotarySwitch], value);
      }
    } 
  }
}

multiplexSelector getSelector(int selectorIndex)
{
  multiplexSelector selector = {bitRead(selectorIndex,0), bitRead(selectorIndex,1), bitRead(selectorIndex,2)};
  return selector;
}

void writeSelector(multiplexSelector selector, multiplexBus pins)
{
  digitalWrite(pins.s0pin, selector.r0);
  digitalWrite(pins.s1pin, selector.r1);
  digitalWrite(pins.s2pin, selector.r2);
}

void multiplexSelect(int selectorIndex, multiplexBus pins)
{
  multiplexSelector selector = getSelector(selectorIndex);
  writeSelector(selector, pins);
}

void readSwitches()
{
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    int sensorValue = digitalRead(digitalPins[i]);
    if (sensorValue != lastSwitchValues[i])
    {
      lastSwitchValues[i] = sensorValue;
      
      if (switchCCs[i] == 102)
      {
        MIDI_NRPN(1, sensorValue * 16383);
      }
      else
      {      
        MIDI_TX(0xB0 + MIDI_CHANNEL - 1, switchCCs[i], sensorValue * 127);
      }
    }    
  }
  
  //External switches
  for (int i = 0; i < NUMBER_OF_EXTERNAL_SWITCHES - 2; i++)
  {
    boolean sensorValue = digitalRead(externalSwitchPins[i]);
    if (sensorValue != lastExternalSwitchValues[i])
    {
      lastExternalSwitchValues[i] = sensorValue;
      MIDI_TX(0xB0 + MIDI_CHANNEL - 1, externalSwitchCCs[i], !sensorValue * 127);      
    }    
  }
  
  int polyModeSwitchValue = digitalRead(externalSwitchPins[4]);
  int legatoModeSwitchValue = digitalRead(externalSwitchPins[5]);
  
  if (polyModeSwitchValue != lastExternalSwitchValues[4] || legatoModeSwitchValue != lastExternalSwitchValues[5])
  {
    lastExternalSwitchValues[4] = polyModeSwitchValue;
    lastExternalSwitchValues[5] = legatoModeSwitchValue;
    
    int maxValue = 127;
    if (legatoModeSwitchValue == LOW)
    {
      maxValue = 64;
    }
    
    MIDI_TX(0xB0 + MIDI_CHANNEL - 1, externalSwitchCCs[4], polyModeSwitchValue * maxValue);
  }
}

void readPots()
{
  multiplexBus pins = {50, 51, 52};
  
  for (int i = 0; i < NUMBER_OF_POTS; i++)
  {
    unsigned long currentMillis = millis();    
    
    if (currentMillis != potReadMillis[i])
    {    
      int pin = i / 8;
      
      int multiplexIndex = i % 8;
      
      multiplexSelect(multiplexIndex, pins);
      double reading = analogRead(analogPins[pin]);
      int sensorValue;
      
      if (USE_NRPN)
      {
        sensorValue = (int)(reading * NRPN_SCALING_FACTOR);
      }
      else
      {
        double fraction = reading / 8.055 + 0.5;
        sensorValue = fraction;        
      }      
      potTotals[i] = potTotals[i] + sensorValue;
      potReadMillis[i] = millis();            
      potReadCount[i]++;
      
      if (potReadCount[i] == SMOOTH_READ_FACTOR)
      {
        int average = ((double) potTotals[i] / SMOOTH_READ_FACTOR) + 0.5;        
        if (average != lastPotValues[i])
        {    
          lastPotValues[i] = average;        
          
          if (USE_NRPN)
          {            
            MIDI_NRPN(potNRPNs[i], average);
          }
          else
          {
            MIDI_TX(0xB0 + MIDI_CHANNEL - 1, potCCs[i], average);
          }     
        }
        
        potReadCount[i] = 0;
        potTotals[i] = 0;
      }        
    }  
  }
  /*
  for (int i = 0; i < NUMBER_OF_POTS; i++)
  {
    //Setup read from multiplexer
      int selectorIndex = i % 8;
      multiplexSelect(selectorIndex);
      int pin = analogPins[i / 8]; 
    
    // read the value from the sensor:
    int sensorValue = floor(analogRead(pin) / 8);
    Serial.println(i);
    Serial.println("=");
    Serial.println(sensorValue);
    if (sensorValue != lastPotValues[i])
    {    
      lastPotValues[i] = sensorValue;
      //MIDI.sendControlChange(7, 127, 1);
      //MIDI_TX(0xB0, cc[i], sensorValue);
      //Serial.println(selectorIndex);
    }
  }
  */
  
  /*
  
  //Let's try a bit of optimizing 
  for (int i = 0; i < NUMBER_OF_POTS; i++)
  {
    unsigned long currentMillis = millis();    
    
    if (currentMillis != potReadMillis[i])
    {
      //Setup read from multiplexer
      int selectorIndex = i % 8;
      multiplexSelect(selectorIndex);
      int pin = analogPins[i / 8];                  
      
      int reading = analogRead(pin) / 8;
      potTotals[i] = potTotals[i] + reading;
      potReadMillis[i] = millis();
      potReadCount[i]++;
    }
    
    if (potReadCount[i] == SMOOTH_READ_FACTOR)
    {
      int average = potTotals[i] / SMOOTH_READ_FACTOR;
      
      if (average != lastPotValues[i])
      {    
        lastPotValues[i] = average;        
        MIDI_TX(0xB0 + MIDI_CHANNEL - 1, potCCs[i], average);        
      }
      
      potReadCount[i] = 0;
      potTotals[i] = 0;
    }    
  }
  */
}

void MIDI_TX(byte MESSAGE, byte CONTROL, byte VALUE) //pass values out through standard Midi Command
{
   Serial.write(MESSAGE);
   Serial.write(CONTROL);
   Serial.write(VALUE);
}

void MIDI_NRPN(int number, int value)
{  
  byte lsbParameter = (byte)(number & 0x7F);
  byte msbParameter = (byte)(number >> 7);
  
  byte lsbValue = (byte)(value & 0x7F);
  byte msbValue = (byte)(value >> 7);
  
  MIDI_TX(0xB0 + MIDI_CHANNEL - 1, 99, msbParameter);
  MIDI_TX(0xB0 + MIDI_CHANNEL - 1, 98, lsbParameter);
  MIDI_TX(0xB0 + MIDI_CHANNEL - 1, 6, msbValue);
  MIDI_TX(0xB0 + MIDI_CHANNEL - 1, 38, lsbValue);
}

int analogSmoothRead(int inputPin)
{  
  int readings[SMOOTH_READ_FACTOR];      // the readings from the analog input
  int index = 0;                  // the index of the current reading
  int total = 0;                  // the running total
  int average = 0;                // the average
  
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < SMOOTH_READ_FACTOR; thisReading++)
    readings[thisReading] = 0;
  
  for (int i = 0; i < SMOOTH_READ_FACTOR; i++)
  {  
    // subtract the last reading:
    total= total - readings[i];        
    // read from the sensor:  
    readings[i] = analogRead(inputPin);
    // add the reading to the total:
    total= total + readings[i];
    delay(1);        // delay in between reads for stability    
  }
  
  // calculate the average:
  average = total / SMOOTH_READ_FACTOR;          
  
  return average;  
}



