#ifndef _MULTIPLEX_SELECTOR_H
#define _MULTIPLEX_SELECTOR_H
#include "multiplexSelector.h"
#endif 

#define SMOOTH_READ_FACTOR 1

//configure inputs and CCs
#define MIDI_CHANNEL 1
#define NUMBER_OF_POTS 20
#define NUMBER_OF_SWITCHES 13
const int analogPins[3] = {A0, A1, A2};    // select the input pin for the potentiometer
const int potCCs[NUMBER_OF_POTS] = {90, 5, 23, 12, 13, 75, 76, 77, 79, 78, 74, 20, 73, 71, 21, 72, 70, 22, 15, 7};
const int digitalPins[NUMBER_OF_SWITCHES] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
const int switchCCs[NUMBER_OF_SWITCHES] = {85, 89, 80, 83, 81, 26, 82, 86, 87, 88, 67, 102, 103};

//pot data arrays
int lastPotValues[NUMBER_OF_POTS];
int potTotals[NUMBER_OF_POTS];
int potReadCount[NUMBER_OF_POTS];
unsigned long potReadMillis[NUMBER_OF_POTS];

//switch data arrays
int lastSwitchValues[NUMBER_OF_SWITCHES];

void setup() {  
  //  Set MIDI baud rate:
  Serial.begin(31250);
  //Serial.begin(9600);
  
  //Init switch pins
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    pinMode(digitalPins[i], INPUT);  
  }

  pinMode(50, OUTPUT);    // s0
  pinMode(51, OUTPUT);    // s1
  pinMode(52, OUTPUT);    // s2  
  
  //Init arrays
  for (int i = 0; i < NUMBER_OF_POTS; i++)
  {  
    lastPotValues[i] = 0;
    potTotals[i] = 0;
    potReadCount[i] = 0;
    potReadMillis[i] = 0;
  }
}

void loop() 
{   
  readPots();

  readSwitches();
 
}

multiplexSelector getSelector(int selectorIndex)
{
  multiplexSelector selector = {bitRead(selectorIndex,0), bitRead(selectorIndex,1), bitRead(selectorIndex,2)};
  return selector;
}

void writeSelector(multiplexSelector selector)
{
  digitalWrite(50, selector.r0);
  digitalWrite(51, selector.r1);
  digitalWrite(52, selector.r2);
}

void multiplexSelect(int selectorIndex)
{
  multiplexSelector selector = getSelector(selectorIndex);
  writeSelector(selector);
}

void readSwitches()
{
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    int sensorValue = digitalRead(digitalPins[i]);
    if (sensorValue != lastSwitchValues[i])
    {
      lastSwitchValues[i] = sensorValue;
      MIDI_TX(0xB0 + MIDI_CHANNEL - 1, switchCCs[i], sensorValue * 127);
    }    
  }  
}

void readPots()
{
  /*
  int r0 = bitRead(0,0);    // use this with arduino 0013 (and newer versions)    
  int r1 = bitRead(0,1);    // use this with arduino 0013 (and newer versions)    
  int r2 = bitRead(0,2);    // use this with arduino 0013 (and newer versions)

  digitalWrite(50, r0);
  digitalWrite(51, r1);
  digitalWrite(52, r2);  
  */
  
  for (int i = 0; i < 20; i++)
  {
    int pin = i / 8;
    
    int multiplexIndex = i % 8;
    
    multiplexSelect(multiplexIndex);
    int sensorValue = analogRead(analogPins[pin]) / 8;
    if (sensorValue != lastPotValues[i])
    {    
      lastPotValues[i] = sensorValue;
      //MIDI.sendControlChange(7, 127, 1);    
      MIDI_TX(0xB0 + MIDI_CHANNEL - 1, potCCs[i], sensorValue);
      //Serial.println(selectorIndex);
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



