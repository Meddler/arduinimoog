#define SMOOTH_READ_FACTOR 10

//configure inputs and CCs
#define MIDI_CHANNEL 1
#define NUMBER_OF_POTS 2
#define NUMBER_OF_SWITCHES 1
const int analogPins[NUMBER_OF_POTS] = {A0, A1};    // select the input pin for the potentiometer
const int potCCs[NUMBER_OF_POTS] = {7, 8};
const int digitalPins[NUMBER_OF_SWITCHES] = {2};
const int switchCCs[NUMBER_OF_SWITCHES] = {9};

//pot data arrays
int lastPotValues[NUMBER_OF_POTS];
int potTotals[NUMBER_OF_POTS];
int potReadCount[NUMBER_OF_POTS];
unsigned long potReadMillis[NUMBER_OF_POTS];

//switch data arrays
int lastSwitchValues[NUMBER_OF_SWITCHES];
unsigned long switchReadMillis[NUMBER_OF_SWITCHES];

void setup() {  
  //  Set MIDI baud rate:
  Serial.begin(31250);
  
  pinMode(2, INPUT_PULLUP);
  
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
  for (int i = 0; i < NUMBER_OF_POTS; i++)
  {
    // read the value from the sensor:
    int sensorValue = floor(analogSmoothRead(sensorPins[i]) / 8);
    if (sensorValue != lastSensorValues[i])
    {    
      lastSensorValues[i] = sensorValue;
      //MIDI.sendControlChange(7, 127, 1);
      MIDI_TX(0xB0, cc[i], sensorValue);
    }
  }
  
  */
  //Let's try a bit of optimizing 
  for (int i = 0; i < NUMBER_OF_POTS; i++)
  {
    unsigned long currentMillis = millis();
    
    if (currentMillis != potReadMillis[i])
    {
      int reading = floor(analogRead(analogPins[i]) / 8);
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



