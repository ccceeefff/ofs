/* Overland Flow Sensor Software 
 * Version 0.1
 * 
 * This software collects range data from two IR sensors and provides
 * latitude, longitude, and datetime information. The data is saved on 
 * a micro-SD card with datetime formated for SQL.
 * 
 * The minimum range of the IR sensor is 4 cm and maximum range is ~20cm.
 * Analog range values are converted to distance by the lookup table values
 * with the key:value pair is array index is the analog voltage, and the 
 * array value is the calibrated distance. The lookup tables are stored in 
 * the program memory of the Arduino.
 */

#include <avr/pgmspace.h>
#include <ctype.h>
#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <SPI.h>
#include <SD.h>

//IR sensor analog pins
#define irSensorPin0   A0  
#define irSensorPin1   A2 
int irSensor0=0;
int irSensor1=0;

//GPS communiction setup
static const int RXPin = 8,TXPin = 9;
static const uint32_t GPSBaud = 38400;
TinyGPSPlus gps;                        // The TinyGPS++ object
AltSoftSerial ss(RXPin, TXPin);         // The serial connection to the GPS device

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 4;

//Sensor0 lookup Table
const int Sensor0[] PROGMEM = {255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,225,223,222,221,220,219,
218,217,216,216,215,215,214,213,212,211,210,209,208,207,205,204,203,202,200,
199,197,196,195,194,192,190,189,188,187,186,185,184,183,182,181,180,179,179,
178,177,176,175,175,174,173,172,171,170,169,168,167,166,165,165,164,164,163,
163,162,162,161,161,160,160,159,158,157,156,155,154,154,153,153,152,151,150,
150,149,149,148,147,147,146,145,145,144,144,143,142,141,141,140,140,139,139,
138,137,136,135,135,134,134,134,133,133,132,132,131,131,130,130,129,129,128,
128,127,127,127,126,126,125,125,124,124,123,123,122,121,120,120,120,119,119,
119,118,118,118,117,117,117,116,116,116,115,115,115,114,114,113,113,113,112,
112,111,111,110,110,109,109,108,108,107,107,107,106,106,106,105,105,104,104,
104,103,103,103,102,102,102,101,101,101,100,100,100,99,99,99,98,98,98,97,
97,97,97,96,96,96,95,95,94,94,94,94,93,93,93,93,92,92,92,92,91,91,91,91,90,
90,89,89,89,88,88,88,88,87,87,87,87,86,86,86,86,85,85,85,85,84,84,84,84,83,
83,83,83,82,82,82,82,81,81,81,81,80,80,80,80,79,79,79,79,78,78,78,78,77,77,
77,77,76,76,76,76,75,75,75,75,75,74,74,74,74,74,73,73,73,73,73,72,72,72,72,
72,71,71,71,71,71,70,70,70,70,70,70,69,69,69,69,69,69,68,68,68,68,68,68,67,
67,67,67,67,67,66,66,66,66,66,66,65,65,65,64,64,64,64,64,64,64,63,63,63,63,
63,63,63,62,62,62,62,62,62,62,61,61,61,61,61,61,61,60,60,60,59,59,59,59,59,
59,59,58,58,58,58,58,58,58,57,57,57,57,57,57,57,57,56,56,56,56,56,56,56,56,
55,55,55,54,54,54,54,54,54,54,54,54,54,53,53,53,53,53,53,53,53,53,53,52,52,
52,52,52,52,52,52,52,52,51,51,51,51,51,51,51,51,51,51,50,50,50,50,50,50,50};

const int Sensor1[] PROGMEM = {255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,225,223,222,221,220,
219,218,217,216,216,215,215,214,213,212,211,210,209,208,207,205,204,203,
202,200,199,197,196,195,194,192,190,189,188,187,186,185,184,183,182,181,
180,179,179,178,177,176,175,175,174,173,172,171,170,169,168,167,166,165,
165,164,164,163,163,162,162,161,161,160,160,159,158,157,156,155,154,154,
153,153,152,151,150,150,149,149,148,147,147,146,145,145,144,144,143,142,
141,141,140,140,139,139,138,137,136,135,135,134,134,134,133,133,132,132,
131,131,130,130,129,129,128,128,127,127,127,126,126,125,125,124,124,123,
123,122,121,120,120,120,119,119,119,118,118,118,117,117,117,116,116,116,
115,115,115,114,114,113,113,113,112,112,111,111,110,110,109,109,108,108,
107,107,107,106,106,106,105,105,104,104,104,103,103,103,102,102,102,101,
101,101,100,100,100,99,99,99,98,98,98,97,97,97,97,96,96,96,95,95,94,94,
94,94,93,93,93,93,92,92,92,92,91,91,91,91,90,90,89,89,89,88,88,88,88,87,
87,87,87,86,86,86,86,85,85,85,85,84,84,84,84,83,83,83,83,82,82,82,82,81,
81,81,81,80,80,80,80,79,79,79,79,78,78,78,78,77,77,77,77,76,76,76,76,75,
75,75,75,75,74,74,74,74,74,73,73,73,73,73,72,72,72,72,72,71,71,71,71,71,
70,70,70,70,70,70,69,69,69,69,69,69,68,68,68,68,68,68,67,67,67,67,67,67,
66,66,66,66,66,66,65,65,65,64,64,64,64,64,64,64,63,63,63,63,63,63,63,62,
62,62,62,62,62,62,61,61,61,61,61,61,61,60,60,60,59,59,59,59,59,59,59,58,
58,58,58,58,58,58,57,57,57,57,57,5757,57,56,56,56,56,56,56,56,56,55,55,
55,54,54,54,54,54,54,54,54,54,54,53,53,53,53,53,53,53,53,53,53,52,52,52,
52,52,52,52,52,52,52,51,51,51,51,51,51,51,51,51,51,50,50,50,50,50,50,50};

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  
  pinMode(10, OUTPUT);
   // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop() {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);  
    irSensor0 = analogRead(irSensorPin0);
    irSensor1 = analogRead(irSensorPin1);


    irSensor0 = pgm_read_word(&Sensor0[irSensor0]);
    irSensor1 = pgm_read_word(&Sensor1[irSensor1]);    
    
  /***  Sensor output test ***
    Serial.print(irSensor0);
    Serial.print(",");
    Serial.println(irSensor2);
  */
     

    while (ss.available())
    {     
       if (gps.encode(ss.read()))
       {  
         /* dataFile.print(gps.date.year());
          dataFile.print("-");
          dataFile.print(gps.date.month());
          dataFile.print ("-"); 
          dataFile.print(gps.date.day());
          dataFile.print(" "); 
          dataFile.print(gps.time.hour());
          dataFile.print( ":");
          dataFile.print(gps.time.minute());
          dataFile.print(":");
          dataFile.print(gps.time.second());
          dataFile.print(",");
          dataFile.print(irSensor0);
          dataFile.print(",");
          dataFile.println(irSensor1);  
          dataFile.close(); */


          
/*** GPS output Test ***
    Serial.print(gps.date.year());
    Serial.print("-");
    Serial.print(gps.date.month());
    Serial.print ("-"); 
    Serial.print(gps.date.day());
    Serial.print(" "); 
    Serial.print(gps.time.hour());
    Serial.print( ":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
    */
     }
    //delay(500);
   } 
}

