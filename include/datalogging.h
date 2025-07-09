#ifndef datalogging_h
#define datalogging_h



#include <Arduino.h>

#include <SD.h>

#include <guidance.h>
#include <sensor_fetch.h>
#include <control.h>
#include <defines.h>



const int chipSelect = BUILTIN_SDCARD;

File dataFile;

unsigned long timeStampLiftoff; //micros with teensy
unsigned long timeSinceLiftoff;

char serialGO = 'n';

Sd2Card card;
SdVolume volume;
SdFile root;



void flightLogging() {
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  
  dataFile.print(timeSinceLiftoff);
  dataFile.print(",");
  dataFile.println(millis());

  dataFile.close();
}



#endif