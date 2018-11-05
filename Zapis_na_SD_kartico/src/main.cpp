#include <Arduino.h>
// knjiznice SD
#include "FS.h"
#include "SD.h"
#include <SPI.h>

// SD card pin
#define SD_CS 5

File besedilo;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

 while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(5)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

 besedilo = SD.open("besedilo.txt", FILE_WRITE);

 // if the file opened okay, write to it:
  if (besedilo) {
    Serial.print("Writing to besedilo.txt...");
    besedilo.println("Hello world");
    // close the file:
    besedilo.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening besedilo.txt");
  }


}

void loop() {
  // put your main code here, to run repeatedly:
}