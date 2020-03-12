#include <SD.h>                                                  //calls the SD.h library
#include <DmxSimple.h>

int a, b, c, i, j;
int rawin;

#define STRIP_LENGTH 61//96 LEDs on this strip and how many pixels WIDE the photoshop file is
#define ARRAY_LENGTH 18111 // this is how many lines in the photoshop file


File myFile;                           //declares the public name of the file to open on the SD card

void setup() {
  pinMode(10, OUTPUT);  

  DmxSimple.usePin(5);
  DmxSimple.maxChannel(7);
  
//  Serial.begin(9600);
  if (!SD.begin(10)) {
//    Serial.println("initialization failed!");         //extra serial.print stuff that i only need for debugging
    return;
  }
}


void loop() {

  myFile = SD.open("rgb61x14.raw");
  if (myFile) {
 //     Serial.write("hello");
    for(j = 0; j<ARRAY_LENGTH; j++){                          
      for(i = 0 ; i < (STRIP_LENGTH*3) ; i++){
 
        rawin = myFile.read();
        DmxSimple.write(i+1, rawin);
//        Serial.write(i + ", " + rawin); 
      delay(5);        
      }

   }
    myFile.close();
  } 
  
}


