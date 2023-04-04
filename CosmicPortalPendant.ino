#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <EEPROMWearLevel.h>
 
#define PIN       A1
#define NUMPIXELS 12

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 100

#define EEPROM_LAYOUT_VERSION 0
#define AMOUNT_OF_INDEXES 1

#define INDEX_CONFIGURATION_MODE 0

int mode;

//old
void writeLEDMode(int ledmode){
  EEPROM.write(0, ledmode);
}
int readLEDMode(){
  return EEPROM.read(0);
}

//new
void writeConfiguration() {
  // write a byte
  EEPROMwl.update(INDEX_CONFIGURATION_MODE, 12);
}

void readConfiguration() {
  byte mode = EEPROMwl.read(INDEX_CONFIGURATION_VAR1);
  Serial.print(F("mode: "));
  Serial.println(mode);
}


void setup() {
  //EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);

  //writeConfiguration();
  //readConfiguration();

//old
  mode = readLEDMode(); //read mode
  if(mode==255){//reset mode
    mode=0;
  }
  if(mode==5){//cycle modes
    mode=1;
  }else{  
    mode=mode+1;
  }
  writeLEDMode(mode); //save config

  pixels.begin();
  pixels.setBrightness(50);
  pixels.show(); // Initialize all pixels to 'off'
}

void loop() {
  // mode 1
  // Some example procedures showing how to display to the pixels:
  if (mode==1){
    colorWipe(pixels.Color(255, 0, 0), 50); // Red
    colorWipe(pixels.Color(0, 255, 0), 50); // Green
    colorWipe(pixels.Color(0, 0, 255), 50); // Blue
    //colorWipe(pixels.Color(0, 0, 0, 255), 50); // White RGBW
  }else if (mode==2){
    // mode 2
    // Send a theater pixel chase in...
    theaterChase(pixels.Color(127, 127, 127), 50); // White
    theaterChase(pixels.Color(127, 0, 0), 50); // Red
    theaterChase(pixels.Color(0, 0, 127), 50); // Blue
  }else if (mode==3){
    // mode 3
    rainbow(20);
  }else if (mode==4){
    // mode 4
    rainbowCycle(20);
  }else if (mode==5){
    // mode 5
    theaterChaseRainbow(50);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c);
    pixels.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixels.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
    }
    pixels.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixels.show();

      delay(wait);

      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixels.show();

      delay(wait);

      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

