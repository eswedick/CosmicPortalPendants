#include <Adafruit_NeoPixel.h>

#define PIN         A1
#define NUMPIXELS   12
#define DELAYVAL    100
#define BUTTON      38
#define NUM_MODES   7
#define NUM_COLORS  6
#define BRIGHTNESS  10

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint32_t BLUE = pixels.Color(0, 0, 255);
uint32_t GREEN = pixels.Color(0, 255, 0);
uint32_t RED = pixels.Color(255, 0, 0);
uint32_t YELLOW = pixels.Color(255, 255, 0);
uint32_t CYAN = pixels.Color(0, 255, 255);
uint32_t PURPLE = pixels.Color(255, 0, 255);
uint32_t WHITE = pixels.Color(255, 255, 255);

int currentMode = 0;
int currentColor = 0;
bool patternRunning = false;

uint32_t getCurrentColor(){
  switch(currentColor){
    case 0:
      return BLUE;
      break;
    case 1:
      return GREEN;
      break;
    case 2:
      return RED;
      break;
    case 3:
      return PURPLE;
      break;
    case 4:
      return YELLOW;
      break;
    case 5:
      return CYAN;
      break;
  }
}

void setMode(){
  switch (currentMode) {
    case 0:
      colorWipe(getCurrentColor(), 50); 
      break;
    case 1:
      theaterChase(getCurrentColor(), 50);
      break;
    case 2:
      singleLEDCircle(getCurrentColor(), 50);
      break;
    case 3:
      singleLEDCircleFade(getCurrentColor(), 50);
      break;
    case 4:
      doubleLEDCircle(getCurrentColor(), 50);
      break;
    case 5:
      tripleLEDCircle(getCurrentColor(), 50);
      break;
    case 6:
      ringPulse(getCurrentColor(), 80);
      break;
  }
}

// button commands
volatile bool buttonPressed = false;
volatile bool buttonDoublePressed = false;
volatile bool buttonHeld = false;
unsigned long lastButtonPressTime = 0;
const unsigned long doublePressTimeout = 500;
const unsigned long holdThreshold = 5000;

//sleep
volatile bool sleepMode = false;

//battery level
#define VBATPIN A13
#define USBPIN 39 

//power config 
#if defined(ADAFRUIT_FEATHER_ESP32_V2)
#define PIN_NEOPIXEL 0
#define NEOPIXEL_I2C_POWER 2
#endif

#if defined(PIN_NEOPIXEL)
  Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

void setup() {
  // Some boards work best if we also make a serial connection
  Serial.begin(115200);

  //enableInternalPower();
  disableInternalPower();

  pinMode(BUTTON, INPUT_PULLUP);  // Use INPUT_PULLUP to enable the internal pull-up resistor

  attachInterrupt(digitalPinToInterrupt(BUTTON), handleButtonPress, FALLING);

  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  pixels.show(); 
}

bool patternTesting = false;

void loop() {
  if (patternTesting){
    ringPulse(CYAN, 80);
  }else{
    float usbLevel = getUSBStatus();
    if (usbLevel > 0.8) {
      batteryCharging();
    } 
    else {
      run();
    }
  }
}

void batteryCharging(){
  float batteryLevel = getBatteryLevel();
  getChargeStatus(batteryLevel);
  delay(1000);

  LEDoff();
  delay(1000);
}

void run(){

  int buttonState = digitalRead(BUTTON);
  if(buttonState == LOW && sleepMode){
    sleepMode = false;
  }

  if (!sleepMode){
    if (buttonPressed) {
      int buttonState = digitalRead(BUTTON);
      if(buttonState == LOW && (millis() - lastButtonPressTime >= holdThreshold)){
        sleepMode = !sleepMode;
        buttonPressed = false;     
      }
      else if (buttonState == HIGH){
        currentMode = (currentMode + 1) % NUM_MODES;
      
        buttonPressed = false;
      }
    }

    if (buttonDoublePressed){
      currentColor = (currentColor + 1) % NUM_COLORS;
      buttonDoublePressed = false;
    }

    setMode();
  }else if (sleepMode){
    esp_sleep_enable_timer_wakeup(3000000); // 3 sec
    esp_light_sleep_start();
  }
}

void handleButtonPress() {
  unsigned long currentTime = millis();

  if (currentTime - lastButtonPressTime < doublePressTimeout) {
    buttonDoublePressed = true;

    // Reset the flag and reset the time
    buttonPressed = false;
    lastButtonPressTime = 0;
  } 
  //else if (millis() - lastButtonPressTime >= holdThreshold) {
    // Check for a long button press (5 seconds)
    //return;
    // Reset the flag and reset the time
    // buttonPressed = false;
    // lastButtonPressTime = 0;
  //} 
  else {
    lastButtonPressTime = currentTime;
    buttonPressed = true;
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c);
    pixels.show();
    delay(wait);
  }

  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 0,0,0);
    pixels.show();
  }
}

void singleLEDCircle(uint32_t c, uint32_t wait){
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c); 
    pixels.show(); 
    delay(wait);
    pixels.setPixelColor(i, 0, 0, 0); 
  }
}

void singleLEDCircleFade(uint32_t c, uint32_t wait){
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c); 
    pixels.setPixelColor((i+pixels.numPixels()-1) % pixels.numPixels(), dimColor(c, 0.7));
    pixels.setPixelColor((i+pixels.numPixels()-2) % pixels.numPixels(), dimColor(c, 0.4));
    pixels.setPixelColor((i+pixels.numPixels()-3) % pixels.numPixels(), dimColor(c, 0.1));
    pixels.show(); 
    delay(wait);
    pixels.setPixelColor((i+pixels.numPixels()-4) % pixels.numPixels(), 0, 0, 0); 
  }
}

uint32_t dimColor(uint32_t color, float brightness) {
  uint8_t red = (color >> 16) & 0xFF;
  uint8_t green = (color >> 8) & 0xFF;
  uint8_t blue = color & 0xFF;

  red = static_cast<uint8_t>(red * brightness);
  green = static_cast<uint8_t>(green * brightness);
  blue = static_cast<uint8_t>(blue * brightness);

  return ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
}

uint32_t brightenColor(uint32_t color, float brightness) {
  uint8_t red = (color >> 16) & 0xFF;
  uint8_t green = (color >> 8) & 0xFF;
  uint8_t blue = color & 0xFF;

  // Brighten each color component
  red = static_cast<uint8_t>(std::min(255, red + static_cast<int>(255 * brightness)));
  green = static_cast<uint8_t>(std::min(255, green + static_cast<int>(255 * brightness)));
  blue = static_cast<uint8_t>(std::min(255, blue + static_cast<int>(255 * brightness)));

  return ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
}

void doubleLEDCircle(uint32_t c, uint32_t wait){
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    uint16_t opposite = (i+(pixels.numPixels()/2)) % pixels.numPixels();
    pixels.setPixelColor(i, c); 
    pixels.setPixelColor(opposite, c);
    pixels.show(); 
    delay(wait);
    pixels.setPixelColor(i, 0, 0, 0); 
    pixels.setPixelColor(opposite, 0, 0, 0); 
  }
}

void tripleLEDCircle(uint32_t c, uint32_t wait){
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    uint16_t opposite1 = (i+(pixels.numPixels()/3)) % pixels.numPixels();
    uint16_t opposite2 = (i+2*(pixels.numPixels()/3)) % pixels.numPixels();
    pixels.setPixelColor(i, c); 
    pixels.setPixelColor(opposite1, c);
    pixels.setPixelColor(opposite2, c);
    pixels.show(); 
    delay(wait);
    pixels.setPixelColor(i, 0, 0, 0); 
    pixels.setPixelColor(opposite1, 0, 0, 0); 
    pixels.setPixelColor(opposite2, 0, 0, 0); 
  }
}

void ringPulse(uint32_t c, uint32_t wait){
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.9)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.8)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.7)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.6)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.5)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.4)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.3)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.2)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.1)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.2)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.3)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.4)); 
    pixels.show(); 
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.5));
    pixels.show();
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.6));
    pixels.show();
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.7));
    pixels.show();
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.8));
    pixels.show();
  }
  delay(wait);
  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, dimColor(c, 0.9));
    pixels.show();
  }
  delay(wait);
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


//battery/ charging
float getBatteryLevel(){
  float measuredvbat = analogReadMilliVolts(VBATPIN);
  measuredvbat *= 2;    
  measuredvbat /= 1000; 
  Serial.print("VBat: " ); 
  Serial.println(measuredvbat);
  return measuredvbat;
}

float getUSBStatus(){
  float measuredvbat = analogReadMilliVolts(USBPIN);
  measuredvbat *= 2;    
  measuredvbat /= 1000; 
  Serial.print("USB: " ); 
  Serial.println(measuredvbat);
  return measuredvbat;
}

void LEDonBatteryLevel(uint32_t color, uint16_t count) {
  for(uint16_t i=0; i<count; i++) {
      pixels.setPixelColor(i, color);
      pixels.show();
    }
}

void getChargeStatus(float voltage){
  if(voltage > 4.05){
    LEDonBatteryLevel(GREEN, 12);
  }
  else if (voltage > 4.00){
    LEDonBatteryLevel(GREEN, 11);
  }
  else if (voltage > 3.95){
    LEDonBatteryLevel(GREEN, 10);
  }
  else if (voltage > 3.90){
    LEDonBatteryLevel(GREEN, 9);
  }
  else if (voltage > 3.85){
    LEDonBatteryLevel(YELLOW, 8);
  }
  else if (voltage > 3.83){
    LEDonBatteryLevel(YELLOW, 7);
  }
  else if (voltage > 3.81){
    LEDonBatteryLevel(YELLOW, 6);
  }
  else if (voltage > 3.76){
    LEDonBatteryLevel(YELLOW, 5);
  }
  else if (voltage > 3.73){
    LEDonBatteryLevel(RED, 4);
  }
  else if (voltage > 3.71){
    LEDonBatteryLevel(RED, 3);
  }
  else if (voltage > 3.65){
    LEDonBatteryLevel(RED, 2);
  }
  else{
    LEDonBatteryLevel(RED, 1);
  }
}

void LEDon() {
  pixels.setPixelColor(1, 255,0,0);
  pixels.show();
}

void LEDoff() {
  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 0,0,0);
    pixels.show();
  }
}

void enableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
}

void disableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to rest state (off)
  pinMode(PIN_I2C_POWER, INPUT);
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif
}
