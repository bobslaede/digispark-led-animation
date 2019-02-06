//#include "LPD8806.h"
#include "tinySPI.h"
#include "Adafruit_WS2801.h"
#include "PinChangeInterrupt.h"

// Example to control LPD8806-based RGB LED Modules in a strip

/*****************************************************************************/

// Number of RGB LEDs in strand:
uint16_t  nLEDs  = 16;

// Chose 2 pins for output; can be any valid output pins:
uint8_t dataPin  = 3;
uint8_t clockPin = 2;
uint8_t buttonPin = 1;

uint8_t order = 0;

// First parameter is the number of LEDs in the strand.  The LED strips
// are 32 LEDs per meter but you can extend or cut the strip.  Next two
// parameters are SPI data and clock pins:
//LPD8806 strip = LPD8806(nLEDs, dataPin, clockPin);
Adafruit_WS2801 strip = Adafruit_WS2801(nLEDs, dataPin, clockPin, order);

// You can optionally use hardware SPI for faster writes, just leave out
// the data and clock pin parameters.  But this does limit use to very
// specific pins on the Arduino.  For "classic" Arduinos (Uno, Duemilanove,
// etc.), data = pin 11, clock = pin 13.  For Arduino Mega, data = pin 51,
// clock = pin 52.  For 32u4 Breakout Board+ and Teensy, data = pin B2,
// clock = pin B1.  For Leonardo, this can ONLY be done on the ICSP pins.
//LPD8806 strip = LPD8806(nLEDs);

bool oldState = HIGH;
int showType = 4;
int numShows = 5;


void setup() {
  // Start up the LED strip
  pinMode(buttonPin, INPUT_PULLUP);
  strip.begin();

  // Update the strip, to start they are all 'off'
  strip.show();
  attachPCINT(digitalPinToPCINT(buttonPin), changeShow, CHANGE);
}


void loop() {
  startShow(showType);

  //  bool newState = digitalRead(buttonPin);
  //
  //  // Check if state changed from high to low (button press).
  //  if (newState == LOW && oldState == HIGH) {
  //    // Short delay to debounce button.
  //    delay(20);
  //    // Check if button is still low after debounce.
  //    newState = digitalRead(buttonPin);
  //    if (newState == LOW) {
  //      showType++;
  //      if (showType > numShows)
  //        showType=0;
  //      startShow(showType);
  //    }
  //  } else {
  //    startShow(showType);
  //  }
  //
  //
  //  // Set the last button state to the old state.
  //  oldState = newState;

  //  // Send a simple pixel chase in...
  //  colorChase(Color(127, 127, 127), 50); // White
  //  colorChase(Color(127,   0,   0), 50); // Red
  //  colorChase(Color(255, 255,   0), 50); // Yellow
  //  colorChase(Color(  0, 127,   0), 50); // Green
  //  colorChase(Color(  0, 127, 127), 50); // Cyan
  //  colorChase(Color(  0,   0, 127), 50); // Blue
  //  colorChase(Color(127,   0, 127), 50); // Violet
  //
  //  // Fill the entire strip with...
  //  colorWipe(Color(127,   0,   0), 50);  // Red
  //  colorWipe(Color(  0, 127,   0), 50);  // Green
  //  colorWipe(Color(  0,   0, 127), 50);  // Blue

  //  rainbow(10);
  //  rainbowCycle(0);  // make it go through the cycle fairly fast
}

void changeShow() {
  showType++;
  if (showType > numShows)
    showType = 0;
}

void startShow(int i) {
  switch (i) {
    case 0:
      colorWipe(Color(0, 0, 0), 50);    // Black/off
      break;
    case 1:
      colorChase(Color(127,   0, 127), 50);
      break;
    case 2:
      rainbow(10);  // Red
      break;
    case 3:
      colorWipe(Color(255, 0, 0), 50);
      break;
    case 4:
      rainbowCycle(0);
      break;
  }
}

void rainbow(uint8_t wait) {
  int i, j;

  for (j = 0; j < 384; j++) {   // 3 cycles of all 384 colors in the wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel( (i + j) % 384));
    }
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed
// along the chain
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  //  int colorNum = 384;
  //  int colorNum = 256;
  for (j = 0; j < 384 * 5; j++) {   // 5 cycles of all 384 colors in the wheel
    for (i = 0; i < strip.numPixels(); i++) {
      // tricky math! we use each pixel as a fraction of the full 384-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 384 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel( ((i * 384 / strip.numPixels()) + j) % 384) );
      //      strip.setPixelColor(i, Wheel(((i * 384 / strip.numPixels()) + j) & 384));
    }
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// Fill the dots progressively along the strip.
void colorWipe(uint32_t c, uint8_t wait) {
  int i;

  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

// Chase one dot down the full strip.
void colorChase(uint32_t c, uint8_t wait) {
  int i;

  // Start by turning all pixels off:
  for (i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, 0);

  // Then display one pixel at a time:
  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c); // Set new pixel 'on'
    strip.show();              // Refresh LED states
    strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
    delay(wait);
  }

  strip.show(); // Refresh to turn off last pixel
}

/* Helper functions */


uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 384 to get a color value.
//The colours are a transition r - g -b - back to r

uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
    return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
