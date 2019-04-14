/* ***************************************************************** 
   neopixels.h: code to initialise the neopixel strip
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */


#define npixels 72

#include <Adafruit_NeoPixel.h>

//#define PIN 4  // PS hack was 6
//#define PIN 15  // PS hack was 6
//#define PIN 6  // for the light arduino with a radio attached
//#define PIN 2  // for the ATTINY yellow
//#define PIN 3  // for the ATTINY black
#define PIN A1   // 19=A1 for the shoe promicro repurposed into necklace

// from Adafruit_NeoPixel-master/examples/strandtest/strandtest.ino: 
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(npixels, PIN, NEO_GRB + NEO_KHZ400);

initialise_neopixels() {
  pinMode(PIN,OUTPUT);
  strip.begin();
}
