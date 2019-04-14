/* ***************************************************************** 
   necklace.ino: top-level code for non-rotating necklace
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */


// Board: Sparkfun Pro Micro ATmega32U4 3.3v 8MHz


//uncomment to print the radio setup details 
//#define PRINT_RADIO_DETAILS

// to output data, for use with client-necklace/client or Arduino
// serial monitor, on either the radio or printing on the serial link, uncomment
// either USE_RADIO or PRINT_DATA or both.  Printing data
// significantly reduces the cycle frequency

//#define USE_RADIO      
//#define PRINT_DATA   // switch client.c data printing on/off

// control of which data to print: 
//   fields understood by client.c:
#define PRINT_RAW
//#define PRINT_COOKED
//#define PRINT_CALC_WXYZ
#define PRINT_CALC_ARM
#define PRINT_CALC_GWZ
//#define PRINT_CALC_MAWZ_MP
//#define PRINT_CALC_RWS /* rolling window size */
#define PRINT_RESULTS     /* with only this printing, rate 52/25 ie  20/40 ms/cycle */

//   fields not understood by client.c, for debugging use with serial monitor only:
//#define PRINT_COLOUR_STATE
//#define PRINT_ANTIALIASING
//#define PRINT_PIXEL_COLOURS
//#define PRINT_MEMORY_USAGE


// uncomment to turn on dynamic gyro recalibration when more-or-less at rest.
// (costs basically no time per cycle)
#define GYRO_DYNAMIC_RECALIBRATION   
// uncomment to blink 8 LEDs white at gyro recalibrations soon after startup
#define GYRO_DYNAMIC_RECALIBRATION_BLINK  

// uncomment to display horizontal acceleration in the LEDs
//#define ACCEL_LEDS  // costs 3-5ms / cycle to display accel

// smooth gravity either with a rolling mean (if uncommented) 
// or a low-pass filter (if commented out)
//#define ROLLING_MEAN

// uncomment to use fixed brightness levels for power consumption measurement
//#define MEASURE_POWER  

// uncomment to suppress mag reading when angular velocity above threshold, to 
// just trust the gyro then
//#define SUPPRESS_MAG_WHEN_ROTATING_FAST

// smooth angle when angular velocity is small, to reduce mag jitter
#define LOW_PASS_LOW_ANGULAR_RATE_SMOOTHED

// uncomment to display the cycle rate superimposed on the LEDs
//#define DISPLAY_RATE_ON_LEDS

// uncomment to use EU flag colours (blue background with twelve
// yellow pixels) instead of the four cartesian random-colour-walk blobs
//#define EUCOLOUR


// set the radio pins for the new-shoe-based h/w
#define NEW_SENDER   
//#define OLD_SENDER


uint16_t i,j,k,l;

#include "poor_mans_printf.h" // poor-man's printf library
#include "vectors.h"          // simple vector library
#ifdef PRINT_MEMORY_USAGE
#include "memory_usage.h"     // code to display memory usage
#endif 
#include "neopixels.h"      // neopixel config and initialisation
#include "dynamics_config.h"// dynamics config values 
#include "dynamics_state.h" // dynamics state variables 
//#ifdef USE_RADIO
#include "radio_type.h"     // type for radio packet
#include "radio_marshalling.h" // marshal and unmarshal radio data
#include "radio.h"          // sending/receiving state over radio 
//#endif
#include "print_state.h"    // printing state to serial
#include "colours.h"        // colour random walk
#include "mpu.h"            // reading data from MPU
#include "dynamics.h"       // calculation of rotation rate and north angle
#include "display.h"        // display on LEDs



// ************* main initialisation ****************************

void setup() {

  // ***************** initialise serial link
  Serial.begin(115200); // fast, for less serial latency
  // to estimate serial latency, the Arduino default is apparently 8
  // data, no parity, 1 stop, so 115200 gives 115200/(9*nbytes) samples /
  // second max

  // initialise neopixel
  initialise_neopixels();
  
  // Initialize all pixels to 'off' 
  for (i=0; i<npixels; i++) {
    strip.setPixelColor(i, 0, 0, 0);
  }

  // delay to let the user open the Arduino serial monitor
  strip.setPixelColor(0, 10, 30, 0); 
  strip.show(); 
  delay(1000); 
  strip.setPixelColor(1, 30, 0, 10);
  delay(1000); 
  strip.show(); 
  strip.setPixelColor(2, 10, 0, 40);
  delay(1000); 
  strip.show(); 

  // initialise MPU
  initialise_mpu();

#ifdef PRINT_RADIO_DETAILS
  printf_begin();
#endif

  // initialise radio
  //#ifdef USE_RADIO
  initialise_radio_sender();
  //#endif

  // initialise random colour walk
  initialise_colour_walk();

}


// *************** main loop ****************************

void loop() {

  read_mpu();

  dynamics_calculation();
  
  display_output();
  
#ifdef USE_RADIO
  radio_send_data();
#endif 

#ifdef PRINT_DATA
  print_data();
#endif
  
  // update step counts
  buffer_index = (buffer_index + 1) % rolling_window_buffer_size; 
  if (first_cycles <= rolling_window_buffer_size) first_cycles++;
  sample_count++;
  
#ifdef PRINT_MEMORY_USAGE
  check_mem("loop"); 
#endif 
}


  

