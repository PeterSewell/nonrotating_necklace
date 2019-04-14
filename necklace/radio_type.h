/* ***************************************************************** 
 radio_type.h: radio payload type, shared between sender and receiver
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */


// this has to fit in a 32-byte packet

typedef struct __attribute__ ((__packed__)) {
  // 18 bytes
  int16_t araw[3],graw[3],mraw[3]; //mcalib[3];  // raw
  //  2 bytes
  uint8_t north_angle,gwz_integrated_magged; // 0..2pi scaled to 0..255
  //  2 bytes
  int16_t gwz; // scaled -40..+40 rad/2 to -32700..32700
  //  6 bytes
  int16_t a_smoothed[3];  // scaled -2g..+2g to -32700..32700
  //  2 bytes
  uint16_t now; // low-order part of the unsigned long
  //  1 byte
  uint8_t rate;    
  //  1 byte 
  uint8_t sample_count; // low-order part of the uint16_t
} radio_data;


