/* ***************************************************************** 
 radio.h: code to initialise the radio and send data
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */


// for reference, here is the debug output from PRINT_RADIO_DETAILS
//
//STATUS		 = 0x0e RX_DR=0 TX_DS=0 MAX_RT=0 RX_P_NO=7 TX_FULL=0
//RX_ADDR_P0-1	 = 0xe8e8f0f0e1 0xc2c2c2c2c2
//RX_ADDR_P2-5	 = 0xc3 0xc4 0xc5 0xc6
//TX_ADDR		 = 0xe8e8f0f0e1
//RX_PW_P0-6	 = 0x20 0x00 0x00 0x00 0x00 0x00
//EN_AA		 = 0x00
//EN_RXADDR	 = 0x03
//RF_CH		 = 0x4c  // ie RF24 default, setChannel(76);
//RF_SETUP	 = 0x07
//CONFIG		 = 0x0e
//DYNPD/FEATURE	 = 0x00 0x00
//Data Rate	 = 1MBPS
//Model		 = nRF24L01+
//CRC Length	 = 16 bits
//PA Power	 = PA_MAX



// **************** Preamble for RF24 radio **************************

#ifdef PRINT_RADIO_DETAILS
#include <printf.h>
#endif
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Hardware configuration


// Set up nRF24L01 radio on SPI bus plus pins 9,10 or 18,10
#ifdef OLD_SENDER
RF24 radio(9,10);
#endif
#ifdef NEW_SENDER
RF24 radio(18,10);
#endif
#ifdef RECEIVER
RF24 radio(9,10);
#endif

// Single radio pipe address for the 2 nodes to communicate.
#define radio_pipe  0xE8E8F0F0E1LL


// **************** radio state **************************


// radio send result
bool radio_send_ok; 
uint16_t radio_send_ok_count=0, radio_send_fail_count=0;



// **************** radio setup  ***********************

void initialise_radio_sender() {
  Serial.println("initialise_radio_sender");

  delay(100);  // just in case this helps

  radio.begin();

  delay(100);  // just in case this helps

  // turn off radio auto ack
  radio.setAutoAck(0);

  // turn off retries
  // setting this to 0,0 makes the bogus-all-bytes-the-same much more common
  radio.setRetries(0, 0);

  // Open pipe to other nodes for communication. This simple sketch
  // opens a single pipe for these two nodes to communicate
  //  back and forth.  One listens on it, the other talks to it.

  //#ifdef RECEIVER
  //  radio.openReadingPipe(1,pipe);
  //  radio.startListening();

  // radio.stopListening();  didn't help with failure on hard start

  radio.openWritingPipe(radio_pipe);


#ifdef PRINT_RADIO_DETAILS
  // Dump the configuration of the rf unit for debugging
  Serial.print("radio printDetails:\n");
  radio.printDetails();
#endif 

  //  rf24_crclength_e crcl = radio.getCRCLength();

  //  if (sizeof(radio_data) > 32) {
  //    Serial.print("static error: radio_data more than 32 bytes\n");
  //    // set some pixels to red
  //    strip.setPixelColor(10, 40, 0, 0);
  //    strip.setPixelColor(20, 40, 0, 0);
  //    strip.setPixelColor(30, 40, 0, 0);
  //    strip.show(); 
  //  }
}


void initialise_radio_receiver() {

  radio.begin();
  
  // PS turn off radio auto ack
  radio.setAutoAck(0);

  // PS turn off retries
  // setting this to 0,0 makes the bogus-all-bytes-the-same much more common
  //radio.setRetries(0, 0);


  // Open pipe to other nodes for communication. This simply
  // opens a single pipe for these two nodes to communicate
  //  back and forth.  One listens on it, the other talks to it.

  radio.openReadingPipe(1,radio_pipe);
  radio.startListening();
  //  radio.openWritingPipe(pipe);

  // Dump the configuration of the rf unit for debugging
  Serial.print("radio printDetails\n");
  radio.printDetails();
  rf24_crclength_e crcl = radio.getCRCLength();


  if (sizeof(radio_data) > 32) {
    Serial.print("static error: radio_data more than 32 bytes\n");
  }
}






// ****************** radio send  ***********************
void radio_send_data() {
  // marshal data to radio buffer
  marshal();
  
  // send it
  //    if (debug) 
  //    printf("Send-");
  radio_send_ok = radio.write( &r, sizeof(radio_data));
  //  if (debug) {
  if (radio_send_ok)
    radio_send_ok_count++;
  else
    radio_send_fail_count++;
  
  //  if (now - last_now_print > 1000000 || now < last_now_print) {
  //    Serial.print("now:");Serial.print((uint32_t)(now&0xFFFFFFFF));
  //    last_now_print=now;
  //    Serial.print("radio send ok: ");Serial.print(radio_send_ok_count); 
  //    Serial.print("fail: ");Serial.print(radio_send_fail_count); 
  //    Serial.println("");
  //  }
}

