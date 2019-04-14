


// fields understood by client.c and present in radio packet

#define PRINT_RAW
//#define PRINT_COOKED
//#define PRINT_CALC_WXYZ
#define PRINT_CALC_ARM
#define PRINT_CALC_GWZ
//#define PRINT_CALC_MAWZ_MP
//#define PRINT_CALC_RWS /* rolling window size */
#define PRINT_RESULTS     /* with only this printing, rate 52/25 ie  20/40 ms/cycle */



#define USE_BIGLED

#define RECEIVER



uint16_t i,j,k,l;

// unclear in the Arduino setup how to share files between two
// executables without putting them in a global library, so this is
// horrible hackery using absolute paths.  

#include "/home/pes20/shoes/Arduino-1.8.5/necklace/poor_mans_printf.h"
#include "/home/pes20/shoes/Arduino-1.8.5/necklace/vectors.h"
#include "/home/pes20/shoes/Arduino-1.8.5/necklace/dynamics_state.h"
#include "/home/pes20/shoes/Arduino-1.8.5/necklace/radio_type.h"
#include "/home/pes20/shoes/Arduino-1.8.5/necklace/radio_marshalling.h"
#include "/home/pes20/shoes/Arduino-1.8.5/necklace/radio.h"


#include "/home/pes20/shoes/Arduino-1.8.5/necklace/print_state.h"








// ****************************************************************
// ****************************************************************
// main initialisation 


// **************** Setup for RF24 radio ***********************


void setup() {
  delay(3000);

  // ***************** initialise serial link
  Serial.begin(115200); // fast, for less serial latency
  // to estimate serial latency, the Arduino default is apparently 8 data, no parity, 1 stop, so 115200 gives 115200/(9*nbytes) samples / second max

  Serial.print("Receiver\n");

  // initialise radio
  initialise_radio_receiver();

  delay(500); 
}

#ifdef USE_BIGLED
// pin for big LED
int bigled=5;
#endif


// ***************** packet timing ****************************

unsigned long time; 
unsigned long last_receive_time=0; //millis();
unsigned long last_blink_time=0; //millis();

unsigned int receive_packet_count=0;
unsigned int last_packet_count=0;


// ****************************************************************
// ****************************************************************
// ****************************************************************
// main loop 


void loop() {

  // wait until there is radio data ready
  while ( !(radio.available() /*|| radio.rx_fifo_full()*/ )) {

#ifdef USE_BIGLED
    // if more than 1000ms since we've last seen data or done a
    // no-recent-data blink of LED, then blink big LED on and off
    time = millis();
    if ((time > last_receive_time + 1000) && (time > last_blink_time + 1000)) {
      Serial.println("no data available");
      last_blink_time = time;
      analogWrite(bigled, 253);
      delay(50);
      analogWrite(bigled, 255);
    }
#endif
    
  }

//  Serial.println("data available");
//  analogWrite(bigled, 250);
//  delay(50);
//  analogWrite(bigled, 255);
//  delay(150); 
//  analogWrite(bigled, 250);
//  delay(50);
//  analogWrite(bigled, 255);


  // read and dump radio data payloads until we've got the most recent
  bool done = false;
  while (!done) {
    done = radio.read( (uint8_t*) &r, sizeof(radio_data) );
  }
  
  last_receive_time = millis();

  // check if the radio data has all bytes equal and output if so (working around an apparent h/w or RF24 library bug) 
  int i;
  bool bytes_equal;
  bytes_equal = true;
  for (i=1; i<sizeof(radio_data); i++)
    if (((unsigned char*)&r)[i]!=((unsigned char *)&r)[0]) {
      bytes_equal = false;
      break;
    };

  if (!bytes_equal) {

    // unmarshal data from radio buffer
    unmarshal();

    // print data to serial, in same format as necklace
    print_data();
  } else {
    Serial.println("bytes equal");
  }

    // update step counts
  //    buffer_index = (buffer_index + 1) % rolling_window_buffer_size; 
  //    if (first_cycles <= rolling_window_buffer_size) first_cycles++;
  //    mcount++;

}


  
