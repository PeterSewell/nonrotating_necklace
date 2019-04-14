/* ***************************************************************** 
 print_state.h: printing data to serial link:
  - human-readable, to use from arduino serial monitor
  - parsable by client.c, to use for live graphs
  - abstracted out so that the radio receiver can produce identical output
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */

void print_data () {

  if (Serial.availableForWrite()==0) 
    return;

#ifdef PRINT_RAW
  print_int16_vector("ar",araw);
  print_int16_vector("gr",graw);
  print_int16_vector("mr",mraw);
  //print_int16_vector("mc",mcalib);
#ifdef GYRO_DYNAMIC_RECALIBRATION
  print_int16_vector("grc",graw_calib);
#endif
#endif

#ifdef PRINT_RESULTS
  // working around the lack of support for uint64_t in Arduino Serial.print
  // (we detect wrapping and fix up in client.c)
  Serial.print("now="); Serial.print((uint32_t)(now & UINT32_MAX)); Serial.print(" ");
#endif

#ifdef PRINT_COOKED
  print_vector("a",a,2); // "g"
  print_vector("g",g,2); // "rad/s"
  print_vector("m",m,2); // "mG"
#endif

#ifdef PRINT_CALC_RWS
  print_int_scalar("rws",rolling_window_size);
#endif    

#ifdef PRINT_CALC_ARM
  print_vector("as",a_smoothed,2); // "g"
#endif    

#ifdef PRINT_CALC_WXYZ
  print_vector("wx",wx,2);    
  print_vector("wy",wy,2);    
  print_vector("wz",wz,2);    
#endif

#ifdef PRINT_CALC_MAWZ_MP
  print_vector("mawz",m_along_wz,2);    
  print_vector("mp",m_prime,2);    
#endif

#ifdef PRINT_CALC_GWZ
  print_scalar("c",c,4); 
  print_scalar("gwz",gwz,4); // "rad/s"
#endif

#ifdef PRINT_RESULTS
  print_scalar("na",north_angle,4); // "rad"
  print_scalar("gi",gwz_integrated,3); // "rad"
  print_scalar("gim",gwz_integrated_magged,3); // "rad"
#ifdef ACCEL_LEDS
  print_scalar("aa",accel_angle,4); // "rad"
  print_scalar("apm",a_prime_mod,4); // "g"
#endif
#endif

#ifdef PRINT_RESULTS
  print_scalar("rate",rate, 1); // "Hz"
  print_int_scalar("sc",sample_count & 0xFF);
#endif

  Serial.println("");

  //  Serial.print("available for write: ");Serial.print(Serial.availableForWrite());Serial.println("");
    
}
