
void display_output() {
  // position around strip for north
  
  iyfloat = ((/*north_angle*/ /*gwz_integrated*/ gwz_integrated_magged * (float)npixels / (twopi)));
  
  // fractional pixel offset for antialiasing
  iyoffset = modff(iyfloat,&dummy);

#ifdef PRINT_ANTIALIASING
  print_scalar("iyoffset",iyoffset,3,"");
#endif

  // range-limit, just in case
  iy = (uint16_t)iyfloat;
  if (iy<0) iy=0;

  // enhance brightness if angular velocity is largish
  float brightness_scale;


#ifdef MEASURE_POWER
  // measuring power for dance colour
#define n_regimes 5
#define dur_regime 5
  uint8_t regime;
  regime = ((now / 1000000) % (n_regimes * dur_regime))/dur_regime;
  if (regime==0)          // mean col    max col  
    brightness_scale=0.0; //  66 mA        67             
  else if (regime==1) 
    brightness_scale=1.0; //  73           80
  else  if (regime==2)
    brightness_scale=2.0; //  87          101
  else  if (regime==3)
    brightness_scale=3.0; // 102          124
  else 
    brightness_scale=4.0; // 117          148
  //
  //                            //  15mA         23mA per brightness_scale step
  //
  //    // adding radio doesn't change the power
  //
  //    max col:  50+15+15 + 10+50+20 + 5+2+40 + 30+10+30  = 277
  //    min col: 20+0+0 + 0+10+0 + 0+0+20 + 20+0+20 = 90
  //    mean col:     183 
  //
  //    times 3 pixels of each colour:
  //    max col: 765     ../255 = 3.00 full-scale single-colour pixel equiv  
  //    mean col: 489    ../255 = 1.92
  //
  //    so we only see around 8mA per full-scale single-colour pixel equiv
  //    vs the advertised 20mA (for a 5v vs these 3.3v, but still...)

#else
  brightness_scale = fabsf(gwz);
  if (brightness_scale < 0.2) 
    brightness_scale = 1.0;
  else
    brightness_scale = 1.0 + 1.0*(brightness_scale - 0.2) / (2.0 - 1.0);
#endif

  // zero all pixels
  for (i=0; i<npixels; i++) {
    strip.setPixelColor(i, 0, 0, 0);
  }

  // take a step in the colour random walks
  step_colour_walk();
  // this costs about 0.7ms : rate 54 to 52

#ifdef EUCOLOUR
  // estimate full-scale power:  
  //   60 * (0x33 + 0x99)/0xff * 20  + 12 * (0xff + 0xcc)/0xff * 20 
  // = (60 * (51+154)*20)/255  + (12 *(255+204)*20)/255
  // = 1396 mA  
  // Arduino regulator is current-limited around 500mA, so say max brightness is a third of that
  // eus  10   0.1        estimate: 140mA
  // eus   5   0.2                  280mA
  // eus   3   0.33
  // eus   2   0.5  
  uint8_t eus=2;
  //    uint8_t eus=10;
  //    //#define eus 10 
  //#define n_regimes 4
  //#define dur_regime 5
  //    uint8_t regime;
  //    regime = ((now / 1000000) % (n_regimes * dur_regime))/dur_regime;
  //    if (regime==0)
  //      eus=10;               // 82mA
  //    else if (regime==1) 
  //      eus=5;               //116
  //    else  if (regime==2)
  //      eus=3;                 // 163
  //    else 
  //      eus=2;                // 207

  //     measured data is a *lot* less than that estimate: 
  // 0.1,   82
  // 0.2,   116
  // 0.333, 163
  // 0.5,   207

  // Initialize all pixels to 'off' except pixel 0 green
  for (i=0; i<npixels; i++) {
    //    strip.setPixelColor(i, 0, 0x33/eus, 0x99/eus);
    strip.setPixelColor(i, 0, 0x22/eus, 0x66/eus);
  }
  for (i=0;i<12;i++)
    //    strip.setPixelColor((i*6+iy)%npixels, 0xFF/eus, 0xCC/eus, 0); 
    strip.setPixelColor((i*6+iy)%npixels, 0xAA, 0x88/eus, 0); 
#else
  // set blobs of pixels at each cardinal direction with the appropriate colour
  setpixels(iy, iyoffset,brightness_scale,gwz);
  // this costs 5ms of the loop: frame rate 52 to 72   19ms to 14ms 
#endif


#ifdef DISPLAY_RATE_ON_LEDS
  // a scale: a dim blue pixel every 10
  for (i=0;i<npixels;i+=10)
    strip.setPixelColor(i,0,4,4);
  // a red pixel for rate
  strip.setPixelColor( ((uint8_t)(((float)magrate)/deltatmag))%npixels,40,00,00);
  // a white pixel for available serial buffer
  strip.setPixelColor(Serial.availableForWrite()%npixels,20,20,20);
#endif






#ifdef ACCEL_LEDS
  // this seems too noisy to be pleasant    

  // position around strip for current horizontal-plane acceleration
  iafloat = gwz_integrated_magged + accel_angle;
  if (iafloat > twopi) iafloat -= twopi;
  iafloat = ( iafloat * (float)npixels / (twopi));
  iaoffset = modff(iafloat,&dummy);
  ia = (uint16_t)iafloat;
  if (ia>=npixels) ia=npixels-1;
  if (ia<0) ia=0;

  // scale brightness by modulus of that horizontal acceleration vector
  brightness_scale = a_prime_mod;
  if (brightness_scale < 0.1) 
    brightness_scale = 0.0;
  else
    brightness_scale = 10.0 * (brightness_scale - 0.1);

  // set a blob of white pixels
  if (a_prime_mod > 0.1) 
    setpixels((ia+(0*npixels/4)),iaoffset,brightness_scale, 20,20,20);
#endif

  // display strip  (this costs about 2ms / cycle)
  strip.show(); 
}
