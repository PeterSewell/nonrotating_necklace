/* ***************************************************************** 
   dynamics.h: orientation calculation, based on the accel/gyro/mag
   data from mpu.h.
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under the
   BSD-2-Clause license in LICENSE  */


void dynamics_calculation() {


  // compute smoothed acceleration to use as a vertical reference,
  // either from a rolling mean or a low-pass filter
  
#ifdef ROLLING_MEAN
  // buffer computed acceleration to compute rolling mean
  for (j=0;j<3;j++) {
    buffer[buffer_index].a[j]=a[j];
    //      buffer[buffer_index].g[j]=g[j];
    //      buffer[buffer_index].m[j]=m[j];
  }

  // calculate current window duration based on sample time to make
  // behaviour roughly invariant with sample rate (used to use rate between last
  // mag samples to avoid excess jitter, but with suppress_mag that is
  // no longer stable)
  if (first_cycles==0) 
    rolling_window_size = 1;
  else {
    rolling_window_size = rolling_window_duration / (deltatmag / magrate);
    if (rolling_window_size > rolling_window_buffer_size)
      rolling_window_size = rolling_window_buffer_size;
    else if (rolling_window_size < 1) 
      rolling_window_size = 1;
  }

  // calculate accel rolling mean 
  for (j=0;j<3;j++) {
    // use current value until the buffer has enough, then take mean
    if (first_cycles < rolling_window_size) {
      a_smoothed[j]=a[j];
    } else {
      tmp_mean = 0.0;
      for (k=0; k<rolling_window_size; k++) {
        i = (buffer_index+(rolling_window_buffer_size-k)) % rolling_window_buffer_size;
        tmp_mean +=  buffer[i].a[j]; 
      }
      a_smoothed[j]= tmp_mean / rolling_window_size;
    }
  }

#else
  // or apply a low-pass filter
  #define tau_arw 3.0
  //  float c_arw = expf(-1.0 / (tau_arw/deltat));
  float c_arw =0.99;
  if (first_cycles==0) {
    for (j=0;j<3;j++) 
      a_smoothed[j]=a[j];
  } else {
    for (j=0;j<3;j++)
      a_smoothed[j]= c_arw * a_smoothed[j] + ((1.0-c_arw)*a[j]);
  }
    
#endif

  // Get the world orientation in device coordinates:
  // 
  // wz is world gravity up, in device coords
  // wx, wy are world horizontal plane, in device coords
  //
  // The gravity acceleration vector wz=normalise(a_smoothed) (world Z) is up.
  // Assume the device Y axis will always be far from world vertical, 
  // which it will with the USB axis of the board in the necklace plane.
  // wx = normalise(y crossproduct wz)  will be in the world horizontal plane 
  // (if g happens to be parallel to z, then wx will be parallel to device X).
  // 
  // wy = normalise(wz crossproduct wx) will also be in the world horizontal 
  // plane (if g happened to be parallel to z, then wy will be parallel to 
  // device Y)

  // redo this only every so often - increases rate from 45 to 47 Hz
  if (first_cycles==0 || ((sample_count+1) % (2*magrate)) == 0u) {
    vector_normalise(wz, a_smoothed);
    vector_cross_product(wx,vector_unit_y,wz);
    vector_normalise(wx,wx);
    vector_cross_product(wy,wz,wx);
    vector_normalise(wy,wy);
  }

  // Project m onto (wx,wy) plane
    
  // sensed mag vector is m, in device coords
  // project mag vector to find the part parallel to wz
  scalar_vector_mul(m_along_wz,vector_dot_product(m,wz),wz);
  // subtract that from m to find the part of m in (wx,wy) plane
  vector_sub(m_prime, m, m_along_wz);
  // and normalise
  vector_normalise(m_prime,m_prime);
  // now find the angle clockwise (looking down wz) from wx to m_prime
  n = vector_dot_product(m_prime,wx);
  if (n>1.0) n=1.0;
  if (n<-1.0) n=-1.0;
  north_angle = acos(n); // between 0 and pi
  if (vector_dot_product(m_prime,wy) < 0) north_angle = twopi- north_angle; // between 0 and 2pi
  // the north angle based only on this accel/mag calculation is thus:
  north_angle = north_angle + offset_to_north;
  if (north_angle > twopi) north_angle -= twopi;


  // sensed gyro vector is g, in device coords
  // project g to find the part parallel to wz
  gwz =  vector_dot_product(g,wz);
  

  // naive integration of gwz gyro rate, for interest: to see how good
  // that is in isolation, without using any mag data
#if 0
  if (first_cycles==0) 
    gwz_integrated = gwz;
  else {
    gwz_integrated = gwz_integrated - gwz * deltat;
    if (gwz_integrated < 0) gwz_integrated += twopi;
    if (gwz_integrated >= twopi) gwz_integrated -= twopi;
  }
#endif

  
  // optionally, suppress use of the mag data when rotating fast.
  // It's not clear that this really helps
#ifdef SUPPRESS_MAG_WHEN_ROTATING_FAST
    if (gwz > SUPPRESS_MAG_GWZ_THRESHOLD || gwz < -SUPPRESS_MAG_GWZ_THRESHOLD)
      suppress_mag = SUPPRESS_MAG;
    else if (suppress_mag == SUPPRESS_MAG || suppress_mag == SUPPRESS_MAG_RECOVERY)
      suppress_mag = SUPPRESS_MAG_RECOVERY;
    else 
      suppress_mag = SUPPRESS_MAG_OFF;
#endif 


  // complementary filter of gyro and mag (as projected into horizontal-wrt-rolling-accel-mean plane)

  /* gim' = c*gest + (1-c)*na
          = (1-1+c)*gest + (1-c)*na
          = gest + (1-c)*(na-gest)
  */

  // first compute complementary filter parameter based on desired time constant, to make behaviour roughly invariant of sample rate
  // time constant for complementary filter:  tau = c / (1-c) * deltat
  // before had c=.99  deltat=1/25  tau =~ 4s

  c = tau / (tau + deltat);
  float oneminusc = 1.0 - c;
    
  if (first_cycles==0) 
    gwz_integrated_magged = north_angle;
  else {
    float gwz_estimate, gwz_error;
    gwz_estimate = gwz_integrated_magged - gwz * deltat;
    normalise_twopi(&gwz_estimate);

    if (
#ifdef SUPPRESS_MAG_WHEN_ROTATING_FAST
        suppress_mag == SUPPRESS_MAG_OFF
#else
        true
#endif
        ) {
      // correct error in shortest way round the circle
      gwz_error = north_angle - gwz_estimate;
      if ( gwz_error > pi) {
        gwz_error = gwz_error - twopi;
      } else if (gwz_error < -pi) {
        gwz_error = twopi + gwz_error;
      } else {
      }
      // actually apply complementary filter
      gwz_integrated_magged = gwz_estimate + oneminusc * gwz_error;
      normalise_twopi(&gwz_integrated_magged);
    } else {
      gwz_integrated_magged = gwz_estimate;
    }
  }



  // calculate smoothed version to use in low-angular-velocity regime
  // to reduce jitter
#ifdef LOW_PASS_LOW_ANGULAR_RATE_SMOOTHED
  float c2 = 0.8;  
  // 0.9 leaves a slow creep of about 3 pixels
  // 0.7 
  // 0.5 leaves a fair amount of jitter
  float oneminusc2 = 1.0 - c2;   // 1.0 for no smoothing
  if (first_cycles==0) 
    gwz_integrated_magged_smoothed = gwz_integrated_magged;
  else {
    gims_error = gwz_integrated_magged - gwz_integrated_magged_smoothed;
    if (gims_error > pi)
      gims_error = gims_error - twopi;
    else if (gims_error < -pi) 
      gims_error = twopi + gims_error;
    gwz_integrated_magged_smoothed = gwz_integrated_magged_smoothed + oneminusc2 * gims_error;
    normalise_twopi(&gwz_integrated_magged_smoothed);
  }

  if (fabs(gwz)<SMOOTH_GWZ_THRESHOLD)
    gwz_integrated_magged = gwz_integrated_magged_smoothed;
#endif 


  // project acceleration onto horizontal plane, if we also want to
  // display that on the LEDs.  This is interesting to see, but it's
  // not clear it really enhances the effect, as it highlights the
  // beginnings and ends of movement.  Something velocity-based would meld
  // better.

#ifdef ACCEL_LEDS
  // original accel vector a, in device coords
  // project accel vector to find the part parallel to wz
  scalar_vector_mul(a_along_wz,vector_dot_product(a,wz),wz);
  // subtract that from m to find the part of m in (wx,wy) plane
  vector_sub(a_prime, a, a_along_wz);
  // and normalise
  a_prime_mod = vector_normalise_mod(a_prime, a_prime);
  // now find the angle clockwise (looking down wz) from wx to m_prime
  n = vector_dot_product(a_prime,wx);
  if (n>1.0) n=1.0;
  if (n<-1.0) n=-1.0;
  accel_angle = acos(n); // between 0 and pi
  if (vector_dot_product(a_prime,wy) < 0) accel_angle = twopi- accel_angle; // between 0 and 2pi

  accel_angle = accel_angle + offset_to_north;
  if (accel_angle > twopi) accel_angle -= twopi;


  // project accel to find the parts in the wx/wy plane
  awx =  vector_dot_product(a,wx);
  awy =  vector_dot_product(a,wy);
#endif




}




