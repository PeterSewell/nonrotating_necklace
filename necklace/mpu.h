/* ***************************************************************** 
   mpu.h: code to manage the MPU 9150: initialise it, read the raw
   accelerometer, gyroscope, and magnetometer sensor data, convert the
   latter to the same coordinate system and apply my calibration to it
   (from dynamics_config.h), and convert to standard units.
   Optionally, automatically recalibrate the gyro from a rolling mean
   when we appear to have been at rest for some time.
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */


/* This code was initially based on MPU9150_AHRS.ino: SFE_MPU9150
   Library AHRS Data Fusion Example Code, Kris Winer for Sparkfun
   Electronics,
   https://github.com/sparkfun/MPU-9150_Breakout/blob/master/firmware/MPU6050/Examples/MPU9150_AHRS.ino,
   in https://github.com/sparkfun/MPU-9150_Breakout, and it uses the
   MPU6050_9Axis_MotionApps41.h library from there.  That code also
   includes Madgewick and Mahoney sensor fusion algorithms, kept in
   comments below, but they seemed to work less well than the custom
   scheme in dynamics.h */


// Coordinate system, for both the original (Sparkfun, red) and
// replacement (blue) MPU-9150 assemblies.  The Pro Micro and MPU9150
// boards are lined up and the same way up, with the Pro Micro USB at the
// VCC end of the MPU9150 board.
//
//              ^
//              | X
//              | 
//    Y     ---------------
//  <----   [USB]  Z up   |
//          ---------------
//
// accel 
// 
// x   +ve to left of usb, from above with usb towards you
// y   in line with usb  +ve going towards usb
// z   +ve up
// 
// gyro
// 
// x   +ve anitclock looking to right of usb
// y   in line with usb +ve anticlock looking away from usb end
// z   +ve anticlock from above 
// 
// mag: the native mag data is:
// 
// x +ve in line with usb, going away   
// y perpendicular to usb to right with usb towards you  
// z up     
//
// but the arduino code immediately puts x=-y and y=-x to shift this into the accel coordinate system:
// x perpendicular to usb to right with usb towards you 
// y +ve in line with usb, towards usb   
// z up     



#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"

// Declare device MPU6050 class
MPU6050 mpu;

//  internal sensor offsets accel xyz gyro xyz...
// -2208	1465	1078	0	1	63	


void initialise_mpu() {
  // initialize MPU6050 device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU9150 connection successful") : F("MPU9150 connection failed"));

  // Set up the accelerometer, gyro, and magnetometer for data output
  mpu.setRate(7); // set gyro rate to 8 kHz/(1 * rate) shows 1 kHz, accelerometer ODR is fixed at 1 KHz

  //  for magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
  // PS: note that each magnetometer read takes quite a chunk of time

  // from MPU9150_AHRS.ino:
  // Digital low pass filter configuration. 
  // It also determines the internal sampling rate used by the device as shown in the table below.
  // The accelerometer output rate is fixed at 1kHz. This means that for a Sample
  // Rate greater than 1kHz, the same accelerometer sample may be output to the
  // FIFO, DMP, and sensor registers more than once.
  /*
   *          |   ACCELEROMETER    |           GYROSCOPE
   * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
   * ---------+-----------+--------+-----------+--------+-------------
   * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
   * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
   * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
   * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
   * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
   * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
   * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
   */
  mpu.setDLPFMode(0/*WAS 3*/); // set bandwidth of both gyro and accelerometer to ~20 Hz

  // Full-scale range of the gyro sensors:
  // 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
  mpu.setFullScaleGyroRange(2); // set gyro range to 500 degrees/sec
  #define gyro_range 1000.0f
  // PS: it seemed to clip with reasonable tango dance motion at 250 degrees/sec

  // Full-scale accelerometer range.
  // The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
  mpu.setFullScaleAccelRange(0); // set accelerometer to 2 g range
  #define accel_range 2.0f

  Serial.println(" internal sensor offsets accel xyz gyro xyz...");
  Serial.print(mpu.getXAccelOffset()); Serial.print("\t"); // 
  Serial.print(mpu.getYAccelOffset()); Serial.print("\t"); // 
  Serial.print(mpu.getZAccelOffset()); Serial.print("\t"); //
  Serial.print(mpu.getXGyroOffset()); Serial.print("\t"); // 
  Serial.print(mpu.getYGyroOffset()); Serial.print("\t"); // 
  Serial.print(mpu.getZGyroOffset()); Serial.print("\t"); // 
  Serial.print("\n");

  mpu.setIntDataReadyEnabled(true); // enable data ready interrupt
}


void read_mpu() {
  
  while (mpu.getIntDataReadyStatus() != 1) {}; // wait for data ready status register to update all data registers

  // read the raw sensor acceleration
  mpu.getAcceleration( &araw[0], &araw[1], &araw[2] );
  for (j=0;j<3;j++) {
    a[j] = araw[j]*accel_range/32768.0f; // transform accel units to g
#ifdef GYRO_DYNAMIC_RECALIBRATION
    raw_buffer[0][buffer_index][j]=araw[j];
#endif
  }
  
  // read the raw sensor gyro
  mpu.getRotation( &graw[0], &graw[1], &graw[2] );

  // checkpoint time of gyro read
  now_sample = (uint64_t)micros();
  // fix w.r.t. Arduino 32-bit unsigned long choice for micros(), which wraps every 70 min or so
  if (now_sample < last_now_sample) now_offset += UINT64_C(1)<<32;
  now = now_sample + now_offset;
  if (first_cycles==0) {
    now_origin=now;
  } else {
    deltat = ((now - last_now)/1000000.0f); // set integration time by time elapsed since last filter update
    rate = (float)1.0f/deltat;
  }
  last_now = now;



  for (j=0;j<3;j++) {
    graw[j] = graw[j]-graw_calib[j];                 // apply my gyro calibration
    g[j] = graw[j]*gyro_range/32768.0f*twopi/360.0f; // transfrom gyro units to rad
#ifdef GYRO_DYNAMIC_RECALIBRATION
    raw_buffer[1][buffer_index][j]=graw[j];
#endif
  }
  
  //  from MPU9150_AHRS.ino:
  //  The gyros and accelerometers can in principle be calibrated in addition to any factory calibration but they are generally
  //  pretty accurate. You can check the accelerometer by making sure the reading is +1 g in the positive direction for each axis.
  //  The gyro should read zero for each axis when the sensor is at rest. Small or zero adjustment should be needed for these sensors.
  //  The magnetometer is a different thing. Most magnetometers will be sensitive to circuit currents, computers, and 
  //  other both man-made and natural sources of magnetic field. The rough way to calibrate the magnetometer is to record
  //  the maximum and minimum readings (generally achieved at the North magnetic direction). The average of the sum divided by two
  //  should provide a pretty good calibration offset. Don't forget that for the MPU9150, the magnetometer x- and y-axes are switched 
  //  compared to the gyro and accelerometer!

  // get mag sample one in every magrate samples - one mag read costs around 21ms
  if ((first_cycles==0 || ((sample_count % magrate) == 0u))
#ifdef SUPPRESS_MAG_WHEN_ROTATING_FAST
      && suppress_mag!=SUPPRESS_MAG
#endif
      ) {
    /* read the raw sensor magnetometer */
    mpu.getMag( &mraw[1], &mraw[0], &mraw[2] );
    /* ...and transform into accelerometer coordinate system */
    mraw[0] = -mraw[0]; 
    mraw[1] = -mraw[1];
#ifdef SUPPRESS_MAG_WHEN_ROTATING_FAST
    if (suppress_mag == SUPPRESS_MAG_RECOVERY)
      suppress_mag = SUPPRESS_MAG_OFF;
#endif
    /* apply my magnetometer calibration*/ 
    for (j=0;j<3;j++) {
      mcalib[j] = mraw[j] - ((mrange[j][0] + mrange[j][1])/2);
    }
    /*transform magnetometer units into milliGauss*/ 
    for (j=0;j<3;j++) 
      m[j] = mcalib[j]*10.0f*1229.0f/4096.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
    /* use same checkpoint time for mag read as for accel/gyro reads (actually this will be some time later, but we're only using this to keep a stable-duration rolling window) */  
    
    deltatmag = ((now - last_mag_now)/1000000.0f); // time elapsed between last pair of mag updates
    last_mag_now = now;
  } else {
    // just use the last sample
  }


#ifdef GYRO_DYNAMIC_RECALIBRATION
// if we seem to be at rest, i.e. all the raw data is within some
// fixed bounds of the moving average for the last
// rolling_window_buffer_size samples (of the order of 1s), reset the
// gyro drift calibration based on the average of those.
// It would be more time and space efficient to compare against a 
// low-pass-filtered gyro signal, but this is robust in the face
// of slow real rotation.  Unclear whether that's worthwhile.
  for (j=0;j<3;j++)
    raw_buffer[2][buffer_index][j] = mcalib[j];
  
  if (first_cycles >= rolling_window_buffer_size && count_since_last_gyro_recalib > rolling_window_buffer_size && count_since_last_gyro_recalib > gyro_recalib_min_interval/deltat) {
    for (k=0;k<3;k++) {   // check each of accel, gyro, and mag remain within the specified bounds of this rolling mean
      for (j=0;j<3;j++) { // check each axis
        int32_t sum = 0;
        for (i=0; i<rolling_window_buffer_size; i++) // compute rolling mean
          sum += raw_buffer[k][i][j];
        int32_t mean;
        mean = sum / rolling_window_buffer_size;
        for (i=0; i<rolling_window_buffer_size; i++) { // check continuously in bounds
          int32_t diff = abs(raw_buffer[k][i][j]-mean);
          if (diff > maxdiff[k]) goto no_recalibration_now;
        }
        if (k==1) graw_mean[j]=mean; // snapshot to use for new gyro calibration
      }
    }
    for (j=0;j<3;j++) 
      graw_calib[j]=graw_calib[j]+graw_mean[j];
    count_since_last_gyro_recalib = 0;
#ifdef GYRO_DYNAMIC_RECALIBRATION_BLINK  
    if (now - now_origin < gyro_dynamic_recalibration_blink_time) {
      for (i=0;i<npixels;i+=npixels/8) 
        strip.setPixelColor(i,20,20,20);
      strip.show(); 
      delay(100);
    }
#endif
  }
    
 no_recalibration_now:

  if (count_since_last_gyro_recalib < 65534)     
    count_since_last_gyro_recalib++;
#endif

}



/*****************************************************************
// from MPU9150_AHRS.ino:
SFE_MPU9150 Library AHRS Data Fusion Example Code
Kris Winer for Sparkfun Electronics
Original Creation Date: April 8, 2014
https://github.com/sparkfun/MPU9150_Breakout

The MPU9150 is a versatile 9DOF sensor. It has a built-in
accelerometer, gyroscope, and magnetometer that
functions over I2C. It is very similar to the 6 DoF MPU6050 for which an extensive library has already been built.
Most of the function of the MPU9150 can utilize the MPU6050 library.

This Arduino sketch utilizes Jeff Rowberg's MPU6050 library to generate the basic sensor data
for use in two sensor fusion algorithms becoming increasingly popular with DIY quadcopter and robotics engineers. 
I have added and slightly modified Jeff's library here.

This simple sketch will demo the following:
* How to create a MPU6050 object, using a constructor (global variables section).
* How to use the initialize() function of the MPU6050 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readAcceleration(), readRotation(), and readMag() functions and the
  gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, magnetic
  field strength using the  specified ranges as described in the data sheet:
  http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/IMU/PS-MPU-9150A.pdf
  and
  http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/IMU/RM-MPU-9150A-00.pdf.
  
In addition, the sketch will demo:
* How to check for data updates using the data ready status register
* How to display output at a rate different from the sensor data update and fusion filter update rates
* How to specify the accelerometer and gyro sampling and bandwidth rates
* How to use the data from the MPU9150 to fuse the sensor data into a quaternion representation of the sensor frame
  orientation relative to a fixed Earth frame providing absolute orientation information for subsequent use.
* An example of how to use the quaternion data to generate standard aircraft orientation data in the form of
  Tait-Bryan angles representing the sensor yaw, pitch, and roll angles suitable for any vehicle stablization control application.

Hardware setup: This library supports communicating with the
MPU9150 over I2C. These are the only connections that need to be made:
	MPU9150 --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VDD ------------- 3.3V
	 GND ------------- GND

The MPU9150 has a maximum voltage of 3.5V. Make sure you power it
off the 3.3V rail! And either use level shifters between SCL
and SDA or just use a 3.3V Arduino Pro.	  

In addition, this sketch uses a Nokia 5110 48 x 84 pixel display which requires 
digital pins 5 - 9 described below. If using SPI you might need to press one of the A0 - A3 pins
into service as a digital input instead.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Pro 3.3V/8MHz
	MPU9150 Breakout Version: 1.0

This code is beerware. If you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please 
buy us a round!

Distributed as-is; no warranty is given.
*/


// from MPU9150_AHRS.ino:
// // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
// #define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
// #define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// // There is a tradeoff in the beta parameter between accuracy and response speed.
// // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
// #define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
// #define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
// #define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
// #define Ki 0.0f
// 
// int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3;     // raw data arrays reading
// uint16_t count = 0;  // used to control display output rate
// uint16_t delt_t = 0; // used to control display output rate
// uint16_t mcount = 0; // used to control display output rate // used for mag sampling 
// uint8_t MagRate;     // read rate for magnetometer data
// float pitch, yaw, roll;
// float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
// float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
// float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

  //  from MPU9150_AHRS.ino:
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  //   MadgwickQuaternionUpdate(a[0], a[1], a[2], g[0], g[1], g[2], m[0],  m[1], m[2]);  // recheck mag coordinate system if reusing this
  //   MahonyQuaternionUpdate(a[0], a[1], a[2], g[0], g[1], g[2], m[0],  m[1], m[2]);  // recheck mag coordinate system if reusing this


  //  from MPU9150_AHRS.ino:
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  //    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  //    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  //    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  //    pitch *= 180.0f / PI;
  //    yaw   *= 180.0f / PI - 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  //    roll  *= 180.0f / PI;
  //


    //  from MPU9150_AHRS.ino:
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!





//  from MPU9150_AHRS.ino:
// // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// // (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// // which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// // device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// // The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// // but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
//         void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
//         {
//             float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
//             float norm;
//             float hx, hy, _2bx, _2bz;
//             float s1, s2, s3, s4;
//             float qDot1, qDot2, qDot3, qDot4;
// 
//             // Auxiliary variables to avoid repeated arithmetic
//             float _2q1mx;
//             float _2q1my;
//             float _2q1mz;
//             float _2q2mx;
//             float _4bx;
//             float _4bz;
//             float _2q1 = 2.0f * q1;
//             float _2q2 = 2.0f * q2;
//             float _2q3 = 2.0f * q3;
//             float _2q4 = 2.0f * q4;
//             float _2q1q3 = 2.0f * q1 * q3;
//             float _2q3q4 = 2.0f * q3 * q4;
//             float q1q1 = q1 * q1;
//             float q1q2 = q1 * q2;
//             float q1q3 = q1 * q3;
//             float q1q4 = q1 * q4;
//             float q2q2 = q2 * q2;
//             float q2q3 = q2 * q3;
//             float q2q4 = q2 * q4;
//             float q3q3 = q3 * q3;
//             float q3q4 = q3 * q4;
//             float q4q4 = q4 * q4;
// 
//             // Normalise accelerometer measurement
//             norm = sqrt(ax * ax + ay * ay + az * az);
//             if (norm == 0.0f) return; // handle NaN
//             norm = 1.0f/norm;
//             ax *= norm;
//             ay *= norm;
//             az *= norm;
// 
//             // Normalise magnetometer measurement
//             norm = sqrt(mx * mx + my * my + mz * mz);
//             if (norm == 0.0f) return; // handle NaN
//             norm = 1.0f/norm;
//             mx *= norm;
//             my *= norm;
//             mz *= norm;
// 
//             // Reference direction of Earth's magnetic field
//             _2q1mx = 2.0f * q1 * mx;
//             _2q1my = 2.0f * q1 * my;
//             _2q1mz = 2.0f * q1 * mz;
//             _2q2mx = 2.0f * q2 * mx;
//             hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
//             hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
//             _2bx = sqrt(hx * hx + hy * hy);
//             _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
//             _4bx = 2.0f * _2bx;
//             _4bz = 2.0f * _2bz;
// 
//             // Gradient decent algorithm corrective step
//             s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//             s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//             s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//             s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//             norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
//             norm = 1.0f/norm;
//             s1 *= norm;
//             s2 *= norm;
//             s3 *= norm;
//             s4 *= norm;
// 
//             // Compute rate of change of quaternion
//             qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
//             qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
//             qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
//             qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
// 
//             // Integrate to yield quaternion
//             q1 += qDot1 * deltat;
//             q2 += qDot2 * deltat;
//             q3 += qDot3 * deltat;
//             q4 += qDot4 * deltat;
//             norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
//             norm = 1.0f/norm;
//             q[0] = q1 * norm;
//             q[1] = q2 * norm;
//             q[2] = q3 * norm;
//             q[3] = q4 * norm;
// 
//         }
//   
//   
//   
//  // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
//  // measured ones. 
//             void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
//         {
//             float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
//             float norm;
//             float hx, hy, bx, bz;
//             float vx, vy, vz, wx, wy, wz;
//             float ex, ey, ez;
//             float pa, pb, pc;
// 
//             // Auxiliary variables to avoid repeated arithmetic
//             float q1q1 = q1 * q1;
//             float q1q2 = q1 * q2;
//             float q1q3 = q1 * q3;
//             float q1q4 = q1 * q4;
//             float q2q2 = q2 * q2;
//             float q2q3 = q2 * q3;
//             float q2q4 = q2 * q4;
//             float q3q3 = q3 * q3;
//             float q3q4 = q3 * q4;
//             float q4q4 = q4 * q4;   
// 
//             // Normalise accelerometer measurement
//             norm = sqrt(ax * ax + ay * ay + az * az);
//             if (norm == 0.0f) return; // handle NaN
//             norm = 1.0f / norm;        // use reciprocal for division
//             ax *= norm;
//             ay *= norm;
//             az *= norm;
// 
//             // Normalise magnetometer measurement
//             norm = sqrt(mx * mx + my * my + mz * mz);
//             if (norm == 0.0f) return; // handle NaN
//             norm = 1.0f / norm;        // use reciprocal for division
//             mx *= norm;
//             my *= norm;
//             mz *= norm;
// 
//             // Reference direction of Earth's magnetic field
//             hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
//             hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
//             bx = sqrt((hx * hx) + (hy * hy));
//             bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);
// 
//             // Estimated direction of gravity and magnetic field
//             vx = 2.0f * (q2q4 - q1q3);
//             vy = 2.0f * (q1q2 + q3q4);
//             vz = q1q1 - q2q2 - q3q3 + q4q4;
//             wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
//             wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
//             wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  
// 
//             // Error is cross product between estimated direction and measured direction of gravity
//             ex = (ay * vz - az * vy) + (my * wz - mz * wy);
//             ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
//             ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
//             if (Ki > 0.0f)
//             {
//                 eInt[0] += ex;      // accumulate integral error
//                 eInt[1] += ey;
//                 eInt[2] += ez;
//             }
//             else
//             {
//                 eInt[0] = 0.0f;     // prevent integral wind up
//                 eInt[1] = 0.0f;
//                 eInt[2] = 0.0f;
//             }
// 
//             // Apply feedback terms
//             gx = gx + Kp * ex + Ki * eInt[0];
//             gy = gy + Kp * ey + Ki * eInt[1];
//             gz = gz + Kp * ez + Ki * eInt[2];
// 
//             // Integrate rate of change of quaternion
//             pa = q2;
//             pb = q3;
//             pc = q4;
//             q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
//             q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
//             q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
//             q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);
// 
//             // Normalise quaternion
//             norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
//             norm = 1.0f / norm;
//             q[0] = q1 * norm;
//             q[1] = q2 * norm;
//             q[2] = q3 * norm;
//             q[3] = q4 * norm;
//  
//         }
        

      




