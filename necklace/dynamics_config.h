/* ***************************************************************** 
   dynamics_config.h: configuration data for orientation calculation
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under the
   BSD-2-Clause license in LICENSE  */


#define rolling_window_buffer_size 30  // buffer size used for acceleration rolling mean and for at-rest detector
#ifdef ROLLING_MEAN
#define rolling_window_duration  1.0f  // desired rolling mean window duration in seconds
#endif

// initial gyro calibration
int16_t graw_calib[3]={-70, 152, 27};  

// magnetometer calibration (in accel/gyro reference frame): max/min
// values from the rolling-mean client graphs of the raw
// magnetometer data, using a graph_samples=30 window
//const int16_t mrange[3][2] = { {180,-100}, {120,-165}, {20,-250} }; 
// after final assembly:
const int16_t mrange[3][2] = { {143,-143}, {128,-180}, {43,-222} }; 

// take a magnetometer sample only every magrate accel/gyro samples,
// as each costs around 21ms
uint8_t magrate = 4;   

// desired time constant for accel/gyro complementary filter
#define tau 0.2f  

// angular offset to put blue blob roughly at true north (radians)
#define offset_to_north 0.454f


#ifdef SUPPRESS_MAG_WHEN_ROTATING_FAST
// rotation speed above which to suppress magnetometer use
//#define SUPPRESS_MAG_GWZ_THRESHOLD 2.0f
#define SUPPRESS_MAG_GWZ_THRESHOLD 5.0f
#endif

#ifdef LOW_PASS_LOW_ANGULAR_RATE_SMOOTHED
// rotation speed below which to use additional smoothing on orientation estimate, to reduce jitter
#define SMOOTH_GWZ_THRESHOLD (2.0*twopi/72.0)  // two pixels/second
#endif


#ifdef GYRO_DYNAMIC_RECALIBRATION
// minimum interval to try gyro recalibration
#define gyro_recalib_min_interval 5.0   // seconds 

// maximum offsets for raw (calibrated) accel,gyro,mag, from zero, allowed
// for at-rest detection
int32_t maxdiff[3]={200,30,20}; 

// within this many microseconds of startup, blink 8 LEDs white to
// indicate a gyro recalibration
#define gyro_dynamic_recalibration_blink_time   20000000
//#define gyro_dynamic_recalibration_blink_time 60000000
#endif




