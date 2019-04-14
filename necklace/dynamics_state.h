/* ***************************************************************** 
   dynamics_state.h: state for mpu samples and orientation calculation 
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under the
   BSD-2-Clause license in LICENSE  */


// sample timestamping (microseconds)
uint64_t now_sample=0, last_now_sample=0, now_offset, now_origin; 
uint64_t now=0, last_now=0;
uint64_t last_mag_now=0;

//uint64_t last_now_print=0;

float deltat = 0.0f;       // interval since last accel/gyro sample (s)
float rate = 1.0f;         // rate of accel/gyro sampling (Hz)
float deltatmag = 0.0f;    // interval since last mag sample (s)

uint16_t first_cycles=0;   // non-wrapping cycle count, for first few cycles
uint16_t sample_count = 0; // wrapping cycle count, used for mag sampling 

// sampled sensor data
int16_t araw[3], graw[3], mraw[3]; // raw sensor data (except mraw is transformed into the same coordinate system as the others), in sensor units
int16_t mcalib[3];       // mraw after applying calibration
float a[3], g[3], m[3];  // accel (g), gyro (rad/s), mag (mG) 

// smoothed acceleration, either rolling mean or low-pass filtered
float a_smoothed[3];
#ifdef ROLLING_MEAN
uint16_t rolling_window_size; // dynamic rolling window size for accel rolling mean
typedef struct { 
  float a[3];          
} sample;
sample buffer[rolling_window_buffer_size];  // circular buffer
float tmp_mean;
#endif

uint16_t buffer_index=0;  // index into acceleration rolling-mean buffer or gyro dynamic recalibration raw_buffer

// calculation of north estimate wrt acceleration rolling mean
float wx[3]; // world horizontal, in device coords, and perpendicular to device Y axis
float wy[3]; // also in world horizontal, in device coords
float wz[3]; // world vertical, in device coords
float m_along_wz[3]; // project mag vector onto wz
float m_prime[3];    // normalise (m - m_along_wz)
float north_angle;   // angle clockwise (looking down wz) from wx to m_prime
float n;             // intermediate 
float gwz;           // size of gyro vector projected onto wz
float gwz_integrated=0;         // gwz naively integrated
float c;                        // gyro/accel complementary filter parameter
float gwz_integrated_magged=0;  // gwz integrated and combined with north_angle using a complementary filter

#ifdef LOW_PASS_LOW_ANGULAR_RATE_SMOOTHED
float gwz_integrated_magged_smoothed=0; 
float gims_error;
#endif 

// rolling buffer of raw values, doing at-rest detection to automatically recalibrate gyro origin
#ifdef GYRO_DYNAMIC_RECALIBRATION
int16_t raw_buffer[3][rolling_window_buffer_size][3];  // circular buffer
int16_t graw_mean[3]={0,0,0};
uint16_t count_since_last_gyro_recalib=0;
#endif 

// calculation of orientation LED positions from north estimate
uint16_t iy;
float iyfloat,iyoffset,dummy;

// calculation of accel perpendicular to acceleration rolling mean
#ifdef ACCEL_LEDS
float a_along_wz[3];
float a_prime[3];
float a_prime_mod;
float accel_angle;
float awx, awy; // projected accel to the parts in the wx/wy plane
#endif

// calculation of accel LED positions
#ifdef ACCEL_LEDS
uint16_t  ia;
float iafloat,iaoffset;
#endif

// suppress smoothing to mag when rotation rate is high
#ifdef SUPPRESS_MAG_WHEN_ROTATING_FAST
#define SUPPRESS_MAG 0
#define SUPPRESS_MAG_RECOVERY 1
#define SUPPRESS_MAG_OFF 2
uint8_t suppress_mag=SUPPRESS_MAG_OFF;
#endif 




