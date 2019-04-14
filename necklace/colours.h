/* ***************************************************************** 
   colours.h: each cardinal-point region of LEDs has a base colour
   from a random walk within a fixed RGB cuboid.  Each random walk
   has a current position, velocity, and duration (in cycles). When
   the duration reaches 0, a new random velocity is chosen. The walk
   bounces off the internal faces of the RGB cuboid.                 
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under the
   BSD-2-Clause license in LICENSE  */



// ************** colour random walk configuration  **************

#define ncols 4        // number of simultaneous random walks
#define col_scale 127  // scaling for colour values within an int16_t
#define typical_cuboid_width 60 /* RGB values */ 
#define typical_rate 50.0f /* Hz */
#define desired_fastest_traversal_duration 10.0f /* s */

#define random_v_max /* scaledpixels/step */ (col_scale * (typical_cuboid_width / typical_rate) / desired_fastest_traversal_duration )

uint16_t random_segment_max /* steps */ = typical_cuboid_width / random_v_max;  /* say one traversal */

// bounding RGB cuboid for each colour (in unscaled values)
int16_t bound_max_min[ncols][3][2] = { 
  {{100,40}, { 30, 0}, { 30, 0}},
  {{ 40, 0}, {100,20}, { 40, 0}},
  {{ 20, 0}, { 60, 0}, { 80,20}},
  {{ 60,30}, { 20, 0}, { 60,30}}};

// **************** colour random walk state **********************

int16_t col[ncols][3]; // current colours
int16_t v[ncols][3];   // current colour velocities
uint16_t remaining_segment_duration[ncols]={0,0,0,0};

/* initialise colour random walk state, with each colour in the middle of its bounds */
void initialise_colour_walk() {
  for (k=0; k<ncols; k++) {
    for (i=0; i<3; i++) {
      bound_max_min[k][i][0]=bound_max_min[k][i][0]*col_scale;
      bound_max_min[k][i][1]=bound_max_min[k][i][1]*col_scale;
#ifdef MEASURE_POWER
      col[k][i]=bound_max_min[k][i][0]; 
#else
      col[k][i]=(bound_max_min[k][i][0]+bound_max_min[k][i][1])/2; 
#endif
      v[k][i]=0;
    }
  }
}

/* take a step in the colour random walks, bouncing off the bounding cuboid in colour space */
void step_colour_walk() {
  for (k=0; k<ncols; k++) {
    /* occasionally pick a new random velocity */
    if (remaining_segment_duration[k] == 0) {
      remaining_segment_duration[k] = random(random_segment_max);
      for (i=0;i<3;i++)
        v[k][i] = random(-random_v_max,random_v_max);
    }
    remaining_segment_duration[k]--;

    int16_t newc;
    for (i=0;i<3;i++) {
      newc=col[k][i]+v[k][i];
      if (bound_max_min[k][i][0]==bound_max_min[k][i][1])
        newc = bound_max_min[k][i][0];
      else {
        if (newc < bound_max_min[k][i][1]) {
          newc = bound_max_min[k][i][1] + (bound_max_min[k][i][1] - newc);
          v[k][i] = -v[k][i]; 
        }
        if (newc > bound_max_min[k][i][0]) {
          newc = bound_max_min[k][i][0] - (newc - bound_max_min[k][i][0]);
          v[k][i] = -v[k][i];
        }
      }
#ifdef MEASURE_POWER
#else
      col[k][i]=newc;
#endif
    }
  }
#ifdef PRINT_COLOUR_STATE
  print_uint16_vector("col",&col[0],"");
  print_int16_vector("v",&v[0],"");
#endif
}


// ****************** set the cardinal-point blobs of pixels ************
void setpixels(uint16_t iy, float iyoffset, float brightness_scale,float gwz) {
  int16_t c[3];
  float colour_offset, antialias, vscale;
  uint16_t iylk;

#ifdef PRINT_MEMORY_USAGE
  check_mem("setpixels");
#endif

  if (gwz>0.1) 
    vscale = 100.0;
  else 
    vscale = -100.0;

  // three conceptual pixels antialiased onto four LED pixels
  //          AAAAABBBBBCCCCC        iyoffset = 0.6
  //       00000111112222233333
  // 1-iyoffset  1     1   iyoffset          <-- brightness

  // colour gradient based on distance of centre of LED pixel from
  //  centre of block of conceptual pixels:
  // 1+iyoffset 
  //            iyoffset
  //                   1-iyoffset
  //                        2-iyoffset
  //  1..2       0..1   1..0   2..1`

  // start with the random-walk colour
  // calculate anti-aliased pixel values for the left and right end of the blob
  // shift the colour of each LED pixel by vscale times its colour velocity
  // flipping vscale based on the direction of rotation, slightly offset from zero to avoid flicker at rest
  // scale by brightness_scale (based on speed of rotation)

  // all this float calculation does hit the frame rate - one might think it better to
  // refactor into fixed-point, but a simple version of that actually
  // made it slower

  float scale = brightness_scale / (float)col_scale;
  for (k=0;k<4;k++) {   // for each LED pixel of the blobs
    switch (k) {
    case 0: colour_offset = 1.0+iyoffset;    antialias = 1.0-iyoffset; break;
    case 1: colour_offset = iyoffset;        antialias = 1.0;          break;
    case 2: colour_offset = -(1.0-iyoffset); antialias = 1.0;          break;
    case 3: colour_offset = -(2.0-iyoffset); antialias = iyoffset;     break;
    default: break; // never happens
    }
    float common_scale = (antialias * brightness_scale / (float)col_scale);
    float vfactor = colour_offset * vscale;
    for (l=0;l<4;l++) { // for each cardinal-point colour blob
      iylk = (iy + (l*npixels)/4 + k) % npixels;
      for (j=0;j<3;j++) { // for each colour channel
        c[j] =  (int16_t)((((float)col[l][j]) + vfactor * (float)v[l][j]) * common_scale);
        if (c[j] > 255) c[j]=255;
        if (c[j] <0)    c[j]=0;
      }
      strip.setPixelColor( iylk , c[0], c[1], c[2]);
    }
  }
}
