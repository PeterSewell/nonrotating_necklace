
/* ***************************************************************** 
   client.c: client code for non-rotating necklace, to read
   data from serial link (either direct or via necklace_receiver
   radio) and plot as live gnuplot graphs.  Can also record
   and replay data streams. 
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */

 
#define _POSIX_C_SOURCE 2
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <inttypes.h>
#include <float.h>
#include <math.h>
#include <unistd.h>
#include <sys/select.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
//#include <linux/kd.h>
//#include "helper_3dmath.h"


#define DEBUG false
//#define DEBUG true

#define show_input true

// send gnuplot commands to stdout instead of the right pipe
//bool fake_gnuplot = true;
bool fake_gnuplot = false;

// show raw a/g/m data graphs
#define showraw  true
//#define showraw  false

// show cooked a/g/m data graphs
//#define showcooked  true
#define showcooked  false

// false to remove graphs of data that isn't included in the radio
#define notradio false

// serial port
const char *device3 =  "/dev/ttyACM4";
const char *device4 =  "/dev/ttyACM3";
const char *device5 =  "/dev/ttyACM5";
const char *device6 =  "/dev/ttyACM2";


// print diagnostic output every this many packets
int print_freq = 1;

// flush file output (if enabled) every this many packets
int flush_file_freq = 100;


// reset world velocity and position every this many packets if faking fsr data
int reset_world_freq = 00; // 1000; // about every 10 seconds

bool reset_position = true; // turn on position reset for pressure/accel step detector position and/or velocity reset


// stored data sample, including that received from the radio and
// derived quantities
typedef struct {
  double ar[3];
  double gr[3];
  double mr[3];
  double mc[3];
  double grc[3];
  double a[3];
  double g[3];
  double m[3];
  double rws;
  double as[3];
  double wx[3];
  double wy[3];
  double wz[3];
  double mawz[3];
  double mp[3];
  double c;
  double gwz;
  double na;
  double gi;
  double gim;
  double aa;
  double apm;
  double now;
  double rate;
  double sc;
  double send_time_reconstructed;
  double received_time;
  double received_rate;
} sample;

double send_time_origin;
double last_now_sample =0;
double now_offset=0;
double last_received_time=0;
double received_time=0;
double last_graph_time=0;

#define buffer_size 1000000
//static_assert(gc.graph_samples <= buffer_size);
sample buffer[buffer_size];  // circular buffer, filled up to count-1

#define rolling_window 5

enum graph_kind { 
  scalar2d,             // 2d graph of a single scalar, with time on the x-axis, drawn with lines
  scalar2dmean,         // ...similar but also drawing the mean over the graph window
  scalar2dmeanrate,     // ...similar but also drawing the mean rate over the graph window
  scalar2ddots,         // ...similar but drawn with dots
  vector2d,             // ...similar but for a vector
  threescalars2d,       // ...similar but for three separate scalars
  twoscalars2d,         // ...similar but for two separate scalars
  scalar2dpersample     // ...similar but with sample count instead of time on the x-axis
};

typedef struct {
  size_t data_offset;
} simple_graph;

typedef struct {
  size_t data_offset[3];
  char label[3][100];
} three_scalars_graph;

typedef struct {
  size_t data_offset[2];
  char label[2][100];
} two_scalars_graph;

union graph_kind_spec { simple_graph simple; three_scalars_graph three_scalars; two_scalars_graph two_scalars;};

typedef struct {
  bool read_from_input;
  bool display;
  char tag[100];
  char title[100];
  enum graph_kind kind;
  union graph_kind_spec gks;
  int xsize, ysize;
  char yrange[100];
  FILE* pipe;   // mutable
  bool data_present; // mutable (only meaningful for read_from_input graphs)
} graph_spec;


// this describes both what (and how) to parse from the serial input, and the graphs to display
graph_spec graphs[] = { 
  //  {true, showraw,       "ar",  "raw acceleration",            vector2d, { .simple={offsetof(sample,ar) }},   300, 250, "[-20000:20000]", NULL, false}, 
  {true, showraw,           "ar",  "raw acceleration",            vector2d, { .simple={offsetof(sample,ar) }},   300, 250, "", NULL, false}, 
  {true, showraw,           "gr",  "raw gyro",                    vector2d, { .simple={offsetof(sample,gr) }},   300, 250, "",   NULL, false}, 
  //  {false,showraw,       "gr",  "raw gyro",                    vector2d, { .simple={offsetof(sample,gr) }},   300, 250, "[-40000:40000]",     NULL, false}, 
  //  {false,showraw,       "gr",  "raw gyro",                    vector2d, { .simple={offsetof(sample,gr) }},   300, 250, "[-40000:40000]",     NULL, false}, 
  //  {false,showraw,       "gr",  "raw gyro",                    vector2d, { .simple={offsetof(sample,gr) }},   300, 250, "[-200:200]",     NULL, false}, 
  {true, showraw,           "mr",  "raw magnetometer",            vector2d, { .simple={offsetof(sample,mr) }},   300, 250, "[-400:400]",     NULL, false}, 
  {true, false,             "mc",  "raw magnetometer (calib)",    vector2d, { .simple={offsetof(sample,mc) }},   300, 250, "",     NULL, false}, 
  {false,false&&showraw,    "",    "raw acceleration x",          scalar2d, { .simple={offsetof(sample,ar) }},   300, 250, "", NULL, false}, 
  {false,false&& showraw,   "",    "raw gyro x",                  scalar2d, { .simple={offsetof(sample,gr) }},   300, 250, "",   NULL, false}, 
  {false,false,             "",    "raw magnetometer (calib) x",  scalar2d, { .simple={offsetof(sample,mc) }},   300, 250, "",   NULL, false}, 
  {false,true&& showraw,    "",    "raw magnetometer x",          scalar2dmean, { .simple={offsetof(sample,mr) }},   300, 250, "",   NULL, false}, 
  {false,true&& showraw,    "",    "raw magnetometer y",          scalar2dmean, { .simple={offsetof(sample,mr)+sizeof(double) }},   300, 250, "",   NULL, false}, 
  {false,true&& showraw,    "",    "raw magnetometer z",          scalar2dmean, { .simple={offsetof(sample,mr)+2*sizeof(double) }},   300, 250, "",   NULL, false}, 
  {true, notradio&&showraw, "grc", "raw gyro calibration",        vector2d, { .simple={offsetof(sample,grc) }},   300, 250, "",   NULL, false}, 
  {true, false,             "now", "now",                         scalar2d, { .simple={offsetof(sample,now) }},  300, 250, "[0:100]",        NULL, false},
  {true, showcooked,        "a",   "acceleration (g)",            vector2d, { .simple={offsetof(sample,a) }},    300, 250, "[-2:2]",         NULL, false}, 
  {true, showcooked,        "g",   "gyro (rad/s)",                vector2d, { .simple={offsetof(sample,g) }},    300, 250, "[-4:4]",         NULL, false}, 
  //  {false,true,          "",   "gyro (rad/s)",                 vector2d, { .simple={offsetof(sample,g) }},    300, 250, "[-10:10]",         NULL, false}, 
  //  {false,true,          "",   "gyro (rad/s)",                 vector2d, { .simple={offsetof(sample,g) }},    300, 250, "[-.05:.05]",         NULL, false}, 
  {true, showcooked,        "m",   "magnetometer (mG)",           vector2d, { .simple={offsetof(sample,m) }},    300, 250, "[-600:600]",     NULL, false}, 
  {true, false,             "rws", "rolling window size",         scalar2d, { .simple={offsetof(sample,rws) }},    300, 250, "[0,100]",     NULL, false}, 
  {true, true||showcooked,  "as",  "acceleration smoothed(g)",    vector2d, { .simple={offsetof(sample,as) }},  300, 250, "[-2:2]",         NULL, false}, 
  {true, false,             "wx",  "world x, in device frame",    vector2d, { .simple={offsetof(sample,wx) }},   300, 250, "[-1.1:1.1]",     NULL, false}, 
  {true, false,             "wy",  "world y, in device frame",    vector2d, { .simple={offsetof(sample,wy) }},   300, 250, "[-1.1:1.1]",     NULL, false}, 
  {true, false,             "wz",  "world z, in device frame",    vector2d, { .simple={offsetof(sample,wz) }},   300, 250, "[-1.1:1.1]",     NULL, false}, 
  {true, false,             "mawz","mawz",                        vector2d, { .simple={offsetof(sample,mawz) }}, 300, 250, "[-600:600]",     NULL, false}, 
  {true, false,             "mp",  "mp",                          vector2d, { .simple={offsetof(sample,mp) }},   300, 250, "[-1.1:1.1]",     NULL, false}, 
  {true, true,              "c",   "filter parameter",            scalar2d, { .simple={offsetof(sample,c) }},    300, 250, ""/*"[0.98:1.01]"*/,     NULL, false}, 
  {true, true,              "gwz", "gwz (gyro wz component,rad/s)",scalar2d, { .simple={offsetof(sample,gwz) }},  300, 250, "[-15:15]",     NULL, false}, 
  {true, false,             "na",  "na (mag north angle,rad)",    scalar2d, { .simple={offsetof(sample,na) }},   300, 250, "[0:6.5]",        NULL, false}, 
  {true, false,             "gi",  "gi (gwz integrated,rad)",     scalar2d, { .simple={offsetof(sample,gi) }},   300, 250, "[0:6.5]",        NULL, false}, 
  {true, false,             "gim", "gim (gyro + mag,rad)",        scalar2d, { .simple={offsetof(sample,gim) }},  300, 250, "[0:6.5]",        NULL, false}, 
  //  {true, true,          "aa", "accel angle",                  scalar2d, { .simple={offsetof(sample,aa) }},  300, 250, "[0:6.5]",        NULL, false}, 
  //  {true, true,          "apm", "accel wxy modulus",           scalar2d, { .simple={offsetof(sample,apm) }},  300, 250, "",        NULL, false}, 
  {false,false,             "",    "na,gim",                      twoscalars2d, { .two_scalars={{offsetof(sample,na),offsetof(sample,gim)},{"na","gim"}}}, 300, 250, "[0:6.5]" , NULL, false},
  {true, true,              "rate","sent rate (Hz)",              scalar2dmeanrate, { .simple={offsetof(sample,rate) }}, 300, 250, "[0:100]",        NULL, false},
  {false,true,              "received rate","received rate (Hz)", scalar2dmeanrate, { .simple={offsetof(sample,received_rate) }}, 300, 250, ""/*"[0:100]"*/,        NULL, false},
  {true, true,              "sc",  "send count",                  scalar2dpersample, { .simple={offsetof(sample,sc) }},   300, 250, "[0:256]",        NULL, false}, 
  {false, true,             "sc",  "send count",                  scalar2ddots, { .simple={offsetof(sample,sc) }},   300, 250, "[0:256]",        NULL, false}, 
  {false,true,              "",    "computed north angles (rad)", threescalars2d, { .three_scalars={{offsetof(sample,na),offsetof(sample,gi),offsetof(sample,gim)},{"magnetometer","gyro integrated","gyro/mag combined"}}}, 400, 250, "[0:6.5]" , NULL, false},
  {false,true,              "",    "computed north angles (rad)", twoscalars2d, { .two_scalars={{offsetof(sample,na),offsetof(sample,gim)},{"magnetometer","gyro/mag combined"}}}, 400, 250, "" , NULL, false}
};

int n_graphs = sizeof(graphs) / sizeof(graph_spec);

int graph_freq=5;
int graph_samples=120;   // plotting doesn't reliably keep up with data rate at 200 
//int graph_samples=30;   // better for mag calibration mean
int graph_plan_samples=300;


bool play_from_file = false;
bool record_to_file = false;
//bool accel_calib = false;
char* filename = NULL;

// blocking reads of a byte and a number of bytes

uint8_t blocking_read(int fd) {
  uint8_t c;
  while (1) {
    int nread = read(fd,&c,1);
    if (nread == 0) continue;
    if (nread == -1) perror("error in blocking_read");
    // printf("%i\n",c);
    // if (c >=32 && c<127) { printf("%c",c); }
    break;
  }
  return(c);
}

void blocking_read2(int fd, uint8_t * buf, size_t n) {
  size_t offset;
  ssize_t nread;
  offset = 0;
  //memset(buf,23,n);
  while (1) {
    nread = read(fd, buf+offset, n-offset);
    //printf("%i\n",nread);
    if (nread == 0) continue;
    if (nread == -1) perror("error in blocking_read2");
    if (nread != n-offset ) {
      offset = offset + nread;
      continue;
    }
    else
      return;
  }
}


#define byte_buffer_size 2000   // definitely more than one line 
char byte_buffer[byte_buffer_size];


int blocking_readline(int fd) {
  // read line into byte_buffer up to the next \n
  int byte_buffer_cursor=0;
  while (true) {
    int bytesAv=0;
    if (ioctl (fd,FIONREAD,&bytesAv) >= 0 ) {
      //printf("Bytes available: %i\n",bytesAv);
      if ((!play_from_file) && bytesAv >= 1000) {
        printf("warning: not keeping up with incoming data rate. Bytes available: %i\n",bytesAv);
        /*
          my_beep(160);
          my_beep(320);
          exit(1);
        */
      }
      char b;
      b = blocking_read(fd);
      //      printf("%c",b);
      byte_buffer[byte_buffer_cursor++] = b;
      if (b=='\n') {
        byte_buffer[byte_buffer_cursor] = 0;
        break;
      }
      if (byte_buffer_cursor == byte_buffer_size) 
        perror("overflowed byte_buffer");
    }
  }
  return byte_buffer_cursor;
}


void perror_exit(const char *s) {
  perror(s);
  exit(1);
}



int kbhit() {
  struct timeval tv = { 0L, 0L };
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv);
}


void my_pclose(FILE *pipe) {
  if (pipe != NULL) pclose(pipe);
}

void my_close(int fd) {
  if (fd != 0) close(fd);
}


void debug(char *s) {
  if (DEBUG) {
    printf("%s",s);
    fflush(stdout);
  }
}


// gnuplot invocation, tiling plots across screen

int gnuplot_pos_x=0, gnuplot_pos_y=0;
int gnuplot_max_x=1600, gnuplot_max_y=900;
int gnuplot_column_max_size_x=0;
int gnuplot_space_x=2, gnuplot_delta_y=65;

void new_gnuplot(graph_spec* g) {
  //int sensor, int size_x, int size_y,char *title,char *init_string) {
  //  size_x = size_x/2;
  //  size_y = size_y/2;
  //printf("new_gnuplot called for %s\n",g->title);
  if (gnuplot_pos_y + g->ysize <= gnuplot_max_y) {
    // place in current column at current y position
    if (g->xsize > gnuplot_column_max_size_x) gnuplot_column_max_size_x = g->xsize;
  }
  else {
    // start new column and place at top
    gnuplot_pos_x = gnuplot_pos_x + gnuplot_column_max_size_x;
    gnuplot_column_max_size_x = g->xsize;
    gnuplot_pos_y = 0;
  }
  char s[100];

  snprintf(s,100,"gnuplot -geometry %ix%i+%i+%i",g->xsize,g->ysize,gnuplot_pos_x,gnuplot_pos_y);
  if (( g->pipe = popen(s, "w")) == NULL) 
    perror_exit("popen for gnuplot");
  else if (fake_gnuplot) 
    g->pipe=stdout;
  gnuplot_pos_y = gnuplot_pos_y+g->ysize+gnuplot_delta_y;
  fprintf(g->pipe,"set term x11 noraise title \"%s\"\n",g->title);
  //if (DEBUG) printf("yrange = %s\n",g->yrange);
  //  fprintf(g->pipe,"\nset xlabel 'sample time (s)'\nset ylabel '%s'\nset xtics\nset ytics\nset format x \"%%g\"\nset yrange %s\nset style data lines\nset grid\n",g->title,g->yrange);
  switch (g->kind) {
  case scalar2dpersample:
    fprintf(g->pipe,"set style data steps\nset grid\nset xlabel 'received sample count'\nset ylabel '%s'\nset xtics 100\nset ytics\nset format x \"%%g\"\n\n",g->title);
    break;
  case scalar2ddots:
    fprintf(g->pipe,"set style data points\nset grid\nset xlabel 'sample time (s)'\nset ylabel '%s'\nset xtics 1.0\nset ytics\nset format x \"%%g\"\n\n",g->title);
    break;
  default:
    fprintf(g->pipe,"set style data steps\nset grid\nset xlabel 'sample time (s)'\nset ylabel '%s'\nset xtics 1.0\nset ytics\nset format x \"%%g\"\n\n",g->title);
    break;
  }
  if (strlen(g->yrange)!=0) fprintf(g->pipe,"set yrange %s\n",g->yrange);
}
      


// ******************************* main ************************



int fd=0, fd_write=0;

void close_all() {
  my_close(fd);
  my_close(fd_write);
  int i;
  for (i=0; i<n_graphs; i++) {
    if (graphs[i].display) 
      my_pclose(graphs[i].pipe);
  }
  exit(1);
}

void intHandler(int dummy) {
  close_all();
}


double sinc(double x) {
  if (fabs(x) < 1.0e-4) return (1.0) - x*x*(0.166666666666666666667);
  else return sin(x)/x;
}

double vector_unit_x[3] = {1,0,0};
double vector_unit_y[3] = {0,1,0};
double vector_unit_z[3] = {0,0,1};

void vector_cross_product(double *v, double *va, double*vb) {
  v[0] = va[1]*vb[2] - va[2]*vb[1];
  v[1] = va[2]*vb[0] - va[0]*vb[2];
  v[2] = va[0]*vb[1] - va[1]*vb[0];
}


double vector_dot_product(double *va, double*vb) {
  return(va[0]*vb[0] + va[1]*vb[1] + va[2]*vb[2]);
}

double vector_modulus(double *va) {
  return(sqrt(va[0]*va[0] + va[1]*va[1] + va[2]*va[2]));
}

void vector_normalise(double *v, double *va) {
  double m = vector_modulus(va);
  v[0]=va[0]/m;
  v[1]=va[1]/m;
  v[2]=va[2]/m;
}

void scalar_vector_mul(double *v, double f, double *va) {
  v[0]=f*va[0];
  v[1]=f*va[1];
  v[2]=f*va[2];
}

void vector_copy(double *v, double *va) {
  v[0]=va[0];
  v[1]=va[1];
  v[2]=va[2];
}

void vector_sub(double *v, double *va, double* vb) {
  v[0]=va[0]-vb[0];
  v[1]=va[1]-vb[1];
  v[2]=va[2]-vb[2];
}

void quaternion_assign(double *q, double *qa) {
  q[0] = qa[0];
  q[1] = qa[1];
  q[2] = qa[2];
  q[3] = qa[3];
}

void quaternion_multiply(double *q, double *qa, double*qb) {
  q[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
  q[1] = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2];
  q[2] = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1];
  q[3] = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0];
}

void quaternion_normalise(double *q, double *qa) {
  double mag = sqrt(qa[0]*qa[0]+qa[1]*qa[1]+qa[2]*qa[2]+qa[3]*qa[3]);
  q[0] = qa[0]/mag;
  q[1] = qa[1]/mag;
  q[2] = qa[2]/mag;
  q[3] = qa[3]/mag;
}

/**
 * Sets a quat from the given angle and rotation axis,
 * then returns it.
 *
 * @param {quat} out the receiving quaternion
 * @param {vec3} axis the axis around which to rotate
 * @param {Number} rad the angle in radians
 * @returns {quat} out
 **/
void quaternion_set_axis_angle(double *out, double *axis, double rad) {
  rad = rad * 0.5;
  double s = sin(rad);
  out[0] = cos(rad);
  out[1] = s * axis[0];
  out[2] = s * axis[1];
  out[3] = s * axis[2];
}

//https://github.com/toji/gl-matrix/blob/f0583ef53e94bc7e78b78c8a24f09ed5e2f7a20c/src/gl-matrix/quat.js#L54
///**
//* Sets a quaternion to represent the shortest rotation from one
//* vector to another.
//*
//* Both vectors are assumed to be unit length.
//*
//* @param {quat} out the receiving quaternion.
//* @param {vec3} a the initial vector
//* @param {vec3} b the destination vector
//* @returns {quat} out
//*/
void quaternion_rotate_between(double *out, double *a, double*b) {
  double tmpvec3[3];
  double xUnitVec3[3] = {1.0f, 0.0f, 0.0f};
  double yUnitVec3[3] = {0.0f, 1.0f, 0.0f};

  double dot = vector_dot_product(a, b);
  if (dot < -0.999999) {
    vector_cross_product(tmpvec3, xUnitVec3, a);
    if (vector_modulus(tmpvec3) < 0.000001) 
      vector_cross_product(tmpvec3, yUnitVec3, a);
    vector_normalise(tmpvec3,tmpvec3);
    quaternion_set_axis_angle(out, tmpvec3, 3.141592654f);
  } else if (dot > 0.999999) {
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
  } else {
    vector_cross_product(tmpvec3, a, b);
    out[0] = 1 + dot;
    out[1] = tmpvec3[0];
    out[2] = tmpvec3[1];
    out[3] = tmpvec3[2];
    quaternion_normalise(out, out);
  }
}

void my_beep(double f) {
  // use sox to make some kind of a beep
  char *system_buffer[200];
  snprintf((char * __restrict__ )system_buffer,200,"play -n output.wav synth 0.05 sine 600 vol %f 2> /dev/null&",f);
  system((char * __restrict__ )system_buffer);
}

double gettime() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return( (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0f);
}



int main(int argc, char **argv) {

  if (argc < 2) 
    {}
  else if ((argc == 3) && (strcmp("-play", argv[1]) == 0)) {
    play_from_file = true;
    filename = argv[2];
  }
  else if ((argc == 3) && (strcmp("-record", argv[1]) == 0)) {
    record_to_file = true;
    filename = argv[2];
  }
  //  else if ((argc == 2) && (strcmp("-accel_calib", argv[1]) == 0)) {
  //    accel_calib = true;
  //  }
  else {
    printf("bad command-line arguments (must be empty, -play <filename>, or -record <filename>)\n");
    //    printf("bad command-line arguments (must be empty, -play filename, -record filename, or -accel_calib)\n");
    exit(1);
  }
  


  signal(SIGINT, intHandler);

  //  int fd_speaker = open("/dev/tty0", O_RDONLY);
  //
  //  if (fd_speaker == -1 ) return -1;
  //  int ms = 5000;
  //  int freq = 440;
  //  ioctl(fd_speaker, KDMKTONE, (ms<<16 | 1193180/freq));

  


  uint64_t i,j,s;


  // open serial port device or file to read from
  char device[100];
  if (play_from_file) {
    fd = open(filename,O_RDONLY,S_IRUSR+S_IWUSR);
    if (fd == -1) {
      printf("file \"%s\":\n",filename);
      perror_exit("error opening file");
    }
  } else {
    fd = open(device3,O_RDWR);
    if (fd != -1) 
      strncpy(device,device3,100);
    else {
      fd = open(device4,O_RDWR);
      if (fd != -1) 
        strncpy(device,device4,100);
      else {
        fd = open(device5,O_RDWR);
        if (fd != -1) 
          strncpy(device,device5,100);
        else {
          fd = open(device6,O_RDWR);
          if (fd != -1) 
            strncpy(device,device6,100);
          else {
            printf("devices \"%s\", \"%s\", \"%s\", \"%s\":\n",device3,device4,device5,device6);
            perror_exit("error opening device");
          }
        }
      }
    }
    printf("opened device \"%s\"\n",device);
  }

  // open file to write to
  if (record_to_file) {
    printf("trying to open %s\n",filename);
    fd_write = open(filename,O_RDWR | O_CREAT, S_IRUSR|S_IWUSR);
    if (fd_write == -1) {
      printf("file \"%s\":\n",filename);
      perror_exit("error opening file");
    }
  }




  uint64_t count=0;   // index of next free space in circular buffer 

  int64_t step_duration_last=0; 
  int64_t step_timestamp_last=0;
  int64_t step_count_last =0;

                                             
  double received_time_origin; 

  // main loop
  while (true) {

    // read data from serial link

    // throw away the first (partial) line of input
    if (count==0) blocking_readline(fd);

    int byte_buffer_read;

    while (true) {
      byte_buffer_read = blocking_readline(fd);

      double t;
      t = gettime();
      if (count==0) {
        received_time_origin=t;
        last_received_time=0;
        received_time=0;
        buffer[count%buffer_size].received_time = 0;
        buffer[count%buffer_size].received_rate = 0;
      } else {
        last_received_time = received_time;
        received_time = t - received_time_origin;
        buffer[count%buffer_size].received_time = received_time;
        buffer[count%buffer_size].received_rate = 1.0 / (buffer[count%buffer_size].received_time - buffer[(count-1)%buffer_size].received_time);
        
      }

      if (DEBUG || show_input) 
        printf("input line:\n%s\n",byte_buffer);
      
      if (record_to_file)
        write(fd_write,byte_buffer,byte_buffer_read);

      
      // now try to parse it, assuming fields in graphs might be absent but are in the right order if present
      i=0;
      for (j=0; j<n_graphs; j++) {
        if (graphs[j].read_from_input) {
          int len = strlen(graphs[j].tag);
          double *p;
          p=(double*)((char*)(&buffer[count % buffer_size]) + graphs[j].gks.simple.data_offset);
          if (memcmp(&byte_buffer[i], graphs[j].tag, len)==0 && byte_buffer[i+len]=='=') {
            int bytes_parsed, nitems;
            switch (graphs[j].kind) {
            case scalar2d:
            case scalar2dmean:
            case scalar2dmeanrate:
            case scalar2dpersample:
            case scalar2ddots:
              nitems = sscanf(&(byte_buffer[i])+len+1," %lf %n",p,&bytes_parsed);
              if (nitems != 1) perror("scalar2d parsing nitems != 1");
              i = i + len + 1 + bytes_parsed;
              graphs[j].data_present=true;
              break;
            case vector2d:
              nitems = sscanf(&(byte_buffer[i])+len+1," %lf %lf %lf %n",p,p+1,p+2,&bytes_parsed);
              if (nitems != 3) perror("vector2d parsing nitems != 3");
              i = i + len + 1 + bytes_parsed;
              graphs[j].data_present=true;
              break;
            default:
              break;
            }
          } else {
            switch (graphs[j].kind) {
            case scalar2d:
            case scalar2dmean:
            case scalar2dmeanrate:
            case scalar2ddots:
            case scalar2dpersample:
              *p=0;
              break;
            case vector2d:
              p[0]=0; p[1]=0; p[2]=0;
              break;
            default:
              break;
            }
          }
        } // read_from_input
        if (i>=byte_buffer_read) goto end_of_parsing;
      } // for
      printf("parse did not consume the whole string\n");
      for (j=0;j<i;j++) printf("%c",byte_buffer[j]);
      printf("***");
      for (j=i;j<byte_buffer_read;j++) printf("%c",byte_buffer[j]);
      printf("\n");
    }

  end_of_parsing:

    debug("end of parsing\n");

    if (true || DEBUG) {
      // print it out again
      printf("parsed data:\n");
      for (j=0; j<n_graphs; j++) {
        if (graphs[j].read_from_input && graphs[j].data_present) {
          double *p;
          p=(double*)((char*)(&buffer[count % buffer_size]) + graphs[j].gks.simple.data_offset);
          printf("%s= ",graphs[j].tag);
          switch (graphs[j].kind) {
          case scalar2d:
          case scalar2dmean:
          case scalar2dmeanrate:
          case scalar2ddots:
          case scalar2dpersample:
            printf("%f ",p[0]);
            break;
          case vector2d:
            printf("%f %f %f ",p[0],p[1],p[2]);
            break;
          default:
            break;
          }
        }
      }
    printf("\n");
    }



    if (count==0) {
      send_time_origin = buffer[count % buffer_size].now;
    } 

    if (buffer[count % buffer_size].now < last_now_sample)
      now_offset += (double)( UINT64_C(1)<<32 );

    last_now_sample = buffer[count % buffer_size].now;

    printf("now: %f\n",buffer[count % buffer_size].now);

    buffer[count % buffer_size].send_time_reconstructed=(buffer[count % buffer_size].now+now_offset-send_time_origin)/1000000.0;

    //    printf("send_time_origin = %f   send_time_reconstructed=%f\n",send_time_origin, buffer[count % buffer_size].send_time_reconstructed);

    double offset = buffer[count % buffer_size].send_time_reconstructed - received_time;
    if (play_from_file && offset > 0.0) {
      double offset_int,offset_frac;
      offset_frac=modf(offset,&offset_int);
      struct timespec req;
      req.tv_sec = (time_t)offset_int;
      req.tv_nsec = (long)(offset_frac * 1e9);
      nanosleep(&req,NULL);
    }


    if (buffer[count % buffer_size].apm < 0.1) buffer[count % buffer_size].aa=0;


    // calculate X range for 2d graphs
    int plot_start_count, plan_start_count, plot_all_start_count;
    if (count < graph_samples) 
      plot_start_count = 0;
    else 
      plot_start_count = count - graph_samples;


    double plot_start_time = buffer[plot_start_count % buffer_size].send_time_reconstructed /*- send_time_origin*/;
    double plot_end_time = buffer[count % buffer_size].send_time_reconstructed /*- send_time_origin*/;
    //printf("plot_start_count %% buffer_size = %i\n",plot_start_count % buffer_size);
    //printf("plot_start_count=%i  count=%i\n",plot_start_count,count);

    double sum,mean;

    printf("count=%" PRId64 "\n",count); 


    if (count!=0 && plot_start_time == plot_end_time) {
      printf("plot_start_time == plot_end_time == %f\n",plot_start_time);
      goto no_graphs;
    } else {
      //      printf("plot_start_time = %f  plot_end_time == %f\n",plot_start_time,plot_end_time);
    }

    int k;
    if (count==0) {
      for (j=0; j<n_graphs; j++) {
        // create pipes for talking to an instance of gnuplot for each graph and initialise gnuplots
        if (graphs[j].display) {
          //printf("calling new_gnuplot for j=%d\n",j);
          new_gnuplot(&graphs[j]);
        }
      }
    } else if (last_graph_time==0.0 || received_time > last_graph_time + 1.0/graph_freq) {
      last_graph_time = received_time;
      for (j=0; j<n_graphs; j++) {
        //      printf("last_graph_time=%f  received_time=%f\n",last_graph_time,received_time);
        if (graphs[j].display) {
          //printf("drawing graph for j=%d\n",j);
          switch (graphs[j].kind) {
          case vector2d : 
            debug("start of graph drawing, vector2d\n");
            //printf("foodf graph=%s, pipe=%p\n",graphs[j].title,(void*)graphs[j].pipe);fflush(stdout);
            fprintf(graphs[j].pipe,"set xrange [%f:%f]\n",plot_start_time,plot_end_time);
            //printf("foodfgh");fflush(stdout);
            fprintf(graphs[j].pipe,"plot \"-\" title 'x', \"-\" title 'y', \"-\" title 'z'\n");
            //printf("foofghj");   fflush(stdout); 
            //fflush(graphs[j].pipe);
            for (k=0; k<3; k++) { 
              for (i=plot_start_count; i<=count; i++) 
                fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.simple.data_offset)+k));
              fprintf(graphs[j].pipe,"e\n");
            }                      
            //          for (i=plot_start_count[sensor]; i<=count[sensor]; i++) 
            //            fprintf(pipe_ypr_2d[sensor],"%" PRIu32 " %f\n", buffer[sensor][i % buffer_size].send_time_reconstructed /* - send_time_origin */, sqrt((double)buffer[sensor][i % buffer_size].rd.ypr_int[0]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[0]+(double)buffer[sensor][i % buffer_size].rd.ypr_int[1]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[1]+(double)buffer[sensor][i % buffer_size].rd.ypr_int[2]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[2]));
            //          fprintf(pipe_ypr_2d[sensor],"e\n");
            fflush(graphs[j].pipe);
            break;
          case scalar2d:
          case scalar2ddots:
            debug("start of graph drawing, scalar2d\n");
            fprintf(graphs[j].pipe,"set xrange [%lf:%lf]\n",plot_start_time,plot_end_time);
            fprintf(graphs[j].pipe,"plot \"-\" title '%s'\n",graphs[j].tag);
            for (i=plot_start_count; i<=count; i++) 
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.simple.data_offset)+0));
            fprintf(graphs[j].pipe,"e\n");
            fflush(graphs[j].pipe);
            break;
          case scalar2dmean:
            debug("start of graph drawing, scalar2dmean\n");
            fprintf(graphs[j].pipe,"set xrange [%lf:%lf]\n",plot_start_time,plot_end_time);
            sum=0;
            for (i=plot_start_count; i<=count; i++) {
              double v;
              v= *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.simple.data_offset)+0);
              sum+=v;
            }
            mean = (sum / (count+1-plot_start_count));
            fprintf(graphs[j].pipe,"plot \"-\" title '%s', \"-\" title 'mean %4.0f'\n",graphs[j].tag,mean);
            for (i=plot_start_count; i<=count; i++) {
              double v;
              v= *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.simple.data_offset)+0);
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, v);
            }
            fprintf(graphs[j].pipe,"e\n");
            for (i=plot_start_count; i<=count; i++) {
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, mean);
            }
            fprintf(graphs[j].pipe,"e\n");
            fflush(graphs[j].pipe);
            break;
          case scalar2dmeanrate:
            debug("start of graph drawing, scalar2dmeanrate\n");
            fprintf(graphs[j].pipe,"set xrange [%lf:%lf]\n",plot_start_time,plot_end_time);
           sum=0;
            for (i=plot_start_count; i<=count; i++) {
              double v;
              v= *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.simple.data_offset)+0);
               sum+=1.0/v;
            }
            mean = 1.0/ (sum / (count+1-plot_start_count));
            fprintf(graphs[j].pipe,"plot \"-\" title '%s', \"-\" title 'mean rate %3.0f'\n",graphs[j].tag,mean);
             for (i=plot_start_count; i<=count; i++) {
              double v;
              v= *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.simple.data_offset)+0);
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, v);
             }
             fprintf(graphs[j].pipe,"e\n");
            for (i=plot_start_count; i<=count; i++) {
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, mean);
            }
            fprintf(graphs[j].pipe,"e\n");
            fflush(graphs[j].pipe);
            break;
          case scalar2dpersample:
            debug("start of graph drawing, scalar2dpersample\n");
            //          fprintf(graphs[j].pipe,"set xrange [%lf:%lf]\n",plot_start_time,plot_end_time);
            fprintf(graphs[j].pipe,"plot \"-\" title '%s'\n",graphs[j].tag);
            for (i=plot_start_count; i<=count; i++) 
              fprintf(graphs[j].pipe,"%f %f\n", (double)i, *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.simple.data_offset)+0));
            fprintf(graphs[j].pipe,"e\n");
            fflush(graphs[j].pipe);
            break;
          case twoscalars2d:
            debug("start of graph drawing, twoscalars2d\n");
            fprintf(graphs[j].pipe,"set style line 2 linecolor 3\n");
            fprintf(graphs[j].pipe,"set xrange [%lf:%lf]\n",plot_start_time,plot_end_time);
            fprintf(graphs[j].pipe,"plot \"-\" title '%s', \"-\" title '%s'\n",graphs[j].gks.two_scalars.label[0],graphs[j].gks.two_scalars.label[1]);
            for (i=plot_start_count; i<=count; i++) 
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.two_scalars.data_offset[0])));
            fprintf(graphs[j].pipe,"e\n");
            for (i=plot_start_count; i<=count; i++) 
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.two_scalars.data_offset[1])));
            fprintf(graphs[j].pipe,"e\n");
            //          for (i=plot_start_count[sensor]; i<=count[sensor]; i++) 
            //            fprintf(pipe_ypr_2d[sensor],"%" PRIu32 " %f\n", buffer[sensor][i % buffer_size].send_time_reconstructed /* - send_time_origin */, sqrt((double)buffer[sensor][i % buffer_size].rd.ypr_int[0]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[0]+(double)buffer[sensor][i % buffer_size].rd.ypr_int[1]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[1]+(double)buffer[sensor][i % buffer_size].rd.ypr_int[2]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[2]));
            //          fprintf(pipe_ypr_2d[sensor],"e\n");
            fflush(graphs[j].pipe);
            break;
          case threescalars2d:
            debug("start of graph drawing, threescalars2d\n");
            fprintf(graphs[j].pipe,"set xrange [%lf:%lf]\n",plot_start_time,plot_end_time);
            fprintf(graphs[j].pipe,"plot \"-\" title '%s', \"-\" title '%s', \"-\" title '%s'\n",graphs[j].gks.three_scalars.label[0],graphs[j].gks.three_scalars.label[1],graphs[j].gks.three_scalars.label[2]);
            for (i=plot_start_count; i<=count; i++) 
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.three_scalars.data_offset[0])));
            fprintf(graphs[j].pipe,"e\n");
            for (i=plot_start_count; i<=count; i++) 
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.three_scalars.data_offset[1])));
            fprintf(graphs[j].pipe,"e\n");
            for (i=plot_start_count; i<=count; i++) 
              fprintf(graphs[j].pipe,"%f %f\n", buffer[i % buffer_size].send_time_reconstructed /* - send_time_origin */, *((double*)((char*)(&(buffer[i % buffer_size]))+graphs[j].gks.three_scalars.data_offset[2])));
            fprintf(graphs[j].pipe,"e\n");
            //          for (i=plot_start_count[sensor]; i<=count[sensor]; i++) 
            //            fprintf(pipe_ypr_2d[sensor],"%" PRIu32 " %f\n", buffer[sensor][i % buffer_size].send_time_reconstructed /* - send_time_origin */, sqrt((double)buffer[sensor][i % buffer_size].rd.ypr_int[0]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[0]+(double)buffer[sensor][i % buffer_size].rd.ypr_int[1]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[1]+(double)buffer[sensor][i % buffer_size].rd.ypr_int[2]*(double)buffer[sensor][i % buffer_size].rd.ypr_int[2]));
            //          fprintf(pipe_ypr_2d[sensor],"e\n");
            fflush(graphs[j].pipe);
            break;
          default:
            break;
          }
        }
      }
    }

    debug("end of graph drawing\n");

  no_graphs:

    count++;
  }  // main loop
}






