/* ***************************************************************** 
   poor_mans_printf.h: crude library to print data to the serial
   output in a readable fixed-width right-justified form, to make
   it readable as it scrolls, without the resource use of the printf
   library  (this might no longer be necessary now that memory 
   consumption is reduced).
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */

void my_print_int(int i) {
  if (i>=-9999 && i<-999) Serial.print("");
  else if (i>-1000 && i<=-100) Serial.print(" ");
  else if (i>-100 && i<=-10) Serial.print("  ");
  else if (i>-10 && i<0) Serial.print("   ");
  else if (i>=0 && i<10) Serial.print("    ");
  else if (i>=10 && i<100) Serial.print("   "); 
  else if (i>=100 && i<1000) Serial.print("  "); 
  else if (i>=1000&& i<10000) Serial.print(" "); 
  Serial.print(i);
}

void my_print_int_tight(int i) {
  if      (i>=0 && i<10) Serial.print("  ");
  else if (i>=10 && i<100) Serial.print(" "); 
  Serial.print(i);
}

void my_print_float(float f, int i) {
  if      (f> -1000 && f<= -100) Serial.print("");
  else if (f> -100  && f<= -10)  Serial.print(" ");
  else if (f> -10   && f<   0 )  Serial.print("  ");
  else if (f>= 0    && f<10   )  Serial.print("   ");
  else if (f>= 10   && f<100  )  Serial.print("  "); 
  else if (f>= 100  && f<1000 )  Serial.print(" "); 
  Serial.print(f,i);
}

void print_scalar(char *s1,float f, int i) {
  Serial.print(s1); Serial.print("=");
  my_print_float(f,i);
  Serial.print(" ");// units: Serial.print(s2);Serial.print(" ");
}

void print_vector(char *s1,float *fp, int i) {
  Serial.print(s1); // units: Serial.print("(");  Serial.print(s2);Serial.print(")");
  Serial.print("=");
  my_print_float(fp[0],i); Serial.print(" ");
  my_print_float(fp[1],i); Serial.print(" ");
  my_print_float(fp[2],i); Serial.print(" ");
}

void print_int16_vector(char *s1,int16_t *ip) {
  Serial.print(s1); Serial.print("=");
  my_print_int(ip[0]); Serial.print(" ");
  my_print_int(ip[1]); Serial.print(" ");
  my_print_int(ip[2]); Serial.print(" ");
  // units:  Serial.print(s2);Serial.print(" ");
}

void print_uint16_vector(char *s1,uint16_t *ip) {
  Serial.print(s1); Serial.print("=");
  my_print_int(ip[0]); Serial.print(" ");
  my_print_int(ip[1]); Serial.print(" ");
  my_print_int(ip[2]); Serial.print(" ");
  // units:  Serial.print(s2);Serial.print(" ");
}

void print_uint16_tight_vector(char *s1,uint16_t *ip) {
  Serial.print(s1); Serial.print("=");
  my_print_int_tight((int)ip[0]); Serial.print(" ");
  my_print_int_tight((int)ip[1]); Serial.print(" ");
  my_print_int_tight((int)ip[2]); Serial.print(" ");
  // units:  Serial.print(s2);Serial.print(" ");
}

//void print_long_vector(char *s1,long *ip) {
//  Serial.print(s1); Serial.print("=");
//  my_print_int(ip[0]); Serial.print(" ");
//  my_print_int(ip[1]); Serial.print(" ");
//  my_print_int(ip[2]); Serial.print(" ");
//  // units:  Serial.print(s2);Serial.print(" ");
//}

void print_int_scalar(char *s1, int i) {
  Serial.print(s1); Serial.print("=");
  my_print_int(i); Serial.print(" ");
  // units:  Serial.print(s2);Serial.print(" ");
}
