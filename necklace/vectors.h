/* ***************************************************************** 
 vectors.h: simple vector library
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */

#define pi 3.14159
#define twopi (2.0*pi)

//const float vector_unit_x[3] = {1,0,0};
const float vector_unit_y[3] = {0,1,0};
//const float vector_unit_z[3] = {0,0,1};

void vector_cross_product(float *v, float *va, float*vb) {
  v[0] = va[1]*vb[2] - va[2]*vb[1];
  v[1] = va[2]*vb[0] - va[0]*vb[2];
  v[2] = va[0]*vb[1] - va[1]*vb[0];
}

float vector_dot_product(float *va, float*vb) {
  return(va[0]*vb[0] + va[1]*vb[1] + va[2]*vb[2]);
}

float vector_modulus(float *va) {
  return(sqrt(va[0]*va[0] + va[1]*va[1] + va[2]*va[2]));
}

void vector_normalise(float *v, float *va) {
  float m;
  m = vector_modulus(va);
  v[0]=va[0]/m;
  v[1]=va[1]/m;
  v[2]=va[2]/m;
}

float vector_normalise_mod(float *v, float *va) {
  float m;
  m = vector_modulus(va);
  v[0]=va[0]/m;
  v[1]=va[1]/m;
  v[2]=va[2]/m;
  return m;
}

void scalar_vector_mul(float *v, float f, float *va) {
  v[0]=f*va[0];
  v[1]=f*va[1];
  v[2]=f*va[2];
}

void vector_copy(float *v, float *va) {
  v[0]=va[0];
  v[1]=va[1];
  v[2]=va[2];
}

void vector_sub(float *v, float *va, float* vb) {
  v[0]=va[0]-vb[0];
  v[1]=va[1]-vb[1];
  v[2]=va[2]-vb[2];
}


// normalise 
void normalise_twopi(float *p) {
  if (*p < 0) *p += twopi;
  if (*p >= twopi) *p -= twopi;
}
