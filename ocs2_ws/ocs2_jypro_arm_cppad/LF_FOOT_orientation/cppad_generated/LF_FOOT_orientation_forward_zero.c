#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void LF_FOOT_orientation_forward_zero(double const *const * in,
                                      double*const * out,
                                      struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[26];

   v[0] = sin(x[10]);
   v[1] = 0 - v[0];
   v[2] = cos(x[13]);
   v[3] = cos(x[10]);
   v[4] = sin(x[11]);
   v[5] = v[3] * v[4];
   v[6] = sin(x[12]);
   v[7] = 0 - v[6];
   v[8] = cos(x[11]);
   v[9] = v[3] * v[8];
   v[10] = cos(x[12]);
   v[11] = v[5] * v[7] + v[9] * v[10];
   v[12] = -1 * sin(x[13]);
   v[13] = 0 - v[12];
   v[14] = v[1] * v[2] + v[11] * v[13];
   v[15] = -1 * sin(x[14]);
   v[11] = v[1] * v[12] + v[11] * v[2];
   v[1] = cos(x[14]);
   v[16] = v[14] * v[15] + v[11] * v[1];
   v[17] = cos(x[9]);
   v[18] = v[17] * v[3];
   v[19] = v[17] * v[0];
   v[20] = sin(x[9]);
   v[21] = v[19] * v[4] - v[20] * v[8];
   v[19] = v[19] * v[8] + v[20] * v[4];
   v[22] = v[21] * v[7] + v[19] * v[10];
   v[23] = v[18] * v[2] + v[22] * v[13];
   v[22] = v[18] * v[12] + v[22] * v[2];
   v[18] = 0 - v[15];
   v[24] = v[23] * v[1] + v[22] * v[18];
   v[0] = v[20] * v[0];
   v[25] = v[0] * v[4] + v[17] * v[8];
   v[0] = v[0] * v[8] - v[17] * v[4];
   v[17] = -1 * -1 * (1 - v[2]) + v[2];
   v[8] = -1 * -1 * (1 - v[1]) + v[1];
   v[4] = (v[25] * v[10] + v[0] * v[6]) * v[17] * v[8];
   v[9] = (v[5] * v[10] + v[9] * v[6]) * v[17] * v[8];
   v[20] = v[20] * v[3];
   v[0] = v[25] * v[7] + v[0] * v[10];
   v[13] = v[20] * v[2] + v[0] * v[13];
   v[0] = v[20] * v[12] + v[0] * v[2];
   v[20] = v[13] * v[15] + v[0] * v[1];
   v[12] = v[9] - v[20];
   v[22] = v[23] * v[15] + v[22] * v[1];
   v[11] = v[14] * v[1] + v[11] * v[18];
   v[14] = v[22] - v[11];
   if( v[24] > v[4] ) {
      v[23] = v[12];
   } else {
      v[23] = v[14];
   }
   v[15] = 0 - v[4];
   v[0] = v[13] * v[1] + v[0] * v[18];
   v[8] = (v[21] * v[10] + v[19] * v[6]) * v[17] * v[8];
   v[17] = v[0] - v[8];
   if( v[24] > v[4] ) {
      v[19] = 1 + v[24] - v[4] - v[16];
   } else {
      v[19] = 1 + v[4] - v[24] - v[16];
   }
   v[21] = 0 - v[4];
   if( v[24] < v[21] ) {
      v[10] = 1 + v[16] - v[24] - v[4];
   } else {
      v[10] = 1 + v[24] + v[4] + v[16];
   }
   if( v[16] < 0 ) {
      v[10] = v[19];
   } else {
      v[10] = v[10];
   }
   if( v[24] < v[15] ) {
      v[19] = v[17];
   } else {
      v[19] = v[10];
   }
   if( v[16] < 0 ) {
      v[19] = v[23];
   } else {
      v[19] = v[19];
   }
   v[23] = 0.5 / sqrt(v[10]);
   v[19] = v[19] * v[23];
   v[8] = v[0] + v[8];
   if( v[24] > v[4] ) {
      v[0] = v[8];
   } else {
      v[0] = v[10];
   }
   v[20] = v[9] + v[20];
   if( v[24] < v[21] ) {
      v[21] = v[20];
   } else {
      v[21] = v[14];
   }
   if( v[16] < 0 ) {
      v[21] = v[0];
   } else {
      v[21] = v[21];
   }
   v[21] = v[21] * v[23];
   if( v[24] > v[4] ) {
      v[8] = v[10];
   } else {
      v[8] = v[8];
   }
   v[11] = v[22] + v[11];
   if( v[24] < v[15] ) {
      v[12] = v[11];
   } else {
      v[12] = v[12];
   }
   if( v[16] < 0 ) {
      v[12] = v[8];
   } else {
      v[12] = v[12];
   }
   v[12] = v[12] * v[23];
   if( v[24] > v[4] ) {
      v[11] = v[11];
   } else {
      v[11] = v[20];
   }
   if( v[24] < v[15] ) {
      v[10] = v[10];
   } else {
      v[10] = v[17];
   }
   if( v[16] < 0 ) {
      v[10] = v[11];
   } else {
      v[10] = v[10];
   }
   v[10] = v[10] * v[23];
   y[0] = v[19] * x[30] + v[21] * x[32] - x[33] * v[12] - v[10] * x[31];
   y[1] = v[19] * x[31] + v[10] * x[30] - x[33] * v[21] - v[12] * x[32];
   y[2] = v[19] * x[32] + v[12] * x[31] - x[33] * v[10] - v[21] * x[30];
}

