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

void RH_FOOT_orientation_forward_zero(double const *const * in,
                                      double*const * out,
                                      struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[27];

   v[0] = sin(x[10]);
   v[1] = cos(x[21]);
   v[2] = -1 * -1 * (1 - v[1]) + v[1];
   v[3] = (0 - v[0]) * v[2];
   v[4] = cos(x[22]);
   v[5] = cos(x[10]);
   v[6] = sin(x[11]);
   v[7] = v[5] * v[6];
   v[8] = -1 * sin(x[21]);
   v[9] = 0 - v[8];
   v[10] = cos(x[11]);
   v[11] = v[5] * v[10];
   v[12] = v[7] * v[9] + v[11] * v[1];
   v[13] = -1 * sin(x[22]);
   v[14] = 0 - v[13];
   v[15] = v[3] * v[4] + v[12] * v[14];
   v[16] = -1 * sin(x[23]);
   v[12] = v[3] * v[13] + v[12] * v[4];
   v[3] = cos(x[23]);
   v[17] = v[15] * v[16] + v[12] * v[3];
   v[18] = cos(x[9]);
   v[19] = v[18] * v[5] * v[2];
   v[20] = v[18] * v[0];
   v[21] = sin(x[9]);
   v[22] = v[20] * v[6] - v[21] * v[10];
   v[20] = v[20] * v[10] + v[21] * v[6];
   v[23] = v[22] * v[9] + v[20] * v[1];
   v[24] = v[19] * v[4] + v[23] * v[14];
   v[23] = v[19] * v[13] + v[23] * v[4];
   v[19] = 0 - v[16];
   v[25] = v[24] * v[3] + v[23] * v[19];
   v[0] = v[21] * v[0];
   v[26] = v[0] * v[6] + v[18] * v[10];
   v[0] = v[0] * v[10] - v[18] * v[6];
   v[18] = -1 * -1 * (1 - v[4]) + v[4];
   v[10] = -1 * -1 * (1 - v[3]) + v[3];
   v[6] = (v[26] * v[1] + v[0] * v[8]) * v[18] * v[10];
   v[11] = (v[7] * v[1] + v[11] * v[8]) * v[18] * v[10];
   v[21] = v[21] * v[5] * v[2];
   v[0] = v[26] * v[9] + v[0] * v[1];
   v[14] = v[21] * v[4] + v[0] * v[14];
   v[0] = v[21] * v[13] + v[0] * v[4];
   v[21] = v[14] * v[16] + v[0] * v[3];
   v[13] = v[11] - v[21];
   v[23] = v[24] * v[16] + v[23] * v[3];
   v[12] = v[15] * v[3] + v[12] * v[19];
   v[15] = v[23] - v[12];
   if( v[25] > v[6] ) {
      v[24] = v[13];
   } else {
      v[24] = v[15];
   }
   v[16] = 0 - v[6];
   v[0] = v[14] * v[3] + v[0] * v[19];
   v[10] = (v[22] * v[1] + v[20] * v[8]) * v[18] * v[10];
   v[18] = v[0] - v[10];
   if( v[25] > v[6] ) {
      v[20] = 1 + v[25] - v[6] - v[17];
   } else {
      v[20] = 1 + v[6] - v[25] - v[17];
   }
   v[22] = 0 - v[6];
   if( v[25] < v[22] ) {
      v[8] = 1 + v[17] - v[25] - v[6];
   } else {
      v[8] = 1 + v[25] + v[6] + v[17];
   }
   if( v[17] < 0 ) {
      v[8] = v[20];
   } else {
      v[8] = v[8];
   }
   if( v[25] < v[16] ) {
      v[20] = v[18];
   } else {
      v[20] = v[8];
   }
   if( v[17] < 0 ) {
      v[20] = v[24];
   } else {
      v[20] = v[20];
   }
   v[24] = 0.5 / sqrt(v[8]);
   v[20] = v[20] * v[24];
   v[10] = v[0] + v[10];
   if( v[25] > v[6] ) {
      v[0] = v[10];
   } else {
      v[0] = v[8];
   }
   v[21] = v[11] + v[21];
   if( v[25] < v[22] ) {
      v[22] = v[21];
   } else {
      v[22] = v[15];
   }
   if( v[17] < 0 ) {
      v[22] = v[0];
   } else {
      v[22] = v[22];
   }
   v[22] = v[22] * v[24];
   if( v[25] > v[6] ) {
      v[10] = v[8];
   } else {
      v[10] = v[10];
   }
   v[12] = v[23] + v[12];
   if( v[25] < v[16] ) {
      v[13] = v[12];
   } else {
      v[13] = v[13];
   }
   if( v[17] < 0 ) {
      v[13] = v[10];
   } else {
      v[13] = v[13];
   }
   v[13] = v[13] * v[24];
   if( v[25] > v[6] ) {
      v[12] = v[12];
   } else {
      v[12] = v[21];
   }
   if( v[25] < v[16] ) {
      v[8] = v[8];
   } else {
      v[8] = v[18];
   }
   if( v[17] < 0 ) {
      v[8] = v[12];
   } else {
      v[8] = v[8];
   }
   v[8] = v[8] * v[24];
   y[0] = v[20] * x[30] + v[22] * x[32] - x[33] * v[13] - v[8] * x[31];
   y[1] = v[20] * x[31] + v[8] * x[30] - x[33] * v[22] - v[13] * x[32];
   y[2] = v[20] * x[32] + v[13] * x[31] - x[33] * v[8] - v[22] * x[30];
}

