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

void Gripper_Point_orientation_forward_zero(double const *const * in,
                                            double*const * out,
                                            struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[34];

   v[0] = sin(x[10]);
   v[1] = 0 - v[0];
   v[2] = sin(x[24]);
   v[3] = 0 - v[2];
   v[4] = cos(x[10]);
   v[5] = sin(x[11]);
   v[6] = v[4] * v[5];
   v[7] = cos(x[24]);
   v[8] = v[1] * v[3] + v[6] * v[7];
   v[9] = cos(x[27]);
   v[6] = v[1] * v[7] + v[6] * v[2];
   v[1] = cos(x[25]);
   v[10] = cos(x[11]);
   v[11] = v[4] * v[10];
   v[12] = sin(x[25]);
   v[13] = 0 - v[12];
   v[14] = v[6] * v[1] + v[11] * v[13];
   v[15] = sin(x[26]);
   v[11] = v[6] * v[12] + v[11] * v[1];
   v[6] = cos(x[26]);
   v[16] = v[14] * v[15] + v[11] * v[6];
   v[17] = sin(x[27]);
   v[18] = v[8] * v[9] + v[16] * v[17];
   v[19] = sin(x[29]);
   v[20] = 0 - v[19];
   v[21] = 0 - v[15];
   v[11] = v[14] * v[6] + v[11] * v[21];
   v[14] = sin(x[28]);
   v[22] = 0 - v[17];
   v[16] = v[8] * v[22] + v[16] * v[9];
   v[8] = cos(x[28]);
   v[23] = v[11] * v[14] + v[16] * v[8];
   v[24] = cos(x[29]);
   v[25] = v[18] * v[20] + v[23] * v[24];
   v[26] = cos(x[9]);
   v[27] = v[26] * v[4];
   v[28] = v[26] * v[0];
   v[29] = sin(x[9]);
   v[30] = v[28] * v[5] - v[29] * v[10];
   v[31] = v[27] * v[7] + v[30] * v[2];
   v[28] = v[28] * v[10] + v[29] * v[5];
   v[32] = v[31] * v[1] + v[28] * v[13];
   v[28] = v[31] * v[12] + v[28] * v[1];
   v[31] = v[32] * v[6] + v[28] * v[21];
   v[30] = v[27] * v[3] + v[30] * v[7];
   v[28] = v[32] * v[15] + v[28] * v[6];
   v[32] = v[30] * v[22] + v[28] * v[9];
   v[27] = 0 - v[14];
   v[33] = v[31] * v[8] + v[32] * v[27];
   v[4] = v[29] * v[4];
   v[29] = v[29] * v[0];
   v[0] = v[29] * v[5] + v[26] * v[10];
   v[3] = v[4] * v[3] + v[0] * v[7];
   v[0] = v[4] * v[7] + v[0] * v[2];
   v[29] = v[29] * v[10] - v[26] * v[5];
   v[13] = v[0] * v[1] + v[29] * v[13];
   v[29] = v[0] * v[12] + v[29] * v[1];
   v[15] = v[13] * v[15] + v[29] * v[6];
   v[0] = v[3] * v[9] + v[15] * v[17];
   v[29] = v[13] * v[6] + v[29] * v[21];
   v[15] = v[3] * v[22] + v[15] * v[9];
   v[3] = v[29] * v[14] + v[15] * v[8];
   v[22] = v[0] * v[24] + v[3] * v[19];
   v[23] = v[18] * v[24] + v[23] * v[19];
   v[3] = v[0] * v[20] + v[3] * v[24];
   v[0] = v[23] - v[3];
   v[28] = v[30] * v[9] + v[28] * v[17];
   v[32] = v[31] * v[14] + v[32] * v[8];
   v[20] = v[28] * v[20] + v[32] * v[24];
   v[16] = v[11] * v[8] + v[16] * v[27];
   v[11] = v[20] - v[16];
   if( v[33] > v[22] ) {
      v[31] = v[0];
   } else {
      v[31] = v[11];
   }
   v[14] = 0 - v[22];
   v[15] = v[29] * v[8] + v[15] * v[27];
   v[32] = v[28] * v[24] + v[32] * v[19];
   v[28] = v[15] - v[32];
   if( v[33] > v[22] ) {
      v[24] = 1 + v[33] - v[22] - v[25];
   } else {
      v[24] = 1 + v[22] - v[33] - v[25];
   }
   v[19] = 0 - v[22];
   if( v[33] < v[19] ) {
      v[29] = 1 + v[25] - v[33] - v[22];
   } else {
      v[29] = 1 + v[33] + v[22] + v[25];
   }
   if( v[25] < 0 ) {
      v[29] = v[24];
   } else {
      v[29] = v[29];
   }
   if( v[33] < v[14] ) {
      v[24] = v[28];
   } else {
      v[24] = v[29];
   }
   if( v[25] < 0 ) {
      v[24] = v[31];
   } else {
      v[24] = v[24];
   }
   v[31] = 0.5 / sqrt(v[29]);
   v[24] = v[24] * v[31];
   v[32] = v[15] + v[32];
   if( v[33] > v[22] ) {
      v[15] = v[32];
   } else {
      v[15] = v[29];
   }
   v[3] = v[23] + v[3];
   if( v[33] < v[19] ) {
      v[19] = v[3];
   } else {
      v[19] = v[11];
   }
   if( v[25] < 0 ) {
      v[19] = v[15];
   } else {
      v[19] = v[19];
   }
   v[19] = v[19] * v[31];
   if( v[33] > v[22] ) {
      v[32] = v[29];
   } else {
      v[32] = v[32];
   }
   v[16] = v[20] + v[16];
   if( v[33] < v[14] ) {
      v[0] = v[16];
   } else {
      v[0] = v[0];
   }
   if( v[25] < 0 ) {
      v[0] = v[32];
   } else {
      v[0] = v[0];
   }
   v[0] = v[0] * v[31];
   if( v[33] > v[22] ) {
      v[16] = v[16];
   } else {
      v[16] = v[3];
   }
   if( v[33] < v[14] ) {
      v[29] = v[29];
   } else {
      v[29] = v[28];
   }
   if( v[25] < 0 ) {
      v[29] = v[16];
   } else {
      v[29] = v[29];
   }
   v[29] = v[29] * v[31];
   y[0] = v[24] * x[30] + v[19] * x[32] - x[33] * v[0] - v[29] * x[31];
   y[1] = v[24] * x[31] + v[29] * x[30] - x[33] * v[19] - v[0] * x[32];
   y[2] = v[24] * x[32] + v[0] * x[31] - x[33] * v[29] - v[19] * x[30];
}

