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

void Gripper_Point_position_forward_zero(double const *const * in,
                                         double*const * out,
                                         struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[27];

   v[0] = cos(x[9]);
   v[1] = sin(x[10]);
   v[2] = v[0] * v[1];
   v[3] = cos(x[11]);
   v[4] = sin(x[9]);
   v[5] = sin(x[11]);
   v[6] = v[2] * v[3] + v[4] * v[5];
   v[7] = cos(x[10]);
   v[8] = v[0] * v[7];
   v[9] = cos(x[24]);
   v[2] = v[2] * v[5] - v[4] * v[3];
   v[10] = sin(x[24]);
   v[11] = v[8] * v[9] + v[2] * v[10];
   v[12] = cos(x[25]);
   v[13] = sin(x[25]);
   v[14] = 0 - v[13];
   v[15] = v[11] * v[12] + v[6] * v[14];
   v[16] = sin(x[26]);
   v[11] = v[11] * v[13] + v[6] * v[12];
   v[17] = cos(x[26]);
   v[18] = v[15] * v[16] + v[11] * v[17];
   v[19] = 0 - v[16];
   v[11] = v[15] * v[17] + v[11] * v[19];
   v[20] = 0 - v[10];
   v[2] = v[8] * v[20] + v[2] * v[9];
   v[8] = cos(x[27]);
   v[21] = sin(x[27]);
   v[22] = v[2] * v[8] + v[18] * v[21];
   v[23] = sin(x[28]);
   v[24] = 0 - v[21];
   v[2] = v[2] * v[24] + v[18] * v[8];
   v[25] = cos(x[28]);
   v[26] = 0 - v[23];
   y[0] = 0.1813 * v[6] + x[6] + 0.0995 * v[6] + 0.3315 * v[15] + 0.065 * v[18] + 0.1121 * v[11] + 0.00805 * v[22] + 0.283 * v[11] + 0.00366 * (v[11] * v[23] + v[2] * v[25]) + 0.00406 * v[22] + 0.10424 * (v[11] * v[25] + v[2] * v[26]);
   v[2] = v[4] * v[1];
   v[22] = v[2] * v[3] - v[0] * v[5];
   v[4] = v[4] * v[7];
   v[2] = v[2] * v[5] + v[0] * v[3];
   v[0] = v[4] * v[9] + v[2] * v[10];
   v[11] = v[0] * v[12] + v[22] * v[14];
   v[0] = v[0] * v[13] + v[22] * v[12];
   v[18] = v[11] * v[16] + v[0] * v[17];
   v[0] = v[11] * v[17] + v[0] * v[19];
   v[2] = v[4] * v[20] + v[2] * v[9];
   v[4] = v[2] * v[8] + v[18] * v[21];
   v[2] = v[2] * v[24] + v[18] * v[8];
   y[1] = 0.1813 * v[22] + x[7] + 0.0995 * v[22] + 0.3315 * v[11] + 0.065 * v[18] + 0.1121 * v[0] + 0.00805 * v[4] + 0.283 * v[0] + 0.00366 * (v[0] * v[23] + v[2] * v[25]) + 0.00406 * v[4] + 0.10424 * (v[0] * v[25] + v[2] * v[26]);
   v[3] = v[7] * v[3];
   v[1] = 0 - v[1];
   v[7] = v[7] * v[5];
   v[10] = v[1] * v[9] + v[7] * v[10];
   v[14] = v[10] * v[12] + v[3] * v[14];
   v[10] = v[10] * v[13] + v[3] * v[12];
   v[16] = v[14] * v[16] + v[10] * v[17];
   v[10] = v[14] * v[17] + v[10] * v[19];
   v[7] = v[1] * v[20] + v[7] * v[9];
   v[21] = v[7] * v[8] + v[16] * v[21];
   v[7] = v[7] * v[24] + v[16] * v[8];
   y[2] = 0.1813 * v[3] + x[8] + 0.0995 * v[3] + 0.3315 * v[14] + 0.065 * v[16] + 0.1121 * v[10] + 0.00805 * v[21] + 0.283 * v[10] + 0.00366 * (v[10] * v[23] + v[7] * v[25]) + 0.00406 * v[21] + 0.10424 * (v[10] * v[25] + v[7] * v[26]);
}

