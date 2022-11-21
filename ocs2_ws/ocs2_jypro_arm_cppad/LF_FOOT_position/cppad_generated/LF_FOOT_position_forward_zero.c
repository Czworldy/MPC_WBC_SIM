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

void LF_FOOT_position_forward_zero(double const *const * in,
                                   double*const * out,
                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[19];

   v[0] = cos(x[9]);
   v[1] = sin(x[10]);
   v[2] = v[0] * v[1];
   v[3] = sin(x[11]);
   v[4] = sin(x[9]);
   v[5] = cos(x[11]);
   v[6] = v[2] * v[3] - v[4] * v[5];
   v[7] = cos(x[10]);
   v[8] = v[0] * v[7];
   v[9] = cos(x[12]);
   v[2] = v[2] * v[5] + v[4] * v[3];
   v[10] = sin(x[12]);
   v[11] = -1 * sin(x[13]);
   v[12] = 0 - v[10];
   v[13] = v[6] * v[12] + v[2] * v[9];
   v[14] = cos(x[13]);
   v[15] = v[8] * v[11] + v[13] * v[14];
   v[16] = 0 - v[11];
   v[17] = -1 * sin(x[14]);
   v[18] = cos(x[14]);
   y[0] = 0.08 * v[6] + 0.292 * v[8] + x[6] + 0.12325 * (v[6] * v[9] + v[2] * v[10]) + -0.3 * v[15] + -0.33 * ((v[8] * v[14] + v[13] * v[16]) * v[17] + v[15] * v[18]);
   v[15] = v[4] * v[1];
   v[13] = v[15] * v[3] + v[0] * v[5];
   v[4] = v[4] * v[7];
   v[15] = v[15] * v[5] - v[0] * v[3];
   v[0] = v[13] * v[12] + v[15] * v[9];
   v[2] = v[4] * v[11] + v[0] * v[14];
   y[1] = 0.08 * v[13] + 0.292 * v[4] + x[7] + 0.12325 * (v[13] * v[9] + v[15] * v[10]) + -0.3 * v[2] + -0.33 * ((v[4] * v[14] + v[0] * v[16]) * v[17] + v[2] * v[18]);
   v[3] = v[7] * v[3];
   v[1] = 0 - v[1];
   v[7] = v[7] * v[5];
   v[12] = v[3] * v[12] + v[7] * v[9];
   v[11] = v[1] * v[11] + v[12] * v[14];
   y[2] = 0.08 * v[3] + 0.292 * v[1] + x[8] + 0.12325 * (v[3] * v[9] + v[7] * v[10]) + -0.3 * v[11] + -0.33 * ((v[1] * v[14] + v[12] * v[16]) * v[17] + v[11] * v[18]);
}

