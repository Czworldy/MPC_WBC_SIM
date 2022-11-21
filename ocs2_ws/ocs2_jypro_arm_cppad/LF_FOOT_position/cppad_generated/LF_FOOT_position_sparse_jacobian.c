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

void LF_FOOT_position_sparse_jacobian(double const *const * in,
                                      double*const * out,
                                      struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[34];

   v[0] = cos(x[14]);
   v[1] = -0.3 + -0.33 * v[0];
   v[2] = cos(x[13]);
   v[3] = -1 * sin(x[14]);
   v[4] = -0.33 * v[3];
   v[5] = -1 * sin(x[13]);
   v[6] = 0 - v[5];
   v[7] = v[1] * v[2] + v[4] * v[6];
   v[8] = cos(x[12]);
   v[9] = sin(x[12]);
   v[10] = v[7] * v[8] + 0.12325 * v[9];
   v[11] = cos(x[11]);
   v[12] = 0 - v[9];
   v[13] = 0.08 + v[7] * v[12] + 0.12325 * v[8];
   v[14] = sin(x[11]);
   v[15] = v[10] * v[11] + v[13] * v[14];
   v[16] = sin(x[10]);
   v[17] = v[1] * v[5] + v[4] * v[2] + 0.292;
   v[18] = cos(x[10]);
   v[19] = sin(x[9]);
   v[20] = 0 - v[13];
   v[21] = cos(x[9]);
   jac[1] = 0 - (v[15] * v[16] + v[17] * v[18]) * v[19] + (v[10] * v[14] + v[20] * v[11]) * v[21];
   v[22] = cos(x[9]);
   v[23] = sin(x[10]);
   v[24] = cos(x[10]);
   jac[2] = 0 - v[17] * v[22] * v[23] + v[15] * v[22] * v[24];
   v[17] = v[22] * v[16];
   v[15] = sin(x[9]);
   v[25] = sin(x[11]);
   v[26] = cos(x[11]);
   jac[3] = 0 - (v[10] * v[17] + v[20] * v[15]) * v[25] + (v[10] * v[15] + v[13] * v[17]) * v[26];
   v[20] = v[17] * v[11] + v[15] * v[14];
   v[17] = v[17] * v[14] - v[15] * v[11];
   v[13] = sin(x[12]);
   v[10] = cos(x[12]);
   jac[4] = 0 - (v[7] * v[20] + 0.12325 * v[17]) * v[13] + (0.12325 * v[20] - v[7] * v[17]) * v[10];
   v[17] = v[17] * v[12] + v[20] * v[8];
   v[20] = v[22] * v[18];
   v[7] = sin(x[13]);
   v[27] = cos(x[13]);
   jac[5] = 0 - (v[1] * v[17] + v[4] * v[20]) * v[7] + (v[1] * v[20] - v[4] * v[17]) * -1 * v[27];
   v[4] = sin(x[14]);
   v[1] = cos(x[14]);
   jac[6] = 0 - -0.33 * (v[20] * v[5] + v[17] * v[2]) * v[4] + -0.33 * (v[20] * v[2] + v[17] * v[6]) * -1 * v[1];
   v[20] = -0.3 + -0.33 * v[0];
   v[17] = -0.33 * v[3];
   v[28] = v[20] * v[2] + v[17] * v[6];
   v[29] = v[28] * v[8] + 0.12325 * v[9];
   v[30] = 0 - v[29];
   v[31] = 0.08 + v[28] * v[12] + 0.12325 * v[8];
   v[32] = v[29] * v[11] + v[31] * v[14];
   v[33] = v[20] * v[5] + v[17] * v[2] + 0.292;
   jac[8] = 0 - (v[30] * v[14] + v[31] * v[11]) * v[19] + (v[32] * v[16] + v[33] * v[18]) * v[21];
   jac[9] = 0 - v[33] * v[15] * v[23] + v[32] * v[15] * v[24];
   v[33] = v[15] * v[16];
   jac[10] = 0 - (v[29] * v[33] + v[31] * v[22]) * v[25] + (v[30] * v[22] + v[31] * v[33]) * v[26];
   v[31] = v[33] * v[11] - v[22] * v[14];
   v[33] = v[33] * v[14] + v[22] * v[11];
   jac[11] = 0 - (v[28] * v[31] + 0.12325 * v[33]) * v[13] + (0.12325 * v[31] - v[28] * v[33]) * v[10];
   v[33] = v[33] * v[12] + v[31] * v[8];
   v[15] = v[15] * v[18];
   jac[12] = 0 - (v[20] * v[33] + v[17] * v[15]) * v[7] + (v[20] * v[15] - v[17] * v[33]) * -1 * v[27];
   jac[13] = 0 - -0.33 * (v[15] * v[5] + v[33] * v[2]) * v[4] + -0.33 * (v[15] * v[2] + v[33] * v[6]) * -1 * v[1];
   v[0] = -0.3 + -0.33 * v[0];
   v[3] = -0.33 * v[3];
   v[15] = v[0] * v[2] + v[3] * v[6];
   v[9] = v[15] * v[8] + 0.12325 * v[9];
   v[33] = 0.08 + v[15] * v[12] + 0.12325 * v[8];
   jac[15] = 0 - (v[9] * v[11] + v[33] * v[14]) * v[23] + (0 - (v[0] * v[5] + v[3] * v[2] + 0.292)) * v[24];
   jac[16] = 0 - v[9] * v[18] * v[25] + v[33] * v[18] * v[26];
   v[11] = v[18] * v[11];
   v[18] = v[18] * v[14];
   jac[17] = 0 - (v[15] * v[11] + 0.12325 * v[18]) * v[13] + (0.12325 * v[11] - v[15] * v[18]) * v[10];
   v[18] = v[18] * v[12] + v[11] * v[8];
   v[16] = 0 - v[16];
   jac[18] = 0 - (v[0] * v[18] + v[3] * v[16]) * v[7] + (v[0] * v[16] - v[3] * v[18]) * -1 * v[27];
   jac[19] = 0 - -0.33 * (v[16] * v[5] + v[18] * v[2]) * v[4] + -0.33 * (v[16] * v[2] + v[18] * v[6]) * -1 * v[1];
   // dependent variables without operations
   jac[0] = 1;
   jac[7] = 1;
   jac[14] = 1;
}

