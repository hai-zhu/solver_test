/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) di_mpc_ca_cost_ext_cost_fun_jac_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fmax CASADI_PREFIX(fmax)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

casadi_real casadi_fmax(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>y ? x : y;
#else
  return fmax(x, y);
#endif
}

static const casadi_int casadi_s0[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s4[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s5[15] = {6, 6, 0, 1, 2, 4, 6, 6, 6, 0, 1, 2, 3, 2, 3};

/* di_mpc_ca_cost_ext_cost_fun_jac_hess:(i0[4],i1[2],i2[15])->(o0,o1[6],o2[6x6,6nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[2]? arg[2][6] : 0;
  a1=arg[2]? arg[2][2] : 0;
  a2=arg[0]? arg[0][0] : 0;
  a3=(a1-a2);
  a4=casadi_sq(a3);
  a5=arg[2]? arg[2][3] : 0;
  a6=arg[0]? arg[0][1] : 0;
  a7=(a5-a6);
  a8=casadi_sq(a7);
  a4=(a4+a8);
  a8=arg[2]? arg[2][0] : 0;
  a1=(a1-a8);
  a1=casadi_sq(a1);
  a8=arg[2]? arg[2][1] : 0;
  a5=(a5-a8);
  a5=casadi_sq(a5);
  a1=(a1+a5);
  a5=1.;
  a1=casadi_fmax(a1,a5);
  a4=(a4/a1);
  a4=(a0*a4);
  a8=arg[2]? arg[2][7] : 0;
  a9=arg[1]? arg[1][0] : 0;
  a10=casadi_sq(a9);
  a11=arg[1]? arg[1][1] : 0;
  a12=casadi_sq(a11);
  a10=(a10+a12);
  a10=(a8*a10);
  a4=(a4+a10);
  a10=arg[2]? arg[2][8] : 0;
  a12=10.;
  a13=arg[2]? arg[2][10] : 0;
  a2=(a2-a13);
  a13=casadi_sq(a2);
  a14=arg[2]? arg[2][4] : 0;
  a15=arg[2]? arg[2][14] : 0;
  a16=arg[2]? arg[2][12] : 0;
  a16=(a15*a16);
  a14=(a14+a16);
  a14=casadi_sq(a14);
  a13=(a13/a14);
  a16=arg[2]? arg[2][11] : 0;
  a6=(a6-a16);
  a16=casadi_sq(a6);
  a17=arg[2]? arg[2][5] : 0;
  a18=arg[2]? arg[2][13] : 0;
  a18=(a15*a18);
  a17=(a17+a18);
  a17=casadi_sq(a17);
  a16=(a16/a17);
  a13=(a13+a16);
  a13=(a13-a15);
  a13=(a12*a13);
  a13=exp(a13);
  a5=(a5+a13);
  a10=(a10/a5);
  a4=(a4+a10);
  if (res[0]!=0) res[0][0]=a4;
  a9=(a9+a9);
  a9=(a9*a8);
  if (res[1]!=0) res[1][0]=a9;
  a11=(a11+a11);
  a11=(a11*a8);
  if (res[1]!=0) res[1][1]=a11;
  a11=(a2+a2);
  a9=(a10/a5);
  a4=(a13*a9);
  a4=(a12*a4);
  a15=(a4/a14);
  a16=(a11*a15);
  a3=(a3+a3);
  a0=(a0/a1);
  a3=(a3*a0);
  a16=(a16+a3);
  a16=(-a16);
  if (res[1]!=0) res[1][2]=a16;
  a16=(a6+a6);
  a4=(a4/a17);
  a3=(a16*a4);
  a7=(a7+a7);
  a7=(a7*a0);
  a3=(a3+a7);
  a3=(-a3);
  if (res[1]!=0) res[1][3]=a3;
  a3=0.;
  if (res[1]!=0) res[1][4]=a3;
  if (res[1]!=0) res[1][5]=a3;
  a3=2.;
  a7=(a3*a8);
  if (res[2]!=0) res[2][0]=a7;
  a8=(a3*a8);
  if (res[2]!=0) res[2][1]=a8;
  a15=(a3*a15);
  a2=(a2+a2);
  a2=(a2/a14);
  a2=(a12*a2);
  a2=(a13*a2);
  a8=(a9*a2);
  a10=(a10/a5);
  a7=(a10*a2);
  a7=(a7/a5);
  a1=(a9/a5);
  a2=(a1*a2);
  a7=(a7+a2);
  a7=(a13*a7);
  a8=(a8-a7);
  a8=(a12*a8);
  a8=(a8/a14);
  a8=(a11*a8);
  a15=(a15+a8);
  a8=-2.;
  a7=(a8*a0);
  a15=(a15+a7);
  a15=(-a15);
  if (res[2]!=0) res[2][2]=a15;
  a6=(a6+a6);
  a6=(a6/a17);
  a6=(a12*a6);
  a6=(a13*a6);
  a9=(a9*a6);
  a10=(a10*a6);
  a10=(a10/a5);
  a1=(a1*a6);
  a10=(a10+a1);
  a13=(a13*a10);
  a9=(a9-a13);
  a12=(a12*a9);
  a14=(a12/a14);
  a11=(a11*a14);
  a11=(-a11);
  if (res[2]!=0) res[2][3]=a11;
  if (res[2]!=0) res[2][4]=a11;
  a3=(a3*a4);
  a12=(a12/a17);
  a16=(a16*a12);
  a3=(a3+a16);
  a8=(a8*a0);
  a3=(a3+a8);
  a3=(-a3);
  if (res[2]!=0) res[2][5]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_cost_ext_cost_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_cost_ext_cost_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_cost_ext_cost_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void di_mpc_ca_cost_ext_cost_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_cost_ext_cost_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void di_mpc_ca_cost_ext_cost_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void di_mpc_ca_cost_ext_cost_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void di_mpc_ca_cost_ext_cost_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int di_mpc_ca_cost_ext_cost_fun_jac_hess_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int di_mpc_ca_cost_ext_cost_fun_jac_hess_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real di_mpc_ca_cost_ext_cost_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* di_mpc_ca_cost_ext_cost_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* di_mpc_ca_cost_ext_cost_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* di_mpc_ca_cost_ext_cost_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* di_mpc_ca_cost_ext_cost_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_cost_ext_cost_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
