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
  #define CASADI_PREFIX(ID) di_mpc_ca_constr_h_e_fun_ ## ID
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
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
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

static const casadi_int casadi_s0[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s1[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* di_mpc_ca_constr_h_e_fun:(i0[4],i1[15])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4;
  a0=arg[0]? arg[0][0] : 0;
  a1=arg[1]? arg[1][10] : 0;
  a0=(a0-a1);
  a0=casadi_sq(a0);
  a1=arg[1]? arg[1][4] : 0;
  a2=1.0500000000000000e+00;
  a3=arg[1]? arg[1][12] : 0;
  a3=(a2*a3);
  a1=(a1+a3);
  a1=casadi_sq(a1);
  a0=(a0/a1);
  a1=arg[0]? arg[0][1] : 0;
  a3=arg[1]? arg[1][11] : 0;
  a1=(a1-a3);
  a1=casadi_sq(a1);
  a3=arg[1]? arg[1][5] : 0;
  a4=arg[1]? arg[1][13] : 0;
  a2=(a2*a4);
  a3=(a3+a2);
  a3=casadi_sq(a3);
  a1=(a1/a3);
  a0=(a0+a1);
  a1=1.;
  a0=(a0-a1);
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_constr_h_e_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_constr_h_e_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_constr_h_e_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void di_mpc_ca_constr_h_e_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_constr_h_e_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void di_mpc_ca_constr_h_e_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void di_mpc_ca_constr_h_e_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void di_mpc_ca_constr_h_e_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int di_mpc_ca_constr_h_e_fun_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int di_mpc_ca_constr_h_e_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real di_mpc_ca_constr_h_e_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* di_mpc_ca_constr_h_e_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* di_mpc_ca_constr_h_e_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* di_mpc_ca_constr_h_e_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* di_mpc_ca_constr_h_e_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int di_mpc_ca_constr_h_e_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
