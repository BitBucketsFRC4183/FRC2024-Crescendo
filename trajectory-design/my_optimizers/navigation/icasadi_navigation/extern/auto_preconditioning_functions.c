/* This file was automatically generated by CasADi 3.6.4.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) auto_preconditioning_functions_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_f3 CASADI_PREFIX(f3)
#define casadi_fabs CASADI_PREFIX(fabs)
#define casadi_fmax CASADI_PREFIX(fmax)
#define casadi_fmin CASADI_PREFIX(fmin)
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

casadi_real casadi_fabs(casadi_real x) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>0 ? x : -x;
#else
  return fabs(x);
#endif
}

casadi_real casadi_fmax(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>y ? x : y;
#else
  return fmax(x, y);
#endif
}

casadi_real casadi_fmin(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x<y ? x : y;
#else
  return fmin(x, y);
#endif
}

static const casadi_int casadi_s0[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* open_preconditioning_w_cost_navigation:(i0[5],i1[2])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6, a7, a8;
  a0=1.;
  a1=0.;
  a2=arg[1]? arg[1][0] : 0;
  a3=arg[0]? arg[0][0] : 0;
  a4=(a2-a3);
  a4=(a4+a4);
  a5=(a3+a3);
  a6=arg[0]? arg[0][1] : 0;
  a3=casadi_sq(a3);
  a3=(a6-a3);
  a3=(a3+a3);
  a7=arg[1]? arg[1][1] : 0;
  a3=(a3*a7);
  a5=(a5*a3);
  a4=(a4+a5);
  a4=(-a4);
  a4=casadi_fabs(a4);
  a1=casadi_fmax(a1,a4);
  a4=(a2-a6);
  a4=(a4+a4);
  a5=(a6+a6);
  a8=arg[0]? arg[0][2] : 0;
  a6=casadi_sq(a6);
  a6=(a8-a6);
  a6=(a6+a6);
  a6=(a6*a7);
  a5=(a5*a6);
  a4=(a4+a5);
  a3=(a3-a4);
  a3=casadi_fabs(a3);
  a1=casadi_fmax(a1,a3);
  a3=(a2-a8);
  a3=(a3+a3);
  a4=(a8+a8);
  a5=arg[0]? arg[0][3] : 0;
  a8=casadi_sq(a8);
  a8=(a5-a8);
  a8=(a8+a8);
  a8=(a8*a7);
  a4=(a4*a8);
  a3=(a3+a4);
  a6=(a6-a3);
  a6=casadi_fabs(a6);
  a1=casadi_fmax(a1,a6);
  a2=(a2-a5);
  a2=(a2+a2);
  a6=(a5+a5);
  a3=arg[0]? arg[0][4] : 0;
  a5=casadi_sq(a5);
  a3=(a3-a5);
  a3=(a3+a3);
  a3=(a3*a7);
  a6=(a6*a3);
  a2=(a2+a6);
  a8=(a8-a2);
  a8=casadi_fabs(a8);
  a1=casadi_fmax(a1,a8);
  a3=casadi_fabs(a3);
  a1=casadi_fmax(a1,a3);
  a0=casadi_fmax(a0,a1);
  a0=(1./a0);
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_cost_navigation(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_cost_navigation_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_cost_navigation_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_cost_navigation_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_cost_navigation_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_cost_navigation_release(int mem) {
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_cost_navigation_incref(void) {
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_cost_navigation_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int open_preconditioning_w_cost_navigation_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int open_preconditioning_w_cost_navigation_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real open_preconditioning_w_cost_navigation_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_preconditioning_w_cost_navigation_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_preconditioning_w_cost_navigation_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_preconditioning_w_cost_navigation_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_preconditioning_w_cost_navigation_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_cost_navigation_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* open_preconditioning_w_f1_navigation:(i0[5],i1[2])->(o0) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f1_navigation(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f1_navigation_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f1_navigation_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_f1_navigation_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f1_navigation_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_f1_navigation_release(int mem) {
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_f1_navigation_incref(void) {
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_f1_navigation_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int open_preconditioning_w_f1_navigation_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int open_preconditioning_w_f1_navigation_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real open_preconditioning_w_f1_navigation_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_preconditioning_w_f1_navigation_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_preconditioning_w_f1_navigation_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_preconditioning_w_f1_navigation_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_preconditioning_w_f1_navigation_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f1_navigation_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* open_preconditioning_w_f2_navigation:(i0[5],i1[2])->(o0[2]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3;
  a0=6.6666666666666663e-01;
  if (res[0]!=0) res[0][0]=a0;
  a0=1.;
  a1=0.;
  a2=arg[0]? arg[0][2] : 0;
  a3=arg[0]? arg[0][3] : 0;
  a2=(a2-a3);
  a3=1.0000000000000001e-01;
  a2=(a2+a3);
  a3=(a1<=a2);
  a2=(a2<=a1);
  a2=(a2+a3);
  a3=(a3/a2);
  a2=casadi_fabs(a3);
  a1=casadi_fmax(a1,a2);
  a3=(-a3);
  a3=casadi_fabs(a3);
  a1=casadi_fmax(a1,a3);
  a0=casadi_fmax(a0,a1);
  a0=(1./a0);
  if (res[0]!=0) res[0][1]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f2_navigation(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f2_navigation_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f2_navigation_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_f2_navigation_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f2_navigation_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_f2_navigation_release(int mem) {
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_f2_navigation_incref(void) {
}

CASADI_SYMBOL_EXPORT void open_preconditioning_w_f2_navigation_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int open_preconditioning_w_f2_navigation_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int open_preconditioning_w_f2_navigation_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real open_preconditioning_w_f2_navigation_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_preconditioning_w_f2_navigation_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_preconditioning_w_f2_navigation_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_preconditioning_w_f2_navigation_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_preconditioning_w_f2_navigation_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int open_preconditioning_w_f2_navigation_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* open_initial_penalty_navigation:(i0[5],i1[5])->(o0) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=1.0000000000000000e-08;
  a1=10.;
  a2=1.;
  a3=arg[1]? arg[1][2] : 0;
  a4=arg[1]? arg[1][1] : 0;
  a5=arg[0]? arg[0][1] : 0;
  a6=arg[0]? arg[0][0] : 0;
  a7=casadi_sq(a6);
  a7=(a5-a7);
  a7=casadi_sq(a7);
  a7=(a4*a7);
  a8=arg[1]? arg[1][0] : 0;
  a9=(a8-a6);
  a9=casadi_sq(a9);
  a7=(a7+a9);
  a9=arg[0]? arg[0][2] : 0;
  a10=casadi_sq(a5);
  a10=(a9-a10);
  a10=casadi_sq(a10);
  a10=(a4*a10);
  a11=(a8-a5);
  a11=casadi_sq(a11);
  a10=(a10+a11);
  a7=(a7+a10);
  a10=arg[0]? arg[0][3] : 0;
  a11=casadi_sq(a9);
  a11=(a10-a11);
  a11=casadi_sq(a11);
  a11=(a4*a11);
  a12=(a8-a9);
  a12=casadi_sq(a12);
  a11=(a11+a12);
  a7=(a7+a11);
  a11=arg[0]? arg[0][4] : 0;
  a12=casadi_sq(a10);
  a11=(a11-a12);
  a11=casadi_sq(a11);
  a4=(a4*a11);
  a8=(a8-a10);
  a8=casadi_sq(a8);
  a4=(a4+a8);
  a7=(a7+a4);
  a3=(a3*a7);
  a3=casadi_fabs(a3);
  a3=casadi_fmax(a2,a3);
  a7=5.0000000000000000e-01;
  a4=arg[1]? arg[1][3] : 0;
  a4=casadi_sq(a4);
  a8=1.5000000000000000e+00;
  a8=(a8*a6);
  a8=(a8-a5);
  a8=casadi_sq(a8);
  a4=(a4*a8);
  a8=arg[1]? arg[1][4] : 0;
  a8=casadi_sq(a8);
  a5=0.;
  a9=(a9-a10);
  a10=1.0000000000000001e-01;
  a9=(a9+a10);
  a5=casadi_fmax(a5,a9);
  a5=casadi_sq(a5);
  a8=(a8*a5);
  a4=(a4+a8);
  a7=(a7*a4);
  a7=casadi_fabs(a7);
  a2=casadi_fmax(a2,a7);
  a3=(a3/a2);
  a1=(a1*a3);
  a3=100000000.;
  a1=casadi_fmin(a1,a3);
  a0=casadi_fmax(a0,a1);
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int open_initial_penalty_navigation(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f3(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int open_initial_penalty_navigation_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int open_initial_penalty_navigation_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_initial_penalty_navigation_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int open_initial_penalty_navigation_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_initial_penalty_navigation_release(int mem) {
}

CASADI_SYMBOL_EXPORT void open_initial_penalty_navigation_incref(void) {
}

CASADI_SYMBOL_EXPORT void open_initial_penalty_navigation_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int open_initial_penalty_navigation_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int open_initial_penalty_navigation_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real open_initial_penalty_navigation_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_initial_penalty_navigation_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_initial_penalty_navigation_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_initial_penalty_navigation_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_initial_penalty_navigation_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int open_initial_penalty_navigation_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif