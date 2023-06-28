/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) eval_hess_g_new_platform_ ## ID
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
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

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

static const casadi_int casadi_s0[40] = {36, 1, 0, 36, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
static const casadi_int casadi_s1[32] = {28, 1, 0, 28, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
static const casadi_int casadi_s2[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s3[81] = {36, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 29, 29, 29, 34, 39, 40, 40, 40, 40, 42, 42, 42, 28, 27, 28, 27, 28, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 28, 29, 27, 28, 33, 34, 35, 28, 29, 33, 34, 35, 29, 34, 35};

/* eval_hess_g_new_platform:(w[36],lambda[28],p[14])->(hess_g[36x36,42nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a9;
  a0=arg[1]? arg[1][11] : 0;
  a1=-6.5719616788869575e-02;
  a2=arg[0]? arg[0][28] : 0;
  a3=cos(a2);
  a4=(a1*a3);
  a4=(a0*a4);
  a5=(-a4);
  if (res[0]!=0) res[0][0]=a5;
  a5=1.4103413400973910e-01;
  a6=cos(a2);
  a7=arg[0]? arg[0][27] : 0;
  a8=cos(a7);
  a9=(a6*a8);
  a9=(a5*a9);
  a9=(a0*a9);
  a10=arg[1]? arg[1][10] : 0;
  a11=sin(a7);
  a12=(a5*a11);
  a12=(a10*a12);
  a9=(a9-a12);
  a12=(-a9);
  if (res[0]!=0) res[0][1]=a12;
  a12=sin(a7);
  a13=sin(a2);
  a14=(a12*a13);
  a14=(a5*a14);
  a14=(a0*a14);
  if (res[0]!=0) res[0][2]=a14;
  a15=-1.4103413400973910e-01;
  a16=(a15*a8);
  a16=(a10*a16);
  a17=(a6*a11);
  a17=(a5*a17);
  a17=(a0*a17);
  a16=(a16-a17);
  a17=(-a16);
  if (res[0]!=0) res[0][3]=a17;
  a17=cos(a7);
  a18=(a17*a13);
  a18=(a5*a18);
  a18=(a0*a18);
  if (res[0]!=0) res[0][4]=a18;
  a19=arg[2]? arg[2][0] : 0;
  a20=arg[1]? arg[1][9] : 0;
  a20=(a19*a20);
  a21=arg[2]? arg[2][3] : 0;
  a22=(a21*a3);
  a23=arg[0]? arg[0][29] : 0;
  a24=sin(a23);
  a25=arg[2]? arg[2][4] : 0;
  a26=(a25*a13);
  a27=(a24*a26);
  a22=(a22-a27);
  a22=(a20*a22);
  if (res[0]!=0) res[0][5]=a22;
  a22=(a25*a6);
  a27=cos(a23);
  a28=(a22*a27);
  a28=(a20*a28);
  a29=(a19*a10);
  a30=sin(a23);
  a31=(a25*a30);
  a31=(a29*a31);
  a28=(a28-a31);
  if (res[0]!=0) res[0][6]=a28;
  a28=arg[2]? arg[2][2] : 0;
  a31=(a28*a3);
  a32=cos(a23);
  a33=(a25*a13);
  a34=(a32*a33);
  a31=(a31-a34);
  a31=(a20*a31);
  a31=(-a31);
  if (res[0]!=0) res[0][7]=a31;
  a31=(a25*a27);
  a31=(a29*a31);
  a34=(a25*a6);
  a35=(a34*a30);
  a35=(a20*a35);
  a31=(a31+a35);
  if (res[0]!=0) res[0][8]=a31;
  a31=(a21*a32);
  a35=(a28*a24);
  a31=(a31-a35);
  a35=(a31*a13);
  a35=(a20*a35);
  a35=(-a35);
  if (res[0]!=0) res[0][9]=a35;
  a35=(a21*a30);
  a36=(a28*a27);
  a35=(a35+a36);
  a35=(a6*a35);
  a35=(a20*a35);
  a36=(a21*a27);
  a37=(a28*a30);
  a36=(a36-a37);
  a36=(a29*a36);
  a35=(a35+a36);
  a35=(-a35);
  if (res[0]!=0) res[0][10]=a35;
  a35=arg[2]? arg[2][6] : 0;
  a36=(a35*a3);
  a37=arg[2]? arg[2][7] : 0;
  a38=(a37*a13);
  a39=(a24*a38);
  a36=(a36-a39);
  a36=(a20*a36);
  if (res[0]!=0) res[0][11]=a36;
  a36=(a37*a6);
  a39=(a36*a27);
  a39=(a20*a39);
  a40=(a37*a30);
  a40=(a29*a40);
  a39=(a39-a40);
  if (res[0]!=0) res[0][12]=a39;
  a39=arg[2]? arg[2][5] : 0;
  a40=(a39*a3);
  a41=(a37*a13);
  a42=(a32*a41);
  a40=(a40-a42);
  a40=(a20*a40);
  a40=(-a40);
  if (res[0]!=0) res[0][13]=a40;
  a40=(a37*a27);
  a40=(a29*a40);
  a42=(a37*a6);
  a43=(a42*a30);
  a43=(a20*a43);
  a40=(a40+a43);
  if (res[0]!=0) res[0][14]=a40;
  a40=(a35*a32);
  a43=(a39*a24);
  a40=(a40-a43);
  a43=(a40*a13);
  a43=(a20*a43);
  a43=(-a43);
  if (res[0]!=0) res[0][15]=a43;
  a43=(a35*a30);
  a44=(a39*a27);
  a43=(a43+a44);
  a43=(a6*a43);
  a43=(a20*a43);
  a44=(a35*a27);
  a45=(a39*a30);
  a44=(a44-a45);
  a44=(a29*a44);
  a43=(a43+a44);
  a43=(-a43);
  if (res[0]!=0) res[0][16]=a43;
  a43=arg[2]? arg[2][9] : 0;
  a44=(a43*a3);
  a45=arg[2]? arg[2][10] : 0;
  a46=(a45*a13);
  a47=(a24*a46);
  a44=(a44-a47);
  a44=(a20*a44);
  if (res[0]!=0) res[0][17]=a44;
  a44=(a45*a6);
  a47=(a44*a27);
  a47=(a20*a47);
  a48=(a45*a30);
  a48=(a29*a48);
  a47=(a47-a48);
  if (res[0]!=0) res[0][18]=a47;
  a47=arg[2]? arg[2][8] : 0;
  a48=(a47*a3);
  a49=(a45*a13);
  a50=(a32*a49);
  a48=(a48-a50);
  a48=(a20*a48);
  a48=(-a48);
  if (res[0]!=0) res[0][19]=a48;
  a48=(a45*a27);
  a48=(a29*a48);
  a50=(a45*a6);
  a51=(a50*a30);
  a51=(a20*a51);
  a48=(a48+a51);
  if (res[0]!=0) res[0][20]=a48;
  a48=(a43*a32);
  a51=(a47*a24);
  a48=(a48-a51);
  a51=(a48*a13);
  a51=(a20*a51);
  a51=(-a51);
  if (res[0]!=0) res[0][21]=a51;
  a51=(a43*a30);
  a52=(a47*a27);
  a51=(a51+a52);
  a51=(a6*a51);
  a51=(a20*a51);
  a52=(a43*a27);
  a53=(a47*a30);
  a52=(a52-a53);
  a52=(a29*a52);
  a51=(a51+a52);
  a51=(-a51);
  if (res[0]!=0) res[0][22]=a51;
  a51=arg[2]? arg[2][12] : 0;
  a52=(a51*a3);
  a53=arg[2]? arg[2][13] : 0;
  a54=(a53*a13);
  a55=(a24*a54);
  a52=(a52-a55);
  a52=(a20*a52);
  if (res[0]!=0) res[0][23]=a52;
  a52=(a53*a6);
  a55=(a52*a27);
  a55=(a20*a55);
  a56=(a53*a30);
  a56=(a29*a56);
  a55=(a55-a56);
  if (res[0]!=0) res[0][24]=a55;
  a55=arg[2]? arg[2][11] : 0;
  a56=(a55*a3);
  a57=(a53*a13);
  a58=(a32*a57);
  a56=(a56-a58);
  a56=(a20*a56);
  a56=(-a56);
  if (res[0]!=0) res[0][25]=a56;
  a56=(a53*a27);
  a56=(a29*a56);
  a58=(a53*a6);
  a59=(a58*a30);
  a59=(a20*a59);
  a56=(a56+a59);
  if (res[0]!=0) res[0][26]=a56;
  a56=(a51*a32);
  a59=(a55*a24);
  a56=(a56-a59);
  a59=(a56*a13);
  a59=(a20*a59);
  a59=(-a59);
  if (res[0]!=0) res[0][27]=a59;
  a59=(a51*a30);
  a60=(a55*a27);
  a59=(a59+a60);
  a59=(a6*a59);
  a59=(a20*a59);
  a27=(a51*a27);
  a30=(a55*a30);
  a27=(a27-a30);
  a27=(a29*a27);
  a59=(a59+a27);
  a59=(-a59);
  if (res[0]!=0) res[0][28]=a59;
  a59=arg[0]? arg[0][34] : 0;
  a27=arg[0]? arg[0][10] : 0;
  a27=(a59-a27);
  a30=(a27*a0);
  a30=(a5*a30);
  a60=(a6*a30);
  a61=arg[0]? arg[0][35] : 0;
  a62=arg[0]? arg[0][11] : 0;
  a62=(a61-a62);
  a63=(a62*a10);
  a15=(a15*a63);
  a60=(a60+a15);
  a15=arg[0]? arg[0][33] : 0;
  a63=(a15*a6);
  a64=7.5314517220869556e-02;
  a65=(a19*a0);
  a64=(a64*a65);
  a63=(a63*a64);
  a65=arg[1]? arg[1][4] : 0;
  a65=(a19*a65);
  a63=(a63-a65);
  a65=(a61*a63);
  a60=(a60-a65);
  a65=-7.5314517220869556e-02;
  a65=(a65*a29);
  a66=(a15*a65);
  a67=arg[1]? arg[1][5] : 0;
  a67=(a19*a67);
  a68=(a67/a6);
  a66=(a66-a68);
  a69=sin(a2);
  a70=arg[1]? arg[1][3] : 0;
  a19=(a19*a70);
  a70=(a19/a6);
  a71=(a69*a70);
  a71=(a66-a71);
  a72=(a59*a71);
  a60=(a60+a72);
  a72=sin(a7);
  a60=(a60*a72);
  a62=(a62*a0);
  a62=(a5*a62);
  a72=(a6*a62);
  a27=(a27*a10);
  a5=(a5*a27);
  a72=(a72+a5);
  a5=(a59*a63);
  a72=(a72+a5);
  a5=(a69*a70);
  a66=(a66-a5);
  a5=(a61*a66);
  a72=(a72+a5);
  a5=cos(a7);
  a72=(a72*a5);
  a60=(a60+a72);
  a60=(-a60);
  if (res[0]!=0) res[0][29]=a60;
  a60=cos(a7);
  a72=(a15*a13);
  a72=(a64*a72);
  a5=(a61*a72);
  a27=(a30*a13);
  a5=(a5-a27);
  a68=(a68/a6);
  a68=(a68*a13);
  a27=(a70/a6);
  a27=(a27*a13);
  a10=(a69*a27);
  a10=(a19+a10);
  a10=(a68+a10);
  a73=(a59*a10);
  a5=(a5-a73);
  a60=(a60*a5);
  a7=sin(a7);
  a5=(a62*a13);
  a73=(a59*a72);
  a5=(a5+a73);
  a73=(a69*a27);
  a73=(a19+a73);
  a68=(a68+a73);
  a73=(a61*a68);
  a5=(a5+a73);
  a7=(a7*a5);
  a60=(a60+a7);
  if (res[0]!=0) res[0][30]=a60;
  a60=(a59*a8);
  a7=(a61*a11);
  a60=(a60-a7);
  a60=(a65*a60);
  a7=(a59*a11);
  a5=(a61*a8);
  a7=(a7+a5);
  a7=(a64*a7);
  a7=(a6*a7);
  a60=(a60-a7);
  if (res[0]!=0) res[0][31]=a60;
  a60=(a63*a11);
  a9=(a9-a60);
  a71=(a71*a8);
  a9=(a9+a71);
  if (res[0]!=0) res[0][32]=a9;
  a63=(a63*a8);
  a16=(a16-a63);
  a66=(a66*a11);
  a16=(a16-a66);
  if (res[0]!=0) res[0][33]=a16;
  a16=arg[0]? arg[0][9] : 0;
  a16=(a15-a16);
  a16=(a16*a0);
  a1=(a1*a16);
  a16=arg[0]? arg[0][22] : 0;
  a0=(a16*a20);
  a66=(a55*a0);
  a1=(a1-a66);
  a66=arg[0]? arg[0][21] : 0;
  a11=(a66*a20);
  a63=(a51*a11);
  a1=(a1+a63);
  a63=arg[0]? arg[0][19] : 0;
  a8=(a63*a20);
  a9=(a47*a8);
  a1=(a1-a9);
  a9=arg[0]? arg[0][18] : 0;
  a71=(a9*a20);
  a60=(a43*a71);
  a1=(a1+a60);
  a60=arg[0]? arg[0][16] : 0;
  a7=(a60*a20);
  a5=(a39*a7);
  a1=(a1-a5);
  a5=arg[0]? arg[0][15] : 0;
  a73=(a5*a20);
  a74=(a35*a73);
  a1=(a1+a74);
  a74=arg[0]? arg[0][12] : 0;
  a75=(a74*a20);
  a76=(a21*a75);
  a1=(a1+a76);
  a76=arg[0]? arg[0][13] : 0;
  a77=(a76*a20);
  a78=(a28*a77);
  a1=(a1-a78);
  a78=(a59*a12);
  a79=(a78*a70);
  a1=(a1-a79);
  a79=(a61*a17);
  a80=(a79*a70);
  a1=(a1-a80);
  a80=sin(a2);
  a1=(a1*a80);
  a80=cos(a2);
  a81=(a78*a27);
  a82=(a79*a27);
  a81=(a81+a82);
  a80=(a80*a81);
  a1=(a1+a80);
  a59=(a59*a17);
  a61=(a61*a12);
  a59=(a59-a61);
  a59=(a59*a64);
  a61=(a15*a59);
  a62=(a17*a62);
  a61=(a61+a62);
  a30=(a12*a30);
  a61=(a61+a30);
  a30=arg[0]? arg[0][23] : 0;
  a62=(a30*a20);
  a56=(a56*a62);
  a61=(a61+a56);
  a56=(a32*a0);
  a56=(a53*a56);
  a61=(a61-a56);
  a56=(a24*a11);
  a56=(a53*a56);
  a61=(a61+a56);
  a56=arg[0]? arg[0][20] : 0;
  a80=(a56*a20);
  a48=(a48*a80);
  a61=(a61+a48);
  a48=(a32*a8);
  a48=(a45*a48);
  a61=(a61-a48);
  a48=(a24*a71);
  a48=(a45*a48);
  a61=(a61+a48);
  a48=arg[0]? arg[0][17] : 0;
  a81=(a48*a20);
  a40=(a40*a81);
  a61=(a61+a40);
  a40=(a32*a7);
  a40=(a37*a40);
  a61=(a61-a40);
  a40=(a24*a73);
  a40=(a37*a40);
  a61=(a61+a40);
  a40=arg[0]? arg[0][14] : 0;
  a20=(a40*a20);
  a31=(a31*a20);
  a61=(a61+a31);
  a24=(a24*a75);
  a24=(a25*a24);
  a61=(a61+a24);
  a32=(a32*a77);
  a32=(a25*a32);
  a61=(a61-a32);
  a32=(a79+a78);
  a32=(a32/a6);
  a24=(a32/a6);
  a31=(a24*a67);
  a61=(a61+a31);
  a31=(a15*a6);
  a82=(a69*a79);
  a31=(a31+a82);
  a69=(a69*a78);
  a31=(a31+a69);
  a31=(a31/a6);
  a69=(a31/a6);
  a82=(a69*a19);
  a61=(a61+a82);
  a70=(a15*a70);
  a61=(a61-a70);
  a70=cos(a2);
  a61=(a61*a70);
  a2=sin(a2);
  a32=(a32/a6);
  a32=(a32*a13);
  a32=(a32/a6);
  a24=(a24/a6);
  a24=(a24*a13);
  a32=(a32+a24);
  a67=(a67*a32);
  a79=(a79*a3);
  a32=(a15*a13);
  a79=(a79-a32);
  a78=(a78*a3);
  a79=(a79+a78);
  a79=(a79/a6);
  a31=(a31/a6);
  a31=(a31*a13);
  a79=(a79+a31);
  a79=(a79/a6);
  a69=(a69/a6);
  a69=(a69*a13);
  a79=(a79+a69);
  a19=(a19*a79);
  a67=(a67+a19);
  a15=(a15*a27);
  a67=(a67-a15);
  a2=(a2*a67);
  a61=(a61+a2);
  a1=(a1+a61);
  a1=(-a1);
  if (res[0]!=0) res[0][34]=a1;
  a1=cos(a23);
  a61=(a62*a13);
  a2=(a55*a61);
  a54=(a11*a54);
  a2=(a2-a54);
  a54=(a80*a13);
  a67=(a47*a54);
  a2=(a2+a67);
  a46=(a71*a46);
  a2=(a2-a46);
  a46=(a81*a13);
  a67=(a39*a46);
  a2=(a2+a67);
  a38=(a73*a38);
  a2=(a2-a38);
  a38=(a20*a13);
  a67=(a28*a38);
  a2=(a2+a67);
  a26=(a75*a26);
  a2=(a2-a26);
  a1=(a1*a2);
  a2=sin(a23);
  a57=(a0*a57);
  a61=(a51*a61);
  a57=(a57-a61);
  a54=(a43*a54);
  a57=(a57-a54);
  a49=(a8*a49);
  a57=(a57+a49);
  a46=(a35*a46);
  a57=(a57-a46);
  a41=(a7*a41);
  a57=(a57+a41);
  a38=(a21*a38);
  a57=(a57-a38);
  a33=(a77*a33);
  a57=(a57+a33);
  a2=(a2*a57);
  a1=(a1-a2);
  if (res[0]!=0) res[0][35]=a1;
  a59=(a59*a13);
  a4=(a4-a59);
  if (res[0]!=0) res[0][36]=a4;
  a4=(a17*a72);
  a14=(a14+a4);
  a10=(a12*a10);
  a14=(a14+a10);
  a14=(-a14);
  if (res[0]!=0) res[0][37]=a14;
  a72=(a12*a72);
  a72=(a72-a18);
  a68=(a17*a68);
  a72=(a72-a68);
  if (res[0]!=0) res[0][38]=a72;
  a16=(a16*a29);
  a16=(a53*a16);
  a30=(a30*a29);
  a72=(a51*a30);
  a16=(a16-a72);
  a56=(a56*a29);
  a72=(a43*a56);
  a16=(a16-a72);
  a63=(a63*a29);
  a63=(a45*a63);
  a16=(a16+a63);
  a48=(a48*a29);
  a63=(a35*a48);
  a16=(a16-a63);
  a60=(a60*a29);
  a60=(a37*a60);
  a16=(a16+a60);
  a76=(a76*a29);
  a76=(a25*a76);
  a16=(a16+a76);
  a40=(a40*a29);
  a76=(a21*a40);
  a16=(a16-a76);
  a62=(a6*a62);
  a76=(a55*a62);
  a16=(a16-a76);
  a52=(a52*a11);
  a16=(a16+a52);
  a80=(a6*a80);
  a52=(a47*a80);
  a16=(a16-a52);
  a44=(a44*a71);
  a16=(a16+a44);
  a81=(a6*a81);
  a44=(a39*a81);
  a16=(a16-a44);
  a36=(a36*a73);
  a16=(a16+a36);
  a20=(a6*a20);
  a36=(a28*a20);
  a16=(a16-a36);
  a22=(a22*a75);
  a16=(a16+a22);
  a22=sin(a23);
  a16=(a16*a22);
  a66=(a66*a29);
  a53=(a53*a66);
  a55=(a55*a30);
  a53=(a53-a55);
  a47=(a47*a56);
  a53=(a53-a47);
  a9=(a9*a29);
  a45=(a45*a9);
  a53=(a53+a45);
  a39=(a39*a48);
  a53=(a53-a39);
  a5=(a5*a29);
  a37=(a37*a5);
  a53=(a53+a37);
  a74=(a74*a29);
  a25=(a25*a74);
  a53=(a53+a25);
  a28=(a28*a40);
  a53=(a53-a28);
  a51=(a51*a62);
  a53=(a53+a51);
  a58=(a58*a0);
  a53=(a53-a58);
  a43=(a43*a80);
  a53=(a53+a43);
  a50=(a50*a8);
  a53=(a53-a50);
  a35=(a35*a81);
  a53=(a53+a35);
  a42=(a42*a7);
  a53=(a53-a42);
  a21=(a21*a20);
  a53=(a53+a21);
  a34=(a34*a77);
  a53=(a53-a34);
  a23=cos(a23);
  a53=(a53*a23);
  a16=(a16+a53);
  a16=(-a16);
  if (res[0]!=0) res[0][39]=a16;
  a64=(a64*a6);
  a6=(a17*a64);
  a16=(a12*a65);
  a6=(a6+a16);
  if (res[0]!=0) res[0][40]=a6;
  a17=(a17*a65);
  a12=(a12*a64);
  a17=(a17-a12);
  if (res[0]!=0) res[0][41]=a17;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_new_platform(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_new_platform_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_new_platform_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_new_platform_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_new_platform_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_new_platform_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_new_platform_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void eval_hess_g_new_platform_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_hess_g_new_platform_n_in(void) { return 3;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_hess_g_new_platform_n_out(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real eval_hess_g_new_platform_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_hess_g_new_platform_name_in(casadi_int i) {
  switch (i) {
    case 0: return "w";
    case 1: return "lambda";
    case 2: return "p";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_hess_g_new_platform_name_out(casadi_int i) {
  switch (i) {
    case 0: return "hess_g";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_hess_g_new_platform_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_hess_g_new_platform_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int eval_hess_g_new_platform_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


