#ifndef SMC_HEADER
#define SMC_HEADER

/* Debug mode */
#ifndef debug
//#define debug
#endif

/* FPGA usage control */
/* enable: use FPGA; disable: use CPU for all processes */
#ifndef FPGA_resampling
#define Use_FPGA 
#endif

/* Resampling control*/
/* enable: resampling on FPGA; disable: resampling on CPU */
#ifndef FPGA_resampling
//#define FPGA_resampling 
#endif

/* Number of CPU threads */
#ifndef THREADS
#define THREADS	4
#endif

/* Number of steps */
#ifndef NT
#define NT		10
#endif

/* Number of particles */
// NP is from run-time argument
//#ifndef NP
//#define NP	6144
//#endif
#ifndef NPMin
#define NPMin	2048
#endif
#ifndef NPMax
#define NPMax	8192
#endif

/* Horizon length */
#ifndef H
#define H  	1
#endif

/* Outer loop */
#ifndef itl_outer
#define itl_outer 1
#endif

/* Spread of Gaussian random numbers */
// S is from run-time argument
//#ifndef S
//#define S	1
//#endif

/* Number of agents */
#ifndef NA
#define NA	1
#endif

/* Number of FPGA cores */
/* NC_inner * NC_P */
#ifndef NC
#define NC	16
#endif

/* Number of state slots */
#ifndef SS
#define SS 1
#endif

#define Pi 3.14159265359

#endif
