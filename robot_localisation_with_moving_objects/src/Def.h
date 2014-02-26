#ifndef SMC_HEADER
#define SMC_HEADER

/* Debug mode */
#ifndef debug
//#define debug
#endif

/* FPGA usage control */
/* enable: use FPGA; disable: use CPU for all processes */
#ifndef Use_FPGA
#define Use_FPGA 
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
//#define NP	4096
//#endif
// 7120*4096
#ifndef NPMin
#define NPMin 29163520
#endif
#ifndef NPMax
#define NPMax 29163520
#endif

/* Number of particles in a block */
#ifndef NPBlock
#define NPBlock 7120
#endif

/* Horizon length */
#ifndef H
#define H  	1
#endif

/* Outer loop */
#ifndef itl_outer
#define itl_outer 4
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
#define NC	2
#endif

/* Number of state slots */
#ifndef SS
#define SS 3
#endif

/* Number of reference slots */
#ifndef RS
#define RS 2
#endif

#define Pi 3.14159265359

#endif
