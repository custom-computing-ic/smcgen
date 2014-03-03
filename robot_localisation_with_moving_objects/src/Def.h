#ifndef SMC_HEADER
#define SMC_HEADER

/* Debug mode */
#ifndef debug
//#define debug
#endif

/* FPGA usage control */
/* enable: use FPGA; disable: use CPU for all processes */
#ifndef Use_FPGA
//#define Use_FPGA 
#endif

/* Number of CPU threads */
#ifndef THREADS
#define THREADS	4
#endif

/* Number of steps */
#ifndef NT
#define NT		10
#endif

/* Number of robot particles */
#ifndef NPMin
#define NPMin 4096
#endif
#ifndef NPMax
#define NPMax 4096
#endif

/* Number of moving object particles per robot particles */
#ifndef NPObj
#define NPObj 1017
#endif

/* Number of moving objects */
#ifndef Obj
#define Obj 7
#endif

/* Number of slots in a particle */
// 1+Obj*NPObj
// R:|0,1,...,Obj-1|...|0,1,...,Obj-1|
#ifndef slotOfP
#define slotOfP 7120
#endif

/* Horizon length */
#ifndef H
#define H  	1
#endif

/* Outer loop */
#ifndef itl_outer
#define itl_outer 4
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

/* Number of sensors on a robot */
#ifndef NSensor
#define NSensor 20
#endif

#define Pi 3.14159265359

#endif
