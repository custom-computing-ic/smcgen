#ifndef SMC_HEADER
#define SMC_HEADER

/* Debug mode */
#define debug 0

/* FPGA usage control */
/* 1: use FPGA; 0: use CPU for all processes */
#define Use_FPGA 1

/* FPGA onboard DRAM control*/
/* 1: use onboard DRAM for state IO; 0: stream in/out state directly from/to host */
#define Use_DRAM 0

/* Number of CPU threads */
#define THREADS	4

/* Number of steps */
#define NT		1

/* Number of robot particles */
#define NPMin 5008
#define NPMax 25008

/* Number of moving object particles per robot particles */
#define NPObj 1017

/* Number of moving objects */
#define Obj 7

/* Number of slots in a particle */
// 1+Obj*NPObj
// R:|0,1,...,Obj-1|...|0,1,...,Obj-1|
#define slotOfP 7120

/* Horizon length */
#define H  	1

/* Number of FPGA cores */
/* NC_inner * NC_P */
#define NC	2

/* Number of FPGA boards */
#define NBoard	1

/* Number of state slots */
#define SS 3

/* Number of reference slots */
#define RS 2

/* Number of sensors on a robot */
#define NSensor 20

/* Number of test to get the average error */
#define NTest 5

/* FPGA clock frequency in MHz*/
#define Clk 120

/* Bandwidth between CPU and FPGA in MB/s (one direction) */
#define BW 1800

/* CPU speed scaling factor */
#define alpha 1.1E-8

#define Pi 3.14159265359

#endif
