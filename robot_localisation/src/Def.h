#ifndef SMC_HEADER
#define SMC_HEADER

/* Debug mode */
#define debug 0

/* FPGA usage control */
/* 1: use FPGA; 0: use CPU for all processes */
#define Use_FPGA 1

/* Resampling control*/
/* 1: resampling on FPGA; 0: resampling on CPU */
#define FPGA_resampling  0

/* FPGA onboard DRAM control*/
/* 1: use onboard DRAM for state IO; 0: stream in/out state directly from/to host */
#define Use_DRAM 0

/* Number of CPU threads */
#define THREADS	4

/* Number of steps */
#define NT		10

/* Number of particles */
// NP is from run-time argument
#define NPMin 2048
#define NPMax 8192

/* Horizon length */
#define H  	1

/* Outer loop */
#define itl_outer 4

/* Number of agents */
#define NA	1

/* Number of FPGA cores per FPGA board */
/* NC_inner * NC_P */
#define NC	2

/* Number of FPGA boards */
#define NBoard	1

/* Number of state slots */
#define SS 3

/* Number of reference slots */
#define RS 2

#define Pi 3.14159265359

#endif
