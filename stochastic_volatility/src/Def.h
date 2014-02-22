#ifndef SMC_HEADER
#define SMC_HEADER

//#define debug

//#define Use_FPGA

/* Number of CPU threads */
#ifndef THREADS
#pragma message "THREADS has been set to 12"
#define THREADS	12
#endif

/* Number of steps */
#ifndef NT
#pragma message "NT has been set to 10"
#define NT		10
#endif

/* Number of particles */
#ifndef NP
#pragma message "NP has been set to 6144"
#define NP	6144
#endif

/* Horizon length */
#ifndef H
#pragma message "H has been set to 1"
#define H  	1
#endif

/* Outer loop */
#ifndef itl_outer
#pragma message "itl_outer has been set to 1"
#define itl_outer 1
#endif

/* Spread of Gaussian random numbers */
#ifndef S
#pragma message "S has been set to 1"
#define S	1
#endif

/* Number of agents */
#ifndef NA
#pragma message "NA has been set to 1"
#define NA	1
#endif

/* Number of FPGA cores */
/* NC_inner * NC_P */
#ifndef NC
#pragma message "NC has been set to 8"
#define NC	8
#endif

/* Number of state slots */
#ifndef SS
#define SS 1
#endif

#define Pi 3.14159265359

#endif
