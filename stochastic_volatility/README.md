# About

This is a simple example of using the SMC engine for stochastic volatility modelling.

# Input


# Output


# Paramters

 * Application specific functions are described in `Func.maxj` and `Func.c`.
```
/* Func.maxj */
/* User customised sampling function */
public static DFEStruct sampling(SmcKernel smc, DFEStruct s_in, DFEStruct seeds, DFEVar S) {
	...
}
/* User customised weighting function */
public static DFEVar weighting(SmcKernel smc, DFEStruct s_in, DFEVar o_in, DFEVar w_in, DFEVar n, DFEVar h, DFEVar p) {
	...
}
```
```
/* Func.c */
/*** FPGA mode: Call SMC core */
#ifdef FPGA_resampling
void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out){
#else
void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out, maxfile_t* maxfile, max_engarray_t* engines){
#endif
	...
}
...
```
* Design and system parameters are described in `Def.maxj` and `Def.h`.
```
/* Def.maxj */
public class Def {

	/*** Computation precision */
	static final DFEType float_t = KernelLib.dfeFloat(8, 24);

	/*** Seeds struct */
	public static final DFEStructType seeds_t = new DFEStructType(
		new StructFieldType("a", KernelLib.dfeRawBits(128))
	);
	public static int seed_slot = 1;

	/*** State struct */
	public static final DFEStructType state_t = new DFEStructType(
		new StructFieldType("x", float_t)
	);
	public static int state_slot = 1;

	/*** Particle struct */
	public static final DFEStructType particle_t = new DFEStructType(
		new StructFieldType("w", float_t), // weight
		new StructFieldType("s", state_t) // state
	);

	/*** Design Parameters */
	public static int NPMin = 96; // Minimum number of particles allowed
	public static int NPMax = 4096; // Maximum number of particle allowed
	...

	/*** System Parameters */
	public static int NC_inner = 1; // Parallelisation along itl_inner, between 1 and 4 inclusive
	public static int NC_P = 16; // Parallelisation along NP, should be divisor of NP
	public static int Clk_core = 150; // FPGA core clock frequency
	public static int Clk_mem = 400; // FPGA onboard DRAM frequency
	public static int FPGA_resampling = 0; // 1: resampling on FPGA; 0: resampling on CPU
	...
}
```
```
/* Def.h */
/* Debug mode */
#ifndef debug
//#define debug
#endif

/* FPGA usage control */
/* enable: use FPGA; disable: use CPU for all processes */
#ifndef Use_FPGA
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
#define NPMin	96
#endif
#ifndef NPMax
#define NPMax	4096
#endif
...
/* Number of FPGA cores */
/* NC_inner * NC_P */
#ifndef NC
#define NC	16
#endif

/* Number of FPGA boards */
#ifndef NBoard
#define NBoard	1
#endif

/* Number of state slots */
#ifndef SS
#define SS 1
#endif
```


# How to run

 * Simulation:
 	* `make runsim`
 * Build hardware:
 	* `make build`
 * Run hardware:
 	* `make run`
