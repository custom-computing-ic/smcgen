### About

This is a simple example of using the SMC engine for localisation of a mobile robot.

### Input

 * `src/data_y.txt` - observations (sensor readings), based on true states (positions) `src/data_x.txt`.

### Output

 * `build/data_xest.txt` - estimated states.

### Functions

 * Application specific functions are described in `src/Func.maxj` and `src/Func.c`.
```
/* Func.maxj */
/* User customised sampling function */
public static DFEStruct sampling(SmcKernel smc, DFEStruct s_in, DFEStruct r_in, DFEStruct seeds, DFEVar S) {
	...
}
/* User customised weighting function */
public static DFEVar weighting(SmcKernel smc, DFEStruct s_in, DFEVar o_in, DFEVar w_in, DFEVar n, DFEVar h, DFEVar p, DFEVar S) {
	...
}
```
```
/* Func.c */
/*** FPGA mode: Call SMC core */
#ifdef FPGA_resampling
void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out){
#else
void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out, max_file_t* maxfile, max_group_t* engines){
#endif
	...
}
...
```

### Paramters

* Design and system parameters are described in `src/Def.maxj` and `src/Def.h`.
```
/* Def.maxj */
public class Def {

	/*** Computation precision */
	static final DFEType float_t = KernelLib.dfeFloat(8, 24);

	/*** Seeds struct */
	public static final DFEStructType seeds_t = new DFEStructType(
		new StructFieldType("a", KernelLib.dfeRawBits(128)),
		new StructFieldType("b", KernelLib.dfeRawBits(128)),
		new StructFieldType("c", KernelLib.dfeRawBits(128))
	);
	public static int seed_slot = 3;

	/*** State struct */
	public static final DFEStructType state_t = new DFEStructType(
		new StructFieldType("x", float_t),
		new StructFieldType("y", float_t),
		new StructFieldType("h", float_t)
	);
	public static int state_slot = 3;

	/*** Reference struct */
	public static final DFEStructType ref_t = new DFEStructType(
		new StructFieldType("d", float_t),
		new StructFieldType("r", float_t)
	);
	public static int ref_slot = 2;

	/*** Particle struct */
	public static final DFEStructType particle_t = new DFEStructType(
		new StructFieldType("w", float_t),
		new StructFieldType("s", state_t)
	);

	/*** Design Parameters */
	public static int NPMin = 2048; // Minimum number of particles allowed
	public static int NPMax = 8192; // Maximum number of particle allowed
	...

	/*** System Parameters */
	public static int NC_inner = 1; // Parallelisation along itl_inner, between 1 and 4 inclusive
	public static int NC_P = 2; // Parallelisation along NP, should be divisor of NP
	public static int Clk_core = 150; // FPGA core clock frequency
	public static int Clk_mem = 350; // FPGA onboard DRAM frequency (MAX3: 300,333,350,400; MAX4:333,400,533)
	public static int FPGA_resampling = 0; // 1: resampling on FPGA; 0: resampling on CPU
	public static int Use_DRAM = 0; // 1: use onboard DRAM for state IO; 0: stream in/out state directly from/to host

	/*** Application Parameters */
	public static int NWall = 8;
	...
}
```
```
/* Def.h */
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
```

### How to run

 * Simulation:
 	* `make runsim`
 * Build hardware:
 	* `make build`
 * Run hardware:
 	* `make run`
