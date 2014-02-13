## Sequential Monte Carlo (SMC) template for FPGA accelerators

This repository consist of a design flow for generating efficient implementation of reconfigurable SMC designs. 
Users can use it to develop efficient multiple-FPGA SMC applications without any knowledge of FPGA/reconfigurable computing. 
The design flow consists of a parametrisable SMC computation engine, and a software template which enables efficient mapping of a variety of SMC designs to FPGAs.

### Sequential Monte Carlo

Sequential Monte Carlo (SMC) method is a simulation-based approach to compute posterior distributions.
SMC methods often work well on applications considered intractable by other methods due to high dimensionality, but they are computationally demanding.

For details of SMC method, please refer to [Sequential Monte Carlo methods in practice](http://www.springer.com/statistics/physical+%26+information+science/book/978-0-387-95146-1) written by Arnaud Doucet.
Springer, 2001.

### System requirements

1. Maxeler MaxCompiler version 2013.2.2 or above
2. One of the Maxeler FPGA accelerator platforms below:
	- MaxWorkstation with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
	- MPC-C500 with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
	- MPC-X1000 with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
- MPC-X2000 with maia dataflow engines (Altera Stratix V GS 5SGSD8 FPGAs)

### Example applications

1. Robot localisation

* Go to robot_localisation/src/
* Application specific functions are described in `Func.maxj` and `Smc.c`.
```
/* User customised sampling function */
public static DFEStruct sampling(SmcKernel smc, DFEStruct s_in, DFEStruct c_in, DFEVector<DFEVar> seeds);
/* User customised weighting function */
public static DFEVar weighting(SmcKernel smc, DFEStruct s_in, DFEVar e_in, DFEVar w_in, DFEVar n, DFEVar h, DFEVar p);
```
* Design and system parameters are described in `Def.maxj` and `Def.h`.
```
public class Def {
	// State Type
	public static final DFEStructType state_t = new DFEStructType(
			new StructFieldType("x", compType),
			new StructFieldType("y", compType),
			new StructFieldType("h", compType)
			);
	public static int state_slot = 3;

	// Control Type
	public static final DFEStructType control_t = new DFEStructType(
			new StructFieldType("d", compType),
			new StructFieldType("r", compType)
			);
	public static int control_slot = 2;

	// Design Parameters
	public static int NPMin = 2048;
	public static int NPMax = 8192;
	public static int NA = 1;
	public static int H = 1;
	public static double S = 0.5;

	// System Parameters
	public static int NC = 2; // Between 1 and 4 inclusive
	public static int Clk = 200;
}
```
