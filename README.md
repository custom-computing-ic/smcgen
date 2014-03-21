## Sequential Monte Carlo for FPGA accelerators

This repository consist of a design flow for generating efficient implementation of reconfigurable SMC designs. 
Users can use it to develop efficient multiple-FPGA SMC applications without any knowledge of FPGA/reconfigurable computing. 
The design flow consists of a parametrisable SMC computation engine, and a software template which enables efficient mapping of a variety of SMC designs to FPGAs.

We have a paper describing details about this design flow:
 * Thomas C.P. Chau, Maciej Kurek, James Stanley Targett, Jake Humphrey, Georgios Skouroupathis, Alison Eele, Jan Maciejowski, Benjamin Cope, Kathryn Cobden, Philip Leong, Peter Y.K. Cheung and Wayne Luk, "SMCGen: Generating Reconfigurable Design for Sequential Monte Carlo Applications," in International Symposium on Field-Programmable Custom Computing Machines (FCCM), page to appear, 2014.

### Sequential Monte Carlo

Sequential Monte Carlo (SMC) method is a simulation-based approach to compute posterior distributions.
SMC methods often work well on applications considered intractable by other methods due to high dimensionality, but they are computationally demanding.

For details of SMC method, please refer to [Sequential Monte Carlo methods in practice](http://www.springer.com/statistics/physical+%26+information+science/book/978-0-387-95146-1) written by Arnaud Doucet.
Springer, 2001.

### System requirements

1. Maxeler MaxCompiler version 2013.2.2 or above.
2. One of the Maxeler FPGA accelerator platforms below:
	- MaxWorkstation with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
	- MPC-C500 with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
	- MPC-X1000 with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
	- MPC-X2000 with maia dataflow engines (Altera Stratix V GS 5SGSD8 FPGAs)
3. GCC 4.4 or newer.
4. Intel Compiler 12.1 or newer.

### Example applications

#### 1. [Stochastic volatility] (stochastic_volatility)

#### 2. [Robot localisation] (robot_localisation)

#### 3. [Robot localisation with moving objects] (robot_localisation_with_moving_objects)
