## Sequential Monte Carlo for FPGA accelerators

This repository consist of a design flow for generating efficient implementation of reconfigurable SMC designs. 
Users can use it to develop efficient multiple-FPGA SMC applications without any knowledge of FPGA/reconfigurable computing. 
The design flow consists of a parametrisable SMC computation engine, and a software template which enables efficient mapping of a variety of SMC designs to FPGAs.

We have a paper describing the details about SMCGen. If you refer to SMCGen in a published work, please cite the following paper:
 * Thomas C.P. Chau, Maciej Kurek, James Stanley Targett, Jake Humphrey, Georgios Skouroupathis, Alison Eele, Jan Maciejowski, Benjamin Cope, Kathryn Cobden, Philip Leong, Peter Y.K. Cheung and Wayne Luk, "SMCGen: Generating Reconfigurable Design for Sequential Monte Carlo Applications," in International Symposium on Field-Programmable Custom Computing Machines (FCCM), page to appear, 2014.

### Sequential Monte Carlo

Sequential Monte Carlo (SMC) method is a simulation-based approach to compute posterior distributions.
SMC methods often work well on applications considered intractable by other methods due to high dimensionality, but they are computationally demanding.

For details of SMC method, please refer to [Sequential Monte Carlo methods in practice](http://www.springer.com/statistics/physical+%26+information+science/book/978-0-387-95146-1) written by Arnaud Doucet.
Springer, 2001.

### System requirements

1. CentOS 5/6 (x86_64) or Red Hat Enterprise Linux 5/6 (x86_64).
2. GNU Make 3.80+ & GCC 3.4+ & Intel Compiler 12.1+ (optional).
4. Xilinx ISE 13.3 (for Xilinx-based FPGAs).
5. Altera Quartus 13.0 (for Altera-based FPGAs).
6. Maxeler MaxCompiler version 2013.2.2+.
7. One of the Maxeler FPGA accelerator platforms below:
	- MaxWorkstation with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
	- MPC-C500 with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
	- MPC-X1000 with vectis dataflow engines (Xilinx Virtex-6 XC6VSX475T FPGAs)
	- MPC-X2000 with maia dataflow engines (Altera Stratix V GS 5SGSD8 FPGAs)

### Example applications

#### 1. [Stochastic volatility] (stochastic_volatility)

#### 2. [Robot localisation] (robot_localisation)

#### 3. [Robot localisation with moving objects] (robot_localisation_with_moving_objects)

### How to contribute?
1. Make sure you have a [GitHub] (https://github.com/) account.
1. Click [Fork] (https://github.com/custom-computing-ic/smcgen/fork) to create a starting point of the project for your own.
2. Commit your code to the forked repository.
3. Perform a [pull request] (https://github.com/custom-computing-ic/smcgen/compare/) on GitHub.
#### For details, check [GitHub Help] (https://help.github.com/articles/fork-a-repo).
