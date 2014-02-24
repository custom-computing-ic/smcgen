#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <mathimf.h>
#include "Maxfiles.h"
#include "MaxSLiCInterface.h"
#include "dSFMT.h"
#include "Def.h"
#include "Func.h"

dsfmt_t dsfmt[NPMax];

int main(int argc, char *argv[]){

	printf("Usage: %s [observation file (Y)] [reference file (R)] [true state file (X)] [NP] [S]\n", argv[0]);

	// Read observation and reference
	// Initialise states
	float *obsrv = (float *)malloc(NT*sizeof(float));
	float *ref = (float *)malloc(NT*RS*sizeof(float));
	float *state = (float *)malloc(NA*NP*SS*sizeof(float));

	int NP = atoi(argv[4]);
	float S = atof(argv[5]);

	init(NP, argv[1], obsrv, argv[2], ref, state);

	// Other array values
	float *state_in = state;
	float *state_out = (float *)malloc(NA*NP*SS*sizeof(float));
#if defined NA==1
	float *ref_in = (float *)malloc(NA*RS*2*sizeof(float));
#else
	float *ref_in = (float *)malloc(NA*RS*sizeof(float));
#endif
#if defined NA==1 || defined NA==2 || defined NA==3
	float *rand_num = (float *)malloc(4*sizeof(float));
#else
	float *rand_num = (float *)malloc(NA*sizeof(float));
#endif
	int *seed = (int *)malloc(NC*SS*16*3*sizeof(int));
#if defined NA==1 || defined NA==2 || defined NA==3
	float *obsrv_in = (float *)malloc(4*sizeof(float));
#else
	float *obsrv_in = (float *)malloc(NA*sizeof(float));
#endif
	int *index_out = (int *)malloc(NA*NP*sizeof(int));

	for(int t=0; t<NT; t++){
		for (int i=0; i<itl_outer; i++) {

			// Determine inner loop iteration, a number divisible by NC
			int itl_inner = 1;
			// Allocate references of the current time step
			for(int a=0; a<NA; a++){
				ref_in[a*RS] = ref[NA*RS*t+a*RS];
				ref_in[a*RS+1] = ref[NA*RS*t+a*RS+1];
			}
			// Setup rand number for FPGA resampler
			for(int a=0; a<NA; a++)
				rand_num[a] = dsfmt_genrand_close_open(&dsfmt[0])/(NP*1.0);
			// Setup seeds for FPGA random number generators
			for(int j=0; j<NC*SS*16*3; j++)
				seed[j] = 7-j;
			// Allocate observation value of the current time step
			for(int a=0; a<NA; a++){
				obsrv_in[a] = obsrv[NA*t+a];
			}
#ifdef Use_FPGA
			// Invoke FPGA kernel
			printf("Calling FPGA kernel...\n");
			smcFPGA(NP, S, i, itl_inner,state_in,ref_in,rand_num,seed,obsrv_in,index_out,state_out);
#else
#endif

		}
		update(NP, state_in, state_out);
		output(NP, t, state_in);
	}
	check(argv[3]);

	return 0;
}

