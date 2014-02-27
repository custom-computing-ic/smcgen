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

	int NP = atoi(argv[4]);
	float S = atof(argv[5]);

	// Read observation and reference
	// Initialise states
	float *obsrv = (float *)malloc(NT*NSensor*sizeof(float));
	float *ref = (float *)malloc(NT*RS*sizeof(float));
	float *state = (float *)malloc(NP*SS*sizeof(float));
	init(NP, argv[1], obsrv, argv[2], ref, state);

	// Other array values
	float *state_in = state;
	float *state_out = (float *)malloc(NP*SS*sizeof(float));
	float *ref_in = (float *)malloc(RS*2*sizeof(float));
	float *rand_num = (float *)malloc(4*sizeof(float));
	int *seed = (int *)malloc(NC*SS*16*3*sizeof(int));
	float *obsrv_in = (float *)malloc(NSensor*sizeof(float));
	int *index_out = (int *)malloc(NP*sizeof(int));

	for(int t=0; t<NT; t++){
		for (int i=0; i<itl_outer; i++) {

			// Determine inner loop iteration, a number divisible by NC
			int itl_inner = 1;
			// Allocate references of the current time step
			ref_in[0] = ref[RS*t];
			ref_in[1] = ref[RS*t+1];
			// Setup rand number for FPGA resampler
			rand_num[0] = dsfmt_genrand_close_open(&dsfmt[0])/(NP*1.0);
			// Setup seeds for FPGA random number generators
			for(int j=0; j<NC*SS*16*3; j++)
				seed[j] = 7-j;
			// Allocate observation value of the current time step
			memcpy(obsrv_in, obsrv+t*NSensor*sizeof(float), NSensor*sizeof(float));
#ifdef Use_FPGA
			// Invoke FPGA kernel
			printf("Calling FPGA kernel...\n");
			smcFPGA(NP,S,i,itl_inner,state_in,ref_in,rand_num,seed,obsrv_in,index_out,state_out);
#else
			printf("Calling CPU function...\n");
			smcCPU(NP,S,i,itl_inner,state_in,ref_in,obsrv_in,state_out);
#endif

		}
		update(NP, state_in, state_out);
		output(NP, t, state_in);
	}
	check(argv[3]);

	return 0;
}

