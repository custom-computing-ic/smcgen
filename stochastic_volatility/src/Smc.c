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

dsfmt_t dsfmt[NP];

int main(int argc, char *argv[]){

	if (argc<2 || argc>3){
		printf("Usage: %s [observation file]\n", argv[0]);
		return 0;
	}

	// Read observation and control
	// Initialise states
	float *obsrv = (float *)malloc(NT*sizeof(float));
	float *state = (float *)malloc(NA*NP*SS*sizeof(float));
	init(argv[1], obsrv, state);

	// Other array values
	float *state_in = state;
	float *state_out = (float *)malloc(NA*NP*SS*sizeof(float));
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

#ifdef debug
			for(int j=0; j<NP; j++)
				printf("State particle %d: (%f %f %f)\n", j, state[j*3], state[j*3+1], state[j*3+2]);
#endif

			// Determine inner loop iteration, a number divisible by NC
			int itl_inner = 1;
			// Allocate state (from initialisation or resampling)
			state_in = state;
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
			smcFPGA(itl_inner,state_in,rand_num,seed,obsrv_in,index_out,state_out);
#ifdef debug
			for(int j=0; j<NP; j++)
				printf("Resampled particle %d index: %d\n", j, index_out[j]);
#endif
			// Resampling particles
			if(i==itl_outer-1)
				resampleFPGA(state_out, index_out);
			else
				resampleFPGA(state_in, index_out);
#else
			printf("Calling CPU function...\n");
			smcCPU(itl_inner,state_in,obsrv_in,state_out);
#endif

		}
		update(state_in, state_out);
		output(t, state_in);
	}

	return 0;
}
