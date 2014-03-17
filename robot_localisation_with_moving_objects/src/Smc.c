/***
	A sequential Monte Carlo CPU host.
*/

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

dsfmt_t dsfmt[NPMax*slotOfP];

int main(int argc, char *argv[]){

	printf("Usage: %s [observation file (Y)] [reference file (R)] [true state file (X)] [NP] [S] [itl_outer]\n", argv[0]);

	int NP = atoi(argv[4]);
	float S = atof(argv[5]);
	int itl_outer = atoi(argv[6]);
	int itl_inner;

	int slotOfAllP = NP*slotOfP;
	
	// Read observation and reference
	// Initialise states
	float *obsrv = (float *)malloc(NT*NSensor*sizeof(float));
	float *ref = (float *)malloc(NT*RS*sizeof(float));
	float *state = (float *)malloc(slotOfAllP*SS*sizeof(float));

	// Other array values
	float *state_in = state;
	float *state_out = (float *)malloc(slotOfAllP*SS*sizeof(float));
	float *ref_in = (float *)malloc(RS*2*sizeof(float));
	int *seed = (int *)malloc(NC*SS*16*3*sizeof(int));
	float *obsrv_in = (float *)malloc(NSensor*sizeof(float));
	float *weightObj = (float *)malloc(NP*NPObj*sizeof(float));

	// Load multiple FPGAs
	max_file_t *maxfile = Smc_init();
	max_engarray_t *engines = max_load_array(maxfile,NBoard,"*");

	for (int cnt=0; cnt<NTest; cnt++){ // Run for ten times to get the average error
		
		init(NP, slotOfAllP, argv[1], obsrv, argv[2], ref, state);
		
		for(int t=0; t<NT; t++){
			for (int i=0; i<itl_outer; i++) {

				// Determine inner loop iteration, a number divisible by NC
				itl_inner = 1;
				// Allocate references of the current time step
				ref_in[0] = ref[RS*t];
				ref_in[1] = ref[RS*t+1];
				// Setup seeds for FPGA random number generators
				for(int j=0; j<NC*SS*16*3; j++)
					seed[j] = 7-j;
				// Allocate observation value of the current time step
				memcpy(obsrv_in, obsrv+t*NSensor, NSensor*sizeof(float));
#ifdef Use_FPGA
				// Invoke FPGA kernel
				printf("Calling FPGA kernel...\n");
#if NC>1
				reOrderParticles(NP, state_in);
#endif
				smcFPGA(NP,slotOfAllP,S,itl_outer,i,itl_inner,state_in,ref_in,seed,obsrv_in,state_out,weightObj,engines);
#else
				printf("Calling CPU function...\n");
				smcCPU(NP,slotOfAllP,S,itl_outer,i,itl_inner,state_in,ref_in,obsrv_in,state_out,weightObj);
#endif

			}
			update(slotOfAllP, state_in, state_out);
			output(cnt, NP, t, state_in);
		}
	}
	check(argv[3], NP, itl_outer);

	// Release FPGA resources
	max_unload_array(engines);
	max_file_free(maxfile);

	free(obsrv);
	free(ref);
	free(state);
	free(state_out);
	free(ref_in);
	free(seed);
	free(obsrv_in);
	free(weightObj);

	return 0;
}

