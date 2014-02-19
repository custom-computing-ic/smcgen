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

void init(char *obsrvFile, float obsrv[], float state[]);
void smcKernel(int itl_inner, float state_in[], float rand_num[], int seed[], float obsrv[], int index[]);
void resample(float state_out[], int index[]);
void output(int step, float state[]);

int main(int argc, char *argv[]){

	if (argc<3 || argc>4){
		printf("Usage: %s [observation file]\n", argv[0]);
		return 0;
	}

	// Read observation and control
	// Initialise states
	float obsrv[NT];
	float state[NA*NP*SS];
	init(argv[1], obsrv, state);

	// Other array values
	float *state_in;
#if defined NA==1 || defined NA==2 || defined NA==3
	float rand_num[4];
#else
	float rand_num[NA];
#endif
	int seed[NC*SS*16*3];
#if defined NA==1 || defined NA==2 || defined NA==3
	float obsrv_in[4];
#else
	float obsrv_in[NA];
#endif
	int index[NA*NP];

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
			// Invoke FPGA kernel
			printf("Calling FPGA kernel...\n");
			smcKernel(itl_inner,state_in,rand_num,seed,obsrv_in,index);
			printf("FPGA kernel finished...\n");

#ifdef debug
			for(int j=0; j<NP; j++)
				printf("Resampled particle %d index: %d\n", j, index[j]);
#endif

			// Resampling particles
			resample(state_in, index);
		}
		output(t, state_in);
	}

	return 0;
}

void init(char *obsrvFile, float obsrv[NT], float state[NA*NP]){
	
	// Read observations
	FILE *fpSensor = fopen(obsrvFile, "r");
	if(!fpSensor) {
		printf("Failed to open the observation file.\n");
		exit(-1);
	}
	for(int t=0; t<NT; t++){
		fscanf(fpSensor, "%f\n", &obsrv[t]);
	}
	fclose(fpSensor);

	// Initialise random number generators
	srand(time(NULL));
	for(int  p=0; p<NP; p++){
		dsfmt_init_gen_rand(&dsfmt[p], rand());
	}

	// Initialise states
	for(int a=0; a<NA; a++){
		for(int p=0; p<NP; p++){
			state[p*SS*NA+a*SS] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*18;
		}
	}
}

void smcKernel(int itl_inner, float state_in[], float rand_num[], int seed[], float obsrv[], int index[]){

	// Copy states to LMEM
	Smc_ram(NP, state_in);

	// Invoke FPGA kernel
	Smc(NP, itl_inner, obsrv, rand_num, seed, index);
}

void resample(float state[], int index[]){

	float temp[NA*NP*SS];

	for (int a=0; a<NA; a++) {
		for (int p=0; p<NP; p++){
			int k = index[p*NA+a];
			temp[p*SS*NA+a*SS] = state[k*SS*NA+a*SS];
			temp[p*SS*NA+a*SS+1] = state[k*SS*NA+a*SS+1];
			temp[p*SS*NA+a*SS+2] = state[k*SS*NA+a*SS+2];
		}
	}
	memcpy(state, temp, sizeof(float)*NA*NP*SS);
}

void output(int step, float state[]){
	for(int a=0; a<NA; a++){
		float sum_x = 0;
		for(int p=0; p<NP; p++){
			sum_x += state[p*SS*NA+a*SS];
		}
		printf("At step %d, state is %f, observation is %f.\n", step, sum_x/(NP*1.0), exp(sum_x/(NP*2.0)));
	}
}

