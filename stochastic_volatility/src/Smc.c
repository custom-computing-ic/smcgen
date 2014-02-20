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

void init(char *obsrvFile, float* obsrv, float* state);
void smcKernel(int itl_inner, float* state_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out);
void resample(float* state_out, int* index);
void output(int step, float* state);
void update(float* state, float* control);

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
			// Invoke FPGA kernel
			printf("Calling FPGA kernel...\n");
			smcKernel(itl_inner,state_in,rand_num,seed,obsrv_in,index_out,state_out);

#ifdef debug
			for(int j=0; j<NP; j++)
				printf("Resampled particle %d index: %d\n", j, index_out[j]);
#endif

			// Resampling particles
			if(i==itl_outer-1)
				resample(state_out, index_out);
			else
				resample(state_in, index_out);
		}
		update(state_in, state_out);
		output(t, state_in);
	}

	return 0;
}

void init(char *obsrvFile, float* obsrv, float* state){
	
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
			state[p*SS*NA+a*SS] = 0;//((float) dsfmt_genrand_close_open(&dsfmt[p]))*18;
		}
	}
}

void smcKernel(int itl_inner, float* state_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out){

	struct timeval tv1, tv2;

	// Copy states to LMEM
	gettimeofday(&tv1, NULL);
	Smc_ram(NP, state_in);
	gettimeofday(&tv2, NULL);
	unsigned long long lmem_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("Copyed data to LMEM in %lu us.\n", (long unsigned int)lmem_time);

	// Invoke FPGA kernel
	gettimeofday(&tv1, NULL);
	Smc(NP, itl_inner, obsrv_in, rand_num, seed, index_out, state_out);
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("FPGA kernel finished in %lu us.\n", (long unsigned int)kernel_time);
}

void resample(float* state, int* index){

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

void output(int step, float* state){
	for(int a=0; a<NA; a++){
		float sum_x = 0;
		for(int p=0; p<NP; p++){
			sum_x += state[p*SS*NA+a*SS];
		}
		printf("At step %d, state is %f.\n", step, sum_x/(NP*1.0));
	}
}

void update(float* state_current, float* state_next){
	for(int a=0; a<NA; a++){
		for(int p=0; p<NP; p++){
			for(int s=0; s<SS; s++)
				state_current[p*SS*NA+a*SS+s] = state_next[p*SS*NA+a*SS+s];
		}
	}
}
