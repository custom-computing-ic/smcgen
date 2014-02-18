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

void init(char *sensorFile, float sensor[], char *controlFile, float control[], float state[]);
void smcKernel(int itl_inner, float state_in[], float control_in[], float rand_num[], int seed[], float sensor[], int index[]);
void resample(float state_out[], int index[]);
void output(int step, float state[]);
void update(float state[], float control[]);

int main(int argc, char *argv[]){

	if (argc<3 || argc>4){
		printf("Usage: %s [sensor file] [control file]\n", argv[0]);
		return 0;
	}

	// Read sensor and control readings
	// Initialise states
	float sensor[NT];
	float control[NT*CS];
	float state[NA*NP*SS];
	init(argv[1], sensor, argv[2], control, state);

	// Other array values
	float *state_in;
#ifdef NA==1
	float control_in[NA*CS*2];
#else
	float control_in[NA*CS];
#endif
#if defined NA==1 || defined NA==2 || defined NA==3
	float rand_num[4];
#else
	float rand_num[NA];
#endif
	int seed[NC*SS*16*3];
#if defined NA==1 || defined NA==2 || defined NA==3
	float sensor_in[4];
#else
	float sensor_in[NA];
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
			// Allocate control of the current time step
			for(int a=0; a<NA; a++){
				control_in[a*CS] = control[NA*CS*t+a*CS];
				control_in[a*CS+1] = control[NA*CS*t+a*CS+1];
			}
			// Setup rand number for FPGA resampler
			for(int a=0; a<NA; a++)
				rand_num[a] = dsfmt_genrand_close_open(&dsfmt[0])/(NP*1.0);
			// Setup seeds for FPGA random number generators
			for(int j=0; j<NC*SS*16*3; j++)
				seed[j] = 7-j;
			// Allocate sensor value of the current time step
			for(int a=0; a<NA; a++){
				sensor_in[a] = sensor[NA*t+a];
			}
			// Invoke FPGA kernel
			printf("Calling FPGA kernel...\n");
			smcKernel(itl_inner,state_in,control_in,rand_num,seed,sensor_in,index);
			printf("FPGA kernel finished...\n");

#ifdef debug
			for(int j=0; j<NP; j++)
				printf("Resampled particle %d index: %d\n", j, index[j]);
#endif

			if (i==itl_outer-1)
				continue;
			// Resampling particles
			resample(state_in, index);
		}
		output(t, state_in);
		update(state_in, control_in);
	}

	return 0;
}

void init(char *sensorFile, float sensor[NT], char *controlFile, float control[NT], float state[NA*NP]){
	
	// Read sensor values
	FILE *fpSensor = fopen(sensorFile, "r");
	if(!fpSensor) {
		printf("Failed to open the sensor file.\n");
		exit(-1);
	}
	for(int t=0; t<NT; t++){
		fscanf(fpSensor, "%f\n", &sensor[t]);
	}
	fclose(fpSensor);

	// Read control values
	FILE *fpControl = fopen(controlFile, "r");
	if(!fpControl) {
		printf("Failed to open the control file.\n");
		exit(-1);
	}
	for(int t=0; t<NT; t++){
		fscanf(fpControl, "%f %f\n", &control[t*2], &control[t*2+1]);
	}
	fclose(fpControl);

	// Initialise random number generators
	srand(time(NULL));
	for(int  p=0; p<NP; p++){
		dsfmt_init_gen_rand(&dsfmt[p], rand());
	}

	// Initialise states
	for(int a=0; a<NA; a++){
		for(int p=0; p<NP; p++){
			state[p*SS*NA+a*SS] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*18;
			state[p*SS*NA+a*SS+1] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*12;
			state[p*SS*NA+a*SS+2] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*2*Pi;
		}
	}
}

void smcKernel(int itl_inner, float state_in[], float control_in[], float rand_num[], int seed[], float sensor[], int index[]){

	// Copy states to LMEM
	Smc_ram(NP, state_in);

	// Invoke FPGA kernel
	Smc(NP, itl_inner, control_in, sensor, rand_num, seed, index);
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
		float sum_y = 0;
		float sum_h = 0;
		for(int p=0; p<NP; p++){
			sum_x += state[p*SS*NA+a*SS];
			sum_y += state[p*SS*NA+a*SS+1];
			sum_h += state[p*SS*NA+a*SS+2];
		}
		printf("Agent %d's position at step %d is (%f, %f, %f).\n", a, step, sum_x/(NP*1.0), sum_y/(NP*1.0), sum_h/(NP*1.0));
	}
}

void update(float state[], float control[]){
	for(int a=0; a<NA; a++){
		for(int p=0; p<NP; p++){
			state[p*SS*NA+a*SS] += control[a*CS] * cos(state[p*SS*NA+a*SS+2]);
			state[p*SS*NA+a*SS+1] += control[a*CS] * sin(state[p*SS*NA+a*SS+2]);
			state[p*SS*NA+a*SS+2] += control[a*CS+1];
		}
	}
}
