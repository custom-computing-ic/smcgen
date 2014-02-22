#include <omp.h>
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

extern dsfmt_t dsfmt[NPMax];

/* FPGA only functions */

// Call FPGA SMC core
void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out){

	struct timeval tv1, tv2;

#ifdef debug
	for(int p=0; p<NP; p++)
		printf("State particle %d: (%f %f %f)\n", p, state_in[p*SS], state_in[p*SS+1], state_in[p*SS+2]);
#endif

	// Copy states to LMEM
	gettimeofday(&tv1, NULL);
	Smc_ram(NP, state_in);
	gettimeofday(&tv2, NULL);
	unsigned long long lmem_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("Copyed data to LMEM in %lu us.\n", (long unsigned int)lmem_time);

	// Invoke FPGA kernel
	gettimeofday(&tv1, NULL);
#ifdef FPGA_resampling
	Smc(NP, S, itl_inner, obsrv_in, rand_num, seed, index_out, state_out);
	// Rearrange particles
	if(outer_idx==itl_outer-1)
		resampleFPGA(NP, state_out, index_out);
	else
		resampleFPGA(NP, state_in, index_out);
#else
	float *weight = (float *)malloc(NA*NP*sizeof(float));
	float *weight_sum = (float *)malloc(NA*sizeof(float));
	Smc(NP, S, itl_inner, obsrv_in, seed, state_out, weight);
	for (int a=0; a<NA; a++) {
		weight_sum[a] = 0;
		#pragma omp parallel for num_threads(THREADS)
		for (int p=0; p<NP; p++){
			weight_sum[a] += weight[p*NA+a];
		}
	}
	// Resampling of robot particles
	if(outer_idx==itl_outer-1)
		resampleCPU(NP, state_out, weight, weight_sum);
	else
		resampleCPU(NP, state_in, weight, weight_sum);
#endif
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("FPGA kernel finished in %lu us.\n", (long unsigned int)kernel_time);

#ifdef debug
	for(int p=0; p<NP; p++)
		printf("Resampled particle %d index: %d\n", p, index_out[p]);
#endif
}

// Rearrange particles based on the resampled indices
void resampleFPGA(int NP, float* state, int* index){

	float *temp = (float *)malloc(NA*NP*SS*sizeof(float));

	for (int a=0; a<NA; a++) {
		for (int p=0; p<NP; p++){
			int k = index[p*NA+a];
			temp[p*SS*NA+a*SS] = state[k*SS*NA+a*SS];
		}
	}
	memcpy(state, temp, sizeof(float)*NA*NP*SS);
}

/* CPU only functions */

// Call CPU SMC core
void smcCPU(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* obsrv_in, float* state_out){

	struct timeval tv1, tv2;
	float *weight = (float *)malloc(NA*NP*sizeof(float));
	float *weight_sum = (float *)malloc(NA*sizeof(float));

	gettimeofday(&tv1, NULL);
	for (int a=0; a<NA; a++) {
		weight_sum[a] = 0;
		#pragma omp parallel for num_threads(THREADS)
		for (int p=0; p<NP; p++){
			// Sampling
			state_out[p*SS*NA+a*SS] = 0.91*state_in[p*SS*NA+a*SS]+nrand(S*1,p);
			// Importance weighting
			weight[p*NA+a] = exp(-0.5*(state_out[p*SS*NA+a*SS]+obsrv_in[0]*obsrv_in[0]*exp(-1*state_out[p*SS*NA+a*SS])));
			weight_sum[a] += weight[p*NA+a];
		}
	}
	// Resampling of robot particles
	if(outer_idx==itl_outer-1)
		resampleCPU(NP, state_out, weight, weight_sum);
	else
		resampleCPU(NP, state_in, weight, weight_sum);
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("CPU function finished in %lu us.\n", (long unsigned int)kernel_time);
}

// Resample particles
void resampleCPU(int NP, float* state, float* weight, float* weight_sum){

	float *temp = (float *)malloc(NA*NP*SS*sizeof(float));
	float *sum_pdf = (float *)malloc((NP+1)*sizeof(float));

	for (int a=0; a<NA; a++) {
		sum_pdf[0] = 0;
		for (int p=1; p<=NP; p++){
			sum_pdf[p] = sum_pdf[p-1] + weight[(p-1)*NA+a]/weight_sum[a];
		}
		int k=0;
		float u1 = ((float) dsfmt_genrand_close_open(&dsfmt[0])) / (NP*1.0);
		for (int p=0; p<NP; p++){
			float u = u1 + p/(NP*1.0);
			while (u-sum_pdf[k]>=0 && k<NP){
				k = k + 1;
			}
			temp[p*SS*NA+a*SS] = state[(k-1)*SS*NA+a*SS];
		}
	}
	memcpy(state, temp, sizeof(float)*NA*NP*SS);
}

/* Common functions */

// Read input files
void init(int NP, char *obsrvFile, float* obsrv, float* state){

	// Read observations
	FILE *fpObsrv = fopen(obsrvFile, "r");
	if(!fpObsrv) {
		printf("Failed to open the observation file.\n");
		exit(-1);
	}
	for(int t=0; t<NT; t++){
		fscanf(fpObsrv, "%f\n", &obsrv[t]);
	}
	fclose(fpObsrv);

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

// Output particle values
void output(int NP, int step, float* state){

	FILE *fpXest;
	if(step==0)
		fpXest = fopen("data_xest.txt", "w");
	else
		fpXest = fopen("data_xest.txt", "a");

	for(int a=0; a<NA; a++){
		float sum_x = 0;
		for(int p=0; p<NP; p++){
			sum_x += state[p*SS*NA+a*SS];
		}
		printf("At step %d, state is %f.\n", step, sum_x/(NP*1.0));
		fprintf(fpXest, "%f\n", sum_x/(NP*1.0));
	}

	fclose(fpXest);
}

// Commit changes to the particles
void update(int NP, float* state_current, float* state_next){
	for(int a=0; a<NA; a++){
		for(int p=0; p<NP; p++){
			for(int s=0; s<SS; s++)
				state_current[p*SS*NA+a*SS+s] = state_next[p*SS*NA+a*SS+s];
		}
	}
}

// Check estimated states with the true states
void check(char *stateFile){

	FILE *fpX;
	FILE *fpXest;
	fpX = fopen(stateFile, "r");
	fpXest = fopen("data_xest.txt", "r");

	if(!fpX) {
		printf("Failed to open the state file.\n");
		exit(-1);
	}
	if(!fpXest) {
		printf("Failed to open the estimated state file.\n");
		exit(-1);
	}

	float total_error, step_error;
	float x, x_est;

	total_error = 0;
	for(int t=0; t<NT; t++){
		fscanf(fpX, "%f\n", &x);
		fscanf(fpXest, "%f\n", &x_est);
		step_error = x_est-x;
		total_error += step_error;
	}
	printf("Average error: %f\n", fabs(total_error)/(NT*1.0));
	fclose(fpX);
	fclose(fpXest);

}

// Gaussian random number generator
float nrand(float sigma, int l){

	float x, y, w;
	float n1;
	static float n2 = 0.0;
	static short n2_cached = 0;

	if (!n2_cached){
		do {
			x = 2.0 * dsfmt_genrand_close_open(&dsfmt[l]) - 1.0;
			y = 2.0 * dsfmt_genrand_close_open(&dsfmt[l]) - 1.0;
			w = x * x + y * y;
		} while (w > 1.0 || w == 0);

		w = sqrt((-2.0*log(w))/w);
		n1 = x * w;
		n2 = y * w;
		n2_cached = 1;

		return sigma * n1;
	}
	else{

		n2_cached = 0;
		return sigma * n2;
	}

}

