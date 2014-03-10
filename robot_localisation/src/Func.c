/***
	Code of application specific functions for the CPU host.
	User has to customise this file.
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

extern dsfmt_t dsfmt[NPMax];

/*** FPGA mode: Call SMC core */
#ifdef FPGA_resampling
void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out){
#else
void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out, max_file_t* maxfile, max_engarray_t* engines){
#endif

	struct timeval tv1, tv2;

#ifdef debug
	for(int p=0; p<NP; p++)
		printf("State particle %d: (%f %f %f)\n", p, state_in[p*SS], state_in[p*SS+1], state_in[p*SS+2]);
#endif

#ifdef FPGA_resampling // Do resampling on FPGA

	// Copy states to LMEM
	gettimeofday(&tv1, NULL);
	Smc_ram(NP, state_in);
	gettimeofday(&tv2, NULL);
	unsigned long long lmem_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("Copyed data to LMEM in %lu us.\n", (long unsigned int)lmem_time);

	// Invoke FPGA kernel
	gettimeofday(&tv1, NULL);
	Smc(NP, S, itl_inner, obsrv_in, rand_num, ref_in, seed, index_out, state_out);
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("FPGA kernel finished in %lu us.\n", (long unsigned int)kernel_time);
	
	// Rearrange particles
	gettimeofday(&tv1, NULL);
	if(outer_idx==itl_outer-1)
		resampleFPGA(NP, state_out, index_out);
	else
		resampleFPGA(NP, state_in, index_out);
	gettimeofday(&tv2, NULL);
	unsigned long long resampling_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("Resampling finished in %lu us.\n", (long unsigned int)resampling_time);

#else // Do resampling on CPU
	
	float *weight = (float *)malloc(NA*NP*sizeof(float));
	float *weight_sum = (float *)malloc(NA*sizeof(float));

	// Copy states to LMEM
	gettimeofday(&tv1, NULL);
	Smc_ram_actions_t *actions_write[NBoard];
	for (int i=0; i<NBoard; i++){
		actions_write[i] = malloc(sizeof(Smc_ram_actions_t));
		actions_write[i]->param_NP = NP;
		actions_write[i]->instream_particle_mem_from_cpu = state_in + i*NA*NP*SS/NBoard;
	}
	Smc_ram_run_array(engines, actions_write); // for NBoard FPGAs
	//Smc_ram(NP, state_in); // for one FPGA
	gettimeofday(&tv2, NULL);
	unsigned long long lmem_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("Copyed data to LMEM in %lu us.\n", (long unsigned int)lmem_time);

	// Invoke FPGA kernel
	gettimeofday(&tv1, NULL);
	Smc_actions_t *actions[NBoard];
	for (int i=0; i<NBoard; i++){
		actions[i] = malloc(sizeof(Smc_actions_t));
		actions[i]->param_NP = NP;
		actions[i]->param_S = S;
		actions[i]->param_itl_inner = itl_inner;
		actions[i]->instream_obsrv_in = obsrv_in;
		actions[i]->instream_ref_in = ref_in;
		actions[i]->instream_seed_in = seed;
		actions[i]->outstream_state_out = state_out;
		actions[i]->outstream_weight_out = weight;
	}
	Smc_run_array(engines, actions); // for NBoard FPGAs
	//Smc(NP, S, itl_inner, obsrv_in, ref_in, seed, state_out, weight); // for one FPGA
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("FPGA kernel finished in %lu us.\n", (long unsigned int)kernel_time);

	// Resample particles
	gettimeofday(&tv1, NULL);
	for (int a=0; a<NA; a++) {
		weight_sum[a] = 0;
		for (int p=0; p<NP; p++){
			weight_sum[a] += weight[p*NA+a];
		}
	}
	if(outer_idx==itl_outer-1)
		resampleCPU(NP, state_out, weight, weight_sum);
	else
		resampleCPU(NP, state_in, weight, weight_sum);
	gettimeofday(&tv2, NULL);
	unsigned long long resampling_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("Resampling finished in %lu us.\n", (long unsigned int)resampling_time);

#endif

}

/*** FPGA mode: Rearrange particles based on the resampled indices */
void resampleFPGA(int NP, float* state, int* index){

	float *temp = (float *)malloc(NA*NP*SS*sizeof(float));

	for (int a=0; a<NA; a++) {
		for (int p=0; p<NP; p++){
			int k = index[p*NA+a];
			temp[p*SS*NA+a*SS] = state[k*SS*NA+a*SS];
			temp[p*SS*NA+a*SS+1] = state[k*SS*NA+a*SS+1];
			temp[p*SS*NA+a*SS+2] = state[k*SS*NA+a*SS+2];
		}
	}
	memcpy(state, temp, sizeof(float)*NA*NP*SS);

#ifdef debug
	for(int a=0; a<NA; a++){
		for(int p=0; p<NP; p++){
			printf("Resampled particle %d value: %f %f %f\n", p, state[p*SS*NA+a*SS],state[p*SS*NA+a*SS+1],state[p*SS*NA+a*SS+2]);
		}
	}
#endif

}

/*** CPU only mode: Call CPU SMC core */
void smcCPU(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* obsrv_in, float* state_out){

	struct timeval tv1, tv2;
	float *weight = (float *)malloc(NA*NP*sizeof(float));
	float *weight_sum = (float *)malloc(NA*sizeof(float));

	gettimeofday(&tv1, NULL);
	for (int a=0; a<NA; a++) {
		weight_sum[a] = 0;
		#pragma omp parallel for num_threads(THREADS)
		for (int p=0; p<NP; p++){
			// Sampling
			state_out[p*SS*NA+a*SS] = state_in[p*SS*NA+a*SS] + (ref_in[0]+nrand(S*0.5,p)) * cos(state_in[p*SS*NA+a*SS+2]);
			state_out[p*SS*NA+a*SS+1] = state_in[p*SS*NA+a*SS+1] + (ref_in[0]+nrand(S*0.5,p)) * sin(state_in[p*SS*NA+a*SS+2]);
			state_out[p*SS*NA+a*SS+2] = state_in[p*SS*NA+a*SS+2] + (ref_in[1]+nrand(S*0.1,p));
			// Importance weighting
			float obsrvEst = est(state_out[p*SS*NA+a*SS],state_out[p*SS*NA+a*SS+1],state_out[p*SS*NA+a*SS+2]);
			weight[p*NA+a] = exp(-1*pow(obsrvEst-obsrv_in[0],2)/(S*0.5));
			weight_sum[a] += weight[p*NA+a];
		}
	}
	// Resampling of particles
	if(outer_idx==itl_outer-1)
		resampleCPU(NP, state_out, weight, weight_sum);
	else
		resampleCPU(NP, state_in, weight, weight_sum);
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("CPU function finished in %lu us.\n", (long unsigned int)kernel_time);
}

/*** CPU only mode: Estimate sensor value */
float est(float x, float y, float h){
	
	float ax[8] = {0,0,18,18,0,8,6,12};
	float ay[8] = {0,12,12,0,6,6,6,6};
	float bx[8] = {0,18,18,0,4,16,6,12};
	float by[8] = {12,12,0,0,6,6,12,12};

	float min_dist = 99;
	for (int i=0; i<8; i++){
		float dist = dist2Wall(x,y,cos(h),sin(h),ax[i],ay[i],bx[i],by[i]);
		if (dist<min_dist)
			min_dist = dist;
	}
	return min_dist;
}

/*** CPU only mode: Calculate distance to wall */
float dist2Wall(float x, float y, float cos_h, float sin_h, float ax, float ay, float bx, float by){

	float dy = by-ay;
	float dx = bx-ax;
	float pa = dy * (ax-x) - dx * (ay-y);
	float pb = dy * cos_h - dx * sin_h;
	float temp = (pb==0) ? 99 : pa/pb;
	float dist = (temp<0) ? 99 : temp;
	float x_check = x + dist * cos_h;
	float y_check = y + dist * sin_h;
	int cond_a = ((x_check-ax)>=-0.01 & (x_check-bx)<=0.01) ? 1 : 0;
	int cond_b = ((x_check-ax)<=0.01 & (x_check-bx)>=-0.01) ? 1 : 0;
	int cond_c = ((y_check-ay)>=-0.01 & (y_check-by)<=0.01) ? 1 : 0;
	int cond_d = ((y_check-ay)<=0.01 & (y_check-by)>=-0.01) ? 1 : 0;

	if ((cond_a || cond_b) && (cond_c || cond_d))
		return dist;
	else
		return 99.0;
}

/*** CPU only mode: Resample particles */
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
			temp[p*SS*NA+a*SS+1] = state[(k-1)*SS*NA+a*SS+1];
			temp[p*SS*NA+a*SS+2] = state[(k-1)*SS*NA+a*SS+2];
		}
	}
	memcpy(state, temp, sizeof(float)*NA*NP*SS);
}

/*** Read input files */
void init(int NP, char* obsrvFile, float* obsrv, char* refFile, float* ref, float* state){

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

	// Read reference values
	FILE *fpRef = fopen(refFile, "r");
	if(!fpRef) {
		printf("Failed to open the reference file.\n");
		exit(-1);
	}
	for(int t=0; t<NT; t++){
		fscanf(fpRef, "%f %f\n", &ref[t*RS], &ref[t*RS+1]);
	}
	fclose(fpRef);

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
			state[p*SS*NA+a*SS+2] = 0;//((float) dsfmt_genrand_close_open(&dsfmt[p]))*2*Pi;
		}
	}
}

/*** Output data */
void output(int NP, int step, float* state){

	FILE *fpXest;

	if(step==0)
		fpXest = fopen("data_xest.txt", "w");
	else
		fpXest = fopen("data_xest.txt", "a");

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
		fprintf(fpXest, "%f %f %f\n", sum_x/(NP*1.0), sum_y/(NP*1.0), sum_h/(NP*1.0));
	}

	fclose(fpXest);
}

/*** Commit changes to the particles */
void update(int NP, float* state_current, float* state_next){
	for(int a=0; a<NA; a++){
		for(int p=0; p<NP; p++){
			for(int s=0; s<SS; s++)
				state_current[p*SS*NA+a*SS+s] = state_next[p*SS*NA+a*SS+s];
		}
	}
}

/*** Check estimated states against true states */
void check(char *stateFile){

	FILE *fpX;
	FILE *fpXest;
	fpX = fopen(stateFile, "r");
	fpXest = fopen("data_xest.txt", "r");

	if(!fpX){
		printf("Failed to open the state file.\n");
		exit(-1);
	}
	if(!fpXest){
		printf("Failed to open the estimated state file.\n");
		exit(-1);
	}

	float total_error, step_error;
	float x, y, h;
	float x_est, y_est, h_est;

	total_error = 0;
	for(int t=0; t<NT; t++){
		fscanf(fpX, "%f %f %f\n", &x, &y, &h);
		fscanf(fpXest, "%f %f %f\n", &x_est, &y_est, &h_est);
		step_error = sqrt(pow(x_est-x,2)+pow(y_est-y,2));
		total_error += step_error;
	}
	printf("Average error: %f\n", fabs(total_error)/(NT*1.0));
	fclose(fpX);
	fclose(fpXest);

}

/*** Gaussian random number generator */
float nrand(float sigma, int l){

	float x, y, w;
	float n1;
	static float n2 = 0.0;
	static short n2_cached = {0};

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

