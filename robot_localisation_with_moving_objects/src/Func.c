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
void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out){

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

	float *weight = (float *)malloc(NP*sizeof(float));
	float *weight_sum = (float *)malloc(sizeof(float));

	// Invoke FPGA kernel
	gettimeofday(&tv1, NULL);
	Smc(NP, S, itl_inner, obsrv_in, ref_in, seed, state_out, weight);
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("FPGA kernel finished in %lu us.\n", (long unsigned int)kernel_time);

	// Resample particles
	gettimeofday(&tv1, NULL);
	weight_sum[a] = 0;
	for (int p=0; p<NP; p++){
		weight_sum[a] += weight[p];
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

/* CPU only functions */

// Call CPU SMC core
void smcCPU(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* obsrv_in, float* state_out){

	struct timeval tv1, tv2;
	float *weight = (float *)malloc(NP*sizeof(float));
	float *weight_sum = (float *)malloc(sizeof(float));

	gettimeofday(&tv1, NULL);
	weight_sum[a] = 0;
#pragma omp parallel for num_threads(THREADS)
	for (int p=0; p<NP; p++){
		// Sampling
		if (p%NPBlock==0){ // robot particles
			state_out[p*SS] = state_in[p*SS] + (ref_in[0]+nrand(S*0.5,p)) * cos(state_in[p*SS+2]);
			state_out[p*SS+1] = state_in[p*SS+1] + (ref_in[0]+nrand(S*0.5,p)) * sin(state_in[p*SS+2]);
			state_out[p*SS+2] = state_in[p*SS+2] + (ref_in[1]+nrand(S*0.1,p));
		}else{ // particles of the moving objects
			float dist = 0.05+nrand(0.02,p);
			float rot = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*18;
			state_out[p*SS] = state_in[p*SS] + (dist+nrand(S*0.5,p)) * cos(state_in[p*SS+2]);
			state_out[p*SS+1] = state_in[p*SS+1] + (dist+nrand(S*0.5,p)) * sin(state_in[p*SS+2]);
			state_out[p*SS+2] = state_in[p*SS+2] + (rot+nrand(S*0.1,p));
		}
		// Importance weighting
		int NSensor = 20;
		float h_step = Pi/(3.0*NSensor);
		float *obsrvEst = (float *)malloc(Nsensor*sizeof(float));
		float *temp = (float *)malloc(Nsensor*sizeof(float));
		int index = p/NPBlock*NPBlock;
		float x_robot = state_out[index*SS];
		float y_robot = state_out[index*SS+1];
		float h_base = state_out[index*SS+2] - Pi/3.0;
		for (int i=0; i<20; i++){
			if (p%NPBlock==0){ // check walls
				obsrvEst[i] = estWall(x_robot,y_robot,h_base+h_step*i);
				temp[i] = obsrvEst[i];
			}else{ // check moving objects
				if (p!=0 && (p-1)%7==0)	obsrvEst[i] = temp[i];
				obsrvEst[i] = min(obsrvEst[i],estObj(x_robot,y_robot,h_base+h_step*i,state_out[p*SS],state_out[p*SS+1]));
			}
		}
		if (p!=0 && (p-1)%7==6){
			float base = 0;
			for (int i=0; i<Nsensor; i++)
				base += obsrvEst[i];
			weight[(p-1)/7] = exp(base*base/-200.0);
			weight_sum[a] += weight[(p-1)/7];
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

// Estimate sensor value (wall)
float estWall(float x, float y, float h){

	float ax[8] = {0,0,18,18,0,8,6,12};
	float ay[8] = {0,12,12,0,6,6,6,6};
	float bx[8] = {0,18,18,0,4,16,6,12};
	float by[8] = {12,12,0,0,6,6,12,12};

	float min_dist = 99;
	for (int i=0; i<8; i++){
		float dist = dist(x,y,cos(h),sin(h),ax[i],ay[i],bx[i],by[i]);
		if (dist<min_dist)
			min_dist = dist;
	}
	return min_dist;
}

// Estimate sensor value (moving objects)
float estObj(float x, float y, float h, float ox, float oy){

	return dist(x,y,cos(h),sin(h),0,0.4,0,0.4);
}

// Calculate distance
float dist(float x, float y, float cos_h, float sin_h, float ax, float ay, float bx, float by){

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

// Resample particles
void resampleCPU(int NP, float* state, float* weight, float* weight_sum){

	int NPObj = (NPBlock-1)/7;
	float *sum_pdf = (float *)malloc((NPObj+1)*sizeof(float));
	float *temp = (float *)malloc(NP*SS*sizeof(float));

	for (int p=0; p<NP; p++){
		// Resampling of people particles
		sum_pdf[0] = 0;
		for (int i=1; i<=NPObj; i++){
			sum_pdf[i] = sum_pdf[i-1] + weight[i]/weight_sum;
		}
		int k=0;
		u1 = ((float) dsfmt_genrand_close_open(&dsfmt)) / (NPObj*1.0);
		for (int i=0; i<NPObj; i++){
			u = u1 + i/(NPObj*1.0);
			while (u-sum_pdf[k]>=0 && k<NPObj){
				k = k + 1;
			}
			memcpy(temp, state+1+(k-1)*7*SS, sizeof(float)*7*SS);
			weight[i] = weight[k-1];
		}
		w[p] = 0;
		// Weight of robot particles
		for (int i=0; i<NPObj; i++){
			w[p] += weight[i];
		}
		memcpy(state+p*7*SS, temp, sizeof(float)*7*SS);
	}
	sum_pdf[0] = 0;
	for (int p=1; p<=NP; p++){
		sum_pdf[p] = sum_pdf[p-1] + weight[p-1]/weight_sum[a];
	}
	int k=0;
	float u1 = ((float) dsfmt_genrand_close_open(&dsfmt[0])) / (NP*1.0);
	for (int p=0; p<NP; p++){
		float u = u1 + p/(NP*1.0);
		while (u-sum_pdf[k]>=0 && k<NP){
			k = k + 1;
		}
		temp[p*SS] = state[(k-1)*SS];
		temp[p*SS+1] = state[(k-1)*SS+1];
		temp[p*SS+2] = state[(k-1)*SS+2];
	}
	memcpy(state, temp, sizeof(float)*NP*SS);
}

/* Common functions */

// Read input files
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
		fscanf(fpRef, "%f %f\n", &ref[t*2], &ref[t*2+1]);
	}
	fclose(fpRef);

	// Initialise random number generators
	srand(time(NULL));
	for(int  p=0; p<NP; p++){
		dsfmt_init_gen_rand(&dsfmt[p], rand());
	}

	// Initialise states
	for(int p=0; p<NP; p++){
		state[p*SS] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*18;
		state[p*SS+1] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*12;
		state[p*SS+2] = 0;//((float) dsfmt_genrand_close_open(&dsfmt[p]))*2*Pi;
	}
}

// Output particle values
void output(int NP, int step, float* state){

	FILE *fpXest;

	if(step==0)
		fpXest = fopen("data_xest.txt", "w");
	else
		fpXest = fopen("data_xest.txt", "a");

	float sum_x = 0;
	float sum_y = 0;
	float sum_h = 0;
	for(int p=0; p<NP; p++){
		sum_x += state[p*SS];
		sum_y += state[p*SS+1];
		sum_h += state[p*SS+2];
	}
	printf("Agent %d's position at step %d is (%f, %f, %f).\n", a, step, sum_x/(NP*1.0), sum_y/(NP*1.0), sum_h/(NP*1.0));
	fprintf(fpXest, "%f %f %f\n", sum_x/(NP*1.0), sum_y/(NP*1.0), sum_h/(NP*1.0));

	fclose(fpXest);
}

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

// Commit changes to the particles
void update(int NP, float* state_current, float* state_next){
	for(int p=0; p<NP; p++){
		for(int s=0; s<SS; s++)
			state_current[p*SS+s] = state_next[p*SS+s];
	}
}

// Gaussian random number generator
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

