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

extern dsfmt_t dsfmt[NPMax*slotOfP];

/* FPGA only functions */

// Call FPGA SMC core
void smcFPGA(int NP, int slotOfAllP, float S, int itl_outer, int outer_idx, int itl_inner, float* state_in, float* ref_in, int* seed, float* obsrv_in, float* state_out){

	struct timeval tv1, tv2;

	float *weightObj = (float *)malloc(NP*sizeof(float));

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
	Smc(NP, S, itl_inner, obsrv_in, ref_in, seed, state_out, weightObj);
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("FPGA kernel finished in %lu us.\n", (long unsigned int)kernel_time);

	// Resample particles
	gettimeofday(&tv1, NULL);
	if(outer_idx==itl_outer-1)
		resampleCPU(NP, slotOfAllP, state_out, weightObj);
	else
		resampleCPU(NP, slotOfAllP, state_in, weightObj);
	gettimeofday(&tv2, NULL);
	unsigned long long resampling_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("Resampling finished in %lu us.\n", (long unsigned int)resampling_time);

}

/* CPU only functions */

// Call CPU SMC core
void smcCPU(int NP, int slotOfAllP, float S, int itl_outer, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* obsrv_in, float* state_out){

	struct timeval tv1, tv2;

	float h_step = 2.0*Pi/(NSensor*1.0);

	float *obsrvTmp = (float *)malloc(NP*NSensor*sizeof(float));
	float *weightObj = (float *)malloc(NP*NPObj*sizeof(float));

	gettimeofday(&tv1, NULL);
	// Sampling
#pragma omp parallel for num_threads(THREADS)
	for (int p=0; p<slotOfAllP; p++){
		int idxInP = p%slotOfP; // Index inside a particle
		if (idxInP==0){ // robot particles
			state_out[p*SS] = state_in[p*SS] + (ref_in[0]+nrand(S*0.2,p)) * cos(state_in[p*SS+2]);
			state_out[p*SS+1] = state_in[p*SS+1] + (ref_in[0]+nrand(S*0.2,p)) * sin(state_in[p*SS+2]);
			state_out[p*SS+2] = state_in[p*SS+2] + (ref_in[1]+nrand(S*0.05,p));
		}else{ // particles of the moving objects
			float dist = 0.05+nrand(0.02,p);
			float rot = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*18;
			state_out[p*SS] = state_in[p*SS];// + (dist + nrand(S*0.2,p)) * cos(state_in[p*SS+2]);
			state_out[p*SS+1] = state_in[p*SS+1];// + (dist+nrand(S*0.2,p)) * sin(state_in[p*SS+2]);
			state_out[p*SS+2] = state_in[p*SS+2];// + (rot+nrand(S*0.05,p));
		}
	}
	// Importance weighting
#pragma omp parallel for num_threads(THREADS)
	for (int p=0; p<slotOfAllP; p++){
		int idxOfP = p/slotOfP; // Index of particle
		int idxInP = p%slotOfP; // Index inside a particle
		float x_robot, y_robot, h_base;
		for (int i=0; i<NSensor; i++){
			if (idxInP==0){
				// Check walls
				x_robot = state_out[p*SS];
				y_robot = state_out[p*SS+1];
				h_base = state_out[p*SS+2];
				obsrvTmp[idxOfP*NSensor+i] = estWall(x_robot,y_robot,h_base+h_step*i);
			}
		}
	}
#pragma omp parallel for num_threads(THREADS)
	for (int p=0; p<slotOfAllP; p++){
		int idxOfP = p/slotOfP; // Index of particle
		int idxInP = p%slotOfP; // Index inside a particle
		float x_robot, y_robot, h_base;
		float obsrvEst[NSensor];
		for (int i=0; i<NSensor; i++){
			if (idxInP!=0){
				// Check moving objects
				if ((idxInP-1)%Obj==0){
					obsrvEst[i] = obsrvTmp[idxOfP*NSensor+i];
				}
				obsrvEst[i] = fmin(obsrvEst[i],estObj(x_robot,y_robot,h_base+h_step*i,state_out[p*SS],state_out[p*SS+1]));
			}
		}
		if ((idxInP-1)%Obj==6){
			float base = 0;
			for (int i=0; i<NSensor; i++){
				base = base + pow(obsrvEst[i]-obsrv_in[i],2);
			}
			weightObj[idxOfP*NPObj+(idxInP-1)/Obj] = exp(base/-10.0*NSensor*S);
		}
	}
	// Resampling of robot particles
	if(outer_idx==itl_outer-1)
		resampleCPU(NP, slotOfAllP, state_out, weightObj);
	else
		resampleCPU(NP, slotOfAllP, state_in, weightObj);
	gettimeofday(&tv2, NULL);
	unsigned long long kernel_time = (tv2.tv_sec - tv1.tv_sec)*1000000 + (tv2.tv_usec - tv1.tv_usec);
	printf("CPU function finished in %lu us.\n", (long unsigned int)kernel_time);
}

// Resample particles
void resampleCPU(int NP, int slotOfAllP, float* state, float* weightObj){

	// Resampling of moving object particles
	float *weightR = (float *)malloc(NP*sizeof(float));
#pragma omp parallel for num_threads(THREADS)
	for (int p=0; p<NP; p++){
		weightR[p] = resampleObj(state+p*slotOfP*SS+1, weightObj+p*NPObj);
	}
	float weightR_sum = 0;
	for (int p=0; p<NP; p++){
		weightR_sum += weightR[p];
	}

	// Resampling of robot particles
	float *sum_pdf = (float *)malloc((NP+1)*sizeof(float));
	float *temp = (float *)malloc(slotOfAllP*SS*sizeof(float));
	sum_pdf[0] = 0;
	for (int p=1; p<=NP; p++){
		sum_pdf[p] = sum_pdf[p-1] + weightR[p-1]/weightR_sum;
	}
	int k=0;
	float u1 = ((float) dsfmt_genrand_close_open(&dsfmt[0])) / (NP*1.0);
	for (int p=0; p<NP; p++){
		float u = u1 + p/(NP*1.0);
		while (u-sum_pdf[k]>=0 && k<NP){
			k = k + 1;
		}
		memcpy(temp+p*slotOfP*SS, state+(k-1)*slotOfP*SS, slotOfP*SS*sizeof(float));
	}
	memcpy(state, temp, slotOfAllP*SS*sizeof(float));
}

float resampleObj(float* state, float* weightObj){

	float *sum_pdf = (float *)malloc((NPObj+1)*sizeof(float));
	float *temp = (float *)malloc(Obj*NPObj*SS*sizeof(float));
	float *weightObjTmp = (float *)malloc(NPObj*sizeof(float));

	float weightObj_sum = 0;
	for (int p=0; p<NPObj; p++){
		weightObj_sum += weightObj[p];
	}
	sum_pdf[0] = 0;
	for (int p=1; p<=NPObj; p++){
		sum_pdf[p] = sum_pdf[p-1] + weightObj[p]/weightObj_sum;
	}
	int k=0;
	float u1 = ((float) dsfmt_genrand_close_open(&dsfmt[0])) / (NPObj*1.0);
	for (int p=0; p<NPObj; p++){
		float u = u1 + p/(NPObj*1.0);
		while (u-sum_pdf[k]>=0 && k<NPObj){
			k = k + 1;
		}
		memcpy(temp+p*Obj*SS, state+(k-1)*Obj*SS, Obj*SS*sizeof(float));
		weightObjTmp[p] = weightObj[k-1];
	}
	float weightR = 0;
	// Weight of robot particles
	for (int p=0; p<NPObj; p++){
		weightR += weightObjTmp[p];
	}
	memcpy(state, temp, Obj*NPObj*SS*sizeof(float));

	return weightR;
}

// Estimate sensor value (wall)
float estWall(float x, float y, float h){

	float ax[8] = {0,0,18,18,0,8,6,12};
	float ay[8] = {0,12,12,0,6,6,6,6};
	float bx[8] = {0,18,18,0,4,16,6,12};
	float by[8] = {12,12,0,0,6,6,12,12};

	float min_dist = 999.9;
	for (int i=0; i<8; i++){
		float dist = dist2Obj(x,y,cos(h),sin(h),ax[i],ay[i],bx[i],by[i]);
		if (dist<min_dist)
			min_dist = dist;
	}
	return min_dist;
}

// Estimate sensor value (moving objects)
float estObj(float x, float y, float h, float ox, float oy){

	return dist2Obj(x,y,cos(h),sin(h),ox-0.2,oy-0.2,ox+0.2,oy+0.2);
}

// Calculate distance
float dist2Obj(float x, float y, float cos_h, float sin_h, float ax, float ay, float bx, float by){

	float dy = by-ay;
	float dx = bx-ax;
	float pa = dy * (ax-x) - dx * (ay-y);
	float pb = dy * cos_h - dx * sin_h;
	float temp = (pb==0) ? 999.9 : pa/pb;
	float dist = (temp<0) ? 999.9 : temp;
	float x_check = x + dist * cos_h;
	float y_check = y + dist * sin_h;
	int cond_a = ((x_check-ax)>=-0.01 & (x_check-bx)<=0.01) ? 1 : 0;
	int cond_b = ((x_check-ax)<=0.01 & (x_check-bx)>=-0.01) ? 1 : 0;
	int cond_c = ((y_check-ay)>=-0.01 & (y_check-by)<=0.01) ? 1 : 0;
	int cond_d = ((y_check-ay)<=0.01 & (y_check-by)>=-0.01) ? 1 : 0;

	if ((cond_a || cond_b) && (cond_c || cond_d))
		return dist;
	else
		return 999.9;
}

/* Common functions */

// Read input files
void init(int NP, int slotOfAllP, char* obsrvFile, float* obsrv, char* refFile, float* ref, float* state){

	// Read observations
	FILE *fpObsrv = fopen(obsrvFile, "r");
	if(!fpObsrv) {
		printf("Failed to open the observation file.\n");
		exit(-1);
	}
	for(int t=0; t<NT; t++){
		for (int i=0; i<NSensor; i++){
			fscanf(fpObsrv, "%f\n", &obsrv[t*NSensor+i]);
		}
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
	for(int  p=0; p<slotOfAllP; p++){
		dsfmt_init_gen_rand(&dsfmt[p], rand());
	}

	// Initialise states
	for (int p=0; p<slotOfAllP; p++){
		int idxInP = p%slotOfP; // Index inside a particle
		if (idxInP==0){ // robot particles
			state[p*SS] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*18;
			state[p*SS+1] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*12;
			state[p*SS+2] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*2*Pi;
		}else{ // particles of the moving objects
			state[p*SS] = 8+((float) dsfmt_genrand_close_open(&dsfmt[p]))*(18-8);
			state[p*SS+1] = 6+((float) dsfmt_genrand_close_open(&dsfmt[p]))*(12-6);
			state[p*SS+2] = ((float) dsfmt_genrand_close_open(&dsfmt[p]))*2*Pi;
		}
	}
}

// Output particle values
void output(int cnt, int NP, int step, float* state){

	FILE *fpXest;

	char buf[20];
	sprintf(buf, "data_x_est_%d.txt", cnt);
	if(step==0){
		fpXest = fopen(buf, "w");
	}else{
		fpXest = fopen(buf, "a");
	}

	float sum_x = 0;
	float sum_y = 0;
	float sum_h = 0;
	for(int p=0; p<NP; p++){
		sum_x += state[p*slotOfP*SS];
		sum_y += state[p*slotOfP*SS+1];
		sum_h += state[p*slotOfP*SS+2];
	}
	printf("Position at step %d is (%f, %f, %f).\n", step, sum_x/(NP*1.0), sum_y/(NP*1.0), sum_h/(NP*1.0));
	fprintf(fpXest, "%f %f %f\n", sum_x/(NP*1.0), sum_y/(NP*1.0), sum_h/(NP*1.0));

	fclose(fpXest);
}

void check(char *stateFile, int NP, int itl_outer){

	char buf[20];

	float total_error = 0;
	for (int cnt=0; cnt<NTest; cnt++){


		FILE *fpX;
		fpX = fopen(stateFile, "r");
		
		FILE *fpXest;
		sprintf(buf, "data_x_est_%d.txt", cnt);
		fpXest = fopen(buf, "r");

		if(!fpX){
			printf("Failed to open the state file.\n");
			exit(-1);
		}
		if(!fpXest){
			printf("Failed to open the estimated state file.\n");
			exit(-1);
		}

		float step_error;
		float x, y, h;
		float x_est, y_est, h_est;

		for(int t=0; t<NT; t++){
			fscanf(fpX, "%f %f %f\n", &x, &y, &h);
			fscanf(fpXest, "%f %f %f\n", &x_est, &y_est, &h_est);
			step_error = sqrt(pow(x_est-x,2)+pow(y_est-y,2));
			total_error += step_error;

		}
		
		fclose(fpX);
		fclose(fpXest);
	}

	printf("Average error: %f\n", fabs(total_error)/(NT*NTest*1.0));
	printf("Time: %f\n", itl_outer*(NP*slotOfP/(NC*150000000.0)+((NP*slotOfP*SS+NP*NPObj)*sizeof(float))/2000000000.0));

}

// Commit changes to the particles
void update(int slotOfAllP, float* state_current, float* state_next){
	memcpy(state_current, state_next, slotOfAllP*SS*sizeof(float));
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

