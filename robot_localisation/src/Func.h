/***
	Code of application specific functions for the CPU host.
	User has to customise this file.
*/

#ifndef FUNC
#define FUNC

/*** State struct */
typedef struct state_t
{
	float x;
	float y;
	float h;
}state_t;

/*** Reference struct */
typedef struct ref_t
{
	float d;
	float r;
}ref_t;

void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* rand_num, int* seed, float* obsrv_in, float* state_out, max_group_t* group);
void resampleFPGA(int NP, float* state_out, int* index);
void smcCPU(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* obsrv_in, float* state_out);
float est(float x, float y, float h);
float dist2Wall(float x, float y, float cos_h, float sin_h, float ax, float ay, float bx, float by);
void resampleCPU(int NP, float* state, float* weight, float* weight_sum);
void init(int NP, char *obsrvFile, float* obsrv, char *refFile, float* ref, float* state);
void output(int NP, int step, float* state);
void update(int NP, float* state_current, float* state_next);
void check(char *stateFile);
float nrand(float sigma, int l);

#endif
