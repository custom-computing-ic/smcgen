#ifndef FUNC
#define FUNC

typedef struct state_t
{
	float x;
	float y;
	float h;
}state_t;

typedef struct ref_t
{
	float d;
	float r;
}ref_t;

void smcFPGA(int NP, float S, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out);
void resampleFPGA(int NP, float* state_out, int* index);
void resampleCPU(int NP, float* state, float* weight, float* weight_sum);
void init(int NP, char *obsrvFile, float* obsrv, char *refFile, float* ref, float* state);
void output(int NP, int step, float* state);
void update(int NP, float* state_current, float* state_next);
void check(char *stateFile);
float nrand(float sigma, int l);

#endif
