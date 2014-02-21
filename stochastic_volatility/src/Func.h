#ifndef FUNC
#define FUNC

typedef struct state_t
{
	float x;
	float y;
	float h;
}state_t;

typedef struct control_t
{
	float d;
	float r;
}control_t;

void smcFPGA(int outer_idx, int itl_inner, float* state_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out);
void resampleFPGA(float* state_out, int* index);
void smcCPU(int outer_idx, int itl_inner, float* state_in, float* obsrv_in, float* state_out);
void resampleCPU(float* state, float* weight, float* weight_sum);
void init(char *obsrvFile, float* obsrv, float* state);
void output(int step, float* state);
void update(float* state, float* control);
float nrand(float sigma, int l);

#endif
