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

void smcFPGA(int itl_inner, float* state_in, float* rand_num, int* seed, float* obsrv_in, int* index_out, float* state_out);
void resampleFPGA(float* state_out, int* index);
void smcCPU(int itl_inner, float* state_in, float* obsrv_in, float* state_out);
void init(char *obsrvFile, float* obsrv, float* state);
void output(int step, float* state);
void update(float* state, float* control);
float nrand(float sigma, int l);

#endif
