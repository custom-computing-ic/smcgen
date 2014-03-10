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

void smcFPGA(int NP, int slotOfAllP, float S, int itl_outer, int outer_idx, int itl_inner, float* state_in, float* ref_in, int* seed, float* obsrv_in, float* state_out, max_file_t* maxfile, max_engarray_t* engines);
void resampleFPGA(int NP, float* state_out, int* index);
void smcCPU(int NP, int slotOfAllP, float S, int itl_outer, int outer_idx, int itl_inner, float* state_in, float* ref_in, float* obsrv_in, float* state_out);
void resampleCPU(int NP, int slotOfAllP, float* state, float* weightObj);
float resampleObj(float* state, float* weightObj);
float estWall(float x, float y, float h);
float estObj(float x, float y, float h, float ox, float oy);
float dist2Obj(float x, float y, float cos_h, float sin_h, float ax, float ay, float bx, float by);
void init(int NP, int slotOfAllP, char *obsrvFile, float* obsrv, char *refFile, float* ref, float* state);
void output(int cnt, int NP, int step, float* state);
void update(int NP, float* state_current, float* state_next);
void check(char *stateFile, int NP, int itl_outer);
float nrand(float sigma, int l);

#endif
