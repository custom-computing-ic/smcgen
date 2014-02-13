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

float nrand(float sigma, int l);

#endif
