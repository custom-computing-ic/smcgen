#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#define Pi 3.141592654
#define NY 		8 	// Number of walls
#define NStep 	10 	// Max step
#define NSensor 20  // Number of sensors
#define NObj    7   // Number of moving objects

int main(int argc, char *argv[])
{
	// For RNG
	srand(time(NULL));
	int seed_1 = rand();

	// Map
	const float dx[NY] =  {  0, 18,  0,-18,  4,  8,  0,  0};
	const float dy[NY] =  { 12,  0,-12,  0,  0,  0,  6,  6};
	const float oxa[NY] = {  0,  0, 18, 18,  0,  8,  6, 12};
	const float oxb[NY] = {  0, 18, 18,  0,  4, 16,  6, 12};
	const float oya[NY] = {  0, 12, 12,  0,  6,  6,  6,  6};
	const float oyb[NY] = { 12, 12,  0,  0,  6,  6, 12, 12};

	// Robot location
	float x[NStep];
	float y[NStep];
	float h[NStep];
	float px[NStep][NObj];
	float py[NStep][NObj];
	float new_min[NSensor];

	// Read robot locations
	FILE *fp_robot;
	fp_robot = fopen("data_x_robot.txt", "r");
	if (!fp_robot){
		puts("Failed to open robot locations file...");
		return 0;
	}
	for (int i=0; i<NStep; i++){
		fscanf(fp_robot, "%f %f %f\n", &x[i], &y[i], &h[i]);
	}
	fclose(fp_robot);

	// Read locations of moving objects
	FILE *fp_obj;
	int pp_step;
	fp_obj = fopen("data_x_obj.txt", "r");
	if (!fp_obj){
		puts("Failed to open people locations file...");
		return 0;
	}
	for (int i=0; i<NStep; i++){
		for (int j=0; j<NObj; j++){
			fscanf(fp_obj, "%f %f\n", &px[i][j], &py[i][j]);
		}
	}
	fclose(fp_obj);

	for (int i=0; i<NStep; i++){
		float h_base = h[i]-Pi/3;
		float h_step = Pi/1.5/NSensor;
		for (int j=0; j<NSensor; j++){
			// Adjust sensor angle
			float h_curr = h_base + h_step*j;
			float cos_h = cos(h_curr);
			float sin_h = sin(h_curr);
			new_min[j] = -1;
			// Test people
			for (int k=0; k<NObj; k++){ // People being tracked
				float poxa = px[i][k]-0.2;
				float poxb = px[i][k]+0.2;
				float poya = py[i][k]-0.2;
				float poyb = py[i][k]+0.2;
				float tmp_a = 0.4 * (poxa-x[i]);
				float tmp_b = 0.4 * (poya-y[i]);
				float tmp_c = 0.4 * cos_h;
				float tmp_d = 0.4 * sin_h;
				float dist = ((tmp_a-tmp_b)/(tmp_c-tmp_d));
				float x_check = x[i] + dist * cos_h;
				float y_check = y[i] + dist * sin_h;
				if ((((x_check-poxa)>=-0.01 && (x_check-poxb)<=0.01) || ((x_check-poxa)<=0.01 && (x_check-poxb)>=-0.01)) && (((y_check-poya)>=-0.01 && (y_check-poyb)<=0.01) || ((y_check-poya)<=0.01 && (y_check-poyb)>=-0.01))) {
					if (new_min[j]<0 || (dist<new_min[j] && dist>=0))
						new_min[j] = dist;
				}
			}
			// Test walls
			for (int k=0; k<NY; k++){ // Walls being tracked
				float tmp_a = dy[k] * (oxa[k]-x[i]);
				float tmp_b = dx[k] * (oya[k]-y[i]);
				float tmp_c = dy[k] * cos_h;
				float tmp_d = dx[k] * sin_h;
				float dist = ((tmp_a-tmp_b)/(tmp_c-tmp_d));
				float x_check = x[i] + dist * cos_h;
				float y_check = y[i] + dist * sin_h;
				if ((((x_check-oxa[k])>=-0.01 && (x_check-oxb[k])<=0.01) || ((x_check-oxa[k])<=0.01 && (x_check-oxb[k])>=-0.01)) && (((y_check-oya[k])>=-0.01 && (y_check-oyb[k])<=0.01) || ((y_check-oya[k])<=0.01 && (y_check-oyb[k])>=-0.01))) {
					if (new_min[j]<0 || (dist<new_min[j] && dist>=0))
						new_min[j] = dist;
				}
			}
			printf("%f\n", new_min[j]);
		}
	}

	return 0;
}
