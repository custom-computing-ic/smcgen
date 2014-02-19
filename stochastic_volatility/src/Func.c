#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <mathimf.h>
#include "dSFMT.h"
#include "Def.h"
#include "Func.h"

extern dsfmt_t dsfmt[NP];

float nrand(float sigma, int l){

	float x, y, w;
	float n1;
	static float n2[NP] = {0.0};
	static short n2_cached[NP] = {0};

	if (!n2_cached[l]){
		do {
			x = 2.0 * dsfmt_genrand_close_open(&dsfmt[l]) - 1.0;
			y = 2.0 * dsfmt_genrand_close_open(&dsfmt[l]) - 1.0;
			w = x * x + y * y;
		} while (w > 1.0 || w == 0);

		w = sqrt((-2.0*log(w))/w);
		n1 = x * w;
		n2[l] = y * w;
		n2_cached[l] = 1;

		return sigma * n1;
	}
	else{

		n2_cached[l] = 0;
		return sigma * n2[l];
	}

}

