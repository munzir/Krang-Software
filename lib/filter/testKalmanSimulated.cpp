/**
 * @file testKalman.cpp
 * @author Can Erdogan
 * @date Mar 30, 2013
 * @brief Tests the Kalman filter by feeding it samples from a zero-mean normal distribution.
 * The results are printed to a file where the first column is the samples and the second
 * is the output of the filter. Visualize them with matlab/octave.
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "filter.h"

int main () {

	// Initialize the kalman filter indicating that the state space and the 
  // measurement space is 1.
	filter_kalman_t* kf = new filter_kalman_t;	
	filter_kalman_init(kf, 1, 0, 1);
	
	// Initialize the first state as 0
	kf->x[0] = 0.0;

	// Set the process and measurement models and covariances
	kf->A[0] = 1.0;
	kf->C[0] = 1.0; 
	kf->R[0] = 1e-3;
	kf->Q[0] = 2.0;			//< Note that the measurement covariance is set to 1.

	// Create data for k-iterations
	size_t numIters = 1000;
	FILE* out = fopen("out", "w");
	for(size_t i = 0; i < numIters; i++) {

		// Create the state
		double state = (i < numIters/ 2) ? 0.0 : 4.0; // cos(((double) i) / 200);

		// Create a sample from a Gaussian using Box-Muller method
		double uniform1 = ((double) rand()) / RAND_MAX, uniform2 = ((double) rand()) / RAND_MAX;
		double sample = state + sqrt(-2.0 * log(uniform1)) * cos(2 * M_PI * uniform2);

		// Set the input to the kalman filter
		kf->z[0] = sample;

		// Predict and correct
		filter_kalman_predict(kf); 
		filter_kalman_correct(kf); 

		// Write the results to the output file
		fprintf(out, "%lf\t%lf\t%lf\n", sample, kf->x[0], kf->E[0]);	
	}

	// Close the file
	fclose(out);
}
