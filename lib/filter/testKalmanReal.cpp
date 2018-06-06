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
#include <iostream>
#include <fstream>
#include <vector>
#include "filter.h"

using namespace std;

int main (int argc, char* argv[]) {

	// Read the data from the input file
	if(argc < 2) return 0;
	ifstream in (argv[1]);
	vector < pair<double, double> > data;
	double temp1, temp2;
	while(in >> temp1 >> temp2)	data.push_back(make_pair(temp1, temp2));

	// Initialize the kalman filter indicating that the state space and the 
  // measurement space is 1.
	filter_kalman_t* kf = new filter_kalman_t;	
	filter_kalman_init(kf, 1, 0, 1);
	
	// Initialize the first state as 0
	kf->x[0] = 0.0; // data.front().first;

	// Set the process and measurement models and covariances
	kf->A[0] = 1.0;
	kf->C[0] = 1.0; 
	kf->R[0] = 1e-2;			//< Process model covariance
	kf->Q[0] = 1.0;			//< Note that the measurement covariance is set to 1.

	// Run the filter
	size_t numIters = 10000000;
	size_t stepSize = 25;
	FILE* out = fopen("out", "w");
	for(size_t i = 0; i < min(numIters, data.size()); i+=stepSize) {

		// Get the sample
		double sample = data[i].first;

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
