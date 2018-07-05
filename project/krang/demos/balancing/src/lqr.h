#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>

using Eigen::Matrix;
using Eigen::Dynamic;

typedef Matrix<double, Dynamic, Dynamic> Mtx;

bool compGainMatrix(const Mtx &A, const Mtx &B, const Mtx &Q, const Mtx &R,
                    const Mtx &N, Mtx *K, double eps);