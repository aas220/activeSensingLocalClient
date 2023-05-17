#include <iostream>

#include "../include/multivariate_gaussian.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
    const int dim = 2;
    Eigen::VectorXd mean(dim);
    Eigen::MatrixXd cov(dim, dim);
    mean << 0, 0;
    cov << 1, 0, 0, 1;
    unsigned long long seed = 0;
    int numTrials = 1000000;
    int i, j;
    double rawSample[dim];
    MultivariateGaussian normalDist(mean, cov, seed);
    Map<VectorXd> sample(rawSample, dim);
    VectorXd estimatedMean(dim);
    MatrixXd estimatedCov(dim, dim);
    estimatedMean = VectorXd::Zero(dim);
    estimatedCov = MatrixXd::Zero(dim, dim);

    for (i = 0; i < numTrials ; i++) {
        normalDist.dev(rawSample, dim);
        estimatedMean += sample;
        estimatedCov += (sample - normalDist.mean_) * (sample - normalDist.mean_).transpose();
    }

    estimatedMean /= numTrials;
    cout << "estimated mean =";

    for(i = 0; i < dim; i++) {
        cout << " " << estimatedMean(i);
    }

    cout << endl;
    estimatedCov /= numTrials;
    cout << "estimated covariance =";

    for(i = 0; i < dim; i++) {
        for (j = 0; j < dim; j++) {
            cout << " " << estimatedCov(i*dim+j);
        }
    }

    cout << endl;

    return 0;
}