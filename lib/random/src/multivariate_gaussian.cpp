#include "../include/multivariate_gaussian.h"


MultivariateGaussian::MultivariateGaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &cov, unsigned long long seed) :
    Rng(seed),
    dim_(static_cast<int>(mean.size())),
    mean_(mean),
    cov_(cov),
    chol_(cov_),
    spt_(dim_)
{}

void MultivariateGaussian::dev(double* sample, int dim)
{
    Eigen::Map<Eigen::VectorXd> map(sample, dim_);
    dev(map);
}
