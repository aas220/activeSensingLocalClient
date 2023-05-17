
#ifndef MULTIVARIATE_GAUSSIAN_MULTIVARIATE_GAUSSIAN_H
#define MULTIVARIATE_GAUSSIAN_MULTIVARIATE_GAUSSIAN_H

#include "rng.h"
#include "Eigen/Dense"
#include "Eigen/Cholesky"

/**
 * \brief This struct generates random vectors sampled from a multivariable Gaussian distribution.
 *        It implements multinormaldev in Numerical Recipe, 3rd edition.
 */
struct MultivariateGaussian : Rng {

    int dim_;
    Eigen::VectorXd mean_;
    Eigen::MatrixXd cov_;
    Eigen::LLT<Eigen::MatrixXd> chol_;
    Eigen::VectorXd spt_;

    MultivariateGaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &cov, unsigned long long seed);

    void dev(double* sample, int dim);

    template<typename Derived>
    void dev(Eigen::MatrixBase<Derived> &sample)
    {
        int i;
        double u, v, x, y, q;

        for (i = 0; i < dim_; i++)
        {
            do
            {
                u = doub();
                v = 1.7156 * (doub()-0.5);
                x = u - 0.449871;
                y = std::abs(v) + 0.386595;
                q = pow(x, 2) + y * (0.19600 * y - 0.25472 * x);
            } while (q > 0.27597 && (q > 0.27846 || pow(v, 2) > -4.0*log(u)*pow(u, 2)));

            spt_(i) = v/u;
        }

        sample.resize(dim_);
        sample = chol_.matrixL() * spt_ + mean_;
    }
};

#endif //MULTIVARIATE_GAUSSIAN_MULTIVARIATE_GAUSSIAN_H
