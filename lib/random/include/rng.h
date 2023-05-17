//
// Created by tipakorng on 5/10/16.
//

#ifndef MULTIVARIATE_GAUSSIAN_RNG_H
#define MULTIVARIATE_GAUSSIAN_RNG_H
//#include"simulator.h"
int i0flagcheck=0;
int i1flagcheck=0;
int globalcheckflag=0;
int nextflag=0;
/**
 * \brief This struct implements Ranq1 in Numerical Recipe, 3rd edition.
 */
struct Rng {

    unsigned long long v;

    Rng(unsigned long long j);

    unsigned long long int64();

    double doub();

    unsigned int int32();

    unsigned long long v64;
};

#endif //MULTIVARIATE_GAUSSIAN_RNG_H
