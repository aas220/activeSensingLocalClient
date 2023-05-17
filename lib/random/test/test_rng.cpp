#include <iostream>

#include "../include/rng.h"

using namespace std;

int main(int argc, char** argv) {
    unsigned long long seed = 1;
    int numTrials = 1000000;
    double min = 0.5;
    double max = 0.5;
    double mean = 0;
    double sample = 0;
    Rng rng(seed);

    for (int i = 0; i < numTrials ; i++) {
        sample = rng.doub();
        mean += sample;

        if (sample < min) min = sample;
        if (sample > max) max = sample;
    }

    mean /= numTrials;
    cout << "min = " << min << endl;
    cout << "max = " << max << endl;
    cout << "mean = " << mean << endl;

    return 0;
}
