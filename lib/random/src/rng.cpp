#include "../include/rng.h"


Rng::Rng(unsigned long long j) : v(4101842887655122017LL) {
    v ^= j;
    v = int64();
}

unsigned long long Rng::int64() {
    v ^= v >> 21;
    v ^= v << 35;
    v ^= v >> 4;
    return v * 2685821657736338717LL;
}

double Rng::doub() {
    return 5.42101086242752217E-20 * int64();
}

unsigned int Rng::int32() {
    return static_cast<unsigned int>(int64());
}

