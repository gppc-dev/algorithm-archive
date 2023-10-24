#ifndef _FP_UTIL_H_
#define _FP_UTIL_H_

#include <cmath>

template<typename T>
T max(T a, T b) {
    return a < b ? b : a;
}

template<typename T>
T min(T a, T b) {
    return a > b ? b : a;
}

#endif // _FP_UTIL_H_