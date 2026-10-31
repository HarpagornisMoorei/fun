// mech — linalg.hpp: tiny dense symmetric solves shared by solver + dynamics
#pragma once
#include <cmath>
#include <vector>

namespace mech {

// Cholesky solve of A x = b (A symmetric positive definite thanks to damping). Returns false if a pivot
// collapses.
//
// The factor has the same envelope as A: row i of L is zero left of the first nonzero of row i of A. A chain
// of joints gives a narrow envelope (each constraint touches a few neighbours), so the factorization visits
// only the terms inside it; a dense matrix has a full envelope and gets the plain algorithm. The terms left
// out are exact zeros, summed in the same order otherwise, so the result is bit for bit the dense one.
inline bool choleskySolve(const std::vector<double>& A, const std::vector<double>& b,
                          int n, std::vector<double>& x) {
    std::vector<double> L(A);
    std::vector<int> first((size_t)n);
    for (int i = 0; i < n; ++i) {
        int f = 0;
        while (f < i && A[(size_t)i * n + f] == 0.0) ++f;
        first[(size_t)i] = f;
    }
    for (int i = 0; i < n; ++i) {
        for (int j = first[(size_t)i]; j <= i; ++j) {
            double s = L[(size_t)i * n + j];
            const int k0 = first[(size_t)i] > first[(size_t)j] ? first[(size_t)i] : first[(size_t)j];
            for (int k = k0; k < j; ++k) s -= L[(size_t)i * n + k] * L[(size_t)j * n + k];
            if (i == j) {
                if (s <= 1e-14) return false;
                L[(size_t)i * n + i] = std::sqrt(s);
            } else {
                L[(size_t)i * n + j] = s / L[(size_t)j * n + j];
            }
        }
    }
    std::vector<double> y(n);
    for (int i = 0; i < n; ++i) {
        double s = b[i];
        for (int k = first[(size_t)i]; k < i; ++k) s -= L[(size_t)i * n + k] * y[k];
        y[i] = s / L[(size_t)i * n + i];
    }
    x.assign(n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        double s = y[i];
        for (int k = i + 1; k < n; ++k)
            if (first[(size_t)k] <= i) s -= L[(size_t)k * n + i] * x[k];
        x[i] = s / L[(size_t)i * n + i];
    }
    return true;
}

} // namespace mech
