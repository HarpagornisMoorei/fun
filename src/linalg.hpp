// mech — linalg.hpp: tiny dense symmetric solves shared by solver + dynamics
#pragma once
#include <cmath>
#include <vector>

namespace mech {

// In-place Cholesky (A is symmetric positive definite thanks to damping).
// Solves A x = b; returns false if a pivot collapses.
inline bool choleskySolve(const std::vector<double>& A, const std::vector<double>& b,
                          int n, std::vector<double>& x) {
    std::vector<double> L(A);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            double s = L[(size_t)i * n + j];
            for (int k = 0; k < j; ++k) s -= L[(size_t)i * n + k] * L[(size_t)j * n + k];
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
        for (int k = 0; k < i; ++k) s -= L[(size_t)i * n + k] * y[k];
        y[i] = s / L[(size_t)i * n + i];
    }
    x.assign(n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        double s = y[i];
        for (int k = i + 1; k < n; ++k) s -= L[(size_t)k * n + i] * x[k];
        x[i] = s / L[(size_t)i * n + i];
    }
    return true;
}

} // namespace mech
