// mech — linalg.hpp: tiny dense symmetric solves shared by solver + dynamics
#pragma once
#include <algorithm>
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


// ---- sparse building blocks (M21) --------------------------------------------------------------------------------
//
// A constraint row of a mechanism touches the bodies it joins and nothing else, so its Jacobian row has a handful
// of nonzeros however big the mechanism is. The pieces below let the solvers work on those rows and on the banded
// matrices they make (J W J^T, J^T J) without ever forming the dense ones. Every sum is taken in the same order as the
// dense algorithm took it, with only exact zeros left out, so the numbers are the dense algorithm's.

struct SparseRow {
    std::vector<int> col;        // strictly ascending
    std::vector<double> val;     // never exactly zero
};

// A symmetric matrix stored by its lower triangle in "skyline" form: row i keeps the columns first[i] .. i. Cholesky
// factors have the same envelope as the matrix, so the factor lives in a Skyline of the same shape.
struct Skyline {
    int n = 0;
    std::vector<int> first;       // first stored column of each row (<= its index)
    std::vector<int> last;        // last row with an entry in each column: rows i+1..last[i] may hold column i
    std::vector<size_t> off;      // start of each row in `a`
    std::vector<double> a;

    // `firstIdx[i]` is the first column row i may use (clamped to <= i).
    void init(const std::vector<int>& firstIdx) {
        n = (int)firstIdx.size();
        first = firstIdx;
        off.assign((size_t)n + 1, 0);
        last.assign((size_t)n, 0);
        for (int i = 0; i < n; ++i) {
            if (first[(size_t)i] > i) first[(size_t)i] = i;
            off[(size_t)i + 1] = off[(size_t)i] + (size_t)(i - first[(size_t)i] + 1);
            last[(size_t)i] = i;
        }
        for (int i = 0; i < n; ++i)
            for (int j = first[(size_t)i]; j < i; ++j) last[(size_t)j] = i > last[(size_t)j] ? i : last[(size_t)j];
        a.assign(off[(size_t)n], 0.0);
    }
    double& at(int i, int j) { return a[off[(size_t)i] + (size_t)(j - first[(size_t)i])]; }
    double at(int i, int j) const { return a[off[(size_t)i] + (size_t)(j - first[(size_t)i])]; }
    double get(int i, int j) const { return j < first[(size_t)i] ? 0.0 : at(i, j); }
};

// A hash of the sparsity pattern of a set of rows (which columns each has), to recognise a pattern seen before: the order
// chosen for it does not have to be worked out again at every step.
inline unsigned long long patternHash(const std::vector<SparseRow>& J) {
    unsigned long long h = 1469598103934665603ULL;
    auto mix = [&](unsigned long long v) { h ^= v; h *= 1099511628211ULL; };
    mix((unsigned long long)J.size());
    for (const SparseRow& r : J) {
        mix(0xFFFFFFFFULL);
        for (int c : r.col) mix((unsigned long long)c);
    }
    return h;
}

// A fill-reducing order for a symmetric sparsity pattern given as neighbour lists: reverse Cuthill-McKee (breadth first from
// a low-degree node, neighbours by degree, reversed), which keeps the envelope of the matrix, and of its Cholesky factor, small
// whichever order the unknowns were created in. Returns perm with perm[new] = old, or an empty vector when the order the
// unknowns came in is already within 70% of it (a chain declared in order is already optimal, and is left alone).
inline std::vector<int> envelopeOrdering(std::vector<std::vector<int>>& adj) {
    const int n = (int)adj.size();
    for (auto& l : adj) {
        std::sort(l.begin(), l.end());
        l.erase(std::unique(l.begin(), l.end()), l.end());
    }
    auto envelope = [&](const std::vector<int>& pos) {            // pos[old] = new index
        long long e = 0;
        for (int i = 0; i < n; ++i) {
            int f = pos[(size_t)i];
            for (int k : adj[(size_t)i]) if (k != i) f = std::min(f, pos[(size_t)k]);
            e += pos[(size_t)i] - f + 1;
        }
        return e;
    };
    std::vector<int> natural((size_t)n);
    for (int i = 0; i < n; ++i) natural[(size_t)i] = i;
    const long long envNat = envelope(natural);
    if (n < 8) return {};
    std::vector<int> order, visited((size_t)n, 0);
    order.reserve((size_t)n);
    std::vector<int> byDegree((size_t)n);
    for (int i = 0; i < n; ++i) byDegree[(size_t)i] = i;
    std::stable_sort(byDegree.begin(), byDegree.end(), [&](int x, int y) { return adj[(size_t)x].size() < adj[(size_t)y].size(); });
    for (int seed : byDegree) {
        if (visited[(size_t)seed]) continue;
        size_t head = order.size();
        order.push_back(seed);
        visited[(size_t)seed] = 1;
        while (head < order.size()) {
            const int v = order[head++];
            std::vector<int> nb;
            for (int k : adj[(size_t)v]) if (!visited[(size_t)k]) nb.push_back(k);
            std::stable_sort(nb.begin(), nb.end(), [&](int x, int y) { return adj[(size_t)x].size() < adj[(size_t)y].size(); });
            for (int k : nb) { visited[(size_t)k] = 1; order.push_back(k); }
        }
    }
    std::reverse(order.begin(), order.end());
    std::vector<int> pos((size_t)n);
    for (int i = 0; i < n; ++i) pos[(size_t)order[(size_t)i]] = i;
    if ((double)envelope(pos) < 0.7 * (double)envNat) return order;
    return {};
}

// L L^T = A, factored into `L` (same shape as A), or in place. False if a pivot collapses (A not positive definite enough).
inline bool skylineFactorInPlace(Skyline& L) {
    const int n = L.n;
    for (int i = 0; i < n; ++i) {
        for (int j = L.first[(size_t)i]; j <= i; ++j) {
            double s = L.at(i, j);
            const int k0 = L.first[(size_t)i] > L.first[(size_t)j] ? L.first[(size_t)i] : L.first[(size_t)j];
            for (int k = k0; k < j; ++k) s -= L.at(i, k) * L.at(j, k);
            if (i == j) {
                if (s <= 1e-14) return false;
                L.at(i, i) = std::sqrt(s);
            } else {
                L.at(i, j) = s / L.at(j, j);
            }
        }
    }
    return true;
}

inline bool skylineFactor(const Skyline& A, Skyline& L) {
    L = A;
    return skylineFactorInPlace(L);
}

// Solve L L^T x = b.
inline void skylineSolve(const Skyline& L, const std::vector<double>& b, std::vector<double>& x) {
    const int n = L.n;
    std::vector<double> y((size_t)n);
    for (int i = 0; i < n; ++i) {
        double s = b[(size_t)i];
        for (int k = L.first[(size_t)i]; k < i; ++k) s -= L.at(i, k) * y[(size_t)k];
        y[(size_t)i] = s / L.at(i, i);
    }
    x.assign((size_t)n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        double s = y[(size_t)i];
        for (int k = i + 1; k <= L.last[(size_t)i]; ++k)
            if (L.first[(size_t)k] <= i) s -= L.at(k, i) * x[(size_t)k];
        x[(size_t)i] = s / L.at(i, i);
    }
}

} // namespace mech
