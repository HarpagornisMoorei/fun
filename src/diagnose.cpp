// mech — diagnose.cpp: separation angles of the constraint Jacobian, and what they say about a pose (M22)
//
// The separation of row i is measured by a Cholesky factorization of the Gram matrix G = S S^T of the (column-scaled) rows S:
// its i-th pivot is the squared length of the part of row i that the rows before it do not span, so
//
//     pivot_i / G_ii = sin^2(angle between row i and the span of rows 0..i-1)
//
// A row whose ratio falls below kDependentRatio is a combination of the earlier rows; it is taken out of the factorization (its
// column of L is zeroed, which is the same as deleting the row), so the rows after it are measured against the independent ones
// only. That makes the factorization rank-revealing, and the count of rows taken out is the rank deficiency.
//
// The Gram matrix of a mechanism is banded like the matrices the solvers factor (two rows meet only if they share a body), so
// it is kept in the same Skyline form, in the same fill-reducing order, and the cost grows the way a solve does.

#include "diagnose.hpp"
#include "linalg.hpp"

#include <algorithm>
#include <cmath>

namespace mech {

namespace {

// Everything jacobianHealth needs, kept between calls: a monitor calls it at every step of a run, and a run is thousands of
// steps over rows that keep one sparsity pattern, so the buffers are reused and the fill-reducing order is worked out again only
// when the pattern changes (the solvers do the same).
struct HealthWorkspace {
    std::vector<int> colStart, colRow;          // the entries of the Jacobian by column: rows ...
    std::vector<double> colVal;                 // ... and values (weighted by the unknowns' units)
    std::vector<int> pos, first, fill;
    std::vector<double> acc, diag, ratio;
    std::vector<char> mark, dep;
    std::vector<int> touched;
    Skyline L;
    unsigned long long hash = 0;
    bool have = false;
    std::vector<int> perm;                      // perm[new] = old for the pattern with `hash`; empty: keep the order
};

} // namespace

JacobianHealth jacobianHealth(const std::vector<SparseRow>& J, int nq, const std::vector<double>* weight) {
    thread_local HealthWorkspace w;
    JacobianHealth h;
    const int m = (int)J.size();
    h.rows = m;
    if (m == 0) return h;

    // The entries by column (compressed), with the values as they are.
    w.colStart.assign((size_t)nq + 1, 0);
    for (const SparseRow& r : J)
        for (int c : r.col) ++w.colStart[(size_t)c + 1];
    for (int j = 0; j < nq; ++j) w.colStart[(size_t)j + 1] += w.colStart[(size_t)j];
    const size_t nnz = (size_t)w.colStart[(size_t)nq];
    w.colRow.resize(nnz);
    w.colVal.resize(nnz);
    w.fill.assign(w.colStart.begin(), w.colStart.begin() + nq);
    for (int i = 0; i < m; ++i)
        for (size_t x = 0; x < J[(size_t)i].col.size(); ++x) {
            const int c = J[(size_t)i].col[x];
            const size_t at = (size_t)w.fill[(size_t)c]++;
            w.colRow[at] = i;
            w.colVal[at] = J[(size_t)i].val[x] * (weight ? (*weight)[(size_t)c] : 1.0);
        }
    for (int j = 0; j < nq; ++j) if (w.colStart[(size_t)j + 1] > w.colStart[(size_t)j]) ++h.unknowns;

    // Two rows are neighbours when they share a column. Number them in the order they came, or by reverse Cuthill-McKee when
    // that makes the envelope much smaller (a mechanism declared out of order); the rank does not depend on the order.
    const unsigned long long hash = patternHash(J);
    if (!w.have || w.hash != hash) {
        std::vector<std::vector<int>> adj((size_t)m);
        for (int j = 0; j < nq; ++j)
            for (int a = w.colStart[(size_t)j]; a < w.colStart[(size_t)j + 1]; ++a)
                for (int b = w.colStart[(size_t)j]; b < w.colStart[(size_t)j + 1]; ++b)
                    if (a != b) adj[(size_t)w.colRow[(size_t)a]].push_back(w.colRow[(size_t)b]);
        w.perm = envelopeOrdering(adj);
        w.hash = hash;
        w.have = true;
    }
    const std::vector<int>& perm = w.perm;
    w.pos.resize((size_t)m);
    for (int n = 0; n < m; ++n) w.pos[(size_t)(perm.empty() ? n : perm[(size_t)n])] = n;

    w.first.resize((size_t)m);
    for (int i = 0; i < m; ++i) w.first[(size_t)w.pos[(size_t)i]] = w.pos[(size_t)i];
    for (int j = 0; j < nq; ++j)
        for (int a = w.colStart[(size_t)j]; a < w.colStart[(size_t)j + 1]; ++a) {
            int& f = w.first[(size_t)w.pos[(size_t)w.colRow[(size_t)a]]];
            for (int b = w.colStart[(size_t)j]; b < w.colStart[(size_t)j + 1]; ++b)
                f = std::min(f, w.pos[(size_t)w.colRow[(size_t)b]]);
        }
    Skyline& L = w.L;
    L.init(w.first);

    // lower triangle of G = S S^T in the new numbering, a row at a time
    w.acc.assign((size_t)m, 0.0);
    w.mark.assign((size_t)m, 0);
    for (int i = 0; i < m; ++i) {
        w.touched.clear();
        for (size_t x = 0; x < J[(size_t)i].col.size(); ++x) {
            const int c = J[(size_t)i].col[x];
            const double v = J[(size_t)i].val[x] * (weight ? (*weight)[(size_t)c] : 1.0);
            for (int a = w.colStart[(size_t)c]; a < w.colStart[(size_t)c + 1]; ++a) {
                const int k = w.colRow[(size_t)a];
                if (w.pos[(size_t)k] > w.pos[(size_t)i]) continue;
                if (!w.mark[(size_t)k]) { w.mark[(size_t)k] = 1; w.touched.push_back(k); }
                w.acc[(size_t)k] += v * w.colVal[(size_t)a];
            }
        }
        for (int k : w.touched) {
            L.at(w.pos[(size_t)i], w.pos[(size_t)k]) = w.acc[(size_t)k];
            w.acc[(size_t)k] = 0.0;
            w.mark[(size_t)k] = 0;
        }
    }

    // Rank-revealing Cholesky, in place.
    w.diag.resize((size_t)m);
    for (int i = 0; i < m; ++i) w.diag[(size_t)i] = L.at(i, i);
    w.dep.assign((size_t)m, 0);
    w.ratio.assign((size_t)m, 1.0);
    for (int i = 0; i < m; ++i) {
        for (int j = L.first[(size_t)i]; j < i; ++j) {
            if (w.dep[(size_t)j]) { L.at(i, j) = 0.0; continue; }
            double s = L.at(i, j);
            const int k0 = std::max(L.first[(size_t)i], L.first[(size_t)j]);
            for (int k = k0; k < j; ++k) s -= L.at(i, k) * L.at(j, k);
            L.at(i, j) = s / L.at(j, j);
        }
        double d = w.diag[(size_t)i];
        for (int k = L.first[(size_t)i]; k < i; ++k) d -= L.at(i, k) * L.at(i, k);
        const double r = w.diag[(size_t)i] > 1e-300 ? d / w.diag[(size_t)i] : 0.0;
        if (!(r > kDependentRatio)) {
            w.dep[(size_t)i] = 1;
            w.ratio[(size_t)i] = 0.0;
            L.at(i, i) = 1.0;                 // never divided by: column i of L is zero below the diagonal
        } else {
            w.ratio[(size_t)i] = r;
            L.at(i, i) = std::sqrt(d);
        }
    }

    h.ratios.assign((size_t)m, 0.0);
    for (int n = 0; n < m; ++n) {
        const int old = perm.empty() ? n : perm[(size_t)n];
        h.ratios[(size_t)old] = w.dep[(size_t)n] ? 0.0 : w.ratio[(size_t)n];
        if (w.dep[(size_t)n]) {
            h.dependent.push_back(old);
        } else {
            ++h.rank;
            if (w.ratio[(size_t)n] < h.minRatio) { h.minRatio = w.ratio[(size_t)n]; h.minRow = old; }
        }
    }
    std::sort(h.dependent.begin(), h.dependent.end());
    return h;
}

std::vector<double> unknownWeights(const Model& m, const std::vector<SparseRow>* J) {
    const size_t nb = m.bodies.size();
    std::vector<double> reach(nb, 0.0);
    // What the body is drawn with: its anchors, and the gears and profiles it carries (a cam shaft has no anchor away from its axle: it
    // reaches as far as its profile does)
    for (size_t b = 1; b < nb; ++b)
        for (const Anchor& a : m.bodies[b].anchors) reach[b] = std::max(reach[b], std::hypot(a.local.x, a.local.y));
    auto hostDistance = [&](int body, int anchor) {
        const Vec2& p = m.bodies[(size_t)body].anchors[(size_t)anchor].local;
        return std::hypot(p.x, p.y);
    };
    for (const Gear& g : m.gears)
        if (g.body > 0) reach[(size_t)g.body] = std::max(reach[(size_t)g.body], hostDistance(g.body, g.hostAnchor) + g.radius);
    for (const Profile& p : m.profiles)
        if (p.body > 0 && p.curve) reach[(size_t)p.body] = std::max(reach[(size_t)p.body], hostDistance(p.body, p.hostAnchor) + p.curve->maxRadius());
    if (J) {
        // What a row reaches: its entry on a body's angle against its entries on that body's position. Both are in the row's own
        // units, so the ratio is a length whatever those are. It can only raise a reach, never lower one: a contact that happens to be
        // radial exerts no torque, and the ~1e-9 that is left would otherwise be taken for the reach of the body.
        std::vector<int> bodyOfCol((size_t)m.nq(), 0);
        for (size_t b = 1; b < nb; ++b)
            if (m.bodies[b].qOffset >= 0)
                for (int k = 0; k < 3; ++k) bodyOfCol[(size_t)m.bodies[b].qOffset + (size_t)k] = (int)b;
        for (const SparseRow& r : *J) {
            for (size_t x = 0; x < r.col.size(); ++x) {
                const int b = bodyOfCol[(size_t)r.col[x]];
                if (b == 0 || r.col[x] != m.bodies[(size_t)b].qOffset) continue;          // an angle entry
                double px = 0.0, py = 0.0;
                for (size_t y = 0; y < r.col.size(); ++y) {
                    if (r.col[y] == r.col[x] + 1) px = r.val[y];
                    if (r.col[y] == r.col[x] + 2) py = r.val[y];
                }
                const double pos = std::hypot(px, py);
                if (pos > 0.0) reach[(size_t)b] = std::max(reach[(size_t)b], std::fabs(r.val[x]) / pos);
            }
        }
    }
    double ref = 0.0;
    for (size_t b = 1; b < nb; ++b) ref = std::max(ref, reach[b]);
    if (!(ref > 0.0)) ref = 1.0;
    std::vector<double> w((size_t)m.nq(), 1.0);
    for (size_t b = 1; b < nb; ++b) {
        if (m.bodies[b].qOffset < 0) continue;
        const double l = reach[b] > 0.0 ? reach[b] : ref;
        w[(size_t)m.bodies[b].qOffset] = 1.0 / l;          // the angle column; the two position columns stay 1
    }
    return w;
}

// ---- naming ---------------------------------------------------------------------------------------------------------

std::vector<std::string> constraintLabels(const Model& m) {
    std::vector<std::string> L;
    auto anchorName = [&](int b, int a) { return m.bodies[(size_t)b].name + "." + m.bodies[(size_t)b].anchors[(size_t)a].name; };
    // the order is that of ConstraintSystem::residuals
    for (const PinJoint& p : m.pins) {
        const std::string s = "pin " + anchorName(p.bodyA, p.anchA) + " -> " + anchorName(p.bodyB, p.anchB);
        L.push_back(s + " (x)");
        L.push_back(s + " (y)");
    }
    for (const SlideJoint& s : m.slides) {
        const std::string g = anchorName(s.guideBody, s.guideAnchor);
        L.push_back(std::string(s.lockDir ? "slide " : "slot ") + g + " (on the rail)");
        if (s.lockDir) L.push_back("slide " + g + " (parallel to the rail)");
    }
    for (const Drive& d : m.drives) {
        if (!d.enabled) continue;
        L.push_back("drive " + m.bodies[(size_t)d.body].name);
    }
    for (const MeshJoint& mm : m.meshes) {
        const std::string s = "mesh " + m.gears[(size_t)mm.gearA].name + " " + m.gears[(size_t)mm.gearB].name;
        L.push_back(s + " (rolling)");
        if (mm.tangent) L.push_back(s + " (tangent)");
    }
    for (const PolarMesh& pm : m.polarMeshes)
        L.push_back("mesh " + m.profiles[(size_t)pm.profA].name + " " + m.profiles[(size_t)pm.profB].name + " (non-circular)");
    for (const TouchJoint& tj : m.touches) {
        if (!tj.engaged) continue;
        L.push_back("touch " + anchorName(tj.body, tj.anchor) + " on " + m.profiles[(size_t)tj.profile].name);
    }
    for (const RackMesh& rm : m.racks) {
        const std::string s = "mesh " + m.gears[(size_t)rm.gear].name + " on a rack";
        if (rm.tangent) L.push_back(s + " (tangent)");
        L.push_back(s + " (rolling)");
    }
    for (const Belt& b : m.belts)
        for (size_t i = 0; i + 1 < b.gears.size(); ++i) L.push_back("belt " + b.name + " (link " + std::to_string(i + 1) + ")");
    for (const LinearDrive& ld : m.linDrives) {
        if (ld.isForce) continue;
        const SlideJoint& s = m.slides[(size_t)ld.slide];
        L.push_back("drive " + anchorName(s.guideBody, s.guideAnchor));
    }
    return L;
}

std::vector<std::pair<int, int>> coincidentBodies(const Model& m, const std::vector<double>& q, double tol) {
    const int nb = (int)m.bodies.size();
    // the size of the mechanism, to say what "the same place" means: how far its bodies and anchors reach
    double size = 0.0;
    for (int b = 1; b < nb; ++b) {
        const Body& bd = m.bodies[(size_t)b];
        if (bd.qOffset < 0) continue;
        size = std::max(size, std::hypot(q[(size_t)bd.qOffset + 1], q[(size_t)bd.qOffset + 2]));
        for (const Anchor& a : bd.anchors) size = std::max(size, std::hypot(a.local.x, a.local.y));
    }
    if (!(size > 0.0)) size = 1.0;
    const double twoPi = 6.283185307179586;
    std::vector<std::pair<int, int>> out;
    for (int i = 1; i < nb; ++i) {
        const Body& bi = m.bodies[(size_t)i];
        if (bi.qOffset < 0) continue;
        for (int j = i + 1; j < nb; ++j) {
            const Body& bj = m.bodies[(size_t)j];
            if (bj.qOffset < 0) continue;
            if (std::hypot(q[(size_t)bi.qOffset + 1] - q[(size_t)bj.qOffset + 1],
                           q[(size_t)bi.qOffset + 2] - q[(size_t)bj.qOffset + 2]) > tol * size) continue;
            double da = std::fmod(std::fabs(q[(size_t)bi.qOffset] - q[(size_t)bj.qOffset]), twoPi);
            da = std::min(da, twoPi - da);
            if (da <= tol) out.emplace_back(i, j);
        }
    }
    return out;
}

// ---- the assembled pose ---------------------------------------------------------------------------------------------

PoseDiagnosis diagnosePose(const Model& m, const ConstraintSystem& sys, const std::vector<double>& q) {
    PoseDiagnosis d;
    std::vector<SparseRow> J;
    sys.jacobianSparse(q, J);
    const std::vector<double> weight = unknownWeights(m, &J);
    d.health = jacobianHealth(J, sys.nq, &weight);
    const int dep = (int)d.health.dependent.size();
    d.explained = std::min(dep, std::max(0, d.health.rows - d.health.unknowns));
    d.unexplained = dep - d.explained;
    if (dep > 0) d.coincident = coincidentBodies(m, q);
    return d;
}

std::vector<Finding> describePose(const Model& m, const PoseDiagnosis& d) {
    std::vector<Finding> out;
    const JacobianHealth& h = d.health;
    if (h.rows == 0) return out;
    char buf[256];
    const int dep = (int)h.dependent.size();
    if (dep == 0) {
        std::snprintf(buf, sizeof buf, "pose regular: the %d constraints are independent (closest to dependent: %.1f deg)",
                      h.rows, h.minDegrees());
        out.push_back({Severity::Ok, buf, {}});
        return out;
    }
    const std::vector<std::string> labels = constraintLabels(m);
    std::string head;
    std::vector<std::string> detail;
    std::snprintf(buf, sizeof buf, "singular pose: %d of %d constraints are dependent here (rank %d)", dep, h.rows, h.rank);
    head = buf;
    if (!d.coincident.empty()) {
        std::string pairs;
        for (size_t k = 0; k < d.coincident.size() && k < 4; ++k) {
            if (k) pairs += ", ";
            pairs += m.bodies[(size_t)d.coincident[k].first].name + " and " + m.bodies[(size_t)d.coincident[k].second].name;
        }
        if (d.coincident.size() > 4) pairs += ", ...";
        detail.push_back(pairs + (d.coincident.size() == 1 ? " lie" : " lie, pair by pair,") +
                         " in exactly the same place: a folded branch, with two copies of one chain on top of each other");
        detail.push_back("the solver found it because the bodies started alike; start the bodies of one chain near where they belong with "
                         "`from: <angle> deg` in their braces (a rough angle is enough)");
        out.push_back({Severity::Warn, head, detail});
        return out;
    }
    std::string which;
    for (size_t k = 0; k < h.dependent.size() && k < 3; ++k) {
        const size_t r = (size_t)h.dependent[k];
        if (k) which += "; ";
        which += r < labels.size() ? labels[r] : "row " + std::to_string(r);
    }
    if (h.dependent.size() > 3) which += "; ...";
    if (d.unexplained == 0) {
        std::snprintf(buf, sizeof buf, "%d dependent constraint%s, as the count of rows against unknowns says (over-constrained)",
                      dep, dep == 1 ? "" : "s");
        out.push_back({Severity::Note, buf, {}});
        return out;
    }
    detail.push_back("dependent: " + which);
    detail.push_back("this pose is a dead point or a branch point, or the design is over-constrained on purpose (parallel cranks, say); "
                     "if it is a dead point, start a body elsewhere with `from:`");
    out.push_back({Severity::Note, head, detail});
    return out;
}

// ---- along a run ------------------------------------------------------------------------------------------------------

void RunMonitor::observe(const std::vector<double>& q, double t, double residual) {
    std::vector<SparseRow> J;
    sys_.jacobianSparse(q, J);
    const std::vector<double> weight = unknownWeights(sys_.m, &J);
    const JacobianHealth h = jacobianHealth(J, sys_.nq, &weight);
    ++samples_;
    const int dep = (int)h.dependent.size();
    if (!haveBase_ || h.rows != baseRows_) {          // what is normal: taken at the start, and again when the rows change
        haveBase_ = true;
        baseRows_ = h.rows;
        baseDependent_ = dep;
    }
    const bool lostRank = dep > baseDependent_;
    const bool near = h.rows > 0 && h.minRatio < kNearRatio;
    const double ratio = lostRank ? 0.0 : h.minRatio;
    if (ratio < closest_) { closest_ = ratio; closestT_ = t; }
    if (!(lostRank || near)) {
        finish();
        return;
    }
    if (!open_) {
        open_ = true;
        lostTold_ = false;
        cur_ = Excursion();
        cur_.t0 = t;
        cur_.worstT = t;
        cur_.worstRatio = 2.0;                        // anything beats this
    }
    cur_.t1 = t;
    ++cur_.samples;
    cur_.maxResidual = std::max(cur_.maxResidual, residual);
    if (ratio < cur_.worstRatio || (lostRank && dep - baseDependent_ > cur_.dependent)) {
        cur_.worstRatio = ratio;
        cur_.worstT = t;
        cur_.dependent = std::max(cur_.dependent, dep - baseDependent_);
        cur_.rows = h.rows;
        cur_.rank = h.rank;
        const std::vector<std::string> labels = constraintLabels(sys_.m);   // named now: the rows can change later
        const int row = lostRank ? h.dependent.back() : h.minRow;
        cur_.involving = row >= 0 && (size_t)row < labels.size() ? labels[(size_t)row] : std::string();
    }
}

void RunMonitor::finish() {
    if (!open_) return;
    open_ = false;
    done_.push_back(cur_);
}

std::string RunMonitor::describe(const Excursion& e) {
    char buf[512];
    std::string span;
    if (e.t1 > e.t0) { std::snprintf(buf, sizeof buf, "t = %.4g .. %.4g s", e.t0, e.t1); span = buf; }
    else { std::snprintf(buf, sizeof buf, "t = %.4g s", e.t0); span = buf; }
    if (e.dependent > 0) {
        std::snprintf(buf, sizeof buf, "singular configuration at %s: %d more constraint%s became dependent on the others (rank %d of %d)",
                      span.c_str(), e.dependent, e.dependent == 1 ? "" : "s", e.rank, e.rows);
    } else {
        std::snprintf(buf, sizeof buf, "singular configuration at %s: the constraints came within %.2g deg of dependent (closest at t = %.4g s)",
                      span.c_str(), e.worstDegrees(), e.worstT);
    }
    std::string line = buf;
    if (!e.involving.empty()) line += "\n  involving, for instance: " + e.involving;
    if (e.maxResidual > kLostResidual) {
        std::snprintf(buf, sizeof buf,
                      "\n  the constraints could no longer be met there (largest residual %.2g): the mechanism has gone past the end "
                      "of its range, and what the run shows after it is not a pose of the mechanism", e.maxResidual);
        line += buf;
    } else {
        line += "\n  the solver may continue on a different branch after this point; if the motion after it looks wrong, keep the drive "
                "away from this pose";
    }
    return line;
}

std::vector<std::string> RunMonitor::report(int maxShown) const {
    std::vector<std::string> out;
    std::vector<Excursion> all = done_;
    if (open_) all.push_back(cur_);
    for (size_t k = 0; k < all.size() && (int)k < maxShown; ++k) out.push_back(describe(all[k]));
    if (all.size() > (size_t)maxShown) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "... and %zu more", all.size() - (size_t)maxShown);
        out.push_back(buf);
    }
    return out;
}

std::vector<std::string> RunMonitor::startedSince() {
    std::vector<std::string> out;
    const size_t total = done_.size() + (open_ ? 1 : 0);
    for (size_t k = announced_; k < total; ++k) {
        const bool isOpen = k >= done_.size();
        const Excursion& e = isOpen ? cur_ : done_[k];
        out.push_back(describe(e));
        if (isOpen) lostTold_ = e.maxResidual > kLostResidual;
    }
    announced_ = total;
    if (open_ && !lostTold_ && cur_.maxResidual > kLostResidual) {
        lostTold_ = true;
        char buf[320];
        std::snprintf(buf, sizeof buf,
                      "t = %.4g s: the constraints could no longer be met (largest residual %.2g): the mechanism has gone past the end "
                      "of its range, and what is shown after it is not a pose of the mechanism", cur_.t1, cur_.maxResidual);
        out.push_back(buf);
    }
    return out;
}

void RunMonitor::reset() {
    announced_ = 0;
    lostTold_ = false;
    samples_ = 0;
    haveBase_ = false;
    baseRows_ = baseDependent_ = 0;
    closest_ = 1.0;
    closestT_ = 0.0;
    open_ = false;
    cur_ = Excursion();
    done_.clear();
}

} // namespace mech
