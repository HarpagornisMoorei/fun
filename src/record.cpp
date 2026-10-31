// mech — record.cpp: time-series export of a simulation (M7)
#include "record.hpp"

#include <cmath>
#include <cstdio>

namespace mech {

namespace {

std::string touchLabel(const Model& m, const TouchJoint& tj) {
    const Body& b = m.bodies[(size_t)tj.body];
    return b.name + "." + b.anchors[(size_t)tj.anchor].name + "@" +
           m.profiles[(size_t)tj.profile].name;
}

} // namespace

Recorder::Recorder(const Model& m, const DynamicsSystem* dyn, std::vector<Probe> probes)
    : m_(m), dyn_(dyn), probes_(std::move(probes)) {
    cols_.push_back("t[s]");
    for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
        const std::string& n = m.bodies[bi].name;
        cols_.push_back(n + ".theta[rad]");
        cols_.push_back(n + ".x[m]");
        cols_.push_back(n + ".y[m]");
    }
    for (const Probe& p : probes_) {
        const std::string n = m.bodies[(size_t)p.body].name + "." +
                              m.bodies[(size_t)p.body].anchors[(size_t)p.anchor].name;
        cols_.push_back(n + ".x[m]");
        cols_.push_back(n + ".y[m]");
    }
    cols_.push_back("maxRes");
    for (const TouchJoint& tj : m.touches) {
        const std::string n = touchLabel(m, tj);
        cols_.push_back(n + ".gap[m]");
        cols_.push_back(n + ".foot.x[m]");
        cols_.push_back(n + ".foot.y[m]");
        cols_.push_back(n + ".normal[rad]");
    }
    if (dyn_) {
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const std::string& n = m.bodies[bi].name;
            cols_.push_back(n + ".omega[rad/s]");
            cols_.push_back(n + ".vx[m/s]");
            cols_.push_back(n + ".vy[m/s]");
        }
        for (const TouchJoint& tj : m.touches) {
            const std::string n = touchLabel(m, tj);
            cols_.push_back(n + ".N[N]");
            cols_.push_back(n + ".engaged");
        }
        for (const Drive& d : m.drives)
            if (d.enabled) cols_.push_back("drive." + m.bodies[(size_t)d.body].name + "[N*m]");
        cols_.push_back("KE[J]");
        cols_.push_back("PE[J]");
        cols_.push_back("E[J]");
        cols_.push_back("ledger.weight[J]");
        cols_.push_back("ledger.impulse[J]");
        cols_.push_back("ledger.drive[J]");
        cols_.push_back("ledger.impact[J]");
        cols_.push_back("ledger.friction[J]");
        cols_.push_back("ledger.residual[J]");
    }
}

std::vector<double> Recorder::sample(double t, const std::vector<double>& q,
                                     const std::vector<double>& qd, double maxRes) const {
    std::vector<double> r;
    r.reserve(cols_.size());
    r.push_back(t);
    for (size_t bi = 1; bi < m_.bodies.size(); ++bi) {
        const int o = m_.bodies[bi].qOffset;
        r.push_back(q[(size_t)o]);
        r.push_back(q[(size_t)o + 1]);
        r.push_back(q[(size_t)o + 2]);
    }
    for (const Probe& p : probes_) {
        const Vec2 w = m_.worldAnchor(p.body, p.anchor, q);
        r.push_back(w.x);
        r.push_back(w.y);
    }
    r.push_back(maxRes);
    for (const TouchJoint& tj : m_.touches) {
        const ContactState cs = m_.touchState(tj, q);
        r.push_back(cs.gap - tj.roller);
        r.push_back(cs.foot.x);
        r.push_back(cs.foot.y);
        r.push_back(std::atan2(cs.normal.y, cs.normal.x));
    }
    if (dyn_) {
        for (size_t bi = 1; bi < m_.bodies.size(); ++bi) {
            const int o = m_.bodies[bi].qOffset;
            r.push_back(qd[(size_t)o]);
            r.push_back(qd[(size_t)o + 1]);
            r.push_back(qd[(size_t)o + 2]);
        }
        // forces and reactions come from one acceleration solve at this state
        std::vector<double> qdd;
        dyn_->accelerations(q, qd, qdd);
        for (size_t ti = 0; ti < m_.touches.size(); ++ti) {
            r.push_back(m_.touches[ti].engaged ? dyn_->contactForces()[ti] : 0.0);
            r.push_back(m_.touches[ti].engaged ? 1.0 : 0.0);
        }
        for (size_t di = 0; di < m_.drives.size(); ++di)
            if (m_.drives[di].enabled) r.push_back(dyn_->driveReactions()[di]);
        const double KE = dyn_->kinetic(q, qd), PE = dyn_->potential(q);
        const HybridStats& hs = dyn_->hybridStats();
        r.push_back(KE);
        r.push_back(PE);
        r.push_back(KE + PE);
        r.push_back(hs.weightDrawn);
        r.push_back(hs.impulseEnergy);
        r.push_back(hs.driveWork);
        r.push_back(hs.impactLoss);
        r.push_back(hs.frictionLoss);
        r.push_back((KE + PE - E0_) - (hs.weightDrawn + hs.impulseEnergy + hs.driveWork -
                                       hs.impactLoss - hs.frictionLoss));
    }
    return r;
}

std::string Recorder::headerLine() const {
    std::string s;
    for (size_t i = 0; i < cols_.size(); ++i) {
        if (i) s += ',';
        s += cols_[i];
    }
    return s;
}

std::string Recorder::csvLine(const std::vector<double>& row) {
    std::string s;
    char buf[40];
    for (size_t i = 0; i < row.size(); ++i) {
        if (i) s += ',';
        std::snprintf(buf, sizeof buf, "%.12g", row[i]);
        s += buf;
    }
    return s;
}

} // namespace mech
