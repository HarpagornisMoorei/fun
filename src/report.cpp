// mech — report.cpp: what a mechanism has to carry (M20)
#include "report.hpp"
#include "sweep.hpp"

#include <algorithm>
#include <cmath>

namespace mech {

namespace {

int findCol(const std::vector<std::string>& cols, const std::string& full) {
    for (size_t i = 0; i < cols.size(); ++i) if (cols[i] == full) return (int)i;
    return -1;
}

// peak |x|, its time, RMS and signed mean of a series sampled uniformly in t
ReportRow reduce(const std::vector<std::vector<double>>& rows, const std::vector<double>& x, const char* group,
                 const std::string& name, const char* quantity, const char* unit) {
    ReportRow r;
    r.group = group; r.name = name; r.quantity = quantity; r.unit = unit;
    double s = 0.0, s2 = 0.0;
    for (size_t i = 0; i < x.size(); ++i) {
        if (std::fabs(x[i]) > r.peak) { r.peak = std::fabs(x[i]); r.peakAt = rows[i][0]; }
        s += x[i];
        s2 += x[i] * x[i];
    }
    if (!x.empty()) { r.mean = s / (double)x.size(); r.rms = std::sqrt(s2 / (double)x.size()); }
    return r;
}

std::vector<double> column(const std::vector<std::vector<double>>& rows, int c) {
    std::vector<double> v(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) v[i] = rows[i][(size_t)c];
    return v;
}

} // namespace

Report buildReport(const Model& m, double seconds) {
    Report rep;
    rep.seconds = seconds;
    if (!m.world.dynamic) {
        rep.ok = false;
        rep.error = "a report needs a dynamic mechanism (a `world` block, with masses): a kinematic one has nothing to carry";
        return rep;
    }
    const SimSeries s = simulate(m, seconds, 1.0, {}, 1, true);
    if (!s.ok) { rep.ok = false; rep.error = s.error; return rep; }
    const auto& cols = s.columns;
    const auto& rows = s.rows;

    // rate drives: torque, speed, power
    for (const Drive& d : m.drives) {
        if (!d.enabled) continue;
        const std::string bn = m.bodies[(size_t)d.body].name;
        const int ct = findCol(cols, "drive." + bn + "[N*m]"), cw = findCol(cols, bn + ".omega[rad/s]");
        if (ct < 0 || cw < 0) continue;
        const std::vector<double> tq = column(rows, ct), w = column(rows, cw);
        std::vector<double> p(tq.size());
        for (size_t i = 0; i < p.size(); ++i) p[i] = tq[i] * w[i];
        rep.rows.push_back(reduce(rows, tq, "drive", bn, "torque", "N*m"));
        rep.rows.push_back(reduce(rows, w, "drive", bn, "speed", "rad/s"));
        rep.rows.push_back(reduce(rows, p, "drive", bn, "power", "W"));
    }
    // linear drives: force, and power from force times the guide's speed along its rail
    for (const LinearDrive& ld : m.linDrives) {
        const SlideJoint& sj = m.slides[(size_t)ld.slide];
        const std::string lab = m.bodies[(size_t)sj.guideBody].name + "." + m.bodies[(size_t)sj.guideBody].anchors[(size_t)sj.guideAnchor].name;
        const int cf = findCol(cols, lab + ".force[N]"), cs = findCol(cols, lab + ".travel[m]");
        if (cf < 0 || cs < 0 || rows.size() < 3) continue;
        const std::vector<double> f = column(rows, cf), tr = column(rows, cs);
        std::vector<double> v(f.size(), 0.0), p(f.size(), 0.0);
        for (size_t i = 0; i < f.size(); ++i) {
            const size_t a = i == 0 ? 0 : i - 1, b = i + 1 < f.size() ? i + 1 : i;
            v[i] = (tr[b] - tr[a]) / (rows[b][0] - rows[a][0]);
            p[i] = f[i] * v[i];
        }
        rep.rows.push_back(reduce(rows, f, "drive", lab, "force", "N"));
        rep.rows.push_back(reduce(rows, v, "drive", lab, "speed", "m/s"));
        rep.rows.push_back(reduce(rows, p, "drive", lab, "power", "W"));
    }
    // joints and everything else that already has a column
    for (size_t c = 1; c < cols.size(); ++c) {
        const std::string& n = cols[c];
        auto ends = [&](const char* suffix) { const std::string sf(suffix); return n.size() > sf.size() && n.compare(n.size() - sf.size(), sf.size(), sf) == 0; };
        auto stem = [&](const char* suffix) { return n.substr(0, n.size() - std::string(suffix).size()); };
        if (ends(".pin[N]")) rep.rows.push_back(reduce(rows, column(rows, (int)c), "joint", stem(".pin[N]"), "force", "N"));
        else if (ends(".normal[N]")) rep.rows.push_back(reduce(rows, column(rows, (int)c), "slide", stem(".normal[N]"), "side force", "N"));
        else if (ends(".moment[N*m]")) rep.rows.push_back(reduce(rows, column(rows, (int)c), "slide", stem(".moment[N*m]"), "moment", "N*m"));
        else if (ends(".spring[N]")) rep.rows.push_back(reduce(rows, column(rows, (int)c), "spring", stem(".spring[N]"), "tension", "N"));
        else if (ends(".N[N]")) rep.rows.push_back(reduce(rows, column(rows, (int)c), "contact", stem(".N[N]"), "normal force", "N"));
        else if (ends(".rack[N]")) rep.rows.push_back(reduce(rows, column(rows, (int)c), "rack", stem(".rack[N]"), "tooth force", "N"));
        else if (n.find(".link") != std::string::npos && ends("[N]") && std::count(n.begin(), n.end(), '.') == 1)
            rep.rows.push_back(reduce(rows, column(rows, (int)c), "belt", n.substr(0, n.size() - 3), "link force", "N"));
    }
    // within a group the heaviest load first (drives keep their torque / speed / power order)
    std::stable_sort(rep.rows.begin(), rep.rows.end(), [](const ReportRow& a, const ReportRow& b) {
        if (a.group != b.group) return false;
        if (a.group == "drive") return false;
        return a.peak > b.peak;
    });
    return rep;
}

} // namespace mech
