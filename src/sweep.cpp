// mech — sweep.cpp: design exploration over parameters (M8)
#include "sweep.hpp"

#include "kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace mech {

SimSeries simulate(const Model& m, double seconds, double dt, const std::vector<Probe>& probes,
                   long every) {
    SimSeries out;
    out.dynamic = m.world.dynamic;
    std::vector<double> q;
    const std::vector<std::string> warns = m.assemble(q);
    (void)warns;
    ConstraintSystem sys(m);
    int iters = 0;
    double res = 0.0;
    sys.solve(q, 0.0, iters, &res);
    const std::vector<std::string> geomErrs = m.validateAssembly(q);
    if (!geomErrs.empty()) {
        out.ok = false;
        out.error = geomErrs[0];
        return out;
    }

    if (!m.world.dynamic) {
        Recorder rec(m, nullptr, probes);
        out.columns = rec.columns();
        const std::vector<double> noVel;
        out.rows.push_back(rec.sample(0.0, q, noVel, sys.maxResidual(q, 0.0)));
        const long n = (long)std::floor(seconds / dt + 1e-9);
        for (long i = 1; i <= n; ++i) {
            const double t = (double)i * dt;
            sys.solve(q, t, iters);
            if (i % every == 0) out.rows.push_back(rec.sample(t, q, noVel, sys.maxResidual(q, t)));
        }
        return out;
    }

    DynamicsSystem dyn(m, sys);
    std::vector<double> qd((size_t)m.nq(), 0.0);
    dyn.projectVelocities(q, qd);
    const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
    Recorder rec(m, &dyn, probes);
    rec.setBaseline(E0);
    out.columns = rec.columns();
    out.rows.push_back(rec.sample(0.0, q, qd, sys.maxResidual(q, 0.0)));
    const double h = m.world.step;
    const long steps = (long)std::llround(seconds / h);
    for (long k = 1; k <= steps; ++k) {
        dyn.step(q, qd, (double)(k - 1) * h);
        if (k % every == 0 || k == steps)
            out.rows.push_back(rec.sample((double)k * h, q, qd, sys.maxResidual(q, (double)k * h)));
    }
    out.stats = dyn.hybridStats();
    return out;
}

// ---- metrics -------------------------------------------------------------------------------

namespace {

int findColumn(const std::vector<std::string>& cols, const std::string& name, std::string& err) {
    for (size_t i = 0; i < cols.size(); ++i)
        if (cols[i] == name) return (int)i;
    int hit = -1, hits = 0;                                   // unit omitted: `follower.y`
    for (size_t i = 0; i < cols.size(); ++i) {
        if (cols[i].compare(0, name.size() + 1, name + "[") == 0) { hit = (int)i; ++hits; }
    }
    if (hits == 1) return hit;
    err = hits > 1 ? "column '" + name + "' is ambiguous" : "no recorded column '" + name + "'";
    return -1;
}

} // namespace

bool parseMetric(const std::string& spec, const std::vector<std::string>& columns, bool dynamic,
                 Metric& out, std::string& err) {
    out = Metric{};
    out.label = spec;
    if (spec == "maxres") { out.kind = Metric::Kind::MaxRes; return true; }
    if (spec == "liftoffs" || spec == "impacts" || spec == "ledger") {
        if (!dynamic) { err = "'" + spec + "' needs a dynamic file (add a world block)"; return false; }
        out.kind = spec == "liftoffs" ? Metric::Kind::Liftoffs
                 : spec == "impacts"  ? Metric::Kind::Impacts : Metric::Kind::Ledger;
        return true;
    }
    const size_t colon = spec.find(':');
    if (colon == std::string::npos) {
        err = "unknown report '" + spec + "' (try max:<col>, min:<col>, mean:<col>, final:<col>, "
              "maxres, liftoffs, impacts, ledger)";
        return false;
    }
    const std::string kind = spec.substr(0, colon), name = spec.substr(colon + 1);
    if (kind == "max") out.kind = Metric::Kind::Max;
    else if (kind == "min") out.kind = Metric::Kind::Min;
    else if (kind == "mean") out.kind = Metric::Kind::Mean;
    else if (kind == "final") out.kind = Metric::Kind::Final;
    else { err = "unknown report kind '" + kind + "' (max, min, mean, final)"; return false; }
    out.col = findColumn(columns, name, err);
    return out.col >= 0;
}

double evalMetric(const Metric& m, const SimSeries& s) {
    if (s.rows.empty()) return std::nan("");
    switch (m.kind) {
    case Metric::Kind::Liftoffs: return (double)s.stats.liftoffs;
    case Metric::Kind::Impacts: return (double)s.stats.impacts;
    case Metric::Kind::MaxRes:
    case Metric::Kind::Ledger: {
        int c = -1;
        const std::string want = m.kind == Metric::Kind::MaxRes ? "maxRes" : "ledger.residual[J]";
        for (size_t i = 0; i < s.columns.size(); ++i) if (s.columns[i] == want) c = (int)i;
        if (c < 0) return std::nan("");
        double w = 0.0;
        for (const auto& r : s.rows) w = std::max(w, std::fabs(r[(size_t)c]));
        return w;
    }
    default: break;
    }
    double mx = -1e300, mn = 1e300, sum = 0.0;
    for (const auto& r : s.rows) {
        const double v = r[(size_t)m.col];
        mx = std::max(mx, v);
        mn = std::min(mn, v);
        sum += v;
    }
    switch (m.kind) {
    case Metric::Kind::Max: return mx;
    case Metric::Kind::Min: return mn;
    case Metric::Kind::Mean: return sum / (double)s.rows.size();
    default: return s.rows.back()[(size_t)m.col];
    }
}

// ---- axes ----------------------------------------------------------------------------------

bool parseAxis(const std::string& spec, SweepAxis& out, std::string& err) {
    const size_t eq = spec.find('=');
    if (eq == std::string::npos || eq == 0 || eq + 1 >= spec.size()) {
        err = "a sweep axis is name=value, name=v1,v2,..., or name=start:stop:step; got '" + spec + "'";
        return false;
    }
    out = SweepAxis{};
    out.name = spec.substr(0, eq);
    const std::string body = spec.substr(eq + 1);
    auto num = [&](const std::string& t, double& v) {
        char* end = nullptr;
        v = std::strtod(t.c_str(), &end);
        return end != t.c_str() && *end == '\0' && std::isfinite(v);
    };
    if (body.find(':') != std::string::npos) {
        std::vector<std::string> parts;
        size_t a = 0;
        for (;;) {
            const size_t b = body.find(':', a);
            parts.push_back(body.substr(a, b == std::string::npos ? std::string::npos : b - a));
            if (b == std::string::npos) break;
            a = b + 1;
        }
        double lo, hi, step;
        if (parts.size() != 3 || !num(parts[0], lo) || !num(parts[1], hi) || !num(parts[2], step)) {
            err = "'" + out.name + "': a range is start:stop:step";
            return false;
        }
        if (step == 0.0 || (hi - lo) * step < 0.0) {
            err = "'" + out.name + "': step " + parts[2] + " never reaches " + parts[1] + " from " + parts[0];
            return false;
        }
        const long n = (long)std::floor((hi - lo) / step + 1e-9) + 1;
        if (n > 5000) { err = "'" + out.name + "': that range has " + std::to_string(n) + " values"; return false; }
        for (long i = 0; i < n; ++i) out.values.push_back(lo + (double)i * step);   // no accumulated drift
        return true;
    }
    size_t a = 0;
    for (;;) {
        const size_t b = body.find(',', a);
        const std::string t = body.substr(a, b == std::string::npos ? std::string::npos : b - a);
        double v;
        if (!num(t, v)) { err = "'" + out.name + "': '" + t + "' is not a number"; return false; }
        out.values.push_back(v);
        if (b == std::string::npos) break;
        a = b + 1;
    }
    return true;
}

// ---- the sweep -----------------------------------------------------------------------------

SweepResult runSweep(const std::string& source, const std::vector<SweepAxis>& axes,
                     const SweepOptions& opt) {
    SweepResult res;
    size_t total = 1;
    for (const SweepAxis& a : axes) {
        if (a.values.empty()) throw std::runtime_error("axis '" + a.name + "' has no values");
        total *= a.values.size();
        if (total > 5000) throw std::runtime_error("that grid is over 5000 runs");
        res.axisNames.push_back(a.name);
    }
    const std::vector<Token> toks = lex(source);

    auto build = [&](const ParamOverrides& ov, std::string& err) -> bool {
        try {
            const Program p = parse(toks, ov);
            if (p.mechanisms.empty()) { err = "the file contains no mechanism"; return false; }
            (void)buildModel(p.mechanisms[0]);
            return true;
        } catch (const ParseError& e) { err = e.msg; }
        catch (const SemanticError& e) { err = e.msg; }
        return false;
    };

    // The unmodified file must build: it fixes the column names and the default reports.
    std::vector<std::string> baseCols;
    bool baseDynamic = false;
    std::vector<Probe> baseProbes;
    {
        ParamOverrides ov;                                      // validates every axis name too
        for (const SweepAxis& a : axes) ov[a.name] = a.values.front();
        std::string err;
        if (!build(ov, err)) throw std::runtime_error("the first design does not build: " + err);
        const Program p = parse(toks, ov);
        const Model m = buildModel(p.mechanisms[0]);
        baseDynamic = m.world.dynamic;
        for (const std::string& ref : opt.probes) {
            const size_t dot = ref.find('.');
            const int bi = dot == std::string::npos ? -1 : m.bodyIndex(ref.substr(0, dot));
            const int ai = bi < 0 ? -1 : m.anchorIndex(bi, ref.substr(dot + 1));
            if (bi < 0 || ai < 0 || m.bodies[(size_t)bi].isGround)
                throw std::runtime_error("--probe expects a moving body.anchor, got '" + ref + "'");
            baseProbes.push_back(Probe{bi, ai});
        }
        baseCols = Recorder(m, nullptr, baseProbes).columns();
        if (baseDynamic) {
            ConstraintSystem sys(m);
            DynamicsSystem dyn(m, sys);
            baseCols = Recorder(m, &dyn, baseProbes).columns();
        }
        res.dynamic = baseDynamic;
        std::vector<std::string> reports = opt.reports;
        if (reports.empty()) {
            reports.push_back("maxres");
            if (baseDynamic) {
                reports.push_back("ledger");
                if (!m.touches.empty()) reports.push_back("liftoffs");
            }
        }
        for (const std::string& r : reports) res.metricLabels.push_back(r);
    }
    std::vector<Metric> metrics;
    for (const std::string& r : res.metricLabels) {
        Metric mt;
        std::string err;
        if (!parseMetric(r, baseCols, baseDynamic, mt, err)) throw std::runtime_error(err);
        metrics.push_back(mt);
    }

    for (size_t flat = 0; flat < total; ++flat) {
        SweepRow row;
        ParamOverrides ov;
        size_t rem = flat;
        std::vector<double> vals(axes.size());
        for (size_t ai = axes.size(); ai-- > 0;) {               // first axis slowest
            vals[ai] = axes[ai].values[rem % axes[ai].values.size()];
            rem /= axes[ai].values.size();
        }
        for (size_t ai = 0; ai < axes.size(); ++ai) ov[axes[ai].name] = vals[ai];
        row.params = vals;
        row.metrics.assign(metrics.size(), std::nan(""));
        try {
            const Program p = parse(toks, ov);
            const Model m = buildModel(p.mechanisms[0]);
            const SimSeries s = simulate(m, opt.seconds, opt.dt, baseProbes);
            if (!s.ok) {
                row.error = s.error;
            } else {
                for (size_t k = 0; k < metrics.size(); ++k) row.metrics[k] = evalMetric(metrics[k], s);
            }
        } catch (const ParseError& e) {
            row.error = e.msg;
        } catch (const SemanticError& e) {
            row.error = e.msg;
        }
        res.rows.push_back(row);
    }
    return res;
}

} // namespace mech
