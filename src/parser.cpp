// mech — parser.cpp: recursive-descent parser for the .mech language (M6)
//
// Grammar (M3 kernel + sugar):
//   program     := mechanism*
//   mechanism   := 'mechanism' IDENT '{' stmt* '}'
//   stmt        := groundDecl | bodyDecl | rodDecl | shaftDecl | worldDecl
//                | gearDecl | meshDecl | springDecl | pinDecl | slideDecl
//                | driveDecl
//
//   groundDecl  := 'ground' IDENT '{' anchor* '}'
//   bodyDecl    := 'body' IDENT '{' (anchor | 'mass' ':' num | 'from' ':' num 'deg')* '}'
//   anchor      := 'anchor' IDENT 'at' '(' num ',' num ')'
//                  [ 'dir' '(' num ',' num ')' ]
//
//   rodDecl     := 'rod' IDENT [ attach ] '{' rodProp* '}' [ attach ]
//   rodProp     := 'length' ':' num | 'from' ':' num 'deg' | 'mass' ':' num
//   shaftDecl   := 'shaft' IDENT attach              (no block)
//   attach      := ('on' | 'at') ref                 sugar: pins anchor A
//
//   worldDecl   := 'world' '{' ('gravity' | 'step') ':' num* '}'
//   gearDecl    := 'gear' IDENT '{' gearProp* '}' 'on' ref
//   gearProp    := ('teeth' | 'module' | 'radius' | 'mass') ':' num
//   meshDecl    := 'mesh' IDENT IDENT [ 'tangent' ]
//   springDecl  := 'spring' IDENT ref '->' ref '{' springProp* '}'
//   springProp  := ('k' | 'rest' | 'damp') ':' num
//
//   pinDecl     := 'pin' ref '->' ref
//   slideDecl   := 'slide' ref '->' ref
//   driveDecl   := 'drive' IDENT 'at' num ('deg/s' | 'rpm') ['from' num 'deg']
//                | 'drive' IDENT 'torque' num
//   ref         := IDENT '.' IDENT
//   num         := ['-'] NUMBER
//
// M4 additions (contextual idents: 'ratio', 'via', 'same', 'opposite',
// 'module', 'spread', 'beat', 'sec' — they stay usable as ordinary names):
//   reduceDecl  := 'reduce' IDENT '->' IDENT 'ratio' num [':' num]
//                  ['via' IDENT (',' IDENT)*] [('same' | 'opposite')]
//                  ['module' ':' num] ['spread' ':' num 'deg']
//   handDecl    := 'hand' IDENT 'on' ref ['.' IDENT] '{' handProp* '}'
//   handProp    := 'length' ':' num
//   dialDecl    := 'dial' ['at' ref] '{' dialProp* '}'
//   dialProp    := ('radius' | 'major' | 'minor') ':' num
//   rodProp     += 'beat' ':' num [('s' | 'sec')]     (period intent)
//   shaftDecl   := 'shaft' IDENT [attach]             (attach now optional:
//                                                     a bare shaft waits
//                                                     for a 'reduce' to
//                                                     place it)
//
// M5 additions ('mode' is a keyword; 'state', 'when', 'hold', 'impulse',
// 'maintain', 'weight', 'barrel', 'drop', 'kg', 'm', 'mJ' are contextual
// identifiers, so they remain usable as ordinary names):
//   driveDecl   += 'drive' IDENT 'weight' num ['kg']
//                  ['barrel' num ['m']] ['drop' num ['m']] ['cw' | 'ccw']
//   modeDecl    := 'mode' IDENT '{' (stateDecl | transition)* '}'
//   stateDecl   := 'state' IDENT '{' ('hold' IDENT)* '}'
//   transition  := IDENT '->' IDENT 'when' guard [':' reset (',' reset)*]
//   guard       := IDENT '.' ('theta' | 'omega' | 'advance')
//                  ('>' | '<') num [('deg' | 'deg/s')]
//   reset       := 'impulse' IDENT num ['mJ']
//                | 'impulse' IDENT 'maintain' num ['deg']
//
// M6 additions ('profile' and 'touch' are keywords; 'circle', 'polar',
// 'points' and 'roller' are contextual identifiers):
//   profileDecl := 'profile' IDENT [ 'on' IDENT ['.' IDENT] ]
//                  '{' shape '}' [ 'on' IDENT ['.' IDENT] ]
//   shape       := 'circle' ':' num
//                | 'polar' ':' ( '(' num ['deg'] ',' num ')' )+
//                | 'points' ':' ( '(' num ',' num ')' )+
//                | 'ellipse' ':' num num [ 'focus' ]          (M6d)
//                | 'polygon' ':' ( '(' num ',' num ')' )+ 'fillet' num   (M6e)
//   touchDecl   := 'touch' ref 'on' IDENT { 'roller' num | 'restitution' num | 'friction' num }
//
// M7 ('param' is contextual at statement start; the names `pi`, sqrt sin cos tan atan
// abs min max pow and the cam item words are reserved as parameter names):
//   paramDecl   := 'param' IDENT '=' expr
//   num         := expr       (every number in the language is now an expression)
//   expr        := term { ('+' | '-') term }
//   term        := unary { ('*' | '/') unary }
//   unary       := ['-' | '+'] unary | primary
//   primary     := NUMBER | 'pi' | PARAM | FUNC '(' expr {',' expr} ')' | '(' expr ')'
//
// M6b ('cam' is contextual at statement start so `cam` stays usable as a
// body name; the items below are contextual identifiers too):
//   camDecl     := 'cam' IDENT [ 'on' IDENT ['.' IDENT] ]
//                  '{' camItem* '}' [ 'on' IDENT ['.' IDENT] ]
//   camItem     := 'base' num | 'roller' num | 'start' num 'deg'
//                | 'turn' ('ccw' | 'cw')
//                | ('rise' | 'fall') num 'over' num 'deg' [law]
//                | 'return' 'over' num 'deg' [law]
//                | 'dwell' [ num 'deg' ]
//   law         := 'cycloidal' | 'harmonic' | 'modtrap'
//
// 'dir', 'tangent' and all property names are contextual identifiers (not
// keywords), so they remain usable as ordinary names. Comments ('//', '#')
// are lexed away.

#include "parser.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace mech {

namespace {

class Parser {
public:
    Parser(const std::vector<Token>& toks, const ParamOverrides& ov) : t_(toks), ov_(ov) {}

    Program program() {
        Program p;
        while (!atEnd()) {
            expectKw("mechanism");
            p.mechanisms.push_back(mechanism());
        }
        // an override that matched no parameter is a typo, not a no-op
        for (const auto& kv : ov_) {
            if (used_.count(kv.first)) continue;
            std::string known;
            for (const MechanismDecl& m : p.mechanisms)
                for (const ParamDecl& pd : m.params) known += (known.empty() ? "" : ", ") + pd.name;
            throw ParseError{1, 1, "--set '" + kv.first + "': the file declares no such parameter (" +
                                   (known.empty() ? std::string("it declares none") : "it declares: " + known) +
                                   ")"};
        }
        return p;
    }

private:
    const std::vector<Token>& t_;
    const ParamOverrides& ov_;
    std::set<std::string> used_;                 // overrides that found their parameter
    std::map<std::string, double> params_;       // parameters declared so far (this mechanism)
    size_t pos_ = 0;

    // --- token helpers -----------------------------------------------------

    const Token& cur() const { return t_[pos_]; }
    bool atEnd() const { return cur().type == Tok::End; }

    [[noreturn]] void fail(const std::string& msg) const {
        throw ParseError{cur().line, cur().col, msg};
    }

    void next() { if (!atEnd()) ++pos_; }

    bool acceptSym(const char* s) {
        if (cur().type == Tok::Sym && cur().text == s) { next(); return true; }
        return false;
    }

    void expectSym(const char* s) {
        if (!acceptSym(s)) fail(std::string("expected '") + s + "', got '" + cur().text + "'");
    }

    bool acceptKw(const char* s) {
        if (cur().type == Tok::Kw && cur().text == s) { next(); return true; }
        return false;
    }

    void expectKw(const char* s) {
        if (!acceptKw(s)) fail(std::string("expected '") + s + "', got '" + cur().text + "'");
    }

    Token expectIdent() {
        if (cur().type != Tok::Ident) fail("expected identifier, got '" + cur().text + "'");
        Token t = cur(); next(); return t;
    }

    void skipSeparators() {
        while (acceptSym(";") || acceptSym(",")) {}
    }

    // --- numeric expressions (M7) --------------------------------------------------------
    //   expr    := term { ('+' | '-') term }
    //   term    := unary { ('*' | '/') unary }
    //   unary   := ('-' | '+') unary | primary
    //   primary := NUMBER | 'pi' | PARAM | FUNC '(' expr { ',' expr } ')' | '(' expr ')'
    // FUNC: sqrt sin cos tan atan abs min max pow (angles in radians). Only an operator
    // continues an expression, so a bare number followed by a unit word (`4 kg`, `1 s`,
    // `100 deg`) or the next item ends it exactly as before.
    double parseNum() { return parseSum(); }

    static bool isFunc(const std::string& n) {
        return n == "sqrt" || n == "sin" || n == "cos" || n == "tan" || n == "atan" ||
               n == "abs" || n == "min" || n == "max" || n == "pow";
    }
    static bool isReservedParam(const std::string& n) {
        return n == "pi" || isFunc(n) || n == "base" || n == "roller" || n == "start" ||
               n == "turn" || n == "rise" || n == "fall" || n == "return" || n == "dwell";
    }
    // Can the next token begin a number? (For items where a number is optional.)
    bool startsNumber() const {
        const Token& t = cur();
        if (t.type == Tok::Number) return true;
        if (t.type == Tok::Sym) return t.text == "-" || t.text == "+" || t.text == "(";
        return t.type == Tok::Ident && (t.text == "pi" || params_.count(t.text) > 0);
    }

    double checkFinite(double v) const {
        if (!std::isfinite(v)) fail("this expression does not evaluate to a finite number");
        return v;
    }

    double parseSum() {
        double v = parseProd();
        while (cur().type == Tok::Sym && (cur().text == "+" || cur().text == "-")) {
            const bool plus = cur().text == "+";
            next();
            const double r = parseProd();
            v = plus ? v + r : v - r;
        }
        return v;
    }

    double parseProd() {
        double v = parseUnary();
        while (cur().type == Tok::Sym && (cur().text == "*" || cur().text == "/")) {
            const bool mul = cur().text == "*";
            next();
            const double r = parseUnary();
            if (!mul && r == 0.0) fail("division by zero");
            v = mul ? v * r : v / r;
        }
        return v;
    }

    double parseUnary() {
        if (acceptSym("-")) return -parseUnary();
        if (acceptSym("+")) return parseUnary();
        return parsePrimary();
    }

    double parsePrimary() {
        const Token t = cur();
        if (t.type == Tok::Number) { next(); return t.num; }
        if (acceptSym("(")) {
            const double v = parseSum();
            expectSym(")");
            return v;
        }
        if (t.type == Tok::Ident) {
            if (t.text == "pi") { next(); return kPi; }
            auto it = params_.find(t.text);
            if (it != params_.end()) { next(); return it->second; }
            if (isFunc(t.text) && pos_ + 1 < t_.size() && t_[pos_ + 1].type == Tok::Sym &&
                t_[pos_ + 1].text == "(") {
                next();
                expectSym("(");
                std::vector<double> a;
                a.push_back(parseSum());
                while (acceptSym(",")) a.push_back(parseSum());
                expectSym(")");
                return callFunc(t, a);
            }
            fail("'" + t.text + "' is not a number or a declared parameter "
                 "(declare it with 'param " + t.text + " = ...' first)");
        }
        fail("expected number, got '" + t.text + "'");
    }

    double callFunc(const Token& f, const std::vector<double>& a) {
        const std::string& n = f.text;
        const bool two = n == "min" || n == "max" || n == "pow";
        if (a.size() != (two ? 2u : 1u)) {
            throw ParseError{f.line, f.col, "'" + n + "' takes " + (two ? "two arguments" : "one argument")};
        }
        double v = 0.0;
        if (n == "sqrt") {
            if (a[0] < 0.0) throw ParseError{f.line, f.col, "sqrt of a negative number"};
            v = std::sqrt(a[0]);
        } else if (n == "sin") v = std::sin(a[0]);
        else if (n == "cos") v = std::cos(a[0]);
        else if (n == "tan") v = std::tan(a[0]);
        else if (n == "atan") v = std::atan(a[0]);
        else if (n == "abs") v = std::fabs(a[0]);
        else if (n == "min") v = std::min(a[0], a[1]);
        else if (n == "max") v = std::max(a[0], a[1]);
        else v = std::pow(a[0], a[1]);
        if (!std::isfinite(v)) throw ParseError{f.line, f.col, "'" + n + "' does not give a finite number here"};
        return v;
    }

    // param NAME = expr
    void paramDecl(MechanismDecl& m) {
        const Token name = expectIdent();
        if (isReservedParam(name.text)) {
            throw ParseError{name.line, name.col, "'" + name.text + "' cannot be a parameter name "
                             "(it is a constant, a function, or part of the cam vocabulary)"};
        }
        if (params_.count(name.text)) {
            throw ParseError{name.line, name.col, "parameter '" + name.text + "' is already declared"};
        }
        expectSym("=");
        ParamDecl pd;
        pd.name = name.text;
        pd.line = name.line;
        pd.declared = pd.value = checkFinite(parseNum());
        auto ov = ov_.find(name.text);
        if (ov != ov_.end()) {
            pd.value = ov->second;
            pd.overridden = true;
            used_.insert(name.text);
        }
        params_[name.text] = pd.value;
        m.params.push_back(pd);
    }

    void parseRef(std::string& body, std::string& anchor) {
        body = expectIdent().text;
        expectSym(".");
        anchor = expectIdent().text;
    }

    // --- grammar -----------------------------------------------------------

    MechanismDecl mechanism() {
        MechanismDecl m;
        params_.clear();                        // parameters are per mechanism
        Token name = expectIdent();
        m.name = name.text;
        m.line = name.line;
        expectSym("{");

        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' (expected '}')");
            stmt(m);
            skipSeparators();
        }
        return m;
    }

    void stmt(MechanismDecl& m) {
        if (acceptKw("ground")) anchorBodyDecl(m, BodyDecl::Kind::Ground);
        else if (acceptKw("body")) anchorBodyDecl(m, BodyDecl::Kind::Free);
        else if (acceptKw("rod")) rodDecl(m);
        else if (acceptKw("shaft")) shaftDecl(m);
        else if (acceptKw("world")) worldDecl(m);
        else if (acceptKw("gear")) gearDecl(m);
        else if (acceptKw("mesh")) meshDecl(m);
        else if (acceptKw("spring")) springDecl(m);
        else if (acceptKw("pin")) jointDecl(m, JointDecl::Kind::Pin);
        else if (acceptKw("slide")) jointDecl(m, JointDecl::Kind::Slide);
        else if (acceptKw("drive")) driveDecl(m);
        else if (acceptKw("reduce")) reduceDecl(m);
        else if (acceptKw("hand")) handDecl(m);
        else if (acceptKw("dial")) dialDecl(m);
        else if (acceptKw("mode")) modeDecl(m);
        else if (acceptKw("profile")) profileDecl(m);
        else if (acceptKw("touch")) touchDecl(m);
        else if (cur().type == Tok::Ident && cur().text == "cam") { next(); camDecl(m); }
        else if (cur().type == Tok::Ident && cur().text == "param") { next(); paramDecl(m); }
        else fail("expected a statement ('ground', 'body', 'rod', 'shaft', 'world', 'gear', "
                  "'mesh', 'spring', 'pin', 'slide', 'drive', 'reduce', 'hand', 'dial', "
                  "'mode', 'profile', 'touch' or 'cam'), got '" + cur().text + "'");
    }

    void anchorBodyDecl(MechanismDecl& m, BodyDecl::Kind kind) {
        BodyDecl b;
        b.kind = kind;
        Token name = expectIdent();
        b.name = name.text;
        b.line = name.line;
        expectSym("{");

        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' in body '" + b.name + "'");
            if (kind == BodyDecl::Kind::Free && cur().type == Tok::Ident &&
                cur().text == "mass") {
                next();
                expectSym(":");
                b.props["mass"] = parseNum();
            } else if (kind == BodyDecl::Kind::Free && cur().type == Tok::Kw &&
                       cur().text == "from") {
                // release angle, as for a rod: assembly seeds this body at the angle
                next();
                expectSym(":");
                b.props["from"] = parseNum();
                if (cur().type == Tok::UnitDeg) { next(); }
                else fail("expected 'deg' after the body's release angle, got '" + cur().text + "'");
            } else {
                expectKw("anchor");
                AnchorDecl a;
                Token an = expectIdent();
                a.name = an.text;
                a.line = an.line;
                expectKw("at");
                expectSym("(");
                a.pos.x = parseNum();
                expectSym(",");
                a.pos.y = parseNum();
                expectSym(")");
                if (cur().type == Tok::Ident && cur().text == "dir") {
                    next();
                    expectSym("(");
                    a.dir.x = parseNum();
                    expectSym(",");
                    a.dir.y = parseNum();
                    expectSym(")");
                    a.hasDir = true;
                }
                b.anchors.push_back(a);
            }
            skipSeparators();
        }
        m.bodies.push_back(b);
    }

    // world { gravity: 9.81, step: 0.002 } — contextual property idents
    void worldDecl(MechanismDecl& m) {
        WorldDecl w;
        w.line = cur().line;
        w.present = true;
        expectSym("{");
        bool sawGravity = false, sawStep = false;
        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' in world block");
            if (cur().type == Tok::Ident &&
                (cur().text == "gravity" || cur().text == "step")) {
                const std::string prop = cur().text;
                next();
                expectSym(":");
                const double v = parseNum();
                if (prop == "gravity") {
                    if (sawGravity) fail("duplicate world property 'gravity'");
                    sawGravity = true;
                    w.gravity = v;
                } else {
                    if (sawStep) fail("duplicate world property 'step'");
                    sawStep = true;
                    w.step = v;
                }
            } else {
                fail("expected world property ('gravity' or 'step'), got '"
                     + cur().text + "'");
            }
            skipSeparators();
        }
        m.world = w;
    }

    // attach := ('on' | 'at') ref — recorded; the model builder desugars it
    // into a pin on anchor A.
    bool parseAttach(BodyDecl& b) {
        if (cur().type == Tok::Kw && (cur().text == "on" || cur().text == "at")) {
            next();
            parseRef(b.attachBody, b.attachAnchor);
            b.hasAttach = true;
            return true;
        }
        return false;
    }

    void rodDecl(MechanismDecl& m) {
        BodyDecl b;
        b.kind = BodyDecl::Kind::Rod;
        Token name = expectIdent();
        b.name = name.text;
        b.line = name.line;
        parseAttach(b);               // 'rod r on A.p { ... }'
        expectSym("{");

        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' in rod '" + b.name + "'");
            if (cur().type == Tok::Kw && cur().text == "length") {
                next();
                expectSym(":");
                b.props["length"] = parseNum();
            } else if (cur().type == Tok::Ident && cur().text == "beat") {
                // M4 period intent: beat = half the swing (horology: tick, tock).
                // The rod length is then derived from the world's gravity.
                next();
                expectSym(":");
                const double v = parseNum();
                if (v <= 0.0) fail("rod 'beat' must be a positive time");
                // unit: optional 's' / 'sec' (contextual idents — '1s' lexes
                // as NUMBER followed by identifier 's')
                if (cur().type == Tok::Ident &&
                    (cur().text == "s" || cur().text == "sec")) {
                    next();
                }
                b.props["beat"] = v;
            } else if (cur().type == Tok::Kw && cur().text == "from") {
                // release angle: assembly seeds this body at the given angle
                next();
                expectSym(":");
                b.props["from"] = parseNum();
                if (cur().type == Tok::UnitDeg) { next(); }
                else fail("expected 'deg' after rod release angle, got '" + cur().text + "'");
            } else if (cur().type == Tok::Ident && cur().text == "mass") {
                next();
                expectSym(":");
                b.props["mass"] = parseNum();
            } else {
                fail("expected rod property ('length', 'mass', 'from' or 'beat'), got '" + cur().text + "'");
            }
            skipSeparators();
        }
        if (!b.hasAttach) parseAttach(b);   // 'rod r { ... } on A.p'
        m.bodies.push_back(b);
    }

    void shaftDecl(MechanismDecl& m) {
        BodyDecl b;
        b.kind = BodyDecl::Kind::Shaft;
        Token name = expectIdent();
        b.name = name.text;
        b.line = name.line;
        parseAttach(b);   // M4: optional — a bare shaft is placed by 'reduce'
        m.bodies.push_back(b);
    }

    void gearDecl(MechanismDecl& m) {
        GearDecl g;
        Token name = expectIdent();
        g.name = name.text;
        g.line = name.line;
        expectSym("{");

        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' in gear '" + g.name + "'");
            if (cur().type == Tok::Ident) {
                std::string prop = cur().text;
                if (prop != "teeth" && prop != "module" && prop != "radius" &&
                    prop != "mass") {
                    fail("unknown gear property '" + prop +
                         "' (expected 'teeth', 'module', 'radius' or 'mass')");
                }
                next();
                expectSym(":");
                g.props[prop] = parseNum();
            } else {
                fail("expected gear property ('teeth', 'module', 'radius' or 'mass'), got '"
                     + cur().text + "'");
            }
            skipSeparators();
        }

        expectKw("on");
        Token host = expectIdent();
        g.host = host.text;
        if (acceptSym(".")) {
            // M3: the pitch circle may sit on any host anchor
            g.hostAnchor = expectIdent().text;
        }
        m.gears.push_back(g);
    }

    void meshDecl(MechanismDecl& m) {
        MeshDecl d;
        d.line = cur().line;
        d.gearA = expectIdent().text;
        d.gearB = expectIdent().text;
        if (cur().type == Tok::Ident) {
            if (cur().text == "tangent") {
                next();
                d.tangent = true;
            } else {
                fail("unexpected identifier '" + cur().text +
                     "' after mesh (did you mean 'tangent'?)");
            }
        }
        m.meshes.push_back(d);
    }

    // M6 — profile lobe on cam.A { circle: 0.03 }
    //      profile lobe { polar: (0 deg, 0.030) (90 deg, 0.034) (180 deg, 0.030) } on cam
    // The host may come before or after the block (like a rod's attach).
    // 'circle', 'polar' and 'points' are contextual identifiers.
    void profileDecl(MechanismDecl& m) {
        ProfileDecl p;
        Token name = expectIdent();
        p.name = name.text;
        p.line = name.line;
        bool hasHost = false;
        auto host = [&]() {
            p.host = expectIdent().text;
            if (acceptSym(".")) p.hostAnchor = expectIdent().text;
            hasHost = true;
        };
        if (acceptKw("on")) host();
        expectSym("{");

        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' in profile '" + p.name + "'");
            if (cur().type != Tok::Ident ||
                (cur().text != "circle" && cur().text != "polar" && cur().text != "points" &&
                 cur().text != "ellipse" && cur().text != "polygon")) {
                fail("expected profile shape ('circle', 'polar', 'points', 'ellipse' or "
                     "'polygon'), got '" + cur().text + "'");
            }
            if (p.kind != ProfileDecl::Kind::None) {
                fail("profile '" + p.name + "' already has a shape — give it exactly one");
            }
            const std::string shape = cur().text;
            next();
            expectSym(":");
            if (shape == "circle") {
                p.kind = ProfileDecl::Kind::Circle;
                p.radius = parseNum();
            } else if (shape == "polygon") {
                p.kind = ProfileDecl::Kind::Polygon;
                while (acceptSym("(")) {
                    const double x = parseNum();
                    expectSym(",");
                    const double y = parseNum();
                    expectSym(")");
                    p.points.push_back(Vec2{x, y});
                    skipSeparators();
                }
                if (p.points.empty()) fail("'polygon:' needs (x, y) vertices");
                if (cur().type == Tok::Ident && cur().text == "fillet") {
                    next();
                    p.hasFillet = true;
                    p.fillet = parseNum();
                }
            } else if (shape == "ellipse") {
                p.kind = ProfileDecl::Kind::Ellipse;
                p.ellipseA = parseNum();
                p.ellipseB = parseNum();
                if (cur().type == Tok::Ident && cur().text == "focus") {
                    next();
                    p.ellipseFocus = true;
                }
            } else if (shape == "polar") {
                p.kind = ProfileDecl::Kind::Polar;
                while (acceptSym("(")) {
                    double a = parseNum();
                    if (cur().type == Tok::UnitDeg) { next(); a = deg2rad(a); }
                    expectSym(",");
                    const double r = parseNum();
                    expectSym(")");
                    p.polar.emplace_back(a, r);
                    skipSeparators();
                }
                if (p.polar.empty()) fail("'polar:' needs (angle, radius) pairs");
            } else {
                p.kind = ProfileDecl::Kind::Points;
                while (acceptSym("(")) {
                    const double x = parseNum();
                    expectSym(",");
                    const double y = parseNum();
                    expectSym(")");
                    p.points.push_back(Vec2{x, y});
                    skipSeparators();
                }
                if (p.points.empty()) fail("'points:' needs (x, y) pairs");
            }
            skipSeparators();
        }
        if (p.kind == ProfileDecl::Kind::None) {
            throw ParseError{name.line, name.col, "profile '" + p.name +
                "' has no shape — give it 'circle', 'polar', 'points', 'ellipse' or 'polygon'"};
        }
        if (!hasHost && acceptKw("on")) host();
        if (!hasHost) {
            throw ParseError{name.line, name.col, "profile '" + p.name +
                "' needs a host — e.g. 'profile " + p.name + " on cam.A { ... }'"};
        }
        m.profiles.push_back(p);
    }

    // M6b — cam lobe on cam.A { base 0.03 roller 0.006 rise 0.01 over 120 deg ... }
    // Every item is a contextual identifier; angles require the 'deg' unit.
    void camDecl(MechanismDecl& m) {
        CamDecl c;
        Token name = expectIdent();
        c.name = name.text;
        c.line = name.line;
        bool hasHost = false;
        auto host = [&]() {
            c.host = expectIdent().text;
            if (acceptSym(".")) c.hostAnchor = expectIdent().text;
            hasHost = true;
        };
        auto expectDeg = [&](const char* what) {
            if (cur().type != Tok::UnitDeg)
                fail(std::string("expected 'deg' after ") + what + ", got '" + cur().text + "'");
            next();
        };
        auto expectWord = [&](const char* w) {
            if (cur().type != Tok::Ident || cur().text != w)
                fail(std::string("expected '") + w + "', got '" + cur().text + "'");
            next();
        };
        auto isItem = [](const std::string& t) {
            return t == "base" || t == "roller" || t == "start" || t == "turn" ||
                   t == "rise" || t == "fall" || t == "return" || t == "dwell";
        };
        // optional motion law after a segment
        auto law = [&](CamSegment& seg) {
            if (cur().type != Tok::Ident || isItem(cur().text)) return;
            if (cur().text == "cycloidal") seg.law = MotionLaw::Cycloidal;
            else if (cur().text == "harmonic") seg.law = MotionLaw::Harmonic;
            else if (cur().text == "modtrap") seg.law = MotionLaw::ModTrap;
            else fail("unknown motion law '" + cur().text +
                      "' (expected 'cycloidal', 'harmonic' or 'modtrap')");
            next();
        };

        if (acceptKw("on")) host();
        expectSym("{");
        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' in cam '" + c.name + "'");
            if (cur().type != Tok::Ident || !isItem(cur().text)) {
                fail("expected cam item ('base', 'roller', 'start', 'turn', 'rise', 'fall', "
                     "'return' or 'dwell'), got '" + cur().text + "'");
            }
            const Token item = cur();
            next();
            if (item.text == "base") {
                c.spec.base = parseNum();
                c.hasBase = true;
            } else if (item.text == "roller") {
                c.spec.roller = parseNum();
            } else if (item.text == "start") {
                c.spec.startRad = deg2rad(parseNum());
                expectDeg("the start angle");
            } else if (item.text == "turn") {
                if (cur().type == Tok::Ident && cur().text == "ccw") c.spec.turn = +1;
                else if (cur().type == Tok::Ident && cur().text == "cw") c.spec.turn = -1;
                else fail("expected 'ccw' or 'cw' after 'turn', got '" + cur().text + "'");
                next();
            } else {
                CamSegment seg;
                if (item.text == "dwell") {
                    seg.kind = CamSegment::Kind::Dwell;
                    if (startsNumber()) {
                        seg.angle = deg2rad(parseNum());
                        expectDeg("the dwell angle");
                    } else {
                        seg.restOfTurn = true;
                    }
                } else {
                    seg.kind = item.text == "rise" ? CamSegment::Kind::Rise
                             : item.text == "fall" ? CamSegment::Kind::Fall
                                                   : CamSegment::Kind::Return;
                    if (seg.kind != CamSegment::Kind::Return) seg.lift = parseNum();
                    expectWord("over");
                    seg.angle = deg2rad(parseNum());
                    expectDeg("the segment angle");
                    law(seg);
                }
                c.spec.segments.push_back(seg);
                c.segLines.push_back(item.line);
            }
            skipSeparators();
        }
        if (!hasHost && acceptKw("on")) host();
        if (!hasHost) {
            throw ParseError{name.line, name.col, "cam '" + c.name +
                "' needs a host — e.g. 'cam " + c.name + " on cam.A { ... }'"};
        }
        m.cams.push_back(c);
    }

    // M6 — touch follower.tip on lobe [roller 0.008]
    void touchDecl(MechanismDecl& m) {
        TouchDecl t;
        t.line = cur().line;
        parseRef(t.body, t.anchor);
        expectKw("on");
        t.profile = expectIdent().text;
        while (cur().type == Tok::Ident &&
               (cur().text == "roller" || cur().text == "restitution" ||
                cur().text == "friction")) {
            const std::string key = cur().text;
            next();
            if (key == "roller") {
                t.hasRoller = true;
                t.roller = parseNum();
            } else if (key == "friction") {
                t.hasFriction = true;
                t.friction = parseNum();
            } else {
                t.hasRestitution = true;
                t.restitution = parseNum();
            }
        }
        m.touches.push_back(t);
    }

    // spring sp a.h -> b.k { k: 9, rest: 1.2, damp: 0.1 }
    void springDecl(MechanismDecl& m) {
        SpringDecl s;
        Token name = expectIdent();
        s.name = name.text;
        s.line = name.line;
        parseRef(s.bodyA, s.anchorA);
        expectSym("->");
        parseRef(s.bodyB, s.anchorB);
        expectSym("{");
        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' in spring '" + s.name + "'");
            if (cur().type == Tok::Ident) {
                const std::string prop = cur().text;
                if (prop != "k" && prop != "rest" && prop != "damp") {
                    fail("unknown spring property '" + prop +
                         "' (expected 'k', 'rest' or 'damp')");
                }
                next();
                expectSym(":");
                s.props[prop] = parseNum();
            } else {
                fail("expected spring property ('k', 'rest' or 'damp'), got '"
                     + cur().text + "'");
            }
            skipSeparators();
        }
        m.springs.push_back(s);
    }

    void jointDecl(MechanismDecl& m, JointDecl::Kind kind) {
        JointDecl j;
        j.kind = kind;
        j.line = cur().line;
        parseRef(j.bodyA, j.anchorA);
        expectSym("->");
        parseRef(j.bodyB, j.anchorB);
        m.joints.push_back(j);
    }

    void driveDecl(MechanismDecl& m) {
        DriveDecl d;
        Token name = expectIdent();
        d.body = name.text;
        d.line = name.line;

        if (acceptKw("torque")) {
            // energy injection: a pure generalized force, no constraint row
            d.isTorque = true;
            d.torque = parseNum();
            m.drives.push_back(d);
            return;
        }

        // M5: weight — an energy source with a budget
        if (cur().type == Tok::Ident && cur().text == "weight") {
            next();
            d.isWeight = true;
            d.weightMass = parseNum();
            if (d.weightMass <= 0.0) {
                fail("drive 'weight' must be a positive mass (kg)");
            }
            if (cur().type == Tok::Ident && cur().text == "kg") next();
            if (cur().type == Tok::Ident && cur().text == "barrel") {
                next();
                d.hasBarrel = true;
                d.barrelR = parseNum();
                if (d.barrelR <= 0.0) {
                    fail("drive 'barrel' must be a positive radius (m)");
                }
                if (cur().type == Tok::Ident && cur().text == "m") next();
            }
            if (cur().type == Tok::Ident && cur().text == "drop") {
                next();
                d.hasDrop = true;
                d.dropM = parseNum();
                if (d.dropM <= 0.0) {
                    fail("drive 'drop' must be a positive length (m)");
                }
                if (cur().type == Tok::Ident && cur().text == "m") next();
            }
            // sense: which way the cord unwinds the barrel. 'cw' is the
            // clock's forward direction (hands sweep clockwise = -theta in
            // the world frame); the default 'ccw' keeps +theta.
            if (cur().type == Tok::Ident && cur().text == "cw") {
                next();
                d.weightSign = -1;
            } else if (cur().type == Tok::Ident && cur().text == "ccw") {
                next();
                d.weightSign = 1;
            }
            m.drives.push_back(d);
            return;
        }

        expectKw("at");
        double rate = parseNum();

        if (cur().type == Tok::UnitDegPerSec) { d.omegaRad = deg2rad(rate); next(); }
        else if (cur().type == Tok::UnitRpm)   { d.omegaRad = rate * kPi / 30.0; next(); }
        else fail("expected unit 'deg/s' or 'rpm' after drive rate, got '" + cur().text + "'");

        if (acceptKw("from")) {
            d.theta0Rad = deg2rad(parseNum());
            d.hasFrom = true;
            if (cur().type == Tok::UnitDeg) { next(); }
            else fail("expected 'deg' after initial angle, got '" + cur().text + "'");
        }
        m.drives.push_back(d);
    }

    // reduce s1->s4 ratio 1:1800 [via s2, s3] [same|opposite]
    //       [module: 0.006] [spread: 30 deg]
    // 'ratio', 'via', 'same', 'opposite', 'module', 'spread' are contextual
    // identifiers, so they remain usable as ordinary names elsewhere.
    void reduceDecl(MechanismDecl& m) {
        ReduceDecl r;
        r.line = cur().line;
        r.bodyA = expectIdent().text;
        expectSym("->");
        r.bodyB = expectIdent().text;

        if (!(cur().type == Tok::Ident && cur().text == "ratio")) {
            fail("expected 'ratio' in reduce, got '" + cur().text + "'");
        }
        next();

        // ratio value: N  or  N:M  (the larger side is the fast side)
        const double a = parseNum();
        long long num = 1, den = 1;
        if (a <= 0.0) fail("ratio must be positive");
        if (acceptSym(":")) {
            const double b = parseNum();
            if (b <= 0.0) fail("ratio must be positive");
            // exact integers keep the tooth search exact
            num = std::llround(a);
            den = std::llround(b);
            if (std::fabs(a - (double)num) > 1e-9 || std::fabs(b - (double)den) > 1e-9) {
                fail("a ':' ratio needs integer sides (e.g. 1:1800); use a single "
                     "number for fractional ratios");
            }
        } else {
            // single number: keep it as an exact rational with 4 decimals
            double scaled = a * 10000.0;
            num = std::llround(scaled);
            den = 10000;
            if (std::fabs(scaled - (double)num) > 1e-6) {
                fail("ratio is too precise to synthesize exactly; use a ':' form");
            }
        }
        r.ratioNum = num;
        r.ratioDen = den;

        // optional clauses, any order
        bool sawVia = false, sawDir = false, sawModule = false, sawSpread = false;
        for (;;) {
            if (cur().type == Tok::Ident && cur().text == "via") {
                if (sawVia) fail("duplicate 'via' clause in reduce");
                sawVia = true;
                next();
                r.via.push_back(expectIdent().text);
                while (acceptSym(",")) r.via.push_back(expectIdent().text);
                // allow space-separated too: consume further bare idents
                while (cur().type == Tok::Ident && cur().text != "via" &&
                       cur().text != "same" && cur().text != "opposite" &&
                       cur().text != "module" && cur().text != "spread") {
                    r.via.push_back(expectIdent().text);
                    while (acceptSym(",")) r.via.push_back(expectIdent().text);
                }
            } else if (cur().type == Tok::Ident &&
                       (cur().text == "same" || cur().text == "opposite")) {
                if (sawDir) fail("duplicate direction clause in reduce");
                sawDir = true;
                r.dir = cur().text == "same" ? ReduceDecl::Dir::Same
                                             : ReduceDecl::Dir::Opposite;
                next();
            } else if (cur().type == Tok::Ident && cur().text == "module") {
                if (sawModule) fail("duplicate 'module' clause in reduce");
                sawModule = true;
                next();
                expectSym(":");
                r.module = parseNum();
                r.hasModule = true;
            } else if (cur().type == Tok::Ident && cur().text == "spread") {
                if (sawSpread) fail("duplicate 'spread' clause in reduce");
                sawSpread = true;
                next();
                expectSym(":");
                r.spreadDeg = parseNum();
                r.hasSpread = true;
                if (cur().type == Tok::UnitDeg) { next(); }
                else fail("expected 'deg' after spread angle, got '" + cur().text + "'");
            } else {
                break;
            }
        }

        m.reduces.push_back(r);
    }

    // hand second on s4 { length: 0.3 }   (host anchor optional, block optional)
    void handDecl(MechanismDecl& m) {
        HandDecl h;
        Token name = expectIdent();
        h.name = name.text;
        h.line = name.line;
        expectKw("on");
        h.host = expectIdent().text;
        if (acceptSym(".")) h.hostAnchor = expectIdent().text;
        if (acceptSym("{")) {
            while (!acceptSym("}")) {
                if (atEnd()) fail("unterminated '{' in hand '" + h.name + "'");
                const bool isLength =
                    (cur().type == Tok::Ident || cur().type == Tok::Kw) &&
                    cur().text == "length";
                if (isLength) {
                    next();
                    expectSym(":");
                    h.props["length"] = parseNum();
                } else {
                    fail("expected hand property ('length'), got '" + cur().text + "'");
                }
                skipSeparators();
            }
        }
        m.hands.push_back(h);
    }

    // dial { radius: 0.36, major: 12, minor: 60 }  [at frame.c]
    // (both the 'at' clause and the block are optional)
    void dialDecl(MechanismDecl& m) {
        DialDecl d;
        d.line = cur().line;
        if (cur().type == Tok::Kw && cur().text == "at") {
            next();
            parseRef(d.atBody, d.atAnchor);
            d.hasAt = true;
        }
        if (acceptSym("{")) {
            while (!acceptSym("}")) {
                if (atEnd()) fail("unterminated '{' in dial");
                if (cur().type == Tok::Ident &&
                    (cur().text == "radius" || cur().text == "major" ||
                     cur().text == "minor")) {
                    const std::string prop = cur().text;
                    next();
                    expectSym(":");
                    d.props[prop] = parseNum();
                } else {
                    fail("expected dial property ('radius', 'major' or 'minor'), got '"
                         + cur().text + "'");
                }
                skipSeparators();
            }
        }
        m.dials.push_back(d);
    }

    // M5 — the hybrid element:
    //   mode esc {
    //     state tick { hold escape }
    //     state run  { }
    //     tick -> run when pend.theta > -87.5 deg
    //     run  -> tock when escape.advance > 6 deg: impulse pend maintain 3 deg
    //   }
    // 'state', 'when', 'hold', 'impulse', 'maintain' are contextual
    // identifiers, so they remain usable as ordinary names elsewhere.
    void modeDecl(MechanismDecl& m) {
        ModeDecl md;
        Token name = expectIdent();
        md.name = name.text;
        md.line = name.line;
        expectSym("{");

        while (!acceptSym("}")) {
            if (atEnd()) fail("unterminated '{' in mode '" + md.name + "'");
            if (cur().type == Tok::Ident && cur().text == "state") {
                next();
                ModeDecl::State st;
                Token sn = expectIdent();
                st.name = sn.text;
                st.line = sn.line;
                expectSym("{");
                while (!acceptSym("}")) {
                    if (atEnd()) fail("unterminated '{' in state '" + st.name + "'");
                    if (cur().type == Tok::Ident && cur().text == "hold") {
                        next();
                        st.holds.push_back(expectIdent().text);
                    } else {
                        fail("expected 'hold <body>' in state '" + st.name +
                             "', got '" + cur().text + "'");
                    }
                    skipSeparators();
                }
                md.states.push_back(st);
            } else if (cur().type == Tok::Ident) {
                // transition:  from -> to when guard [: reset {, reset}]
                ModeDecl::Transition tr;
                Token from = expectIdent();
                tr.from = from.text;
                tr.line = from.line;
                expectSym("->");
                tr.to = expectIdent().text;

                if (!(cur().type == Tok::Ident && cur().text == "when")) {
                    fail("expected 'when' in transition of mode '" + md.name +
                         "', got '" + cur().text + "'");
                }
                next();

                // guard: body.field (>|<) num [deg | deg/s]
                tr.body = expectIdent().text;
                expectSym(".");
                Token fld = expectIdent();
                if (fld.text == "theta") {
                    tr.field = ModeDecl::Transition::Field::Theta;
                } else if (fld.text == "omega") {
                    tr.field = ModeDecl::Transition::Field::Omega;
                } else if (fld.text == "advance") {
                    tr.field = ModeDecl::Transition::Field::Advance;
                } else {
                    fail("unknown guard field '" + fld.text +
                         "' (expected 'theta', 'omega' or 'advance')");
                }
                if (acceptSym(">")) {
                    tr.greater = true;
                } else if (acceptSym("<")) {
                    tr.greater = false;
                } else {
                    fail("expected '>' or '<' in guard of mode '" + md.name +
                         "', got '" + cur().text + "'");
                }
                double v = parseNum();
                if (cur().type == Tok::UnitDeg) {
                    if (tr.field == ModeDecl::Transition::Field::Omega) {
                        fail("guard field 'omega' takes 'deg/s' (or raw rad/s), not 'deg'");
                    }
                    v = deg2rad(v);
                    next();
                } else if (cur().type == Tok::UnitDegPerSec) {
                    if (tr.field != ModeDecl::Transition::Field::Omega) {
                        fail("guard field '" + fld.text + "' takes 'deg' (or raw radians), "
                             "not 'deg/s'");
                    }
                    v = deg2rad(v);
                    next();
                }
                tr.threshold = v;

                // optional resets, applied at the crossing instant
                if (acceptSym(":")) {
                    for (;;) {
                        if (!(cur().type == Tok::Ident && cur().text == "impulse")) {
                            fail("expected reset 'impulse <body> ...' in transition of "
                                 "mode '" + md.name + "', got '" + cur().text + "'");
                        }
                        next();
                        ModeDecl::Reset rs;
                        rs.line = cur().line;
                        rs.body = expectIdent().text;
                        if (cur().type == Tok::Ident && cur().text == "maintain") {
                            next();
                            rs.kind = ModeDecl::Reset::Kind::Maintain;
                            const double sw = parseNum();
                            if (sw <= 0.0) {
                                fail("'maintain' needs a positive half-swing angle");
                            }
                            if (cur().type == Tok::UnitDeg) {
                                rs.swingDeg = sw;
                                next();
                            } else {
                                rs.swingDeg = rad2deg(sw);   // raw: radians
                            }
                        } else {
                            rs.kind = ModeDecl::Reset::Kind::Impulse;
                            rs.energy = parseNum();
                            if (cur().type == Tok::Ident && cur().text == "mJ") {
                                rs.energy *= 1.0e-3;
                                next();
                            }
                        }
                        tr.resets.push_back(rs);
                        if (!acceptSym(",")) break;
                    }
                }
                md.transitions.push_back(tr);
            } else {
                fail("expected 'state' or a transition ('a -> b when ...') in mode '" +
                     md.name + "', got '" + cur().text + "'");
            }
            skipSeparators();
        }
        m.modes.push_back(md);
    }
};

} // namespace

Program parse(const std::vector<Token>& tokens, const ParamOverrides& overrides) {
    Parser p(tokens, overrides);
    return p.program();
}

} // namespace mech
