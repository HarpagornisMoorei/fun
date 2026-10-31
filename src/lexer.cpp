// mech — lexer.cpp: tokenizer for the .mech language (M3 subset)
#include "lexer.hpp"

#include <cctype>
#include <cstdlib>
#include <unordered_set>

namespace mech {

namespace {

const std::unordered_set<std::string>& keywords() {
    static const std::unordered_set<std::string> kw = {
        "mechanism", "ground", "rod", "anchor", "at", "pin", "drive",
        "length", "from",
        // M2
        "body", "shaft", "gear", "mesh", "slide", "on",
        // M3
        "world", "spring", "torque",
        // M4
        "reduce", "hand", "dial",
        // M5
        "mode",
        // M6
        "profile", "touch",
    };
    return kw;
}

inline bool isIdentStart(unsigned char c) { return std::isalpha(c) || c == '_'; }
inline bool isIdentChar(unsigned char c) { return std::isalnum(c) || c == '_'; }

} // namespace

std::vector<Token> lex(const std::string& src) {
    std::vector<Token> out;
    size_t i = 0;
    int line = 1, col = 1;

    auto advance = [&](size_t count = 1) {
        for (size_t k = 0; k < count; ++k) {
            if (src[i] == '\n') { ++line; col = 1; }
            else ++col;
            ++i;
        }
    };

    while (i < src.size()) {
        // whitespace
        unsigned char c = (unsigned char)src[i];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') { advance(); continue; }

        // comments: '//' and '#' to end of line
        if (c == '#' || (c == '/' && i + 1 < src.size() && src[i + 1] == '/')) {
            while (i < src.size() && src[i] != '\n') advance();
            continue;
        }

        int tl = line, tc = col;

        // multi-char unit tokens (checked before identifiers so 'deg'/'rpm'
        // never lex as plain identifiers when standing alone)
        auto tryUnit = [&](const char* lit, Tok type) -> bool {
            size_t n = std::char_traits<char>::length(lit);
            if (src.compare(i, n, lit) != 0) return false;
            if (i + n < src.size() && isIdentChar((unsigned char)src[i + n])) return false;
            Token t; t.type = type; t.text = lit; t.line = tl; t.col = tc;
            out.push_back(t); advance(n);
            return true;
        };
        if (tryUnit("deg/s", Tok::UnitDegPerSec)) continue;
        if (tryUnit("deg", Tok::UnitDeg)) continue;
        if (tryUnit("rpm", Tok::UnitRpm)) continue;

        // identifiers / keywords
        if (isIdentStart(c)) {
            size_t s = i;
            while (i < src.size() && isIdentChar((unsigned char)src[i])) advance();
            Token t;
            t.text = src.substr(s, i - s);
            t.type = keywords().count(t.text) ? Tok::Kw : Tok::Ident;
            t.line = tl; t.col = tc;
            out.push_back(t);
            continue;
        }

        // numbers
        if (std::isdigit(c)) {
            size_t s = i;
            while (i < src.size() && std::isdigit((unsigned char)src[i])) advance();
            if (i < src.size() && src[i] == '.' &&
                i + 1 < src.size() && std::isdigit((unsigned char)src[i + 1])) {
                advance();
                while (i < src.size() && std::isdigit((unsigned char)src[i])) advance();
            }
            Token t;
            t.type = Tok::Number;
            t.text = src.substr(s, i - s);
            t.num = std::strtod(t.text.c_str(), nullptr);
            t.line = tl; t.col = tc;
            out.push_back(t);
            continue;
        }

        // "text": a string, for the path of an `import`. No escapes, no line breaks.
        if (c == '"') {
            advance();
            const size_t s0 = i;
            while (i < src.size() && src[i] != '"' && src[i] != '\n') advance();
            if (i >= src.size() || src[i] != '"')
                throw LexError{tl, tc, "unterminated string (a string ends with '\"' on the same line)"};
            Token t;
            t.type = Tok::Str;
            t.text = src.substr(s0, i - s0);
            t.line = tl; t.col = tc;
            advance();
            out.push_back(t);
            continue;
        }

        // arrow before single '-'
        if (src.compare(i, 2, "->") == 0) {
            Token t; t.type = Tok::Sym; t.text = "->"; t.line = tl; t.col = tc;
            out.push_back(t); advance(2);
            continue;
        }

        // single-character punctuation ('<' and '>' are M5 guard comparators;
        // '+' '*' '/' '=' are the M7 expression operators and `param name = ...`;
        // a '/' that starts "//" was taken as a comment above, and "deg/s" as a unit)
        if (std::string("{}(),:;.-<>+*/=[]").find((char)c) != std::string::npos) {
            Token t; t.type = Tok::Sym; t.text = std::string(1, (char)c);
            t.line = tl; t.col = tc;
            out.push_back(t); advance();
            continue;
        }

        throw LexError{tl, tc, std::string("unexpected character '") + (char)c + "'"};
    }

    Token end;
    end.type = Tok::End;
    end.line = line; end.col = col;
    out.push_back(end);
    return out;
}

} // namespace mech
