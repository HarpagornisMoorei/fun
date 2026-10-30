// mech — lexer.hpp: tokens for the .mech language (M1 subset)
#pragma once
#include <string>
#include <vector>

namespace mech {

enum class Tok : int {
    End,            // end of input
    Ident,          // identifier
    Number,         // numeric literal
    Kw,             // keyword (mechanism/ground/rod/anchor/at/pin/drive/length/from)
    Sym,            // punctuation: { } ( ) , : ; . - ->
    UnitDegPerSec,  // deg/s
    UnitRpm,        // rpm
    UnitDeg,        // deg
};

struct Token {
    Tok type = Tok::End;
    std::string text;   // raw text (for Ident/Kw/Sym) or spelled number
    double num = 0.0;   // value when type == Number
    int line = 1, col = 1;
};

struct LexError {
    int line, col;
    std::string msg;
};

// Tokenize source text. Throws LexError on illegal characters.
std::vector<Token> lex(const std::string& src);

} // namespace mech
