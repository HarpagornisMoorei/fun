// mech — parser.hpp: recursive-descent parser for the .mech language (M1 subset)
#pragma once
#include "ast.hpp"
#include "lexer.hpp"

namespace mech {

struct ParseError {
    int line, col;
    std::string msg;
};

// Parse a token stream into a Program. Throws ParseError.
Program parse(const std::vector<Token>& tokens);

} // namespace mech
