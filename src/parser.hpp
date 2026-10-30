// mech — parser.hpp: recursive-descent parser for the .mech language (M1 subset)
#pragma once
#include "ast.hpp"
#include "lexer.hpp"
#include <map>

namespace mech {

struct ParseError {
    int line, col;
    std::string msg;
};

// Values for `param`s that replace the ones written in the file (`--set name=value`).
using ParamOverrides = std::map<std::string, double>;

// Parse a token stream into a Program. Throws ParseError. An override naming a
// parameter the file does not declare is an error (a typo must not pass silently).
Program parse(const std::vector<Token>& tokens, const ParamOverrides& overrides = {});

} // namespace mech
