// mech — loader.hpp: source files and `import` (M10)
//
//   import "lib/plate_cam.mech"
//
// An import is resolved before parsing, on the token stream: the parts of the named
// file replace the statement, exactly as if they had been written there. The parser
// never sees a second file; what the loader adds is that every token remembers which
// file it came from, so an error inside a library points at the library.
//
// Rules:
//  * the path must end in ".mech", and is looked for next to the importing file, then in
//    each directory of the MECH_PATH environment variable (colon-separated);
//  * a library file contains parts (and imports of other libraries) and nothing else;
//  * a file is imported once however many files import it; a file that (indirectly)
//    imports itself is an error.
#pragma once
#include "lexer.hpp"
#include <string>
#include <vector>

namespace mech {

// A token's line carries its file: line = file * kFileStride + line-in-file. File 0 is the
// file being loaded, so single-file programs see ordinary line numbers.
constexpr int kFileStride = 1 << 20;
inline int fileOfLine(int line) { return line / kFileStride; }
inline int lineInFile(int line) { return line % kFileStride; }

struct LoadedSource {
    std::vector<Token> tokens;
    std::vector<std::string> files;   // [0] = the path given (may be empty); the rest in import order
};

// Lex `source` (the text of `path`) and splice in its imports. `path` anchors relative
// imports; "" means the current directory. Throws LexError for a lexical error in any
// file, and ParseError (declared in parser.hpp) for a bad import; both carry file-tagged lines.
// `files`, when given, is filled even if loading throws, so the error can be located.
LoadedSource loadSource(const std::string& source, const std::string& path,
                        std::vector<std::string>* files = nullptr);

// "name:line" for a file-tagged line, for messages that have no ToolError to carry it.
std::string describeLine(const std::vector<std::string>& files, int taggedLine);

} // namespace mech
