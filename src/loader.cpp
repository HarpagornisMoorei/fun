// mech — loader.cpp: `import` resolution on the token stream (M10)
#include "loader.hpp"
#include "parser.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

namespace mech {

namespace {

namespace fs = std::filesystem;

struct Ctx {
    std::vector<std::string> files;
    std::set<std::string> done;            // canonical paths already spliced in
    std::vector<std::string> active;       // canonical paths being loaded (cycle detection)
};

[[noreturn]] void importError(const Token& t, const std::string& msg) {
    throw ParseError{t.line, t.col, msg};
}

std::vector<std::string> searchDirs(const std::string& importer) {
    std::vector<std::string> dirs;
    if (!importer.empty()) {
        const fs::path d = fs::path(importer).parent_path();
        dirs.push_back(d.empty() ? "." : d.string());
    } else {
        dirs.push_back(".");
    }
    if (const char* mp = std::getenv("MECH_PATH")) {
        std::stringstream ss(mp);
        std::string d;
        while (std::getline(ss, d, ':')) if (!d.empty()) dirs.push_back(d);
    }
    return dirs;
}

bool readText(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::stringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return true;
}

std::string canonicalOf(const std::string& path) {
    std::error_code ec;
    const fs::path c = fs::weakly_canonical(fs::path(path), ec);
    return ec ? path : c.string();
}

// Tokens of one file with its imports spliced in, lines tagged with `fileIdx`, no End token.
std::vector<Token> loadFile(const std::string& text, const std::string& path, int fileIdx, bool library, Ctx& cx) {
    std::vector<Token> toks;
    try {
        toks = lex(text);
    } catch (const LexError& e) {
        throw LexError{fileIdx * kFileStride + e.line, e.col, e.msg};
    }
    for (Token& t : toks) t.line += fileIdx * kFileStride;
    toks.pop_back();   // the End token; the caller puts one back

    std::vector<Token> out;
    int depth = 0;
    for (size_t i = 0; i < toks.size(); ++i) {
        const Token& t = toks[i];
        if (t.type == Tok::Sym && t.text == "{") ++depth;
        if (t.type == Tok::Sym && t.text == "}") --depth;

        if (depth == 0 && t.type == Tok::Kw && t.text == "mechanism" && library)
            importError(t, "a library file contains parts only; 'mechanism' belongs in the file that imports it");

        if (depth == 0 && t.type == Tok::Ident && t.text == "import") {
            if (i + 1 >= toks.size() || toks[i + 1].type != Tok::Str)
                importError(t, "expected a quoted path after 'import', e.g. import \"lib/plate_cam.mech\"");
            const Token& sp = toks[i + 1];
            const std::string& rel = sp.text;
            if (rel.size() < 6 || rel.compare(rel.size() - 5, 5, ".mech") != 0)
                importError(sp, "an import must name a .mech file, got \"" + rel + "\"");

            std::string found, body, tried;
            for (const std::string& dir : searchDirs(path)) {
                const std::string cand = (fs::path(dir) / rel).lexically_normal().string();
                tried += (tried.empty() ? "" : ", ") + cand;
                if (readText(cand, body)) { found = cand; break; }
            }
            if (found.empty()) importError(sp, "cannot find \"" + rel + "\" (looked in: " + tried + ")");

            const std::string canon = canonicalOf(found);
            for (size_t k = 0; k < cx.active.size(); ++k) {
                if (cx.active[k] != canon) continue;
                std::string chain;
                for (size_t j = k; j < cx.active.size(); ++j) chain += fs::path(cx.active[j]).filename().string() + " -> ";
                importError(sp, "import cycle: " + chain + fs::path(canon).filename().string());
            }
            if (cx.done.insert(canon).second) {
                const int idx = (int)cx.files.size();
                cx.files.push_back(found);
                cx.active.push_back(canon);
                const std::vector<Token> sub = loadFile(body, found, idx, true, cx);
                cx.active.pop_back();
                out.insert(out.end(), sub.begin(), sub.end());
            }
            ++i;   // the string
            continue;
        }
        out.push_back(t);
    }
    return out;
}

} // namespace

LoadedSource loadSource(const std::string& source, const std::string& path, std::vector<std::string>* files) {
    Ctx cx;
    cx.files.push_back(path);
    if (!path.empty()) {
        cx.active.push_back(canonicalOf(path));
        cx.done.insert(canonicalOf(path));
    }
    LoadedSource ls;
    try {
        ls.tokens = loadFile(source, path, 0, false, cx);
    } catch (...) {
        if (files) *files = cx.files;
        throw;
    }
    Token end;
    end.type = Tok::End;
    end.line = 1 + (int)std::count(source.begin(), source.end(), '\n');
    end.col = 1;
    ls.tokens.push_back(end);
    ls.files = cx.files;
    if (files) *files = cx.files;
    return ls;
}

std::string describeLine(const std::vector<std::string>& files, int taggedLine) {
    const size_t f = (size_t)fileOfLine(taggedLine);
    const std::string name = f < files.size() && !files[f].empty() ? files[f] : std::string("<input>");
    return name + ":" + std::to_string(lineInFile(taggedLine));
}

} // namespace mech
