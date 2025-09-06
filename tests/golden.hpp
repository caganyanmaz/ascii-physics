#pragma once
#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>

#ifndef GOLDEN_DIR
#  define GOLDEN_DIR "tests/golden"
#endif

namespace golden {
namespace fs = std::filesystem;

inline std::string full_path(const std::string& rel) {
    fs::path p(rel);
    if (!p.is_absolute()) {
        // allow callers to pass "sim/foo.txt" or "tests/golden/sim/foo.txt"
        if (p.string().rfind("tests/golden/", 0) == 0) {
            p = p.string().substr(std::string("tests/golden/").size());
        }
        p = fs::path(GOLDEN_DIR) / p;
    }
    return p.string();
}

inline std::string read_file(const std::string& rel) {
    std::ifstream in(full_path(rel), std::ios::binary);
    if (!in) throw std::runtime_error("Failed to open golden file: " + full_path(rel));
    std::ostringstream ss; ss << in.rdbuf();
    return ss.str();
}

inline void write_file(const std::string& rel, const std::string& content) {
    fs::path p(full_path(rel));
    fs::create_directories(p.parent_path());             // <-- ensure dirs exist
    std::ofstream out(p, std::ios::binary);
    if (!out) throw std::runtime_error("Failed to write golden file: " + p.string());
    out << content;
}

inline void verify(const std::string& rel, const std::string& got) {
#ifdef UPDATE_GOLDEN
    write_file(rel, got);
    GTEST_SKIP() << "Golden updated: " << full_path(rel);
#else
    const std::string want = read_file(rel);
    ASSERT_EQ(got, want) << "Golden mismatch for " << full_path(rel);
#endif
}
} // namespace golden