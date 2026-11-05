#pragma once

#include <cmath>
#include <cstdio>
#include <exception>
#include <functional>
#include <string>
#include <vector>

// A minimal test harness, since the project has no test framework dependency.
namespace testing {

struct TestCase {
    std::string name;
    std::function<void()> body;
};

inline std::vector<TestCase> &registry() {
    static std::vector<TestCase> tests;
    return tests;
}

inline int &currentFailures() {
    static int failures = 0;
    return failures;
}

struct Registrar {
    Registrar(const char *name, std::function<void()> body) {
        registry().push_back({name, std::move(body)});
    }
};

inline void reportFailure(const char *file, int line, const std::string &message) {
    std::printf("    %s:%d: %s\n", file, line, message.c_str());
    currentFailures()++;
}

inline int runAll() {
    int failed = 0;
    for (const TestCase &test : registry()) {
        currentFailures() = 0;
        try {
            test.body();
        } catch (const std::exception &e) {
            std::printf("    unexpected exception: %s\n", e.what());
            currentFailures()++;
        }
        std::printf("[%s] %s\n", currentFailures() == 0 ? "PASS" : "FAIL", test.name.c_str());
        if (currentFailures() != 0) failed++;
    }
    std::printf("%zu tests, %d failed\n", registry().size(), failed);
    return failed == 0 ? 0 : 1;
}

}  // namespace testing

#define TEST(name)                                                         \
    static void test_##name();                                             \
    static testing::Registrar registrar_##name(#name, test_##name);        \
    static void test_##name()

// Variadic, so that braced initializers containing commas can be passed.
#define CHECK(...)                                                         \
    do {                                                                   \
        if (!(__VA_ARGS__)) testing::reportFailure(__FILE__, __LINE__, #__VA_ARGS__); \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                            \
    do {                                                                   \
        double a_ = (actual);                                              \
        double e_ = (expected);                                            \
        if (!(std::abs(a_ - e_) <= (tolerance))) {                         \
            testing::reportFailure(__FILE__, __LINE__,                     \
                                   std::string(#actual " = ") + std::to_string(a_) + \
                                       ", expected " + std::to_string(e_)); \
        }                                                                  \
    } while (0)

#define CHECK_THROWS(exception, ...)                                       \
    do {                                                                   \
        bool thrown_ = false;                                              \
        try {                                                              \
            (void)(__VA_ARGS__);                                           \
        } catch (const exception &) {                                      \
            thrown_ = true;                                                \
        }                                                                  \
        if (!thrown_) {                                                    \
            testing::reportFailure(__FILE__, __LINE__, #__VA_ARGS__ " did not throw"); \
        }                                                                  \
    } while (0)
