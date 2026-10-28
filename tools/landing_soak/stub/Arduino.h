// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's take-off, landing and taxi simulation: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

// Just enough of Arduino for the firmware's follow, telemetry, airports,
// runways and ground code to build on the PC: String, Serial, millis() - on
// the harness's own clock, set each frame - and constrain().
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

class String : public std::string {
public:
    String() {}
    String(const char *s) : std::string(s ? s : "") {}
    String(const std::string &s) : std::string(s) {}
    String(char c) : std::string(1, c) {}
    String(int v) : std::string(std::to_string(v)) {}
    String(unsigned v) : std::string(std::to_string(v)) {}
    String(long v) : std::string(std::to_string(v)) {}
    String(unsigned long v) : std::string(std::to_string(v)) {}
    String(float v, int places = 2) {
        char b[32];
        snprintf(b, sizeof b, "%.*f", places, v);
        assign(b);
    }
    String(double v, int places = 2) {
        char b[32];
        snprintf(b, sizeof b, "%.*f", places, v);
        assign(b);
    }
    long toInt() const { return atol(c_str()); }
    float toFloat() const { return (float)atof(c_str()); }
    unsigned length() const { return (unsigned)size(); }
    bool startsWith(const char *p) const { return compare(0, strlen(p), p) == 0; }
    bool startsWith(const String &p) const { return compare(0, p.size(), p) == 0; }
    bool endsWith(const char *p) const {
        size_t n = strlen(p);
        return size() >= n && compare(size() - n, n, p) == 0;
    }
    void trim() {
        size_t a = find_first_not_of(" \t\r\n");
        size_t b = find_last_not_of(" \t\r\n");
        if (a == std::string::npos) {
            clear();
        } else {
            assign(substr(a, b - a + 1));
        }
    }
    int indexOf(char c) const {
        size_t i = find(c);
        return i == std::string::npos ? -1 : (int)i;
    }
    int indexOf(const char *s) const {
        size_t i = find(s);
        return i == std::string::npos ? -1 : (int)i;
    }
    String substring(unsigned from) const { return from >= size() ? String() : String(substr(from)); }
    String substring(unsigned from, unsigned to) const {
        return from >= size() ? String() : String(substr(from, to > from ? to - from : 0));
    }
    char charAt(unsigned i) const { return i < size() ? (*this)[i] : 0; }
    void toUpperCase() {
        for (char &c : *this) c = (char)toupper((unsigned char)c);
    }
    bool equals(const String &o) const { return *this == o; }
};

inline String operator+(const String &a, const String &b) { return String(static_cast<const std::string &>(a) + b); }
inline String operator+(const String &a, const char *b) { return String(static_cast<const std::string &>(a) + b); }
inline String operator+(const char *a, const String &b) { return String(a + static_cast<const std::string &>(b)); }

// The harness's clock: millis() is whatever the harness says it is now.
extern uint32_t g_harnessMs;
inline uint32_t millis() { return g_harnessMs; }
inline uint32_t micros() { return g_harnessMs * 1000u; }

// Serial: quiet unless the harness asks for the firmware's own log lines.
extern bool g_harnessSerial;
struct HarnessSerial {
    int printf(const char *fmt, ...) {
        if (!g_harnessSerial) return 0;
        va_list ap;
        va_start(ap, fmt);
        int n = vprintf(fmt, ap);
        va_end(ap);
        return n;
    }
    void println(const char *s) {
        if (g_harnessSerial) puts(s);
    }
    void println(const String &s) { println(s.c_str()); }
    void print(const char *s) {
        if (g_harnessSerial) fputs(s, stdout);
    }
};
extern HarnessSerial Serial;

template <typename T, typename L, typename H> T constrain(T v, L lo, H hi) {
    return v < (T)lo ? (T)lo : v > (T)hi ? (T)hi : v;
}

inline size_t strlcpy(char *d, const char *s, size_t n) {
    size_t l = strlen(s);
    if (n) {
        size_t c = l < n - 1 ? l : n - 1;
        memcpy(d, s, c);
        d[c] = 0;
    }
    return l;
}
