#pragma once

#include <Arduino.h>

// The on-screen keyboard shared by the location search and the WiFi password
// entry. Location search only ever needed uppercase letters; a WPA passphrase
// needs case, digits and punctuation, so rather than keep two keyboards this
// one carries all three modes and both screens use it.
namespace keyboard {

enum class Result {
    NONE,     // touch wasn't on the keyboard, or was on a dead cell
    EDITED,   // text changed - redraw whatever displays it
    SUBMIT,   // OK pressed
};

constexpr int TOP_Y = 436;
constexpr int HEIGHT = 284;

void reset();  // back to lowercase
void draw();
bool contains(int x, int y);

// Applies the key at (x, y) to text. maxLen caps growth: 32 suits an SSID,
// 63 a WPA passphrase.
Result handleTouch(int x, int y, String &text, size_t maxLen);

}  // namespace keyboard
