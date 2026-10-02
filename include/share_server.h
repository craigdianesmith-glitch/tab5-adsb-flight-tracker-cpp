#pragma once

#include <Arduino.h>

// A small web server for getting videos and recordings off the card over WiFi:
// a page listing them, with each video playable in the browser, downloadable
// and deletable, and each recording downloadable. Meant for a phone on the
// same network - a QR code on the share screen leads to it.
//
// There is no password, so it only runs while the share screen is up: start()
// on the way in, stop() on the way out. It has a task of its own, so a
// download doesn't hold up the screen, and it takes the recorder's lock a
// chunk at a time, so recording carries on while it serves.
namespace share {

bool start();  // false if WiFi isn't up or the server won't start
void stop();
bool running();

// http://<address>/ - always works on the local network.
String url();
// http://overhead.local/, or "" if mDNS didn't start. Easier to type, but not
// every phone or network resolves .local names.
String localUrl();

// For the share screen to say what is going on.
struct Status {
    uint32_t requests;     // served since start()
    char current[48];      // the file being sent, or "" between transfers
    uint32_t sent, total;  // of the current file, in bytes
};
Status status();

}  // namespace share
