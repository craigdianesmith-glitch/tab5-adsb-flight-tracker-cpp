#pragma once

#include <Arduino.h>

// The WiFi watchdog: for a link that has gone but that nothing on the Tab5
// has noticed.
//
// The Tab5's WiFi is an ESP32-C6 of its own, reached over SDIO through
// ESP-Hosted, and WiFi.status() is a copy the P4 keeps of what the C6 last
// told it. It changes only when an event comes across. On 26 Oct the C6 left
// the network three times - the router logged it going, "disassociated due
// to inactivity", at a strong signal - and the event saying so never reached
// the P4. WiFi.status() stayed WL_CONNECTED, so ensureWifi() in main.cpp
// never reconnected and nothing else ever looked: every poll failed its name
// lookup for hours, until the Tab5 was restarted. The likeliest cause is a
// known fault of ESP-Hosted 2.12.13, the version in this framework
// (esp-hosted-mcu #243): one failed allocation of internal DMA memory, under
// the 3D view's heaviest load, and the P4 stops reading from the C6 for
// good, without a word in the log - and with nothing read, the C6's own WiFi
// stalls behind it. Only resetting both ends brings it back, and on this
// version of ESP-Hosted that can only be done safely by restarting.
//
// So, from the poll task, three things that do not rely on the events:
//
// - The link's truth, asked of the C6 itself about every 10s while the P4
//   believes it is connected: whether it is still associated. Two answers of
//   "no" in a row mean the link is down; no answer at all in the request's
//   5s means the link to the C6 itself is stuck.
// - The feed: requests failing at the network - see adsb_client.h's
//   AdsbFeedHealth - three times running over a minute while the link looks
//   up. Unless the router still answers, in which case the trouble is beyond
//   it - the internet, or the providers - and rejoining would not help, so
//   the WiFi is left alone; though after half an hour of that the Tab5 is
//   restarted all the same, in case what is stuck is its own networking.
// - The network taking the Tab5 back again and again without ever giving it
//   an address: the rejoining every 12s the router logged on 24 Oct, which
//   ensureWifi() and the framework's own reconnect would otherwise keep up
//   for as long as the Tab5 was left.
//
// Any of them starts a ladder, each step only once the one before has been
// given time and failed: 1, the network rejoined; 2, the C6's WiFi stopped
// and started again; 3, last of all and not for several minutes, the Tab5
// restarted - which resets the C6 too, its power cut for half a second first
// to be sure of it. A step between 2 and 3 that reset the C6 and brought
// ESP-Hosted's link up again without restarting would be gentler, but on
// 2.12.13 taking ESP-Hosted down corrupts the heap (fixed in 2.13.1), so
// there is none. The steps are spaced and counted, so a network that is
// simply gone is never hammered, and the WiFi screen - or the screen it is
// about to put up - always has the radio to itself. The restart is the
// loop's to make, when nothing would be lost by it: never while a recording
// is being written, a replay is playing or the WiFi screen is open.
//
// No log line here names the network.
namespace wifiwatch {

// What it needs from main.cpp, which keeps these under its lock.
struct Hooks {
    // Whether the WiFi screen has the radio: everything here stands down.
    bool (*uiOwnsWifi)();
    // The network to join; false if there isn't one.
    bool (*credentials)(String &ssid, String &pass);
};

// Early in setup(), before the splash: says what started the Tab5 this time,
// and whether it was this watchdog's restart - true then, for the boot to be
// a quiet one, without the take-off.
bool noteBoot();

// In setup(), straight after WiFi.mode(WIFI_STA) has brought ESP-Hosted up:
// the line with both ends' ESP-Hosted versions, and the handlers the
// watchdog keeps its count of the link's events with.
void begin(const Hooks &hooks);

// From the poll task after every poll, and in every slice of its wait. Most
// calls return at once; a check of the link takes up to the C6's 5s, and a
// step of the ladder up to half a minute.
void tick();

// For loop(): whether a restart has been asked for. If now is a time it may
// happen, restartNow(); otherwise asked again on the next pass.
bool restartWanted();
const char *restartReason();
[[noreturn]] void restartNow();

// The bench's commands - "##!" and a letter, in BENCH3D builds: see the list
// in main.cpp. Carried out on the poll task, where the C6 is asked; so within
// 200ms while the poll task is waiting between polls, after a poll otherwise.
void benchCommand(char c);

}  // namespace wifiwatch
