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
// (esp-hosted-mcu #243): internal DMA memory short under the 3D view's
// heaviest load, and the P4 stops reading from the C6, without a word in the
// log - and with nothing read, the C6's own WiFi stalls behind it. The
// bench's "##!m" tries that for real and "##!z" makes the stall itself; see
// benchCommand(). Only resetting both ends brings it back, and on this
// version of ESP-Hosted that can only be done safely by restarting. That
// allocation is now never let fail - see hosted_rx_rescue.c - and this
// watchdog stays for whatever else leaves the link stuck.
//
// So, from the poll task, two things that do not rely on the events:
//
// - The link's truth, asked of the C6 itself about every 10s while the P4
//   believes it is connected: whether it is still associated. Two answers of
//   "no" in a row mean the link is down; no answer at all in the request's
//   5s means the link to the C6 itself is stuck.
// - The feed: requests failing at the network - see adsb_client.h's
//   AdsbFeedHealth - three times running over a minute while the link looks
//   up. Unless the router still answers: then the WiFi works, both ways
//   through the C6, and the trouble is beyond it - the internet, or the
//   providers - which neither a rejoin nor a restart can mend. The WiFi is
//   left alone for as long as that lasts, however long, and it is said once.
//
// Either starts a ladder, each step only once the one before has been given
// time and failed: 1, the network rejoined; 2, the C6's WiFi stopped and
// started again; 3, last of all and not for several minutes, the Tab5
// restarted - which resets the C6 too, its power cut for half a second first
// to be sure of it. A step between 2 and 3 that reset the C6 and brought
// ESP-Hosted's link up again without restarting would be gentler, but on
// 2.12.13 taking ESP-Hosted down corrupts the heap (fixed in 2.13.1), so
// there is none. A C6 that has answered neither of the last two requests
// made of it - the check's and then the rejoin's own - hears nothing at all,
// and every step is another request it cannot hear: the restart is asked for
// at once. The steps are spaced and counted, so a network that is simply
// gone is never hammered.
//
// The restart is the loop's to make, when it costs least: see
// checkWatchdogRestart() in main.cpp. One for the WiFi down waits while a
// replay is on the screen, the WiFi screen is open, a phone is using the
// share page or the screen has just been touched. It does not wait for a
// recording: with the feed down a recording gets nothing, so it is ended
// cleanly first.
//
// A request that went unanswered may yet be answered, late, and in 2.12.13 a
// late answer is taken by the next request made, whatever it was, and that
// one's by the one after: every answer out of step from then on, for every
// task that asks the C6 anything, until a restart. So once one has gone
// unanswered the link check stops, and when the trouble is over a restart is
// still wanted - not at once, the WiFi working, but at a quiet moment, with
// nobody using the device and nothing being recorded or followed; or just
// before the WiFi screen opens, whose own requests would otherwise be at
// risk of another's answer.
//
// While the WiFi screen has the radio - or is about to, ensureWifi() having
// given up on the network - everything here stands down. And every request
// the poll task makes of the C6 is made holding its turn at it - see
// RadioTurn - which the screen waits to be free before its first: the two
// never have requests in flight at once.
//
// No log line here names the network.
namespace wifiwatch {

// What it needs from main.cpp, which keeps these under its lock.
struct Hooks {
    // Whether the WiFi screen has the radio, or is about to - ensureWifi()
    // having given up on the network: everything here stands down.
    bool (*uiOwnsWifi)();
    // The network to join; false if there isn't one.
    bool (*credentials)(String &ssid, String &pass);
};

// Early in setup(), before the splash: says what started the Tab5 this time,
// and whether it was this watchdog's restart - true then, for the boot to be
// a quiet one, without the take-off.
bool noteBoot();
// After noteBoot(): whether this boot follows a restart made just as the
// WiFi screen was being opened - see restartBeforeWifiScreen() - so the
// screen should be put up now.
bool bootOpensWifiScreen();

// In setup(), straight after WiFi.mode(WIFI_STA) has brought ESP-Hosted up:
// the line with both ends' ESP-Hosted versions, and the handlers the
// watchdog keeps its count of the link's events with.
void begin(const Hooks &hooks);

// From the poll task after every poll, and in every slice of its wait. Most
// calls return at once; a check of the link takes up to the C6's 5s, and a
// step of the ladder up to half a minute.
void tick();

// The poll task's turn at the C6, held for as long as the object lives:
// every request the poll task makes of the C6 - the watchdog's, and
// ensureWifi()'s disconnect and begin - is made holding it. Whoever holds it
// should then look at whether the WiFi screen has claimed the radio, and ask
// nothing if it has. Only for the poll task, which is the only task that
// waits for it.
class RadioTurn {
  public:
    RadioTurn();
    ~RadioTurn();
    RadioTurn(const RadioTurn &) = delete;
    RadioTurn &operator=(const RadioTurn &) = delete;
};

// Whether no request of the poll task's is in flight to the C6. For the WiFi
// screen, which, once it has claimed the radio, holds back its first
// request until this says so: from then on the poll task asks nothing, as
// everything it does looks at the claim with its turn held. Never waits.
bool radioFree();

// The network the C6 last said it had joined, from its own "connected"
// event rather than asked of it, or "" once it has said it left; and, if
// given somewhere to put it, how many joinings there have been since boot.
// For the WiFi screen, in place of WiFi.SSID(), which asks the C6 and copies
// what comes back through a pointer that, in another request's answer, is
// nothing.
String joinedNetwork(uint32_t *joinings = nullptr);

// The restart asked for, if any. NOW: the WiFi is down and nothing else has
// mended it. WHEN_IDLE: the WiFi works, but a request to the C6 went
// unanswered earlier, and its answers may be out of step until a restart -
// one at a quiet moment.
enum class Restart : uint8_t { NONE, NOW, WHEN_IDLE };
// For loop(): if now is a time it may happen, restartNow(); otherwise asked
// again on the next pass.
Restart restartWanted();
const char *restartReason();
// Whether a restart should come before the WiFi screen opens: one is wanted
// and a request to the C6 has gone unanswered, whose late answer the
// screen's own requests could take. restartNow(true) then, for the boot to
// open the screen.
bool restartBeforeWifiScreen();
[[noreturn]] void restartNow(bool thenWifiScreen = false);

// The bench's commands - "##!" and a letter, in BENCH3D builds: see the list
// in main.cpp. Carried out on the poll task, where the C6 is asked; so within
// 200ms while the poll task is waiting between polls, after a poll otherwise.
// Except 'z' and 'Z', which stop and start ESP-Hosted's reads at once, from
// the loop, as they must be free to while the poll task waits on the C6.
void benchCommand(char c);

}  // namespace wifiwatch
