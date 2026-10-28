#pragma once

#include <Arduino.h>

#include "config.h"

enum class AlertsAction {
    NONE,
    BACK,  // caller should apply and persist the values below
    OPEN_WATCHLIST,
};

// Seeds the screen with the values in force. Reached from the settings screen.
void alertsScreenSet(uint8_t alertMask, bool autoRecord, bool autoFollow, bool followIn3d, const String &watchlist);

// Whether there is a card to record to. Without one the auto-record switch is
// greyed out and can't be changed, though what it was set to is kept.
void alertsScreenSetCard(bool present);

// The watchlist as it came back from the watchlist screen.
void alertsScreenSetWatchlist(const String &watchlist);

// Read aloud - see sound.h.
void alertsScreenSetReadAloud(bool readAloud);

void alertsScreenDraw();
AlertsAction alertsScreenHandleTouch(int x, int y);

uint8_t alertsScreenMask();
bool alertsScreenAutoRecord();
bool alertsScreenAutoFollow();
// Where an auto-follow opens: the 3D view, or the radar.
bool alertsScreenFollowIn3d();
String alertsScreenWatchlist();
bool alertsScreenReadAloud();
