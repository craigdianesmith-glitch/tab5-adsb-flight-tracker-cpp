#pragma once

#include <Arduino.h>

#include "settings.h"

enum class SettingsAction {
    NONE,
    BACK,           // caller should apply and persist the values below
    OPEN_WIFI,
    OPEN_LOCATION,
    OPEN_ALERTS,
};

// Seeds the screen with the values currently in force. The label is only
// displayed - the location itself is changed on the location screen, which
// this screen is now the only way into.
void settingsScreenSet(TrafficFilter traffic, int radiusNm, bool showRefresh, int pollIntervalS,
                       const String &locationLabel, AdsbSource source);

// Whether the radar's navaids switch is on, to seed it and read it back.
void settingsScreenSetNavaids(bool on);
bool settingsScreenNavaids();

// Updates the location shown on the button, after the location screen has
// been in and changed it.
void settingsScreenSetLocation(const String &locationLabel);

// The one-line summary shown on the alerts button, e.g. "3 of 4 on, auto-record on".
void settingsScreenSetAlertSummary(const String &summary);

void settingsScreenDraw();

// pressed/clicked both matter here: the radius slider tracks a held finger,
// everything else acts on a completed tap.
SettingsAction settingsScreenHandleTouch(int x, int y, bool pressed, bool clicked);

TrafficFilter settingsScreenTraffic();
int settingsScreenRadius();
bool settingsScreenShowRefresh();
int settingsScreenPollInterval();
AdsbSource settingsScreenSource();
