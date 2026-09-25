#pragma once

#include <Arduino.h>

#include "settings.h"

enum class SettingsAction {
    NONE,
    BACK,           // caller should apply and persist the values below
    OPEN_WIFI,
    OPEN_LOCATION,
};

// Seeds the screen with the values currently in force. The label is only
// displayed - the location itself is changed on the location screen, which
// this screen is now the only way into.
void settingsScreenSet(TrafficFilter traffic, int radiusNm, bool showRefresh, int pollIntervalS,
                       const String &locationLabel);

// Updates the location shown on the button, after the location screen has
// been in and changed it.
void settingsScreenSetLocation(const String &locationLabel);

void settingsScreenDraw();

// pressed/clicked both matter here: the radius slider tracks a held finger,
// everything else acts on a completed tap.
SettingsAction settingsScreenHandleTouch(int x, int y, bool pressed, bool clicked);

TrafficFilter settingsScreenTraffic();
int settingsScreenRadius();
bool settingsScreenShowRefresh();
int settingsScreenPollInterval();
