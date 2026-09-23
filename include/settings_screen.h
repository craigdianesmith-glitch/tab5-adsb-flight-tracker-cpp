#pragma once

#include <Arduino.h>

#include "settings.h"

enum class SettingsAction {
    NONE,
    BACK,       // caller should apply and persist the values below
    OPEN_WIFI,
};

// Seeds the screen with the values currently in force.
void settingsScreenSet(TrafficFilter traffic, int radiusNm, bool showRefresh);
void settingsScreenDraw();

// pressed/clicked both matter here: the radius slider tracks a held finger,
// everything else acts on a completed tap.
SettingsAction settingsScreenHandleTouch(int x, int y, bool pressed, bool clicked);

TrafficFilter settingsScreenTraffic();
int settingsScreenRadius();
bool settingsScreenShowRefresh();
