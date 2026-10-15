#pragma once

#include <M5Unified.h>
#include <vector>

#include "adsb_client.h"

void displayInit();

// How many aircraft rows actually fit on screen. The poll task trims its
// results to this, so the rows that could never be drawn aren't carried
// around and copied every refresh.
int displayMaxRows();

// True if (x, y) is on the settings cog in the header.
bool displayHitCog(int x, int y);

// True if (x, y) is on the radar button in the header.
bool displayHitRadar(int x, int y);

// True if (x, y) is on the replays button in the header.
bool displayHitReplays(int x, int y);

// True if (x, y) is on the mute speaker in the header.
bool displayHitMute(int x, int y);

// True if (x, y) is on the filter funnel in the header.
bool displayHitFilter(int x, int y);

// Whether the list the table is given has been filtered, and how many there
// were before: the funnel lit, and the strip under the rows saying so.
void displaySetFiltered(bool on, int unfiltered);

// True if (x, y) lands on a row the table actually drew, with outHex set to
// the hex of the aircraft on it - the rows being in whichever order was picked. Bounded by the rows on screen rather than by the size of the
// result set: the poll keeps up to MAX_CONTACTS aircraft for the radar to
// plot, so without the bound the gap below the last drawn row resolves to an
// aircraft that was never on screen.
bool displayHitRow(int x, int y, String &outHex);

// A tap on a column's heading: the rows put in its order - again, the other
// way - and the headings redrawn to say so. True if it was on one; the caller
// has the rows drawn again, and keeps the choice - see displaySetSort().
bool displayHitHeader(int x, int y);
void displaySetSort(int col, bool descending);
int displaySortColumn();
bool displaySortDescending();

// More aircraft than rows: the first shown - the table dragged up or down
// through them - and moving it, kept to the list. True if that changed it,
// for the caller to have the table drawn again.
int displayScroll();
bool displayScrollTo(int first);
constexpr int DISPLAY_ROW_HEIGHT = 52;

// Drop the render cache - call when another screen has drawn over the shared
// canvas, so the next render repaints from scratch.
void displayInvalidate();

// The header's subtitle: the nearest airport to
// the configured location (empty if nothing is near enough), and which
// provider the data came from (empty until a poll has succeeded). Anything
// changing here forces the full repaint that draws it - a failover picking a
// different provider mid-run would otherwise not show until something else
// happened to invalidate the header.
void displaySetHeader(const String &airportCode, const String &provider);

// Whether the speaker icon is drawn muted. Repaints the icon on its own.
void displaySetMuted(bool muted);

// Turns the shading of just-changed cells on or off.
void displaySetShowRefresh(bool enabled);

// Fades out cell shading once it has had its time on screen. Call every loop
// while the main screen is up; it does nothing until something expires.
void displayTickHighlights();

// The state of the polling, for the status line under the table. A
// lastSuccessMs of 0 (everSucceeded false) means nothing has arrived yet.
void displaySetPollState(bool linkUp, bool lastPollOk, bool everSucceeded, uint32_t lastSuccessMs);

// Redraws the status line when what it would say has changed - which includes
// the data ageing by a second. Call every loop while the main screen is up.
void displayTickStatus();

// Puts a short message on the status line in place of the poll state, for a
// few seconds - the answer to something done on the table itself.
void displayShowNotice(const String &text);

// isNew[i] true => aircraft[i]'s row is drawn in dark green for this frame.
void displayRenderAircraft(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew);
