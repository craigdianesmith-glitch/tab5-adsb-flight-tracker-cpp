#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"

// Recordings of the radar scene on the microSD card: every contact, at every
// successful poll, for as long as a recording runs. Started by hand from the
// radar screen, or automatically while an alerted aircraft is about or one is
// being followed.
//
// A recording is a text file under /overhead - a short header, then for each
// poll an `F` line followed by one `A` line per aircraft.
//
// A recording is of whatever the screens are showing, whichever way it was
// started: the radar around home; the radar centred on a followed aircraft;
// or an airport's zoom, with or without one being followed in it. Each `F`
// line says which, and who answered the poll -
//
//   F <ms> <epoch> H <provider>           the radar around home
//   F <ms> <epoch> -<hex> <provider>      centred on aircraft <hex>
//   F <ms> <epoch> GLA/<hex> <provider>   GLA's zoom, following <hex> if given
//
// - so that a replay shows each poll the way it was seen, and can tell an
// aircraft that dropped out of a poll because a different provider answered
// it from one the feed lost. Older files end the line at the epoch, or have a
// bare `-` or airport code for a following recording's views, its aircraft
// named in the header. Text rather than a
// packed binary because a card pulled from the device can be read on a laptop
// as it stands, and at one poll every few seconds the size is no concern: an
// hour of a busy sky is well under a megabyte.
//
// Writes come from the poll task and reads from the UI, so every function
// here takes the card's lock itself; none may be called with it held.
namespace recorder {

// FOLLOW is auto-record's too, started by following an aircraft rather than
// by an alert: the alerts' tail never stops it, its landing does.
enum class Trigger : uint8_t { MANUAL, AUTO, FOLLOW };

struct Header {
    uint32_t startEpoch = 0;  // UTC seconds; 0 if the clock had not been set
    Trigger trigger = Trigger::MANUAL;
    String note;  // what set an automatic recording off, or who it followed
    double lat = 0, lon = 0;  // the home location at the time
    int radiusNm = 0;
    bool military = false;
    // 1 for a recording's first file; a long recording carries on in further
    // parts, each a recording in its own right. Absent from older files, which
    // read as part 1.
    int part = 1;
    // A recording that follows an aircraft: its ICAO hex, and the range the
    // radar showed it at. Its polls are the ones fetched around it, not
    // around home - which lat, lon and radiusNm still give, for the list.
    // Empty for any other recording, and absent from older files.
    String followHex;
    int followRangeNm = 0;
};

// Mounts the card if it isn't already, and proves it can be written to - a
// locked or failing card mounts happily and then loses every recording. Cheap
// once mounted; when it isn't, it tries again, so a card inserted after boot
// is found on next asking.
bool mount();
// Whether a usable card is mounted right now. Lock-free, for the UI to grey
// out what needs one; it goes false if a write fails and the card is dropped.
bool mounted();
uint64_t cardBytes();

bool start(const Header &header);  // false if there is no card or no file
void stop();
bool active();
Trigger activeTrigger();
// millis() when the running recording began - its first part, so a counter
// built on this runs on across the split into parts.
uint32_t activeSinceMs();
String activePath();  // the part being written now

// One poll's worth of contacts, appended if a recording is running. Called
// from the poll task, which is the one place a write can take its time.
// `view` is the `F` line's view token, as above, and `provider` the name of
// the source that answered.
void addFrame(const std::vector<Aircraft> &aircraft, const String &view, const char *provider);

// Every recording on the card, newest first.
std::vector<String> list();
// Every exported video (VIDEO_DIR), newest first.
std::vector<String> videos();
// The videos made from a recording, given the list from videos().
std::vector<String> videosOf(const String &recordingPath, const std::vector<String> &videos);
// Deletes a video. Refuses anything that isn't an .mp4 in VIDEO_DIR.
bool removeVideo(const String &path);
bool readHeader(const String &path, Header &out, uint32_t &bytes);
bool remove(const String &path);

// --- for replay -----------------------------------------------------------

// Holds the card for the lifetime of the object, for a reader working through
// a file outside the functions above.
class CardLock {
public:
    CardLock();
    ~CardLock();
    CardLock(const CardLock &) = delete;
    CardLock &operator=(const CardLock &) = delete;
};

// Parses a header line into `h`; false for a line that isn't one.
bool parseHeaderLine(const char *line, Header &h);

// Parses an `A` line (with its leading "A ") into an aircraft.
bool parseAircraftLine(char *line, Aircraft &out);

}  // namespace recorder
