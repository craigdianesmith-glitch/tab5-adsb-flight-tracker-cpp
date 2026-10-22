#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "filter.h"

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
// - and after the provider, where ZOOM IN had the radar around home or a
// zoom closer in than its own range, the range shown, in nm: `... adsb.lol
// 0.5`; and centred on a followed aircraft, the range it had then. A build
// that predates it reads the line as it always did, at the view's own range.
// Following, each change of that range - ZOOM IN or OUT, auto zoom, closing
// in on a landing - has a line of its own, with when it was made, written
// ahead of the next poll's `F` line; and so does each change of a zoom's:
//
//   R <ms> <nm>       the radar centred on the aircraft followed
//   Z <ms> <nm>       an airport's zoom
//
// so a replay changes range when the live radar did, between polls as often
// as not. And the filter the table and the radar showed the traffic through,
// at the first poll of a file and again whenever it changes - see
// filterText() -
//
//   Q <ms> 0 45000 0 600 -G- - -
//
// so a replay shows the radar through it as it was seen; the recording keeps
// every aircraft all the same. Older builds pass over these, as over any line
// they don't know -
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
// by an alert: the alerts' tail never stops it. Auto-follow's landing does;
// following by hand, only the user does.
enum class Trigger : uint8_t { MANUAL, AUTO, FOLLOW };

// The runway ends of one airport the live zoom had lately seen in use when a
// file began, and how long before: so a replay starts out knowing which way
// the airport was working, as the live zoom did, rather than only once
// something lines up in the recording. Written as
//   runways <airport> <end> <seconds ago> [<end> <seconds ago> ...]
// - absent from older files, and from any begun with nothing remembered.
struct RunwaysInUse {
    String code;  // the airport's IATA code; empty for none
    struct End {
        char ident[4];  // "05", "27L"
        uint32_t agoS;
    };
    std::vector<End> ends;
};

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
    // And whether auto zoom had the range then - see the display of it in the
    // recordings list - written after the range, where an older build's
    // reading of the line stops short of it.
    bool followAuto = false;
    RunwaysInUse runways;
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

// Where each file's runways line comes from: called as the file is opened,
// from whichever task opens it. Until set, none is written.
void setRunwaysInUseSource(void (*source)(RunwaysInUse &out));

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
// the source that answered. `rangeNm`, when not 0, is the range ZOOM IN had
// the view at - the radar around home, or a zoom - written after them.
// `marks`, if given, are the follow range's changes since the last poll -
// see `R` above - on millis()'s clock.
struct RangeMark {
    uint32_t ms;
    float nm;
    bool zoom;  // a zoom's range rather than the radar's: a `Z` line
};
// `filter`, if given, is the filter in force, written when it has changed.
void addFrame(const std::vector<Aircraft> &aircraft, const String &view, const char *provider,
              float rangeNm = 0, const std::vector<RangeMark> *marks = nullptr,
              const TrafficFilterSet *filter = nullptr);

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

// Parses an `A` line (with its leading "A ") into an aircraft. False too for
// one the poll would now leave out - recorded standing still in the air
// before it did (see stillInTheAir()) - so a replay is of the sky as the
// screens show it now.
bool parseAircraftLine(char *line, Aircraft &out);

}  // namespace recorder
