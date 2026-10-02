#include "share_server.h"

#include <ESPmDNS.h>
#include <SD.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <time.h>
#include <vector>

#include "config.h"
#include "recorder.h"

namespace share {
namespace {

constexpr const char *HOSTNAME = "overhead";
// Read from the card and sent in pieces this size, the card lock taken for
// each read only, so the poll task can write a recording in between.
constexpr size_t CHUNK = 16 * 1024;

httpd_handle_t g_server = nullptr;
bool g_mdns = false;
// Set by stop() so a transfer in progress gives up promptly: the server can't
// shut down until the handler sending it returns.
volatile bool g_stopping = false;

portMUX_TYPE g_statusLock = portMUX_INITIALIZER_UNLOCKED;
Status g_status = {};

void setCurrent(const char *name, uint32_t sent, uint32_t total) {
    portENTER_CRITICAL(&g_statusLock);
    strlcpy(g_status.current, name, sizeof(g_status.current));
    g_status.sent = sent;
    g_status.total = total;
    portEXIT_CRITICAL(&g_statusLock);
}

void countRequest() {
    portENTER_CRITICAL(&g_statusLock);
    g_status.requests++;
    portEXIT_CRITICAL(&g_statusLock);
}

// A file name as it may arrive in a URL: letters, digits, '-', '_' and one
// extension. Anything else - a slash, "..", an escape - is refused rather than
// cleaned up, since nothing this device writes needs it.
bool safeName(const String &name, const char *ext) {
    if (name.length() == 0 || name.length() > 64 || !name.endsWith(ext)) {
        return false;
    }
    int dots = 0;
    for (size_t i = 0; i < name.length(); i++) {
        char c = name[i];
        if (c == '.') {
            dots++;
        } else if (!isAlphaNumeric(c) && c != '-' && c != '_') {
            return false;
        }
    }
    return dots == 1;
}

// The last part of the URI, without its query string.
String nameFromUri(const char *uri) {
    String u(uri);
    int q = u.indexOf('?');
    if (q >= 0) {
        u = u.substring(0, q);
    }
    return u.substring(u.lastIndexOf('/') + 1);
}

bool queryValue(httpd_req_t *req, const char *key, char *val, size_t len) {
    char query[128];
    return httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
           httpd_query_key_value(query, key, val, len) == ESP_OK;
}

uint32_t fileSize(const String &path) {
    recorder::CardLock lock;
    File f = SD.open(path, FILE_READ);
    if (!f) {
        return UINT32_MAX;
    }
    uint32_t size = f.size();
    f.close();
    return size;
}

bool sendAll(httpd_req_t *req, const char *buf, size_t len) {
    while (len > 0) {
        int n = httpd_send(req, buf, len);
        if (n <= 0) {
            return false;
        }
        buf += n;
        len -= n;
    }
    return true;
}

esp_err_t sendStatus(httpd_req_t *req, const char *status, const char *body) {
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, body);
}

// Sends a file, or the byte range of it asked for. Ranges are what let a
// phone's browser play a video as it arrives and seek in it - Safari won't
// play one from a server that can't do them. The headers are written by hand,
// so the response carries its length rather than being chunked, which a
// video player also wants.
esp_err_t sendFile(httpd_req_t *req, const String &path, const char *type, bool download) {
    uint32_t size = fileSize(path);
    if (size == UINT32_MAX) {
        return sendStatus(req, "404 Not Found", "Not found");
    }
    uint32_t start = 0, end = size ? size - 1 : 0;
    bool partial = false;
    char range[64];
    if (size > 0 && httpd_req_get_hdr_value_str(req, "Range", range, sizeof(range)) == ESP_OK &&
        strncmp(range, "bytes=", 6) == 0) {
        const char *spec = range + 6;
        char *dash = strchr((char *)spec, '-');
        if (dash != nullptr) {
            if (dash == spec) {  // "bytes=-500": the last 500
                uint32_t n = strtoul(dash + 1, nullptr, 10);
                start = n >= size ? 0 : size - n;
            } else {
                start = strtoul(spec, nullptr, 10);
                if (dash[1] != '\0') {
                    end = std::min<uint32_t>(strtoul(dash + 1, nullptr, 10), size - 1);
                }
            }
            if (start > end || start >= size) {
                char hdr[160];
                int n = snprintf(hdr, sizeof(hdr),
                                 "HTTP/1.1 416 Range Not Satisfiable\r\nContent-Range: bytes */%u\r\n"
                                 "Content-Length: 0\r\n\r\n",
                                 (unsigned)size);
                sendAll(req, hdr, n);
                return ESP_OK;
            }
            partial = true;
        }
    }
    uint32_t length = size ? end - start + 1 : 0;
    String name = path.substring(path.lastIndexOf('/') + 1);

    char hdr[512];
    int n = snprintf(hdr, sizeof(hdr), "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %u\r\nAccept-Ranges: bytes\r\n",
                     partial ? "206 Partial Content" : "200 OK", type, (unsigned)length);
    if (partial) {
        n += snprintf(hdr + n, sizeof(hdr) - n, "Content-Range: bytes %u-%u/%u\r\n", (unsigned)start,
                      (unsigned)end, (unsigned)size);
    }
    if (download) {
        n += snprintf(hdr + n, sizeof(hdr) - n, "Content-Disposition: attachment; filename=\"%s\"\r\n", name.c_str());
    }
    n += snprintf(hdr + n, sizeof(hdr) - n, "Cache-Control: no-cache\r\n\r\n");
    if (!sendAll(req, hdr, n)) {
        return ESP_FAIL;
    }
    if (req->method == HTTP_HEAD || length == 0) {
        return ESP_OK;
    }

    auto *buf = (char *)heap_caps_malloc(CHUNK, MALLOC_CAP_SPIRAM);
    if (buf == nullptr) {
        return ESP_FAIL;
    }
    File f;
    {
        recorder::CardLock lock;
        f = SD.open(path, FILE_READ);
        if (f) {
            f.seek(start);
        }
    }
    uint32_t sent = 0;
    bool ok = (bool)f;
    while (ok && sent < length && !g_stopping) {
        size_t want = std::min<size_t>(CHUNK, length - sent);
        size_t got;
        {
            recorder::CardLock lock;
            got = f.read((uint8_t *)buf, want);
        }
        ok = got > 0 && sendAll(req, buf, got);
        sent += got;
        setCurrent(name.c_str(), sent, length);
    }
    {
        recorder::CardLock lock;
        if (f) {
            f.close();
        }
    }
    heap_caps_free(buf);
    setCurrent("", 0, 0);
    // A short send leaves the client with a response shorter than it was told
    // to expect; failing the request has the server drop the connection, which
    // is the honest thing to do with it.
    return ok && sent == length ? ESP_OK : ESP_FAIL;
}

// "20261002-143155-4x.mp4" -> "Fri 02 Oct 2026 14:31 UTC". Names that don't
// carry a time - a recording made before the clock was set - stay as they are.
String whenFromName(const String &name) {
    struct tm tm = {};
    if (name.length() < 15 || name[8] != '-' ||
        sscanf(name.c_str(), "%4d%2d%2d-%2d%2d%2d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min,
               &tm.tm_sec) != 6) {
        return name;
    }
    tm.tm_year -= 1900;
    tm.tm_mon -= 1;
    time_t t = mktime(&tm);  // the device's zone is UTC, so this round-trips
    gmtime_r(&t, &tm);
    char buf[40];
    strftime(buf, sizeof(buf), "%a %d %b %Y %H:%M UTC", &tm);
    return String(buf);
}

String sizeText(uint32_t bytes) {
    if (bytes >= 1024 * 1024) {
        return String(bytes / (1024.0f * 1024.0f), 1) + " MB";
    }
    return String((bytes + 1023) / 1024) + " KB";
}

const char PAGE_HEAD[] = R"(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Overhead</title>
<style>
:root{color-scheme:dark}
body{margin:0;font:16px -apple-system,system-ui,sans-serif;background:#04120a;color:#cfffe2}
main{max-width:720px;margin:0 auto;padding:16px}
h1{font-size:22px;margin:8px 0 4px}
.sub{color:#7cd9a0;font-size:14px;margin-bottom:12px}
h2{font-size:14px;color:#7cd9a0;text-transform:uppercase;letter-spacing:.06em;margin:24px 0 8px}
.item{background:#0b2216;border:1px solid #1c5a38;border-radius:10px;padding:12px;margin:10px 0}
.name{font-weight:600}
.meta{color:#7cd9a0;font-size:14px;margin:2px 0 10px;word-break:break-all}
video{width:100%;aspect-ratio:16/9;border-radius:6px;background:#000;margin-bottom:10px}
.btn{display:inline-block;padding:9px 16px;border-radius:8px;background:#1c5a38;color:#e6ffee;text-decoration:none;border:0;font:inherit;margin:0 8px 0 0;cursor:pointer}
.del{background:#6b2018}
.empty{color:#7cd9a0}
</style></head><body><main>
<h1>Overhead</h1><div class="sub">Videos and recordings on the tracker's SD card</div>
)";

const char PAGE_TAIL[] = R"(<script>
function del(n){if(confirm('Delete '+n+'?'))fetch('/delete?f='+encodeURIComponent(n),{method:'POST'}).then(function(){location.reload()})}
</script></main></body></html>)";

esp_err_t handleIndex(httpd_req_t *req) {
    countRequest();
    std::vector<String> videos = recorder::videos();
    std::vector<String> recordings = recorder::list();
    String active = recorder::activePath();

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_sendstr_chunk(req, PAGE_HEAD);

    String html = "<h2>Videos</h2>";
    if (videos.empty()) {
        html += "<p class=\"empty\">None yet. Open a recording under Replays on the tracker and tap Export.</p>";
    }
    // Only names this device would have written: anything else, copied on
    // from a computer, could carry quotes or markup into the page.
    for (const String &path : videos) {
        String name = path.substring(path.lastIndexOf('/') + 1);
        if (!safeName(name, ".mp4")) {
            continue;
        }
        String speed = name.substring(name.lastIndexOf('-') + 1, name.lastIndexOf('.'));
        html += "<div class=\"item\"><div class=\"name\">" + whenFromName(name) + " &middot; " + speed +
                "</div><div class=\"meta\">" + name + " &middot; " + sizeText(fileSize(path)) +
                "</div><video controls playsinline preload=\"none\" src=\"/videos/" + name +
                "\"></video><a class=\"btn\" href=\"/videos/" + name +
                "?dl=1\">Download</a><button class=\"btn del\" onclick=\"del('" + name + "')\">Delete</button></div>";
        httpd_resp_sendstr_chunk(req, html.c_str());
        html = "";
    }

    html += "<h2>Recordings</h2>";
    if (recordings.empty()) {
        html += "<p class=\"empty\">None yet.</p>";
    }
    for (const String &path : recordings) {
        String name = path.substring(path.lastIndexOf('/') + 1);
        if (!safeName(name, ".rec")) {
            continue;
        }
        html += "<div class=\"item\"><div class=\"name\">" + whenFromName(name) + "</div><div class=\"meta\">" +
                name + " &middot; " + sizeText(fileSize(path)) + (path == active ? " &middot; recording now" : "") +
                "</div><a class=\"btn\" href=\"/recordings/" + name + "?dl=1\">Download</a></div>";
        httpd_resp_sendstr_chunk(req, html.c_str());
        html = "";
    }
    httpd_resp_sendstr_chunk(req, PAGE_TAIL);
    return httpd_resp_sendstr_chunk(req, nullptr);
}

bool wantsDownload(httpd_req_t *req) {
    char dl[4];
    return queryValue(req, "dl", dl, sizeof(dl)) && dl[0] == '1';
}

esp_err_t handleVideo(httpd_req_t *req) {
    countRequest();
    String name = nameFromUri(req->uri);
    if (!safeName(name, ".mp4")) {
        return sendStatus(req, "404 Not Found", "Not found");
    }
    return sendFile(req, String(VIDEO_DIR) + "/" + name, "video/mp4", wantsDownload(req));
}

esp_err_t handleRecording(httpd_req_t *req) {
    countRequest();
    String name = nameFromUri(req->uri);
    if (!safeName(name, ".rec")) {
        return sendStatus(req, "404 Not Found", "Not found");
    }
    return sendFile(req, "/overhead/" + name, "text/plain; charset=utf-8", wantsDownload(req));
}

esp_err_t handleDelete(httpd_req_t *req) {
    countRequest();
    char f[72];
    if (!queryValue(req, "f", f, sizeof(f)) || !safeName(String(f), ".mp4")) {
        return sendStatus(req, "400 Bad Request", "Bad name");
    }
    bool ok = recorder::removeVideo(String(VIDEO_DIR) + "/" + f);
    Serial.printf("[share] delete %s: %s\n", f, ok ? "done" : "failed");
    return ok ? sendStatus(req, "200 OK", "Deleted") : sendStatus(req, "404 Not Found", "Not found");
}

esp_err_t handleFavicon(httpd_req_t *req) {
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, nullptr, 0);
}

}  // namespace

bool start() {
    if (g_server != nullptr) {
        return true;
    }
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }
    g_stopping = false;
    portENTER_CRITICAL(&g_statusLock);
    g_status = {};
    portEXIT_CRITICAL(&g_statusLock);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.lru_purge_enable = true;  // a phone opens several connections and leaves some idle
    cfg.stack_size = 8192;        // the page is built in Strings
    cfg.core_id = 0;              // off the UI's core
    cfg.task_priority = tskIDLE_PRIORITY + 3;
    cfg.send_wait_timeout = 10;
    cfg.recv_wait_timeout = 10;
    if (httpd_start(&g_server, &cfg) != ESP_OK) {
        g_server = nullptr;
        Serial.println("[share] web server failed to start");
        return false;
    }
    const httpd_uri_t uris[] = {
        {"/", HTTP_GET, handleIndex, nullptr},
        {"/videos/*", HTTP_GET, handleVideo, nullptr},
        {"/videos/*", HTTP_HEAD, handleVideo, nullptr},
        {"/recordings/*", HTTP_GET, handleRecording, nullptr},
        {"/delete", HTTP_POST, handleDelete, nullptr},
        {"/favicon.ico", HTTP_GET, handleFavicon, nullptr},
    };
    for (const httpd_uri_t &u : uris) {
        httpd_register_uri_handler(g_server, &u);
    }
    g_mdns = MDNS.begin(HOSTNAME);
    if (g_mdns) {
        MDNS.addService("http", "tcp", 80);
    }
    Serial.printf("[share] serving at %s%s%s\n", url().c_str(), g_mdns ? " and " : "", localUrl().c_str());
    return true;
}

void stop() {
    if (g_server == nullptr) {
        return;
    }
    g_stopping = true;
    httpd_stop(g_server);
    g_server = nullptr;
    if (g_mdns) {
        MDNS.end();
        g_mdns = false;
    }
    Serial.println("[share] stopped");
}

bool running() { return g_server != nullptr; }

String url() { return "http://" + WiFi.localIP().toString() + "/"; }

String localUrl() { return g_mdns ? String("http://") + HOSTNAME + ".local/" : String(""); }

Status status() {
    portENTER_CRITICAL(&g_statusLock);
    Status s = g_status;
    portEXIT_CRITICAL(&g_statusLock);
    return s;
}

}  // namespace share
