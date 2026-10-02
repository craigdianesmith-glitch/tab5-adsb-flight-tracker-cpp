#include "video_writer.h"

#include <esp_cache.h>
#include <esp_h264_enc_single_hw.h>
#include <esp_heap_caps.h>

#include "recorder.h"

namespace {

constexpr int WIDTH = 1280, HEIGHT = 720;
// A ceiling rather than a target: the radar is a dark, mostly still picture,
// and in practice it encodes at a fraction of this. It is what keeps a frame
// full of moving labels from turning to mush.
constexpr uint32_t BITRATE = 1000000;
// A keyframe every three seconds of video, so a player can seek in it.
constexpr int KEYFRAME_SECONDS = 3;
// On the core the UI isn't on, above the poll task: it mostly waits on the
// encoder block, but when it has work the next frame is waiting on it.
constexpr int ENCODER_CORE = 0;
constexpr UBaseType_t ENCODER_PRIORITY = 2;

esp_h264_enc_handle_t handle(void *p) { return static_cast<esp_h264_enc_handle_t>(p); }

// 64-byte aligned for the PPA and encoder DMA, in PSRAM, and written back once
// so that no dirty line from the zeroing can later land on top of what the
// hardware writes.
uint8_t *dmaBuffer(size_t len) {
    auto *p = (uint8_t *)heap_caps_aligned_calloc(64, 1, len, MALLOC_CAP_SPIRAM);
    if (p != nullptr) {
        esp_cache_msync(p, len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
    return p;
}

// Adds a region to a buffer's list of what it is behind on, collapsing the
// list to its bounding box when it is full: converting more than changed is
// only slower, never wrong.
void addRegion(screen::Region *list, int &count, const screen::Region &r) {
    for (int i = 0; i < count; i++) {
        const screen::Region &c = list[i];
        if (r.x >= c.x && r.y >= c.y && r.x + r.w <= c.x + c.w && r.y + r.h <= c.y + c.h) {
            return;
        }
    }
    if (count < screen::MAX_REGIONS) {
        list[count++] = r;
        return;
    }
    int l = r.x, t = r.y, rt = r.x + r.w, b = r.y + r.h;
    for (int i = 0; i < count; i++) {
        l = std::min<int>(l, list[i].x);
        t = std::min<int>(t, list[i].y);
        rt = std::max<int>(rt, list[i].x + list[i].w);
        b = std::max<int>(b, list[i].y + list[i].h);
    }
    list[0] = {(int16_t)l, (int16_t)t, (int16_t)(rt - l), (int16_t)(b - t)};
    count = 1;
}

}  // namespace

VideoWriter::~VideoWriter() { abort(); }

bool VideoWriter::open(const String &path, int fps) {
    abort();
    path_ = path;
    fps_ = fps;
    pts_ = 0;
    next_ = 0;
    failed_ = false;
    filled_[0] = filled_[1] = false;
    pendingCount_[0] = pendingCount_[1] = 0;
    convertUs_ = encodeUs_ = writeUs_ = 0;

    yuv_[0] = dmaBuffer(screen::YUV420_BYTES);
    yuv_[1] = dmaBuffer(screen::YUV420_BYTES);
    // As large as the input, as the encoder's documentation asks: a frame can
    // never encode bigger than that.
    outLen_ = screen::YUV420_BYTES;
    out_ = dmaBuffer(outLen_);
    if (yuv_[0] == nullptr || yuv_[1] == nullptr || out_ == nullptr) {
        Serial.println("[video] out of memory for frame buffers");
        release();
        return false;
    }

    esp_h264_enc_cfg_hw_t cfg = {};
    cfg.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY;  // the only input this chip revision's encoder takes
    cfg.gop = (uint8_t)(fps * KEYFRAME_SECONDS);
    cfg.fps = (uint8_t)fps;
    cfg.res.width = WIDTH;
    cfg.res.height = HEIGHT;
    cfg.rc.bitrate = BITRATE;
    cfg.rc.qp_min = 18;
    cfg.rc.qp_max = 40;
    esp_h264_enc_handle_t enc = nullptr;
    if (esp_h264_enc_hw_new(&cfg, &enc) != ESP_H264_ERR_OK || esp_h264_enc_open(enc) != ESP_H264_ERR_OK) {
        Serial.println("[video] couldn't start the H.264 encoder");
        if (enc != nullptr) {
            esp_h264_enc_del(enc);
        }
        release();
        return false;
    }
    enc_ = enc;

    {
        recorder::CardLock lock;
        if (!recorder::mount()) {
            release();
            return false;
        }
        int slash = path.lastIndexOf('/');
        if (slash > 0) {
            String dir = path.substring(0, slash);
            if (!SD.exists(dir)) {
                SD.mkdir(dir);
            }
        }
        file_ = SD.open(path, FILE_WRITE);
        if (!file_ || !mp4_.begin(file_, WIDTH, HEIGHT, fps)) {
            Serial.printf("[video] can't write %s\n", path.c_str());
            if (file_) {
                file_.close();
                SD.remove(path);
            }
            release();
            return false;
        }
    }

    jobs_ = xQueueCreate(2, sizeof(int));
    stopped_ = xSemaphoreCreateBinary();
    for (int i = 0; i < 2; i++) {
        free_[i] = xSemaphoreCreateBinary();
        if (free_[i] != nullptr) {
            xSemaphoreGive(free_[i]);
        }
    }
    if (jobs_ == nullptr || stopped_ == nullptr || free_[0] == nullptr || free_[1] == nullptr ||
        xTaskCreatePinnedToCore(encoderTask, "encode", 6144, this, ENCODER_PRIORITY, &task_, ENCODER_CORE) != pdPASS) {
        Serial.println("[video] couldn't start the encoder task");
        task_ = nullptr;
        abort();
        return false;
    }
    // Whatever was marked before now belongs to some other screen's drawing;
    // the first frame into each buffer is converted whole regardless.
    screen::Region discard[screen::MAX_REGIONS];
    screen::takeChanged(discard, screen::MAX_REGIONS);
    return true;
}

bool VideoWriter::convertInto(int buf) {
    if (!filled_[buf]) {
        filled_[buf] = true;
        pendingCount_[buf] = 0;
        return screen::toYuv420(yuv_[buf], screen::YUV420_BYTES);
    }
    for (int i = 0; i < pendingCount_[buf]; i++) {
        if (!screen::toYuv420(yuv_[buf], screen::YUV420_BYTES, pending_[buf][i])) {
            return false;
        }
    }
    pendingCount_[buf] = 0;
    return true;
}

bool VideoWriter::addCanvasFrame() {
    if (task_ == nullptr || failed_) {
        return false;
    }
    // What changed since the last frame, owed now by both buffers: the one
    // about to be filled, and the other, which will next be filled after it.
    screen::Region changed[screen::MAX_REGIONS];
    int n = screen::takeChanged(changed, screen::MAX_REGIONS);
    for (int b = 0; b < 2; b++) {
        for (int i = 0; i < n; i++) {
            addRegion(pending_[b], pendingCount_[b], changed[i]);
        }
    }

    int buf = (int)(next_ & 1);
    xSemaphoreTake(free_[buf], portMAX_DELAY);  // the encoder may still be reading it
    if (failed_) {
        xSemaphoreGive(free_[buf]);
        return false;
    }
    uint32_t t0 = micros();
    bool ok = convertInto(buf);
    convertUs_ += micros() - t0;
    if (!ok) {
        xSemaphoreGive(free_[buf]);
        return false;
    }
    xQueueSend(jobs_, &buf, portMAX_DELAY);
    next_++;
    return true;
}

void VideoWriter::encoderTask(void *self) {
    auto *w = static_cast<VideoWriter *>(self);
    int buf;
    for (;;) {
        xQueueReceive(w->jobs_, &buf, portMAX_DELAY);
        if (buf < 0) {
            break;
        }
        if (!w->failed_) {
            w->encode(buf);
        }
        xSemaphoreGive(w->free_[buf]);
    }
    xSemaphoreGive(w->stopped_);
    vTaskDelete(nullptr);
}

void VideoWriter::encode(int buf) {
    uint32_t t0 = micros();
    esp_h264_enc_in_frame_t in = {};
    in.raw_data.buffer = yuv_[buf];
    in.raw_data.len = screen::YUV420_BYTES;
    in.pts = pts_;
    esp_h264_enc_out_frame_t out = {};
    out.raw_data.buffer = out_;
    out.raw_data.len = outLen_;
    esp_h264_err_t err = esp_h264_enc_process(handle(enc_), &in, &out);
    uint32_t t1 = micros();
    if (err != ESP_H264_ERR_OK) {
        Serial.printf("[video] encoder error %d at frame %u\n", (int)err, (unsigned)frames());
        failed_ = true;
        return;
    }
    pts_ += 1000 / fps_;
    bool ok;
    {
        recorder::CardLock lock;
        ok = mp4_.addFrame(out_, out.length);
    }
    encodeUs_ += t1 - t0;
    writeUs_ += micros() - t1;
    if (!ok) {
        Serial.printf("[video] write failed on %s - card full or removed?\n", path_.c_str());
        failed_ = true;
    }
}

// Lets the encoder finish what it has been given, then ends its task.
void VideoWriter::stopTask() {
    if (task_ == nullptr) {
        return;
    }
    for (int i = 0; i < 2; i++) {
        xSemaphoreTake(free_[i], portMAX_DELAY);
    }
    int stop = -1;
    xQueueSend(jobs_, &stop, portMAX_DELAY);
    xSemaphoreTake(stopped_, portMAX_DELAY);
    task_ = nullptr;
}

bool VideoWriter::close() {
    if (enc_ == nullptr) {
        return false;
    }
    stopTask();
    bool ok = !failed_;
    {
        recorder::CardLock lock;
        ok = ok && mp4_.finish();
        file_.close();
        if (!ok) {
            SD.remove(path_);
        }
    }
    release();
    return ok;
}

void VideoWriter::abort() {
    stopTask();
    if (file_) {
        recorder::CardLock lock;
        file_.close();
        SD.remove(path_);
    }
    release();
}

void VideoWriter::release() {
    if (enc_ != nullptr) {
        esp_h264_enc_close(handle(enc_));
        esp_h264_enc_del(handle(enc_));
        enc_ = nullptr;
    }
    for (int i = 0; i < 2; i++) {
        heap_caps_free(yuv_[i]);
        yuv_[i] = nullptr;
        if (free_[i] != nullptr) {
            vSemaphoreDelete(free_[i]);
            free_[i] = nullptr;
        }
    }
    heap_caps_free(out_);
    out_ = nullptr;
    if (jobs_ != nullptr) {
        vQueueDelete(jobs_);
        jobs_ = nullptr;
    }
    if (stopped_ != nullptr) {
        vSemaphoreDelete(stopped_);
        stopped_ = nullptr;
    }
}
