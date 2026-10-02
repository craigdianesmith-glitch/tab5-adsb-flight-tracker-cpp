#pragma once

#include <Arduino.h>
#include <SD.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "mp4_writer.h"
#include "screen.h"

// Writes what is on the canvas to an MP4 on the SD card, a frame per call:
// converted to YUV by the PPA, encoded by the P4's H.264 block, and written by
// Mp4Writer. Every frame is the whole 1280x720 canvas, so whoever calls this
// draws a frame, adds it, and draws the next.
//
// Two things keep it from taking much longer than the video it makes. Only
// what has changed since a buffer last held a frame is converted - on a replay
// the title, labels and frame of the plot never move. And encoding and writing
// run on a task of their own, on the other core, into one buffer while the
// caller draws and converts the next frame into the other.
//
// All card access takes the recorder's lock, so an export can run while the
// poll task goes on recording.
class VideoWriter {
public:
    ~VideoWriter();

    // Creates the file and gets the encoder ready. False, with nothing left
    // behind, if either can't be done.
    bool open(const String &path, int fps);
    // Hands the canvas as it is now to the encoder. Returns once it has been
    // converted, so the canvas is free to draw on again.
    bool addCanvasFrame();
    // Completes the file. False if it couldn't be, in which case it has been
    // deleted.
    bool close();
    // Gives up: the file is deleted.
    void abort();

    uint32_t frames() const { return mp4_.frames(); }
    uint64_t bytes() const { return mp4_.bytes(); }
    // Time spent per stage so far, for the log. Encoding and writing overlap
    // the caller's drawing, so they add up to more than the time taken.
    uint32_t convertMs() const { return (uint32_t)(convertUs_ / 1000); }
    uint32_t encodeMs() const { return (uint32_t)(encodeUs_ / 1000); }
    uint32_t writeMs() const { return (uint32_t)(writeUs_ / 1000); }

private:
    static void encoderTask(void *self);
    void encode(int buf);
    bool convertInto(int buf);
    void stopTask();
    void release();

    void *enc_ = nullptr;  // esp_h264_enc_handle_t, kept out of this header
    uint8_t *yuv_[2] = {nullptr, nullptr};
    uint8_t *out_ = nullptr;
    size_t outLen_ = 0;
    uint32_t pts_ = 0;
    int fps_ = 0;
    File file_;
    String path_;
    Mp4Writer mp4_;

    // Which regions each buffer is behind the canvas by, and whether it has
    // ever held a whole frame.
    screen::Region pending_[2][screen::MAX_REGIONS];
    int pendingCount_[2] = {0, 0};
    bool filled_[2] = {false, false};
    uint32_t next_ = 0;  // frames handed over, deciding which buffer is next

    TaskHandle_t task_ = nullptr;
    QueueHandle_t jobs_ = nullptr;          // buffer numbers to encode; -1 to stop
    SemaphoreHandle_t free_[2] = {nullptr, nullptr};  // given when the encoder is done with a buffer
    SemaphoreHandle_t stopped_ = nullptr;
    volatile bool failed_ = false;

    uint64_t convertUs_ = 0, encodeUs_ = 0, writeUs_ = 0;
};
