#pragma once

#include <Arduino.h>
#include <FS.h>
#include <vector>

#include "psram_alloc.h"

// A minimal MP4 writer for one H.264 video track: what a phone or a laptop
// needs to play the stream the P4's hardware encoder produces, and no more.
//
// The encoder hands back Annex-B - NAL units behind 00 00 00 01 start codes,
// with the SPS and PPS ahead of every IDR frame. MP4 wants them otherwise:
// each NAL behind its length, and the SPS and PPS once, in the track's sample
// description. So each frame is rewritten on its way to the file, and the
// index MP4 keeps - every frame's size, which are keyframes - is gathered as
// it goes.
//
// The file is ftyp, then mdat (every frame, back to back as one chunk), then
// moov, written by finish() once the index is complete. A file that never
// reaches finish() has no moov and won't play; the caller deletes it.
//
// Card access is the caller's to serialise: these write to the File directly.
class Mp4Writer {
public:
    bool begin(fs::File &file, int width, int height, int fps);
    // One encoded frame, as the encoder returned it.
    bool addFrame(const uint8_t *annexB, size_t len);
    bool finish();

    uint32_t frames() const { return (uint32_t)sizes_.size(); }
    uint64_t bytes() const { return mdatOffset_ + 8 + mdatBytes_; }

private:
    fs::File *file_ = nullptr;
    int width_ = 0, height_ = 0, fps_ = 0;
    uint32_t mdatOffset_ = 0;
    uint64_t mdatBytes_ = 0;
    std::vector<uint8_t> sps_, pps_;
    std::vector<uint32_t, PsramAllocator<uint32_t>> sizes_;
    std::vector<uint32_t> syncs_;  // 1-based numbers of the keyframes
    std::vector<uint8_t, PsramAllocator<uint8_t>> sample_;
};
