#include "mp4_writer.h"

namespace {

// The track's clock. 90kHz is the usual one for video, and every frame rate
// worth asking for divides it exactly.
constexpr uint32_t TRACK_TIMESCALE = 90000;
// The movie's clock, which only the overall duration is given in.
constexpr uint32_t MOVIE_TIMESCALE = 1000;

using Bytes = std::vector<uint8_t, PsramAllocator<uint8_t>>;

void u8(Bytes &b, uint8_t v) { b.push_back(v); }
void u16(Bytes &b, uint16_t v) {
    b.push_back(v >> 8);
    b.push_back(v & 0xFF);
}
void u32(Bytes &b, uint32_t v) {
    for (int s = 24; s >= 0; s -= 8) {
        b.push_back((v >> s) & 0xFF);
    }
}
void fourcc(Bytes &b, const char *t) { b.insert(b.end(), t, t + 4); }
void zeros(Bytes &b, size_t n) { b.insert(b.end(), n, 0); }

// Boxes nest, and each begins with its own size, which isn't known until its
// contents are written: so a box is opened with a placeholder and the size
// patched in when it is closed.
size_t boxOpen(Bytes &b, const char *type) {
    size_t at = b.size();
    u32(b, 0);
    fourcc(b, type);
    return at;
}
void boxClose(Bytes &b, size_t at) {
    uint32_t size = (uint32_t)(b.size() - at);
    for (int i = 0; i < 4; i++) {
        b[at + i] = (size >> (24 - 8 * i)) & 0xFF;
    }
}

// The unity transform every track and movie header carries.
void matrix(Bytes &b) {
    const uint32_t m[9] = {0x00010000, 0, 0, 0, 0x00010000, 0, 0, 0, 0x40000000};
    for (uint32_t v : m) {
        u32(b, v);
    }
}

// Where the next start code begins at or after `from`, and how long it is, or
// `len` if there isn't one. A start code can't occur inside a NAL - the
// encoder's emulation prevention sees to that - so a plain scan is enough.
size_t findStart(const uint8_t *d, size_t len, size_t from, size_t &codeLen) {
    for (size_t i = from; i + 2 < len; i++) {
        if (d[i] != 0 || d[i + 1] != 0) {
            continue;
        }
        if (d[i + 2] == 1) {
            codeLen = 3;
            return i;
        }
        if (i + 3 < len && d[i + 2] == 0 && d[i + 3] == 1) {
            codeLen = 4;
            return i;
        }
    }
    codeLen = 0;
    return len;
}

bool writeAll(fs::File &f, const uint8_t *data, size_t len) {
    // In pieces: one very large write through the VFS can come back short
    // without an error, and a short write here is a corrupt file.
    while (len > 0) {
        size_t n = f.write(data, len > 16384 ? 16384 : len);
        if (n == 0) {
            return false;
        }
        data += n;
        len -= n;
    }
    return true;
}

}  // namespace

bool Mp4Writer::begin(fs::File &file, int width, int height, int fps) {
    file_ = &file;
    width_ = width;
    height_ = height;
    fps_ = fps;
    mdatBytes_ = 0;
    sps_.clear();
    pps_.clear();
    sizes_.clear();
    syncs_.clear();

    Bytes head;
    size_t ftyp = boxOpen(head, "ftyp");
    fourcc(head, "isom");
    u32(head, 0x200);
    fourcc(head, "isom");
    fourcc(head, "iso2");
    fourcc(head, "avc1");
    fourcc(head, "mp41");
    boxClose(head, ftyp);
    mdatOffset_ = (uint32_t)head.size();
    u32(head, 0);  // mdat's size, patched by finish()
    fourcc(head, "mdat");
    return writeAll(*file_, head.data(), head.size());
}

bool Mp4Writer::addFrame(const uint8_t *d, size_t len) {
    sample_.clear();
    bool key = false;
    size_t codeLen;
    size_t pos = findStart(d, len, 0, codeLen);
    while (pos < len) {
        size_t nal = pos + codeLen;
        size_t nextLen;
        size_t next = findStart(d, len, nal, nextLen);
        size_t end = next;
        while (end > nal && d[end - 1] == 0) {
            end--;  // trailing zero bytes are padding before the next start code, not NAL
        }
        if (end > nal) {
            uint8_t type = d[nal] & 0x1F;
            if (type == 7) {
                if (sps_.empty()) {
                    sps_.assign(d + nal, d + end);
                }
            } else if (type == 8) {
                if (pps_.empty()) {
                    pps_.assign(d + nal, d + end);
                }
            } else if (type != 9) {  // an access unit delimiter has no place in MP4
                if (type == 5) {
                    key = true;
                }
                uint32_t n = (uint32_t)(end - nal);
                sample_.push_back(n >> 24);
                sample_.push_back((n >> 16) & 0xFF);
                sample_.push_back((n >> 8) & 0xFF);
                sample_.push_back(n & 0xFF);
                sample_.insert(sample_.end(), d + nal, d + end);
            }
        }
        pos = next;
        codeLen = nextLen;
    }
    if (sample_.empty()) {
        return true;  // parameter sets alone: nothing to show
    }
    if (!writeAll(*file_, sample_.data(), sample_.size())) {
        return false;
    }
    sizes_.push_back((uint32_t)sample_.size());
    if (key) {
        syncs_.push_back((uint32_t)sizes_.size());
    }
    mdatBytes_ += sample_.size();
    return true;
}

bool Mp4Writer::finish() {
    if (sizes_.empty() || sps_.size() < 4 || pps_.empty() || mdatBytes_ + 8 > 0xFFFFFFFFull) {
        return false;
    }
    uint32_t n = (uint32_t)sizes_.size();
    uint32_t delta = TRACK_TIMESCALE / fps_;
    uint32_t movieDuration = (uint32_t)((uint64_t)n * MOVIE_TIMESCALE / fps_);

    Bytes b;
    b.reserve(1024 + n * 4 + syncs_.size() * 4);
    size_t moov = boxOpen(b, "moov");

    size_t mvhd = boxOpen(b, "mvhd");
    u32(b, 0);  // version, flags
    u32(b, 0);  // created
    u32(b, 0);  // modified
    u32(b, MOVIE_TIMESCALE);
    u32(b, movieDuration);
    u32(b, 0x00010000);  // rate 1.0
    u16(b, 0x0100);      // volume 1.0
    zeros(b, 10);
    matrix(b);
    zeros(b, 24);
    u32(b, 2);  // next track ID
    boxClose(b, mvhd);

    size_t trak = boxOpen(b, "trak");
    size_t tkhd = boxOpen(b, "tkhd");
    u32(b, 0x00000003);  // enabled, in the movie
    u32(b, 0);
    u32(b, 0);
    u32(b, 1);  // track ID
    u32(b, 0);
    u32(b, movieDuration);
    zeros(b, 8);
    u16(b, 0);  // layer
    u16(b, 0);  // alternate group
    u16(b, 0);  // volume: none, it's video
    u16(b, 0);
    matrix(b);
    u32(b, (uint32_t)width_ << 16);
    u32(b, (uint32_t)height_ << 16);
    boxClose(b, tkhd);

    size_t mdia = boxOpen(b, "mdia");
    size_t mdhd = boxOpen(b, "mdhd");
    u32(b, 0);
    u32(b, 0);
    u32(b, 0);
    u32(b, TRACK_TIMESCALE);
    u32(b, n * delta);
    u16(b, 0x55C4);  // language "und"
    u16(b, 0);
    boxClose(b, mdhd);

    size_t hdlr = boxOpen(b, "hdlr");
    u32(b, 0);
    u32(b, 0);
    fourcc(b, "vide");
    zeros(b, 12);
    const char name[] = "VideoHandler";
    b.insert(b.end(), name, name + sizeof(name));  // with its terminator
    boxClose(b, hdlr);

    size_t minf = boxOpen(b, "minf");
    size_t vmhd = boxOpen(b, "vmhd");
    u32(b, 1);  // flags 1, as the spec requires
    zeros(b, 8);
    boxClose(b, vmhd);

    size_t dinf = boxOpen(b, "dinf");
    size_t dref = boxOpen(b, "dref");
    u32(b, 0);
    u32(b, 1);
    size_t url = boxOpen(b, "url ");
    u32(b, 1);  // the data is in this file
    boxClose(b, url);
    boxClose(b, dref);
    boxClose(b, dinf);

    size_t stbl = boxOpen(b, "stbl");
    size_t stsd = boxOpen(b, "stsd");
    u32(b, 0);
    u32(b, 1);
    size_t avc1 = boxOpen(b, "avc1");
    zeros(b, 6);
    u16(b, 1);  // data reference index
    zeros(b, 16);
    u16(b, (uint16_t)width_);
    u16(b, (uint16_t)height_);
    u32(b, 0x00480000);  // 72 dpi
    u32(b, 0x00480000);
    u32(b, 0);
    u16(b, 1);  // frames per sample
    zeros(b, 32);  // compressor name
    u16(b, 0x0018);
    u16(b, 0xFFFF);
    size_t avcC = boxOpen(b, "avcC");
    u8(b, 1);
    u8(b, sps_[1]);  // profile
    u8(b, sps_[2]);  // compatibility
    u8(b, sps_[3]);  // level
    u8(b, 0xFF);     // NAL lengths are 4 bytes
    u8(b, 0xE1);     // one SPS
    u16(b, (uint16_t)sps_.size());
    b.insert(b.end(), sps_.begin(), sps_.end());
    u8(b, 1);  // one PPS
    u16(b, (uint16_t)pps_.size());
    b.insert(b.end(), pps_.begin(), pps_.end());
    boxClose(b, avcC);
    boxClose(b, avc1);
    boxClose(b, stsd);

    size_t stts = boxOpen(b, "stts");
    u32(b, 0);
    u32(b, 1);
    u32(b, n);
    u32(b, delta);
    boxClose(b, stts);

    size_t stss = boxOpen(b, "stss");
    u32(b, 0);
    u32(b, (uint32_t)syncs_.size());
    for (uint32_t s : syncs_) {
        u32(b, s);
    }
    boxClose(b, stss);

    size_t stsc = boxOpen(b, "stsc");
    u32(b, 0);
    u32(b, 1);
    u32(b, 1);  // from the first chunk - the only one -
    u32(b, n);  // every sample
    u32(b, 1);
    boxClose(b, stsc);

    size_t stsz = boxOpen(b, "stsz");
    u32(b, 0);
    u32(b, 0);  // sizes vary, so each is listed
    u32(b, n);
    for (uint32_t s : sizes_) {
        u32(b, s);
    }
    boxClose(b, stsz);

    size_t stco = boxOpen(b, "stco");
    u32(b, 0);
    u32(b, 1);
    u32(b, mdatOffset_ + 8);
    boxClose(b, stco);

    boxClose(b, stbl);
    boxClose(b, minf);
    boxClose(b, mdia);
    boxClose(b, trak);
    boxClose(b, moov);

    if (!writeAll(*file_, b.data(), b.size())) {
        return false;
    }
    // Now that its size is known, mdat's header can say it.
    uint32_t mdatSize = (uint32_t)(mdatBytes_ + 8);
    uint8_t sz[4] = {(uint8_t)(mdatSize >> 24), (uint8_t)(mdatSize >> 16), (uint8_t)(mdatSize >> 8),
                     (uint8_t)mdatSize};
    if (!file_->seek(mdatOffset_) || file_->write(sz, 4) != 4) {
        return false;
    }
    file_->flush();
    return true;
}
