// The innermost loop of stamping the 3D view's labels into its picture (see
// labels3d.cpp's stamp()), eight pixels at a time with the ESP32-P4's PIE
// vector instructions: 128-bit registers q0-q7 holding eight 16-bit lanes.
//
// void labels3d_stamp8(uint16_t *row, int chunks, const uint8_t *bits, const uint16_t *constants);
//
// `chunks` lots of eight pixels from `row`, 16-byte aligned, and a byte of
// `bits` for each - the first pixel the top bit. Where a bit is set the
// pixel takes the colour; where it isn't, the pixel is left as it is: the
// letter's shape cut into the picture with an AND, and the colour ORed in.
// A chunk with no bits set isn't touched at all - most of a label's box, and
// all of the rows between its lines. `constants` is two rows of eight lanes,
// 16-byte aligned: the colour, as the picture's pixels are; and each lane's
// bit, in both its bytes, from 0x8080 for the first down to 0x0101 for the
// last.
//
// A PIE load or store takes its address only from x8-x15 or x24-x31.

    .text
    .align  2
    .global labels3d_stamp8
    .type   labels3d_stamp8, @function
labels3d_stamp8:
    blez    a1, 3f                  // none - or fewer: a count under nought would run on through memory
    esp.vld.128.ip  q4, a3, 16      // the colour, in every lane
    esp.vld.128.ip  q7, a3, 0       // each lane's bit
1:
    lbu     t0, 0(a2)
    beqz    t0, 2f                  // none of the letters in these eight
    esp.vldbc.8.ip  q1, a2, 0       // this chunk's byte, in every byte
    esp.andq        q1, q1, q7
    esp.vcmp.eq.u16 q1, q1, q7      // all ones in the lanes whose bit is set
    esp.vld.128.ip  q0, a0, 0       // the pixels
    esp.andq        q2, q4, q1      // the colour where the bit is set...
    esp.notq        q1, q1
    esp.andq        q0, q0, q1      // ...the picture where it isn't
    esp.orq         q0, q0, q2
    esp.vst.128.ip  q0, a0, 0
2:
    addi    a0, a0, 16
    addi    a2, a2, 1
    addi    a1, a1, -1
    bnez    a1, 1b
3:
    ret
    .size   labels3d_stamp8, .-labels3d_stamp8
