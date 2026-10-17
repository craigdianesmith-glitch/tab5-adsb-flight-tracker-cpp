#pragma once

// The 3D landing (scene3d.h) on the device, for measuring - compiled in only
// with -DBENCH3D. Over serial, behind "##": '3' times frames of it every way
// they could reach the screen; '4', '5' and '6' play the landing through the
// cinema's shots by day, at dusk and at night, a governor holding the frame
// rate; '7' leaves one frame in the canvas, for an "##S" straight after it to
// take; '8' puts back the screen that was up, with `repaint`; '9' plays the
// busiest half minute at four sizes of view, the smaller with buttons and the
// aircraft's details in a border round them; '0' lands at Heathrow and at
// Kennedy, busy, at 960x540; 'h' and 'n' fly the cruise over the Firth of
// Clyde, by day and by night.
#ifdef BENCH3D
void bench3dCommand(char c, void (*repaint)());
#endif
