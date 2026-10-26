# The Typhoon's sound: the shared base (eufi-base)

What every Eurofighter Typhoon build starts from, so all of them take the same inputs and
are judged on the same scenes. Research: `~/.cache/terrain-lab/sound/research/eufi-sources.md`
(sections 7, 11, 12) and `eufi-methods.md` (section 9, the recipe).

## The interface (`include/engine_noise.h`)

| What | Default | Meaning |
|---|---|---|
| `Kind::LOW_BYPASS` | (already there) | a fighter's; until now silent by default |
| `Engine::EJ200` | appended, value 5 | the Typhoon's two reheated EJ200s; the A320 values 0-4 are unchanged |
| `Sound::reheat` | `0` | reheat asked for, 0 dry to 1 full. The synth lights and cancels it at the engines' own pace. A fighter hears the larger of `reheat` and `afterburner` (counted as 1) |
| `Sound::afterburner` | `false` | unchanged: lit or not, as the 3D view sets it today |
| `Sound::spoilers` | `0` | for a fighter, its airbrake (the one big panel behind the cockpit), 0 shut to 1 open. No new field: the recipe uses `spoilers` for it |
| `Sound::reverse` | `false` | a fighter has no reversers and ignores this flag (see the landing roll below) |

The A320 never reads `reheat` or `EJ200`. `sizeof(Synth)` was 49,496 B and is now 50,532 B
(+1,036 B, all in PSRAM): one float in `Sound` times 259 copies (`asked_[256]`, `target_`,
`sound_`, `heard_`). The stub adds no state of its own.

## What the 3D view should set for a fighter (`view3d.cpp`, not changed here)

- **`enginesOf()`**: `"EUFI"` gives `kind = LOW_BYPASS`, `engine = EJ200`, `engines = 2`.
  `size` comes from the look, as for any aircraft; the preview uses 15.96/37.57 = 0.425.
- **`reheat`**: `afterburner ? 1 : 0`. The view knows only lit or not. The rule stays as it
  is: power at least 0.95, on the ground or below 3,000 ft at an airfield.
- **Power in level flight**: today 0.65 at any speed. For fighters the integration will make
  it follow speed. The recipe suggests max(vertical-speed power, smoothstep(250, 480, kt) x 0.92).
- **Landing roll**: today it sets `reverse = true` and power 0.7 above 60 kt. A Typhoon has
  no reversers. It holds its nose high to about 100 kt with its engines idling, then uses
  its wheel brakes. The fighter ignores `reverse`, but it does hear `power`, so the view must
  give a fighter's landing roll idle power (0.05), not 0.7. The scenes keep `reverse = true`
  above 60 kt to prove it is ignored, and give power 0.05 as the table says. One help is
  already in the stub: the shared `n1_` idles whenever `reverse` is set, whatever the power,
  because the EJ200 row's reverse figure is its idle. A build that takes N1 from `n1_` is
  therefore right with today's view (reverse plus 0.7) as well.
- **Airbrake (`spoilers`)**: open on the landing roll down to taxi speed, otherwise shut. In
  the scenes it opens over 0.8 s at touchdown and closes as the aircraft slows from 60 kt to
  40 kt, the same shape as an airliner's spoilers. Whether the RAF opens it on the roll-out
  is not published.
- **`flapsOut`**: never, because flaperons are not flaps.
- **`running`**: one flag for both engines, as for the A320. The second engine's start, about
  30 s after the first, is up to the synth.
- **`apu`**: today the view gives one only to a `TURBOFAN`. The stand-start scene sets it from
  0 s until the aircraft taxis, as the table has it. The recipe uses no APU, so each build
  decides whether to play it. If a build does, the view needs the airliner's rule extended to
  `LOW_BYPASS`.
- **Engines' height**: the view takes `pose.at + 2.5 x look.scale`. The preview uses 1.5 m,
  where a Typhoon's nozzles are, instead of the airliner's 3 m.

## The stub (`src/engine_noise.cpp`, search for FIGHTER STUB)

- `update()` fades a `LOW_BYPASS` sound in as it fades an airliner's, then calls
  `fighterStep()` every 2 ms step.
- `slowStep()` computes the shared figures (`gain_`, meaning spreading, lateral attenuation
  and convection; `air_`) and then calls `fighterSlowStep()` in place of the airliner's sums
  (hunt, bands, tones, combs).
- `render()` calls `fighterRender(mix, noiseA, noiseB, rng, n)` before the air's muffling, so
  the fighter goes through the muffling (now gated `on || fighter`), the echo, the level, the
  180 Hz high-pass and the limiter.
- All three are empty, so the output is silence.
- Still shared with the airliner in `update()`:
  - `engines()`: N1 from `wantN1()` with the new EJ200 row of `ENGINES` (estimates: idle 30 %,
    approach 75 %, level 85 %, climb and take-off 100 %). It keeps the A320's 50 % hold before
    take-off power on the ground. A build may want to skip that hold for the fighter.
  - The start spool (`motorS` 25 s).
  - `tyresStep()`. Its squeal and thump are rendered only for a `TURBOFAN` (`on`).
- The fighter's level is its own: set it inside its own sums. `LEVEL` is the airliner's.

## The scenes (`preview.cpp`, `eufiScenes()`)

All are placed, with the ear as given, and set 50 times a second. Decisions the table left
open are in each scene's comment. In short:

| Scene | s | Geometry | Power and reheat |
|---|---|---|---|
| eufi-stand-start | 80 | camera 35 m off at the front quarter (45°), 2 m up, apron | APU from 0 s; `running` from 5 s; idle 0.05; taxis away at 65 s (0.5 m/s², power 0.14), passing the camera 25 m off |
| eufi-taxi-past | 30 | 8 m/s, 35 m off, abeam at 15 s | 0.14; breakaway burst 0.22 (50 % N1) from 9 to 12 s |
| eufi-takeoff | 60 | 150 m off the centreline, 500 m from brake release, 4 m up | brakes held at MIL 0.88 for 0-3 s; release with reheat (power 1); lift-off 8 s after release (1 g, 80 m/s, 320 m); 13.5° climb; reheat cancelled at 3,000 ft (51 s) to 0.88. Passes the camera 40 m up |
| eufi-climbout-overhead | 45 | under the climb 3 km from brake release; overhead at 22 s, 600 m up, 120 m/s, 12.6° | power 1 with reheat; cancelled at 3,000 ft (34 s) to 0.88 |
| eufi-display-pass | 40 | 30 m up at 154 m/s past the tower (230 m off, 20 m up), abeam at 20 s | 0.65 before; reheat lit at the pass (power 1); 5 g pull to 60° from 21 s; reheat cancelled at 3,000 ft (29 s), power still 1, as the view does |
| eufi-low-pass | 30 | 76 m up at 216 m/s, 50 m off the track, abeam at 15 s | 0.65, or `POWER=` (render_eufi.sh also renders 0.85 as eufi-low-pass-085) |
| eufi-flypast | 36 | 3,962 m at 298 m/s; the FLY PAST camera 238 m off and a third of that below, reset every 12 s | 0.65 |
| eufi-chase-cruise | 45 | FL250 at 263 m/s; camera 54 m behind and 10 m above | 0.65, then 0.05, then 0.88, 15 s each |
| eufi-approach-chase | 45 | the chase on a 3° final at 70 m/s from 120 m up; touchdown at 32.7 s | 0.45 with corrections (below); idle after touchdown; aerobraking to 50 m/s; `reverse` on throughout the roll |
| eufi-landing | 80 | 60 m beside the touchdown zone, 2 m up; from 3.6 km out; touchdown at 51.4 s | the same approach; idle after touchdown; aerobraking 1.6 m/s² to 100 kt, then brakes at 2.5 m/s² to 10 m/s by 79.6 s; `reverse` on above 60 kt |
| x-eufi-zeltweg | 40 | 154 m/s, 50 m up and 320 m off a microphone 2.5 m up (324 m slant), abeam at 20 s | power 1, reheat |
| x-eufi-close-pass | 15 | 150 m/s, 40 m up and 70 m off (80 m slant), abeam at 7.5 s | power 1, reheat. The measured crackle (3.8-5.0) is reheat's, not MIL's |
| x-eufi-display-100m | 60 | 60 m up, 80 m off; three passes 20 s apart, turning about | MIL at 300 kt; reheat at 350 kt; at 250 kt, dry until the pass and lit at it |

- **Approach power**: 0.45 plus a correction of up to 3 points held for 3.3-4.6 s each, the
  same on both approach scenes. The table asks for this; the 3D view holds 0.45.
- **The cuts in x-eufi-display-100m**: the aircraft jumps from the end of one pass to the start
  of the next, on the same side 1.3-1.8 km off. The synth hears a jump at once from the new
  place, as it does after a camera cut.

## Tools

- `render_eufi.sh <worktree> <outdir>`:
  - builds the preview with `ENGINE_NOISE_DEBUG`;
  - renders the 13 scenes, eufi-low-pass-085 and the A320's take-off (`ref/`, the level to
    beat by 3-5 dB) side by side, 28 Typhoon WAVs in all;
  - writes `summary.txt` with dBFS every 2 s and each scene's limiter share (it must stay
    under 0.1 %).
- `TRACE=1` on the preview prints what the synth is given, once a second.
- `eufi_cases.inc` for `eufi-tools/cost.sh`, six cases of 250 blocks:
  - eufi-takeoff-roll (reheat, 150 m off);
  - eufi-display-reheat;
  - eufi-low-pass-085;
  - eufi-chase-cruise;
  - eufi-approach-chase;
  - eufi-taxi-past.
- Stub cost (warm, model): 0.35-0.47 ms a block. That is the shared noise draws, the echo, the
  high-pass and the limiter, and means nothing for the sound. The A320's take-off is
  1.23-1.27 ms, and the fighter must stay under it.
- After any change to shared code, run both A320 checks (`eufi-tools/a320_pccheck.sh`: 106
  identical; `a320_bitcheck.sh`: 7 of 7 as approved).

## Build "fit" (branch eufi-fit): what was built on this base

The full notes, every measured target and the renders: `~/.cache/terrain-lab/sound/renders/eufi-fit/NOTES.md`
and `metrics.json`; the fitting scripts in `~/.cache/terrain-lab/sound/eufi-fit-work/`.

- **The roar**: the A320's 8-band bank, each band Tam's big-eddy (F) plus fine-turbulence (G) spectrum with its own
  lobe, at the band's frequency less 0.15 of the Doppler; level by the F404's sound power against thrust, the
  flight effect, each engine's spool and fire; reheat +4 dB with the jet's speed x1.44.
- **Surges**: two processes (the low end's, the treble's), each band moved by both as it is low or high (0 deg at
  400 Hz to 70 deg from 3 kHz); each band's own shimmer.
- **The unplayed low band** at the big eddies' frequency x Doppler: its envelope moves the bank (0.12 low half,
  0.3 treble half); a 20 ms flutter on the treble half.
- **Shocklets** at the low band's upward crossings: a self-exciting, bursty event process gated by the jet's
  convective Mach number against the air (0.8 to 1.5), strength 8.5 x the roar's RMS x a crackle lobe (-5 dB ahead,
  -10 behind) x a formation-and-wear distance law; a 32-tap sawtooth kernel through a 64-float ring. The roar's 4 and
  6.3 kHz bands at 0.3 of their power, the shocks filling the top; at full power the air's treble loss as over 0.8
  of the way (the jet's steepening).
- **Low power**: an intake whine (BPF and 2 x BPF, ahead, gone above N1 0.75-0.9); the second engine 30 s after the
  first; the airbrake's buffet; the A320's touchdown squeal at half level. `reverse` never read.
- **Reheat**: lit at N1 >= 0.95 over ~0.6 s, cancelled in 0.25 s; as it catches, a whoomp (noise at 320 Hz x
  Doppler, 10 ms up, 100 ms down) and a cluster of shocks 0.15 s on.
- **Propagation** on its own path: aspect and closing eased in 35 ms, closing clamp 0.8c, convective gain 0.5-2.5,
  the air's loss compensated over each band's width; the roar ridden down at its very loudest for the limiter.
- **Fitted** on x-eufi-close-pass (vs ILA 2022, 10 passes), x-eufi-display-100m (vs fs69878, 14 passes) and
  x-eufi-zeltweg (vs all 11 pass clips), six realisations each, through the same chains (asymmetry before the
  speaker against the references through hp180c, impulsiveness after it through hp180c + tab5): 99, 109 and 110
  of 114 values within the references' 10th-90th percentiles (one realisation, the default render: 84, 98, 99).
- **Level**: reheat take-off -15.4 LUFS, +2.2 LU over the A320's take-off (-17.6); limiter 0.045 % there, 0.096 %
  at the 80 m reheat pass, under 0.03 % in the other scenes. +0.5 dB put the close pass's roar over 0.1 %.
- **Cost**: 0.89-1.07 ms warm a block (the A320's take-off 1.23-1.27). sizeof(Synth) 51,008 B (+476 B on the base).
- **A320**: a320_pccheck 106 identical; a320_bitcheck 7 of 7 as approved.
- **For the 3D view, beyond the list above**: level-flight power should follow speed for fighters (at 0.65 the
  420-knot low pass is 16 LU quieter and crackle-free); no APU is played; a cut's one-frame speed jump in
  `airspeedMs` is clamped to 400 m/s.
- **Scene notes**: x-eufi-close-pass's first 3.3 s of sound is its start position (1.1 km off, nothing arrived yet);
  x-eufi-display-100m's legs start 1.3-1.8 km off after cuts. Neither changed.
