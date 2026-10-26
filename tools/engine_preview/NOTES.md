<!-- SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 -->

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

Since done, in `include/engines3d.h`, which `engines3d_check.cpp` runs through each phase.
What follows is what this build asked of the view, as it was written. The view now differs
in places: full dry power (0.88) lined up on its brakes, power level and in a fast descent
by equivalent airspeed rather than ground speed, and the engines 1.5 m up.

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
  x-eufi-display-100m's legs start 1.3-1.8 km off after cuts. Neither changed. x-eufi-display-100m's 100 m is
  probably too close for r69878: the same passes rendered 300 m off (a copy of the scene) give its pass envelope
  (-20 / -8 / -8 / -14 dB at -5 / -2 / +2 / +5 s against -15 / -9 / -8 / -11); at 100 m, -31 / -12 / -8 / -20.
  "100 yards" in the recording's title is its altitude.
- **Limiter across realisations**: these renders 0.096 % at the close pass; over six realisations 0.03-0.28 % (the
  surges at its peak move it +-1.4 LU), the take-off 0.02-0.09 %.

## Build "fit", polished (v2: edc594f, 9e3f4e8)

- **v1 is 4571cd9.** It is the build on the user's listening page (renders in `renders/eufi-fit/`). To restore
  it: `git checkout 4571cd9 -- src include`.
- **v2 renders and notes**: `renders/eufi-fit-p/` (`NOTES.md` there has every number before and after).
- **How measured**: over 7 airframes (default, WHO=1..6), through the judges' chains.

What changed:
- **Low powers**: the law under full dry is squeezed (`TY_Q3_IDLE_DB` -14.5, the physics' -38). The idle
  jet's beaming is flatter, the spools' sound lags their speed, and the starter's air plays before light-off.
  - Stand -30.6 to -9.8 LU and taxi -27.3 to -8.4 LU against the A320 take-off.
  - Approach chase -9.6 to -2.4 LU.
  - The chase's idle drop is -16.8 dB over 3-4 s, not -24.6 in 1.5 s.
- **Whine** (`TY_Q1_WHINE_DB` -35): one line, ahead and beside only, nothing over 3 kHz.
  - 24-31 dB lines are now 17-20 dB ahead, and noise's 13-14 dB behind.
- **Cuts**: the convective gain is in the sources' levels, the distance is not eased, and the low band is
  normalised for its tuning. Fly-past cuts +12.4/+15.2 to +6.2/+7.7 dB.
- **Limiter**: the shocks are ridden with the roar and capped at the limiter's threshold. The ride follows in
  50 ms, with a knee of 0.2 and a ceiling of 0.27, and the ceiling comes down while the share over the
  threshold runs high.
  - Every page scene is under 0.1 % in all 7 airframes; the worst is the close pass at 0.036 %.
  - The judges' probes are all under too; the worst is static front at 0.092 %.
  - The loud scenes are -0.2 to -1.5 LU against the A320 take-off, against v1's +1.1 to +2.3. The standing
    probes hold them there.
- **Bank Doppler**: 0.5 of the Doppler, not 0.15. The squeal is at 0.04 of the A320's.
- **Crackle** (`TY_Q2_CRACKLE` 1): 3 dB weaker ahead.
  - The 80 m pass: 0.55 coming, 1.01 at the peak, 0.56 going (the ILA passes: 0.43 / 1.00 / 0.69).
- **Fitted figures**: 414 of 450 in range (v1 412 of 452 in the same run).
- **Checks on 9e3f4e8**:
  - A320 PC: 106 identical. A320 P4: 7 of 7.
  - Worst cost: 329,450 model cycles against the A320 take-off's 402,514.
  - Stack: fighterRender 448 B, deepest chain 640 B (restart).
  - sizeof(Synth): 51,012 B (+4 B on v1).

## Build "fit", final (v3: 918faef)

- **The user heard v2** and said: "the typhoon sounds are all pretty good, but i think the crackle should be a
  tad less in the final version". The other three questions keep their defaults.
- **One change**: `TY_Q2_CRACKLE` 1 to 0.85, so each shock is 1.4 dB weaker. The shocks fire at the same
  moments; only their sizes change, and the ride's answer to them.
- **Renders and notes**: `renders/eufi-fit-f/` (`NOTES.md` there has every number before and after, and why
  0.85).
- **How measured**: over 7 airframes on the five reheat scenes, through the judges' chains.

What it did:
- **Crackle**: the derivative skewness is 24 % less at the passes' loudest 2 s and 29 % less over the passes.
  - The 80 m pass's peak goes from 1.01 to 0.81, and the take-off's from 2.31 to 1.80.
  - Its size through the Tab5 stand-in is 27 % less. The shocks' rates and sizes are 12 % less.
  - 0.8 took 34-38 % off the skewness, and 0.9 took 18-19 %.
- **It still crackles** at every reheat pass's peak, and there is none where there was none.
- **Loudness**: the reheat passes are within 0.1 LU of v2.
- **Shapes**: every pass peaks at the same moment. The close pass's ducking dip just after its peak is 0.6 dB
  shallower. The other dips and fall-offs move by 0.4 dB or less in the medians. The one exception is
  display-100m's third pass 5 s after its peak, -14.4 to -15.7 dB, where one airframe's 1.6 dB moves the median.
- **Limiter**: the worst is static front at 0.089 % (v2 0.092 %). Every page scene is at most 0.034 %.
- **Fitted figures**: 411 of 450 in range (v2 414), from the crackle figures moving away from the recordings.
- **Checks on 918faef**:
  - A320 PC: 106 identical. A320 P4: 7 of 7.
  - Worst cost: 329,450 model cycles (taxi), against the A320 take-off's 402,514.
  - Stack: deepest chain 640 B (restart).
  - sizeof(Synth): 51,012 B.

Copyright (c) 2026 Craig Smith. Part of Overhead's sound engine: PolyForm Noncommercial 1.0.0 ([LICENSE-POLYFORM.md](../../LICENSE-POLYFORM.md)), not the MIT licence of the rest of the repository.
