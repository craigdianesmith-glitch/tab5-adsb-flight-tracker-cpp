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


## The Boeing 777 (b777-sound, 27 Oct, first version for review)

The A320's turbofan voice on the 777's engines, from `b777-sources.md` - no new sources, only figures:

- **Engines**: GE90-94B, GE90-115B and Trent 800 rows in `ENGINES[]`. B77W and B77L get GE90-115Bs, B772 GE90-94Bs
  60 times in a hundred or Trent 800s, B773 Trent 800s (`b777Engines()`, `engines3d::enginesOf`). No 777X (GE9X) yet.
- **New per-engine figures** (all neutral for the A320s and the Typhoon): `jetScale` (the jet's roar an octave lower
  per doubling; the fan's intake hiss and the airframe's rush surging that many times slower; the jet's surges 25 %
  deeper per doubling), `h2Db` (the fan's second harmonic, the fan's voice on the Tab5 when its fundamental is below
  the speaker), `buzzLo`/`buzzHi` (the buzz-saw's shaft orders: 7-13 on the GE90, a few clear lines, not the CFM56's
  comb), `spool` (0.75: the big fan's slow glide) and `frameDb` (+8 dB: twelve main wheels, 3.5 times the wing).
  The fan's broadband corner is 0.75 octave lower per doubling. The GE90's turbine tones are at shaft orders 100 and
  104 [I: blade counts not published].
- **Against the recordings** (`targets/work/b777`, the same measures as the references): 85 of 136 figures in the
  references' range, against the old synthesiser's 46. Still short: the approach's 4 kHz octave by about 6 dB (its
  slope -9 dB/oct against -7.3), the take-off pass's slow swell (0.25-1 Hz) at 0.16-0.2 against 0.43, and spool-up
  (one reference clip only).
- **Checks**: a320-takeoff (CFM56 and LEAP), a320-landing and eufi-takeoff byte-identical to v1.5.1-dev's. The release
  image builds, 12,178,208 B. `sizeof(Synth)` +36 B (`surgeNp_`).
- **To listen**: `~/.cache/terrain-lab/sound/review/b777-v1/` - each scene full-range and `-tab5`; `-as-a320` is the
  same flight with an A320's engines; `-trent800` the take-off with Trent 800s.


# The ATR 72's sound: the shared base (atr72-base, 27 Oct)

What every ATR build starts from, so all of them take the same inputs and are judged on the same scenes. The
first turboprop, and the base for every propeller aircraft after it. Its sound is not made here: a turboprop
plays silence. Research, all in `~/.cache/terrain-lab/sound/`:

- `research/atr72-sources.md`: sources, physics, published numbers, the phase tables (section 14);
- `research/turboprop-methods.md`: how to make it, the recipe (section 13), 25 validation targets (section 14);
- `targets/atr72.md` and `.json`: the measured targets, the judging list (section 9), and where they disagree with
  the other two (section 10.3).

Tags as in the code: [P] published, [P*] secondary, [M] measured on CC0 or CC BY recordings, [I] inferred, [O] this
project's own.

## The interface (`include/engine_noise.h`)

| What | Meaning |
|---|---|
| `Kind::TURBOPROP` | a propeller airliner's, the ATR 72's and 42's; silent until a build makes it |
| `Engine::PW127_568F` ... `PW120_14SF` | five values appended after the 777's, one per installation: a type's engines, propellers and airframe together. The A320's, Typhoon's and 777's values are unchanged |
| `struct PropData`, `propData(engine)`, `isProp(engine)` | the per-type table, public so the 3D view, the preview and the checks read the same figures. Any engine without a row gets a generic six-bladed 1,200 rpm twin (blades passing at `Sound::bladeHz`) |
| `Sound::np` (new, default −1) | the propellers' speed as the crew set it, a share of `rpm100`: asked for, not reached; the synth moves there at `npRise` and `npGlide`. Below nought: not said, the synth guesses from power. `sizeof(Synth)` +1,036 B (259 copies of `Sound`, PSRAM) |
| `Sound::power` | for a turboprop, its shaft power as a share of its take-off rating: how hard the blades bite, at whatever speed they turn |
| `Sound::apu` | for a turboprop with a propeller brake (`hotel`), its hotel mode: the right engine's gas generator running with its propeller held still. With `running`, the left propeller turns and the right is held until hotel mode ends |
| `Sound::spoilers` | nought for a turboprop without ground spoilers (`engines3d::airframe()`): an ATR's two spoilers work with the ailerons [P*][I] |
| `Sound::reverse` | its blades turned past flat to reverse pitch |

`PropData`, by group (each row's comment cites each figure):

- **Propellers**: `engines`, `blades`, `rpm100` (the governor's 100 %), `diameter`, `halfSpan` (each axis from the
  centre line; nought for a single), `axisUp` (axis height on the wheels).
- **Speeds the crew set** (shares of `rpm100`): `npGround` (beta: ground idle, taxi, the roll out of reverse),
  `npClimb` (climb, cruise, descent), `npTakeoff`, `npApproach`, `npReverse`; and how fast they move, `npRise` (up)
  and `npGlide` (down), shares a second.
- **Two propellers**: `synchro` (one speed and phase while governed), `splitBeta` (how far apart in beta).
- **Power by phase**, shares of take-off: `climbPower`, `cruisePower`, `levelPower` (level low down about an
  airfield), `descentPower`, `approachPower`, `reversePower`.
- **Gas generator and gearbox**: `nh100` (HP spool rpm), `whineHz` (as heard on the approach, at the source),
  `gearA`, `gearB` (gearbox lines as orders of the propeller's turning).
- **Ground**: `hotel` (propeller brake, no APU), `groundSpoilers`.

The rows (`src/engine_noise.cpp`, `PROPS`):

| Type (ICAO) | Row | Blades | Ø m | BPF at 71 / 82 / 100 % | Power: climb / cruise / level / descent / approach / reverse |
|---|---|---|---|---|---|
| ATR 72-500/-600 (AT75, AT76) | `PW127_568F` | 6 | 3.93 | 85.2 / 98.4 / 120 Hz | 0.886 / 0.861 / 0.50 / 0.05 / 0.22 / 0.70 |
| ATR 72-200 (AT72) | `PW124_14SF` | 4 | 3.96 | 56.8 / 65.6 / 80 Hz | 0.967 / 0.967 / 0.58 / 0.05 / 0.25 / 0.70 |
| ATR 72-210 (AT73) | `PW127_247F` | 4 | 3.93 [I] | 56.8 / 65.6 / 80 Hz | as the -600's [I] |
| ATR 42-400/-500/-600 (AT44, AT45, AT46) | `PW127_568F_42` | 6 | 3.93 | 85.2 / 98.4 / 120 Hz | 1.00 / 0.987 / 0.48 / 0.05 / 0.20 / 0.70 |
| ATR 42-300/-320 (AT43) | `PW120_14SF` | 4 | 3.96 [I] | 56.8 / 65.6 / 80 Hz; the approach at 91 % [M], 72.8 Hz | 0.95 / 0.90 / 0.55 / 0.05 / 0.24 / 0.70 [I] |

All five: 1,200 rpm at 100 %; axes 8.10 m apart and 2.80 m up [P: the fact sheets]; synchrophased, 0.5 % apart in
beta [I]; up from beta at 0.12 a second, down 100 to 82 % at 0.022 a second [M: aboard]; whine 5,682 Hz [M: the
PW127's, assumed for the others]; gearbox lines at orders 48.8 and 72.9 [M: aboard; assumed for the others] (line A
47.7 since the recipe build, below); hotel
mode, no ground spoilers. The climb and cruise power are the ratings' shares [P: the fact sheets]: so the ATR
42-600 climbs at its take-off power (2,160 shp both) with its propellers at 82 %, which is why the propellers'
speed is its own input (`np`) rather than read from power.

Things in the rows a build should know:

- **The Kajaani clip (fs680325) was recorded aboard** [targets 10.3]. Its propeller speeds and the gearbox orders
  hold either way, but its harmonic levels (turboprop-methods 1.3, "ATR static") are cabin spectra, not exterior
  targets.
- **Gearbox line B (72.9 orders) was not found outside**: at 450 m on the approach it is below the noise, while line
  A is heard at 47.7 ± 0.6 orders, 6-16 dB prominent [targets 5.2]. Treat B as aboard-only until heard outside.
- **The 0.95-1.1 kHz approach tone**: targets 5.2 read it as gearbox line A, locked to the propeller; atr72-sources
  3.7 saw its ratio to the blade rate wander 7.74-7.85 and suspected the gas generator. Either reading is open.
- **ATR 42-300 approach**: seven FedEx clips give 67-79 Hz at the source, median 73 Hz, so 91 % rather than 100 %
  [targets 4.2; camera sound, fits unreliable]. Its row has `npApproach` 0.91.

## What the 3D view sets (`include/engines3d.h`, `src/view3d.cpp`)

- **`enginesOf()`**: AT75, AT76 → `PW127_568F`; AT72 → `PW124_14SF`; AT73 → `PW127_247F`; AT44, AT45, AT46 →
  `PW127_568F_42`; AT43 → `PW120_14SF`. `kind = TURBOPROP`, `engines` from the row. AT72 is the four-bladed -200 by
  ICAO Doc 8643 and by the app's own `aircraft_db.cpp`; if a database files -500s as AT72 they are heard with four
  blades (open question).
- **`ask()`** hands a turboprop to `askTurboprop()`, which sets `power`, `np` and `reverse` from the row:

  | Phase (what the view knows) | power | np (ATR 72-600) |
  |---|---|---|
  | on the ground, not taking off (stand, taxi, the roll below 60 kt) | 0.05 + kt/150, at most 0.17 | 71 % (beta) |
  | lined up or rolling (the runway simulation's take-off, or above 30 kt) | 1.0 | 100 % |
  | landing roll above 60 kt | `reversePower` 0.70, `reverse` | `npReverse` 100 % |
  | in the air, the simulation's take-off (to its hand-over at 1,500 ft over the field) | 1.0 | 100 % |
  | climbing (> 300 ft/min) otherwise, a go-around too | `climbPower` | 82 % |
  | the simulation's landing, or coming down (< −400 ft/min) under 3,000 ft over the field | `approachPower` | 100 % |
  | coming down from height | `descentPower` | 82 % |
  | level under 3,000 ft over the field / level higher | `levelPower` / `cruisePower` | 82 % |

  The hand-over at 1,500 ft sits in the published 800-1,500 ft for the power reduction (ANP's default 1,000 ft); the
  glide to 82 % then takes about 8.5 s at the synth's `npGlide`. A go-around is heard as a climb: the view cannot
  tell one.
- **`airframe()`**: `spoilers` nought for a turboprop without ground spoilers.
- **`enginesUpPerScale()`**: the row's `axisUp` whatever the look's scale (2.80 m; the view's 2.5 × scale gave an
  ATR 72 2.50 m and an ATR 42 2.08 m).
- **`view3d.cpp`**: `apu` (hotel mode) for a turboprop with `hotel`, by the airliner's rule: at its stand, for 90 s
  after its engines start, and taxiing in.
- **A view quirk, not changed**: after the runway simulation hands a take-off back at 1,500 ft, scene3d's
  `configFor()` treats the climb as an arrival and puts the gear down again from there to 600 m (any type). The
  preview's scenes copy it (`viewConfig()`), so a build hears what the device would.

## The stub (`src/engine_noise.cpp`, search for THE TURBOPROP)

- **Four places, each empty**, in the same spots as the fighter's:
  - `propRestart()` from `restart()`;
  - `propStep()` every 2 ms from `update()`;
  - `propSlowStep()` every 32 ms from `slowStep()`, in place of the airliner's sums, after `gain_`, `heldDb_` and
    `air_` are worked out;
  - `propRender(mix, lateA, lateB, noiseA, noiseB, rng, n)` from `render()`, before the air's muffling.
- **Shared already**: the fade in and out; the placed way, with the propellers `halfSpan` either side (4.05 m, not
  5.75); `engines()` (N1 is the gas generator's NH share here, from the neutral `ENGINES` rows); the start's spool;
  `tyresStep()`'s timing; the muffling, echo, level, high-pass and limiter.
- **Left for each build to decide**:
  - `engines()` still holds N1 at 50 % for 2 s before take-off power on the ground, the A320's procedure. For a
    turboprop's NH that is wrong; gate it by kind if `n1_` is used.
  - The airliner's render loops (bank, combs, tones, thump, squeal) run only for `on` (`TURBOFAN`). Widen them to
    `on || prop` (the recipe's 13.6), or copy what is needed into `propRender()`, as the fighter does. The squeal's
    and the thump's levels are the A320's, so set your own.
  - The closing speed's 0.25 s easing (the recipe wants 35-60 ms, as the fighter's) and the echo clamp (±6 dB below
    400 Hz, recipe 11).
  - **History for a far source**: `Synth::HEARD` keeps 256 snapshots, about 5 s at 50 sets a second. The cruise
    flyover from the ground (5.2 km, 15 s late overhead) is heard from where it was 5 s ago, its pass about 10 s
    early. Only that scene and the TOWER camera under a high aircraft reach it.

## The scenes (`preview.cpp`, `atrScenes()`)

Each is fed as the 3D view feeds a turboprop: power, `np` and `reverse` from `engines3d::ask()` on the path's
speed and climb and the scene's runway movement; gear and flaps as the view's set-up gives them; spoilers through
`engines3d::airframe()`; `size` 27.17/37.6 (ATR 72) or 22.67/37.6 (ATR 42); axes `axisUp` up. The phase tables are
atr72-sources section 14.

| Scene | s | Geometry | What the view gives |
|---|---|---|---|
| atr72-start-idle | 90 | apron, 35 m at the front quarter, 2 m up | hotel mode 0-60 s; `running` from 8 s; idle in beta (the view keeps hotel mode 90 s after a start; 52 s here, to hear the brake come off) |
| atr72-taxi-past | 30 | 8 m/s, 35 m off, abeam at 15 s, 1.5 m up | beta, power 0.15, 71 % |
| atr72-takeoff | 75 | beside the runway 700 m from the line-up, 120 m off, 4 m up | beta until 4 s; lined up 4 s (power 1, 100 %); rolls at 6 s, 1.8 m/s²; past the camera at 34 s, 50 m/s; off at 38 s, 57 m/s; climbing at 6° |
| atr72-climbout-flyover | 50 | under the climb, overhead at 22 s, 400 m up, 75 m/s, 6° | take-off power and 100 % until 1,500 ft at 29 s; then 0.886 and 82 % going away |
| atr72-cruise-flyover | 70 | FL170 (5,182 m) at 141 m/s (275 kt), overhead a microphone 1.5 m up at 35 s | cruise 0.861, 82 % (see History above) |
| atr72-approach-side | 75 | 300 m up and 300 m off at the closest (424 m), 3° at 60 m/s (117 kt), abeam at 40 s: fs315660's geometry | approach 0.22, 100 %, gear and flaps |
| atr72-approach-behind | 55 | the CHASE camera: 39.7 m back (55 sizes), 10° up from its middle; from 120 m up, touchdown 38 s | approach; reverse 0.7 at 100 % to 60 kt (50 s); beta |
| atr72-touchdown-reverse | 75 | beside the touchdown zone, 50 m off, 2 m up; from 2.4 km out, touchdown 40 s | approach; reverse to 60 kt (52 s); braking 2.5 m/s² to taxiing by 61 s |
| atr42-takeoff | 75 | ATR 42-500/-600: 600 m from the line-up, 120 m off | as atr72-takeoff; off at 54 m/s after 810 m, climbing at 8.5° (1,800 ft/min [P]), its take-off power 1.0 |
| atr72-cruise-cinema | 45 | the view's CINEMA high up: chase (70 sizes), wide (230), from below (120), 10 s each, at FL170 | cruise 30 s, then down at 2,000 ft/min, flight idle, 82 % |

`ENGINE=` for these: `atr72-600` (the default; `atr42-500` in atr42-takeoff), `atr72-200`, `atr72-210`, `atr42-500`,
`atr42-300`. A turbofan's name leaves them alone, as an ATR's leaves the airliners' scenes alone. `TRACE=1` adds a
line a second with the row and the propellers' speed asked.

## Tools

- `render_atr72.sh <worktree> <outdir>`: the ten scenes, `-atr72-200` take-off and taxi, `-atr42-300` take-off, and
  the A320's take-off in `ref/` (26 WAVs and 2), built with `ENGINE_NOISE_DEBUG`; `summary.txt` has dBFS every 2 s
  and each scene's limiter share (keep it under 0.1 %).
- `render_same.sh <worktree> <reference tree or commit> <outdir>`: every scene the reference makes, in 15 runs (no
  ENGINE=, each turbofan, FOPP, SYNC, DEVICE, WHO=7, POWER=0.85), compared byte for byte with the worktree's, the
  loudness lines too. Run it against `atr72-base` (or 28518ba) after any change to shared code. About 20 s; KEEP=1
  keeps the WAVs (4 GB).
- `bench.cpp`: seven ATR cases after the A320's (PC time).
- `atr72_cases.inc`: seven ATR cases for `eufi-tools/cost.sh` (the P4's time by its model), included at the end of
  `eufi_cases.inc`, so `cost.sh <worktree> <out>` runs them after the A320's and the Typhoon's.
- `engines3d_check.cpp`: now also every ATR type's row, axes height and size, and power, `np` and reverse through 19
  phases for the ATR 72-600, 42-600 and 72-200. Its baseline for the unchanged types now includes the 777's engines,
  which the 777's build had left failing (1 check).

## Checks on this base

- **render_same.sh against 28518ba** (the 777's): 960 identical, 0 differ, 0 missing - every A320, Typhoon and 777
  scene and the splash, full range and `-tab5`, in all 15 runs, with identical loudness lines. A deliberate 0.02 %
  change to `LEVEL` made all 960 differ, so the comparison is not empty.
- **a320_pccheck**: 106 identical against the approved 9d83380 renders.
- **a320_bitcheck (P4 under qemu)**: 5 of 7, the same seven checksums as 28518ba's. **The 777's build itself
  changed the A320's P4 sound for the V2500 and LEAP-1A take-off cases** (c5a233e2 and 6991f2b0, approved eff345bf
  and 6c1a8df4); v1.5.1-dev (c9db0de) gives 7 of 7. The PC renders stayed identical, so most likely the P4's
  fused multiply-adds were rearranged by the 777's new per-engine terms [I]. Not changed here.
- **cost.sh (P4 model)**: the 17 cases both trees run (A320, device, Typhoon) give identical checksums. The silent
  turboprop costs 0.45-0.47 ms warm a block (the shared noise draws, way, echo, muffling, high-pass and limiter),
  against the A320 take-off's 1.24-1.27 ms. `sizeof(Synth)` 52,084 B (the 777's 51,048).
- **engines3d_check**: 349 checks, 0 failed.
- **Firmware**: `view3d.cpp`, `sound.cpp` and `engine_noise.cpp` compile with the firmware's own commands, and the
  release image builds: 12,180,064 B (the 777's 12,178,208; the partition holds 14,680,064).

## Open questions carried for the user

From turboprop-methods 16, with the base's own:

1. The speaker lift for the missing fundamental (`vbDb` in the recipe): physics only, Moliner's bounds, or more.
2. AT72 from the aircraft database: four-bladed -200s, as ICAO says, or -500s filed under it.
3. Hotel mode at the stand: as wired (the view's `apu`), or skipped by the synth.
4. The gearbox line's level, and whether line B (aboard only) is used at all.
5. After take-off: the 100 → 82 % glide at the hand-over (1,500 ft), as wired, or earlier.
6. Reverse: power and speed inferred (none recorded cleanly).
7. When crews select 100 % on the approach: wired from 3,000 ft over the field on the way down; measured at about
   1,000 ft (Schiphol); ANP's approach class looks like 82 %.

## Build "recipe" (atr72-recipe, 27 Oct, first version for review)

turboprop-methods 13 followed on the A320's resonator design, every figure fitted to `targets/atr72.md` and `.json`
and tagged in the code; where the two differ the outside recordings win (the harmonic series' shapes, the steady
line's K, the gearbox's order, line B unplayed). Renders: `~/.cache/terrain-lab/sound/renders/atr72-recipe/` (the ten scenes and
the three four-bladed ones, full range and `-tab5`); the fitting scripts in
`~/.cache/terrain-lab/sound/atr72-recipe-work/scripts/` (`round.sh` renders, measures and judges; `multi.sh` does
seven airframes; `judge.py` scores the judging list, targets 9).

What it is (`src/engine_noise.cpp`, THE TURBOPROP; nothing replayed):

- **Harmonics 1-6**: each a resonance on noise, 0.2 and 0.4 Hz wide for the first two, 0.85 Hz and 0.35 more a
  harmonic above, 2.2 times that standing [M: targets 4.4]. Governed, the fundamental has a steady line under it
  (K = 4, as heard outside: the cabin's 5.6-10.6 was measured aboard) and the first two swell and fade with the two
  propellers' path difference and the synchrophaser's phase (one per airframe). In beta the two propellers run at
  their own speeds: two resonances for each harmonic heard at 300 Hz or more, one, widened, under it.
- **From the 7th up**: one comb (the buzz-saw's delay line), only while its 7th line is within 10 dB of the rush in
  its band: on the ground and at take-off power, never in clean flight.
- **The series' shape by phase**: five envelope tables (clean flight, climb, standing at take-off power, beta,
  reverse), blended by power, the static share (1 standing, 0 by 40 m/s), governed or beta, and reverse; two lobe
  pairs (the first two, the rest) narrowing ahead and behind at low power, broader at high power and standing.
- **The speaker lift** (`PR_LIFT` 1, the recipe's `vbDb` default): the first four harmonics heard at 300 Hz or more,
  up to the 6th, raised to the fundamental's level less 10 dB an octave of harmonic number, never past less 3, with
  the harmonic's own lobe.
- **Propeller speed**: `Sound::np` followed up from beta critically damped (2.2 s 10-90 %) and down in a straight
  glide at `npGlide`; in beta ungoverned, wandering 1 % over 0.8 s, the right 0.5 % off; governed, hunting 0.12 %
  together. Feathered on shutdown, the line 20 % down in a second; hotel mode plays the right gas generator alone.
- **Lines**: the gearbox's line A at 47.7 shaft orders, 22 dB under the fundamental on the approach (line B,
  aboard only, not played); two gas-generator whines at 5,682 Hz x NH, 1-3 % apart and wandering, and a 250 Hz
  haystack at 5.9 kHz in the air.
- **Broadband**: the A320's eight bands and a ninth at 140 Hz (the low rush, the only sound under 250 Hz besides
  the harmonics): the loading rush (power cubed, 450 Hz), the blades' edges (speed to the fifth, 1.2 kHz), exhaust,
  intakes, the reverse roar (630 Hz), the A320's airframe 7 dB down. Each source on the A320's surge processes at
  its own depth and pace, the rush's surge deeper as the bearing turns; shimmer at 0.67 and flicker at 0.6 of the
  A320's (a turboprop churns more and its octaves move together more).
- **Its way to the ear**: Doppler eased in 35 ms; each tone's ground echo free to swing 6 dB at 300 Hz and under,
  2 dB from 1.5 kHz; the squeal at 0.3 and the nose wheel's thump at 0.5 of the A320's.
- **Shared code**: one call added in `render()` (`propTones()` after the echo); everything else is in the
  turboprop's noinline functions. The rows' `gearA` is 47.7 for all five (was 48.8, the aboard figure). Preview:
  `NOREVERSE=1` rolls a landing out in beta, to set beside the reverse.

Measured (each render through the targets' own `measure_atr72.measure()`, pass centres and blade-rate track as the
references were cut):

- **Judging list: 25 of 29 rows in range** (`judge.py`, its tolerances beside each row). The four out are the
  approach's harmonic excess with the lift on: the 3rd-5th stand 7-11 dB over the rush where the recording's stand
  0-4.5. With `PR_LIFT` 0 the same four rows are in range (coming: 14.5 / 8.6 / 3.6 / 3.6 / 2.0 / -0.6 dB).
- **Inside the references' own figures**: blade rate 119.7 Hz at the source on the approach; propeller speed 850 to
  1,199 rpm in 2.2 s 10-90 % (about 3 s in all), overshoot 0.8 %, and the glide 1,199 to 982 rpm at 2.4 %/s,
  linear (r -0.987); one line a harmonic while governed; the take-off roll from ahead 14.7 / 11.2 / 6.9 / 7.1 / 6.0
  / 3.7 dB over the rush (references 15 / 12 / 9 / 8 / 8 / 3); the gearbox line at 7.97 x the blade rate, 11.4 dB
  prominent (median); the whine at 5,661 Hz, 13.6 dB, two lines 2.2 % apart in 71 % of frames; ground fading
  4.8-5.4 dB (amplitude SD over mean 0.47-0.52), the governed fundamental 3.3-3.4 dB (0.35-0.38); harmonics fading
  independently (r -0.07 to 0.04); pitch wander 0.35-0.52 % governed; churn on the approach 1.5-1.8 dB (far out
  1.3), climb 1.8-2.1, taxi 0.8; approach centroid 953 Hz at -7.0 dB/oct, climb 703 Hz at -9.3; the shutdown line
  21 % down in 1.05 s.
- **In by the judge's tolerance only**: Doppler x1.41 (145.6 to 103.3 Hz; x1.33 at 450 m, this scene 424 m and
  faster); the slow taxi six harmonics over 6 dB (7-11), up to 681 Hz (535-620: the Dash 8s' slower blade rates),
  the fundamental -3.5 dB (0-5); take-off power 8.6 dB over the approach at the same slant (ANP 9-12; 9.8 with the
  lift off, which leaves the climb as it is); reverse +4.1 dB over the same roll in beta (+8, one far camera) and
  SEL +0.8 dB (+1.4).
- **Over seven airframes** (`multi.sh`): 261 of 513 per-phase medians inside the references' ranges (most phases
  have one or two reference clips).
- **Levels** (Tab5 band, loudest 2 s, seven airframes, against the A320 from the same place): take-off 6.1 dB under
  (5.5-6.9; ANP 5-8); approach 3.1 under (1.5-5.0; ANP 4-6), 1.0 dB of it the lift (4.3 under with it off); taxi 0.8
  over (0.3-2.0). Limiter: nothing rounded off in any scene but the reverse's 0.0013 %.
- **Cost** (`cost.sh`, P4 model, a 40 ms block): take-off roll 392,679 cycles, 1.20-1.24 ms warm (the A320's
  take-off 402,864, 1.24-1.27); approach 391,591; the worst, the taxi in beta, 427,261, 1.31-1.35 ms, the A320's
  touchdown with reversers 428,965, 1.32-1.35. `sizeof(Synth)` 52,872 B (+788 on the base).
- **Unchanged elsewhere**: `render_same.sh` against 28518ba 960 identical, 0 differ; a320_pccheck 106 identical;
  a320_bitcheck 5 of 7, the same checksums as the base (the 777's V2500 and LEAP difference, above); `cost.sh`'s 17
  A320, device and Typhoon checksums identical to the base's. engines3d_check 349 checks, 0 failed. The release
  image builds, 12,204,880 B (+24,816).

To listen: `renders/atr72-recipe/` as above; `checks/lift-off/` the same 26 with `PR_LIFT` 0, to hear the lift;
`checks/` the standing scene (x-atr72-static, 300 m and 50 m: ground idle, take-off, climb, idle, shutdown) and the
landing without reverse; `checks/airframes/` the five matched scenes for seven airframes.

Open questions for the user, by ear on the Tab5:

1. **The lift**: on (the recipe's default) gives the propeller's low pitch on the speaker, at the price of the
   approach's 3rd-5th standing 7-11 dB over the rush (the recording's 0-4.5) and 1 dB of loudness; off is the
   physics.
2. **The fundamental through the speaker**: the shared output high-pass (Butterworth, 180 Hz) takes 7.8 dB off a
   120 Hz fundamental and 13 dB off beta's 85 Hz, in the full-range renders too; left as it is so the other aircraft
   stay byte-identical, and the envelopes fitted through it.
3. **Still as the base left them**: the cruise flyover heard from 5 s back (`Synth::HEARD`); AT72 as the four-bladed
   -200; hotel mode as wired; the 100 to 82 % glide at 1,500 ft; reverse power and speed inferred; 100 % from 3,000 ft
   on the way down.

## turboprop-v2: the ATR's first faults fixed (27 Oct, on ace261f)

On the approved recipe voice (lift on, as approved); renders before and after in
`~/.cache/terrain-lab/sound/renders/turboprop-v2-fix/` (`before/` from ace261f), the checks in each `checks/`;
work, measures and judge output in `~/.cache/terrain-lab/sound/turboprop-v2-work/`.

- **Take-off power's spool** (`PR_SPOOL_GROW` 0.30, `PR_SPOOL_FLOOR` 0.05, `propDoing()`): the power rises by at most
  30 % of itself a second and 0.05 more, ground idle to take-off in about 5 s [P: 14 CFR 33.73, 5 s flight idle to
  95 %]. The level's 10-90 % rise, Tab5 band [M]: ATR 72 take-off 2.25 -> 4.5 s (+24.5 dB, steepest 20 -> 9 dB/s in
  0.25 s frames); ATR 42 1.75 -> 4.5 s; ATR 72-200 1.5 -> 5.25 s; ATR 42-300 2.25 -> 4.5 s; standing check 2.25 ->
  3.75-4.25 s. The Q400 at Itami 16 dB over about 4 s, the A320 from the same place 17 dB over about 7 s [M].
- **Propeller speed's rise** (`PR_NP_RISE_W` 1.2, was 2.0): 10-90 % 2.2 -> 3.0 s on the standing check (target
  2.5-3.5 s). The speed itself never overshoots (it is clamped at the speed asked); the 0.8-1.3 % the judges read is
  the 0.25 s tracker's noise on the governed line (its SD 0.48-0.53 % over 25-39 s, its maximum some 9 s after the
  rise) [M]. Now 0.5 % by the same measure.
- **Approach against take-off** (`PR_FLIGHT_POWER_DB` 3.5): the whole sound in the air 3.5 dB a decade of power
  share, nought at take-off, fading out on the ground with the static share: approach -2.3 dB, climb -0.2, cruise
  -0.2; the approach's spectrum moved as a whole. Take-off power over the approach at the same slant 8.6 -> 10.9 dB
  (builder's measure, in band; ANP 9-12 [P]); the approach in the Tab5 band -34.9 -> -37.2 dBFS, the A320's from
  the same place -31.3, so 3.6 -> 5.9 dB under it (ANP 4-6). Every approach harmonic row as before.
- **Cruise hiss** (`PR_CUT_*`, `propTones()`): the hiss at FL170 was not the bands' air-loss floor (lowering it,
  `PR_AIR_FLOOR_DB` -95 in the turboprop's own loop, changed nothing measurable; kept, physics) but the tones'
  resonances: each feeds its noise straight to its output, a white skirt some 30 dB under the line, wider as lines
  widen with distance. Beyond 3 km the tones alone go through two one-poles, corner 8 kHz at 3 km falling 1.8
  octaves a km to 400 Hz, each tone's input raised by what they take at its pitch. Cruise, full range, loudest 2 s:
  4-7 kHz under 0.5-1 kHz 9.5 -> 29.3 dB (Tab5 band 8.7 -> 34.9); 2 kHz octave 5.6 -> 19.9 dB, 4 kHz 8.0 -> 27.4 dB
  under. Cruise against the 424 m approach, Tab5 band: 17.1 -> 18.8 dB under (base's law 15-19). Nearer than 3 km
  (approach at its closest, climb-out, taxi, stand, cinema) the path is not run.
- **Judge** (`judge.py`, before -> after): 25 -> 24 of 29. Changed rows: Np rise 2.2 s -> 3.0 s; take-off against
  approach 8.6 -> 10.9 dB; reverse over the beta roll +4.1 -> +5.5 dB (London City +8). Out: the shutdown row on the
  50 m standing check, -21 % in 1.05 s -> -33 % by 0.55 s: the shutdown's code is untouched, but the waveform after
  the take-off at 15 s is a new noise realisation (uncorrelated with before from 14 s), and the row's 0.05 s
  peak-follower is realisation-sensitive [I: not proven by a seed sweep - open]. The approach's four lift rows out as
  approved.
- **Checks**: render_same.sh against atr72-base 960 identical (every A320, Typhoon, 777 and splash render, 15 runs;
  the 300 that differ are the ATR scenes, and every non-ATR loudness line is identical); a320_pccheck 106 identical;
  a320_bitcheck the base's seven (5 of 7; c5a233e2 and 6991f2b0 the 777's); cost.sh's 17 non-ATR checksums
  identical to ace261f's; ATR worst the taxi in beta 428,337 cycles (was 427,261; the A320's touchdown 428,965), the
  far path (not in the cases) some 640 x 8 cycles a block more [I]; engines3d_check 349 checks, 0 failed; release
  image 12,204,768 B; `sizeof(Synth)` 52,884 B (+12).
- **Not done tonight, still open**: the stand (propellers thin, the left one appearing at idle speed); the
  touchdown beside the runway (+18 dB in the last 0.9 s); the climb going away too dull; reverse's roar at 315 Hz and
  the gearbox line and whine through it; the whine's lower line at 5,530 Hz.

## turboprop-v2: the other turboprops as rows of the ATR's voice (27 Oct, on f1ab630)

Twenty-three types on the approved ATR voice (lift on, the ATR's character untouched), each a value of `Engine`
appended, a row of `PROPS` with its figures and their sources, a neutral `ENGINES` row and its ICAO type(s) in
`engines3d::enginesOf()`; research in `~/.cache/terrain-lab/sound/research/turboprops/` (regional, business and
military, `.json` and `.md`); renders in `~/.cache/terrain-lab/sound/renders/turboprop-v2-rows/<engine>/` by
`render_turboprops.sh` (take-off beside the runway, approach from beside, taxi past; full band and `-tab5`; the ATR
72-600's same three in `atr72-600/`); work and logs in `~/.cache/terrain-lab/sound/turboprop-v2-rows-work/`.

- **Rows**, commonest first (ENGINE= name, ICAO): Dash 8 Q400 (q400, DH8D), Saab 340 (saab340, SF34), Twin Otter
  (twinotter, DHC6), King Air 350 (kingair350, B350 BE30), King Air 200 (kingair200, BE20), PC-12 (pc12, PC12),
  Caravan (caravan, C208), Beech 1900D (beech1900, B190), TBM 700/850 (tbm700, TBM7 TBM8), TBM 900 (tbm900, TBM9),
  C-130J (c130j, C30J), A400M (a400m, A400), Dash 8-100/-200 (dash8-100, DH8A DH8B), Dash 8-300 (dash8-300, DH8C),
  C295 (c295), CN-235 (cn235, CN35), Fokker 50 (fokker50, F50), Dornier 328 (do328, D328), EMB 120 (emb120, E120),
  Saab 2000 (saab2000, SB20), King Air 90 (kingair90, BE9L BE9T), Kodiak (kodiak, KODI), PC-6 (pc6, PC6T PC6).
- **Left silent**, as the research advises: the Jetstreams (JS31, JS32, JS41) and the Dornier 228 (D228) - their
  TPE331's single-shaft shriek is what they sound like and is not measured (a row with whineHz 0 would play the
  ATR's 5,682 Hz); the C-130H (C130, L100) and P-3 (P3) - single-shaft T56s, NH held at 100 % even idling, which
  the voice can't yet do; the T-6 (TEX2) and PC-21 (PC21) - they can't reverse, and `askTurboprop()` reverses every
  landing roll.
- **PropData** gains `levelDb`, `ahead`, `halfSpanOuter` and `handed`, all nought on the ATR's rows:
  - `levelDb`, the whole sound against the ATR 72-600 (`gain_`, after the flight-power law). From EPNdB
    certificates: the mean of the lateral, flyover and approach differences, at 0.6 - the ATR's own level was set
    to the A-weighted gap to the A320 (ANP 5-8 dB, mean 6.5) where their lateral gap is 11.1 EPNdB, 0.59 of it [I].
    Taken whole they put the C-130J 11 dB over the A320's take-off and its taxi's limiter at 2.5 %. The business
    types' are take-off dB(A) at 300 m against the ATR's 81, taken whole [P/I]; the rest estimates [I].
  - `ahead`: a single's propeller on its nose, 0.4 of its length ahead of its middle [I] (`placedWay()`).
  - `halfSpanOuter`, a four-engined one's outer pair (C-130J 9.7 m, A400M 11.9 m; inner 5.0 and 6.2) [I].
  - `handed`: the A400M's two on each wing turning opposite ways, down between the engines [P*].
- **The voice for what the ATR never needed** (`propSlowStep()`, each in its own branch, so the ATR's arithmetic
  is untouched):
  - a single (`engines` 1): one propeller - never apart in beta, no pair lobes (pair 1), no spread, one gas
    generator's whine at both's power (+3 dB), the right one's off;
  - a twin without a synchrophaser (`synchro` false, the Twin Otter): governed, its right propeller 0.4 % off
    (`PR_UNSYNC` [I]), the two apart in the air as in beta, the steady lines' phase moving at their difference
    - beating at some 0.4 Hz;
  - four engines: governed, all four on one line, the steady parts' lobes the coherent sum of four sources
    (`fourPair()`): the inner pair's path difference and phase as the twins', the outer's in proportion to its
    span, each its own synchrophaser phase (drawn after the twins' draws, so the ATR's are as they were) - 1 +
    0.7 of the coherent sum's departure from the powers', as the twins' 0.3-1.7 is, so 0.3-3.1; the A400M's outer
    pair's phase against the inner's moving by 2kB times the bearing about the axes [I: a spinning mode's phase],
    the left wing's one way and the right's the other. In beta each harmonic one resonance as wide as the four's
    speeds' spread (twice the twins'): the twins' two resonances took the A400M's taxi to 429,875 cycles, over the
    A320's worst; now 394,123.
- **Measured on the renders** [M]: every row's take-off peak sits at its own blade passing times the roll's Doppler
  (1.02-1.04), and its taxi's strongest line at a whole harmonic of its blade passing at its ground idle (e.g. Q400
  197 Hz = 3 x 66, King Air 200 312 Hz = 4 x 77, PC-12 254 = 3 x 85, A400M 175 = 2 x 87, C-130J 148 = 2 x 73.5).
  Loudest 2 s, full band (dBFS; take-off / approach / taxi): ATR 72-600 -19 / -35 / -26; Q400 -20 / -35 / -27;
  Twin Otter -22 / -38 / -33; PC-12 -21 / -33 / -33; Caravan -21 / -37 / -31; C-130J -10 / -28 / -18; A400M
  -15 / -28 / -18 (all in `levels.txt` beside the renders). The limiter rounds 0.33 % of the C-130J's take-off and
  0.017 % of the A400M's, under 0.01 % of every other render.
- **Identity**: `render_atr72.sh` from this tree gives all 26 ATR renders and the A320 reference byte-identical to
  f1ab630's (`renders/turboprop-v2-fix/`); `cost.sh` gives all 24 of f1ab630's checksums, ATR cases too (an
  earlier draft shared a product between the twins' pair and `fourPair()`, which changed the governed ATR cases'
  checksums on the P4 by its fused multiply-adds; written apart, they are as they were).
- **Checks**: render_same.sh against atr72-base 960 identical (the 300 that differ the ATR scenes; every non-ATR
  loudness line identical in all 15 runs); a320_pccheck 106 identical; a320_bitcheck the base's seven (5 of 7, the
  777's two as before); cost.sh's 17 non-ATR checksums unchanged, the A320's touchdown 429,085 cycles here (428,965
  in f1ab630's run, its code untouched: the cache model's layout), the ATR's taxi 428,572, every new case under
  both - C-130J taxi 394,126, A400M taxi 394,123, roll 388,911, approach 382,066, PC-12 taxi 392,900, Twin Otter
  taxi 414,077 and climb 379,319, Q400 taxi 414,072; engines3d_check 442 checks, 0 failed (the A400M, a turboprop
  now, out of the "as before" list for the C-130H; every new type's row, engines, axes and phases; the silent
  ones still silent); `sizeof(Synth)` 52,892 B (+8).
- **Open**: the C-130J's take-off limiter share - 0.33 % on the renders' airframe, 0.04-0.13 % on six others
  (WHO=1-6), its four co-rotating fundamentals at 102 Hz coherent at their lobes as each airframe's synchrophaser
  phases have them (the A400M 0.017 % on the renders', 0.05-0.10 % on three others); every PT6, CT7, AE 2100 and TP400 whine is the ATR's 5,682 Hz (none measured) and only the PW100 rows have
  its gearbox line; the C-130J's ground idle (0.72, its low-speed ground idle, if it has one [I]); the singles'
  `ahead` and every axis height and spacing are estimates; the TPE331, T56 and no-reverse types still to make.

Copyright (c) 2026 Craig Smith. Part of Overhead's sound engine: PolyForm Noncommercial 1.0.0 ([LICENSE-POLYFORM.md](../../LICENSE-POLYFORM.md)), not the MIT licence of the rest of the repository.
