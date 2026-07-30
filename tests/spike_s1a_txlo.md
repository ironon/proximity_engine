# Spike S1a — the part that still needs a human

**Most of S1a is done.** The level table, the receiver threshold, the cliff shape,
the slot-vs-PDU margin and the active-scan hazard were all measured on
2026-07-29 with `bench/pdr_sweep/`; the results and their caveats live in
`prox_v2_spikes.md` §S1a. `BEACON_TX_LO_DBM` has been moved from the spec's
placeholder of −21 dBm to **−12 dBm** on the strength of that.

What is left is the one thing a bench with both boards tethered to the same
laptop cannot do: **measure two positions.** It should take about ten minutes.

## Why this is quick now

The original plan was a 12-level sweep at each of two positions. That is no
longer necessary. The sweep established that slot-hit is a function of *received*
power — 100 % at ≥ −95 dBm, ≈50 % at −97, 0 % by −101 — and that threshold is a
property of the C3's receiver, so it is the same in your bedroom as on the bench.
Only the path loss is per-install, and the full-power reference slot measures
that for you.

So instead of sweeping, read one number at each position.

## Procedure

1. **Flash the bench pair** (any two C3s; they are interchangeable):

   ```
   cd proximity_engine/bench/pdr_sweep
   ~/.platformio/penv/bin/pio run -e tx -t upload --upload-port /dev/ttyACM1
   ~/.platformio/penv/bin/pio run -e rx -t upload --upload-port /dev/ttyACM2
   ```

   Put the TX board where the anchor will live. The RX board is the stand-in for
   the watch and is the one you carry.

2. **Start the schedule on the TX and leave it running for the whole session:**

   ```
   # on TX:  sweep on
   ```

   This is not optional and it is easy to miss. `fixed <dbm>` only pins the level
   *inside* the slot loop, and that loop only runs while the sweep is on — with
   the sweep off the board emits no LO slots at all, and the receiver reports a
   flawless `lo_pct=0`, which looks exactly like "the level is too low" rather
   than "nothing was transmitted". Two tells that this has happened:
   `cyc_range=1..1` (a single cycle_seq forever) and `lo_pdu=0` in every row.

3. **At the INSIDE position** (where you actually sit or sleep), rest the RX
   board on something and run `reset`, wait ~20 s, `report`.

   Read `hi_rssi` — the full-power reference slot, which is +9 dBm regardless of
   what the sweep is doing. Call it `RSSI_INSIDE`.

   > The `dbm` column is **not** the reference slot's power. It labels the ladder
   > entry for the row's level, i.e. the power of the *swept* slot. `hi_rssi` is
   > always the +9 dBm reference.

   Repeat 3–4 times with the board moved a few cm between captures, and take the
   **median**. A single capture can land in a fade; see the traps below.

4. **At the EDGE position** (just past the tolerance you want to enforce), same
   thing. Record `RSSI_EDGE`.

5. **Check there is anything to work with.** This is a *gate*, not the answer:

   ```
   RSSI_INSIDE - RSSI_EDGE  >=  ~6 dB
   ```

   If the two positions differ by less than that, no TX level separates them and
   PDR cannot be the feature that distinguishes these two spots — say so rather
   than picking a level anyway. (This is a real possible outcome for a small
   bedroom, and it is worth knowing.)

6. **Compute the level.** This is a *different* number from step 5 — it is not
   the separation. Round **down** to a multiple of 3 (rounding up is silently
   ignored: NimBLE rounds toward higher power).

   ```
   BEACON_TX_LO_DBM  =  -91 - RSSI_EDGE          <- note: minus RSSI_EDGE alone
   ```

   Worked example, because the sign juggling is easy to get wrong: with
   `RSSI_EDGE = -94`, this is `-91 - (-94) = -91 + 94 = +3 dBm`. It is *not*
   `94 - 71 = 23`; that subtraction is step 5's gate.

   Sanity check the answer: the level should land somewhere in the −24…+9 ladder.
   If you get a number above +9 you have almost certainly computed the separation
   instead.

7. **Confirm it.** With `sweep on` still running, re-capture at both positions
   and read the row for your chosen level. Pass criterion, unchanged from the
   original spike:

   - **≥ 90 %** `lo_pct` at INSIDE
   - **≤ 30 %** `lo_pct` at EDGE

   One 4-minute sweep per position gives you the entire curve, so you can see the
   whole cliff rather than one point on it — worth the extra time.

8. Write the result into `prox_v2_spikes.md` §S1a and set `BEACON_TX_LO_DBM` in
   `src/proximity.h`.


### result
INSIDE MINUS EDGE TEST LOGS
this is for fixed=9


RX ? unknown: report
fixed 9
RX ? unknown: fixed 9
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,525,0,-71,0,0,0,0
RPT end
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,349,0,-94,0,0,0,0
RPT end


94 - 71 = 23 db worth of margin

round down to multiple of 3 -> 21 db

this is for fixed=21
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,2742,0,-87,0,0,0,0
RPT end
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,286,0,-93,0,0,0,0
RPT end


### Analysis of the above (2026-07-30)

**`lo_pct` was never measured in this session.** `sweep on` was not sent, so the
TX never entered the slot loop and emitted no LO slots at all — it just kept
re-advertising the static payload from `setup()`. The evidence is in the reports
themselves: `cyc_range=1..1` (one single cycle_seq for the whole capture),
`lo_pdu=0` in all four, and `hi_pdu` of 525/349/2742/286 being the same
advertisement counted repeatedly. So `fixed 9` and `fixed 21` both did nothing.
(`fixed 9` in the first block also went to the RX board — `RX ? unknown: fixed 9`.)

The `dbm` column reading `-24` in every row is the same artifact: with cycle_seq
pinned at 1, `bench_level_for_cycle(1) = 0`, so every packet lands in the level-0
row. The actual transmit power throughout was **+9 dBm**, the reference slot.

**What IS valid: the four `hi_rssi` readings.** They are genuine RSSI of a +9 dBm
advertiser at the two positions, which is exactly what steps 3–4 ask for:

| position | capture 1 | capture 2 |
|---|---|---|
| INSIDE | **−71** | **−87** |
| EDGE | **−94** | **−93** |

- **EDGE is solid: −93.5 dBm** (two captures agreeing within 1 dB).
- **INSIDE is not yet usable: −71 vs −87 is a 16 dB spread** at nominally the same
  spot — well past the ~5 dB flag in the traps below. One of those captures was
  almost certainly sitting in a fade (or the board moved/reoriented). This needs
  re-measuring as a median of 3–4 captures before any level is chosen.

**The +21 dBm figure is a misapplication of the formula.** `94 - 71 = 23` is
step 5's *separation gate*, not step 6's level. Step 6 gives
`-91 - (-93.5) = +2.5 → round down → 0 dBm`. (+21 is also outside the −24…+9
ladder, which is the sanity check now written into step 6.)

**Provisional answer, pending a good INSIDE number: `BEACON_TX_LO_DBM = 0`.**

Whether 0 dBm works depends entirely on which INSIDE reading is right, and the
two give opposite verdicts. With `PL = 9 - RSSI_HI`:

| if INSIDE is | PL_inside | LO rx at INSIDE @ 0 dBm | verdict |
|---|---|---|---|
| −71 | 80 dB | **−80 dBm** | far above the −95 reliable-hit threshold ⇒ 100 % hit |
| −87 | 96 dB | **−96 dBm** | below −95 ⇒ marginal, ~50 % hit ⇒ fails |

At EDGE either way: `PL_edge = 102.5 dB`, LO received `= -102.5 dBm`, below the
−101 zero-hit point ⇒ 0 % hit. **The EDGE side of the criterion is already
satisfied by construction; only the INSIDE side is in doubt.**

Solving the constraint properly, the usable window for `TX_LO` is

```
-95 + PL_inside   <=   TX_LO   <=   -101 + PL_edge  =  +1.5
```

which is `[-15, +1.5]` if INSIDE is −71 (comfortable, pick 0 or −3) and
`[+1, +1.5]` if INSIDE is −87 (empty in practice). So: **re-measure INSIDE.**

**One thing worth noting about this install regardless.** `RSSI_EDGE = -93.5` for
a *+9 dBm* transmitter means the full-power reference slot is itself only ~1.5 dB
above the −95 dBm reliable-reception threshold at the edge. That is fine — and in
fact it is precisely the band where PDR produces clean miss evidence, since
"covered" comes from hearing the HI slot while the LO slot is lost. But it means
the working band is narrow: a little further out and the HI slot drops too, at
which point PDR abstains rather than reporting misses. That is the designed
fail-safe direction, not a bug, but it does mean **PDR will hand off to abstention
quickly past this edge** rather than degrading gradually.

## result 2

anchor logs:
== S1a bench: TX role ===
TX ready. adv_itvl=50ms readback_pwr=9 dBm
TX send 'probe' then 'sweep on'
TX sweep START cycles_per_level=40 levels=12 cycle_ms=500
TX level=0 dbm=-24 cycle=1
TX ? unknown: sweeo
TX sweep START cycles_per_level=40 levels=12 cycle_ms=500
TX level=0 dbm=-24 cycle=1


watch logs
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,395,0,-83,0,0,0,0
RPT end
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,460,0,-80,0,0,0,0
RPT end
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,598,0,-75,0,0,0,0
RPT end
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,357,0,-83,0,0,0,0
RPT end

Median -> average of 83 and 80 because there is no middle -> -81.5 db
RSSI_INSIDE = -81.5 db


EDGE READING
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,275,0,-92,0,0,0,0
RPT end
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,387,0,-93,0,0,0,0
RPT end
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,189,0,-95,0,0,0,0
RPT end
reset
RX reset
report
RPT cyc_range=1..1 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,1,1,0,100,0,270,0,-91,0,0,0,0
RPT end

Median -> -92.5 DB = EDGE_DB

margin = 92.5db - 81.5db = 21db

BEACON_TX_LO_DBM = -91 - (-92.5) = -91 + 92.5 = 1.5 DB


i think i may have needed to run "probe" before sweep, as i dont have any lo_pct so i redid this entire trial

### result 2 part 2
reset
RX reset
report
RPT cyc_range=97..408 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 2,-18,23,23,0,100,0,106,0,-77,0,0,0,0
RPT 3,-15,40,40,11,100,27,184,11,-78,-101,-104,-99,0
RPT 4,-12,40,40,16,100,40,178,35,-81,-98,-102,-96,0
RPT 5,-9,40,40,25,100,62,184,43,-82,-98,-102,-94,0
RPT 6,-6,40,40,39,100,97,189,127,-83,-96,-101,-92,0
RPT 7,-3,40,40,40,100,100,188,161,-83,-94,-98,-91,0
RPT 8,0,40,40,40,100,100,189,174,-83,-92,-95,-89,0
RPT 9,3,40,40,40,100,100,183,176,-82,-88,-92,-83,0
RPT 10,6,9,9,8,100,88,41,37,-80,-84,-90,-79,0
RPT end
report
RPT cyc_range=97..511 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,32,32,0,100,0,148,0,-80,0,0,0,0
RPT 2,-18,23,23,0,100,0,106,0,-77,0,0,0,0
RPT 3,-15,40,40,11,100,27,184,11,-78,-101,-104,-99,0
RPT 4,-12,40,40,16,100,40,178,35,-81,-98,-102,-96,0
RPT 5,-9,40,40,25,100,62,184,43,-82,-98,-102,-94,0
RPT 6,-6,40,40,39,100,97,189,127,-83,-96,-101,-92,0
RPT 7,-3,40,40,40,100,100,188,161,-83,-94,-98,-91,0
RPT 8,0,40,40,40,100,100,189,174,-83,-92,-95,-89,0
RPT 9,3,40,40,40,100,100,183,176,-82,-88,-92,-83,0
RPT 10,6,40,40,40,100,100,190,180,-80,-83,-90,-79,0
RPT 11,9,40,40,40,100,100,190,184,-80,-80,-83,-77,0
RPT end
report
RPT cyc_range=97..524 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,40,40,0,100,0,186,0,-80,0,0,0,0
RPT 1,-21,5,5,0,100,0,23,0,-80,0,0,0,0
RPT 2,-18,23,23,0,100,0,106,0,-77,0,0,0,0
RPT 3,-15,40,40,11,100,27,184,11,-78,-101,-104,-99,0
RPT 4,-12,40,40,16,100,40,178,35,-81,-98,-102,-96,0
RPT 5,-9,40,40,25,100,62,184,43,-82,-98,-102,-94,0
RPT 6,-6,40,40,39,100,97,189,127,-83,-96,-101,-92,0
RPT 7,-3,40,40,40,100,100,188,161,-83,-94,-98,-91,0
RPT 8,0,40,40,40,100,100,189,174,-83,-92,-95,-89,0
RPT 9,3,40,40,40,100,100,183,176,-82,-88,-92,-83,0
RPT 10,6,40,40,40,100,100,190,180,-80,-83,-90,-79,0
RPT 11,9,40,40,40,100,100,190,184,-80,-80,-83,-77,0
RPT end
reset
RX reset
report
RPT cyc_range=609..750 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 3,-15,31,31,0,100,0,121,0,-90,0,0,0,0
RPT 4,-12,40,40,0,100,0,165,0,-87,0,0,0,0
RPT 5,-9,40,39,0,97,0,153,0,-89,0,0,0,0
RPT 6,-6,31,31,0,100,0,118,0,-93,0,0,0,0
RPT end
report
RX ? unknown: report
report
RPT cyc_range=609..1065 wrapped=0 scan=passive legacy_minor=0
RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism
RPT 0,-24,40,40,0,100,0,114,0,-92,0,0,0,0
RPT 1,-21,40,40,0,100,0,112,0,-92,0,0,0,0
RPT 2,-18,26,26,0,100,0,78,0,-93,0,0,0,0
RPT 3,-15,31,31,0,100,0,121,0,-90,0,0,0,0
RPT 4,-12,40,40,0,100,0,165,0,-87,0,0,0,0
RPT 5,-9,40,39,0,97,0,153,0,-89,0,0,0,0
RPT 6,-6,40,40,0,100,0,147,0,-93,0,0,0,0
RPT 7,-3,40,39,6,97,15,112,6,-92,-101,-102,-100,0
RPT 8,0,40,40,20,100,50,112,26,-92,-99,-100,-98,0
RPT 9,3,40,39,35,97,87,109,45,-92,-96,-97,-95,0
RPT 10,6,40,40,39,100,97,110,73,-92,-94,-100,-92,0
RPT 11,9,40,40,39,100,97,110,113,-92,-92,-96,-89,0
RPT end


### Analysis of result 2 (2026-07-30) — **PASS, `BEACON_TX_LO_DBM = -6`**

First, the aside: **`probe` was not the problem.** `probe` only walks the
`setPower()` API and prints readbacks; it has no bearing on the sweep. What broke
"result 2 part 1" is the same thing as before — the captures were taken while the
sweep was not running (`cyc_range=1..1`, `lo_pdu=0` again; the TX log shows
`TX ? unknown: sweeo`). Part 2, with the sweep genuinely running, is the good run:
`cyc_range=97..524` and `mism=0` throughout.

**The two positions, from the most complete report at each:**

| TX_LO | INSIDE `lo_pct` | EDGE `lo_pct` | verdict |
|---|---|---|---|
| −24 … −18 | 0 | 0 | INSIDE < 90 |
| −15 | 27 | 0 | INSIDE < 90 |
| −12 | 40 | 0 | INSIDE < 90 |
| −9 | 62 | 0 | INSIDE < 90 |
| **−6** | **97** | **0** | **PASS** |
| **−3** | **100** | **15** | **PASS** |
| 0 | 100 | 50 | EDGE > 30 |
| +3 | 100 | 87 | EDGE > 30 |
| +6 / +9 | 100 | 97 | EDGE > 30 |

Reference slot: INSIDE median **−80 dBm** (spread −83…−77), EDGE median **−92 dBm**
(spread −93…−87). **Separation 12 dB**, comfortably past the feasibility gate.
Path loss 89 dB inside, 101 dB at the edge. `hi_pct` was 97–100 % everywhere, so
every denominator is sound.

**The two positions trace one common curve**, which is the check that the model is
real rather than fitted twice:

| notional received (`TX_LO − PL`) | INSIDE | EDGE |
|---|---|---|
| −104 | 27 % | 15 % |
| −101 | 40 % | 50 % |
| −98 | 62 % | 87 % |
| −95 | 97 % | 97 % |

**Why −6 and not −3, when both pass.** The pass criterion is not the sharpest
tool here; the number that actually matters is where PDR's evidence *changes
sign*. With hits worth `LL_PDR_HIT_Q8 = 532` and misses `LL_PDR_MISS_Q8 = −385`,
the log-LR crosses zero at a hit rate of `385/917 = 42 %`, which this install
reaches at about **−101 dBm** received. Margins to that crossover:

| | INSIDE | EDGE | under ±3 dB wander |
|---|---|---|---|
| **−6 dBm** | **+6 dB** | **−6 dB** | INSIDE→75 %, EDGE→20 %: **both hold** |
| −3 dBm | +9 dB | −3 dB | INSIDE→97 %, **EDGE→45 %: sign flips** |

Both positions showed ±3 dB of RSSI wander across the sweep, so −3 dBm is not
safe at the edge: a 3 dB gain there tips PDR into contributing *NEAR* evidence
while the user is outside the zone. **−6 dBm is symmetric — 6 dB either way — and
survives that wander on both sides.**

Independently, centring the level on the crossover gives the same answer:

```
BEACON_TX_LO_DBM = -101 + (PL_inside + PL_edge)/2
                 = -92  - (RSSI_HI_inside + RSSI_HI_edge)/2
                 = -92  - (-80 + -92)/2   =  -6 dBm
```

### The rule in step 6 was wrong — corrected

The old rule, `-91 - RSSI_EDGE`, used **only the edge** and targeted "land at
−100 dBm there". Applied to this install it gives **+1.5 dBm**, where the edge
actually measures **50 % hit** — a clear fail. Two errors: it ignored the inside
position entirely, and it treated `lo_rssi` as an unbiased estimate of received
power when it is **conditioned on reception**, so at low hit rates it reads
several dB stronger than the truth.

Steps 5–6 above now carry the corrected two-position rule. Anyone re-running this
should use that, not the `-91 - RSSI_EDGE` form.

### One overclaim to retract

Earlier text in this file said the received-power threshold "is the same in your
bedroom as on the bench". Within a single install that holds well — the two
positions above trace one curve. **Across installs it shifts.** The bench read
82 % at −102 dBm where this install reads ~40 %, i.e. the bench was ~3–4 dB more
sensitive, almost certainly a quieter local noise floor. Hit rate depends on SNR,
not signal alone. So the *shape* of the curve transfers and the rule is a good
starting point, but **the sweep is the calibration** — which is exactly what this
run was.

## The traps that still apply

Most of the original list is now closed by measurement (`setPower` does reach the
PHY; the schedule is decodable; the level ladder is exact). Three remain, and two
are new:

- **Hold position.** λ/2 ≈ 6.2 cm at 2.44 GHz, so moving the wrist 10 cm mid-capture
  is a fresh fading draw and reads as a level change. Rest the board on something
  at each position, and prefer a couple of short captures a few cm apart over one
  long one — if they disagree by more than ~5 dB you are sitting in a fade and the
  reading is not representative of the position.
- **Measure both positions in one session.** RF environments drift over tens of
  minutes; a gap between the two captures can invent separation that is not there.
- **Keep the receiver PASSIVE.** This is new and it matters: active scanning cost
  40–70 % of advertisement receptions on the bench and moved the apparent cliff by
  ~9 dB. The bench defaults to passive; do not switch it.
- **−24 dBm is the floor** and only multiples of 3 exist. If the arithmetic asks
  for less than −24, the honest answer is that the positions are too far apart for
  the LO slot to distinguish — which is fine, PDR just says NEAR-or-not rather
  than resolving finely.
- **The bench's own path loss was 90 dB**, which is high for a desk (cables across
  the PCB antennas). Do not carry the bench's absolute numbers into your install;
  carry the rule in step 5.

## After it passes

P2's PDR pipeline is already built and unit-tested against this measurement —
`prox_obs_begin/note/close` on the watch, motion-decayed Beta counts, and an
asymmetrically-capped log-LR folded into the HMM as watch-local evidence (no wire
change). Setting the level is the last input it is waiting on.
