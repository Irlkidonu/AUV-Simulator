# Study 3 — interactive control-window testing (V1)

Classification: **INTERACTIVE TESTING. Not campaign evidence.**
Date: 2026-08-11. Role: tester, not developer.

Nothing in the controller, thresholds, scenarios or existing evidence was
modified. No parameter was tuned, before or after seeing any outcome. Two
findings that would normally invite a fix are reported here and deliberately
left unfixed.

## Method

Ten sessions were recorded at a fresh interactive band (seeds 34,000,000+,
roots 34,100,000+), horizon 600 s, dt 1 s. Each was recorded once under
REACTIVE and then replayed unchanged against DEPLOYMENT_FIXED, REACTIVE and
PREDICTIVE.

Disturbances were injected through `InteractiveEnvironment.set_control`, the
same call `control_window.py:220` makes when a slider moves or a toggle flips.
The Tk GUI was not driven with synthetic clicks: the recording/replay contract
requires reproducible event timing, which clicking cannot give. Every schedule
was authored in full before any session ran, and none was revised after seeing a
result. Times span 40–520 s at deliberately irregular intervals and severities.

**Replay fidelity was verified, not assumed.** In all nine sessions that
completed, the REACTIVE replay reproduced the live REACTIVE run's `trace_digest`
exactly, so the replays observe the recorded disturbance sequence.

Note: the interactive baseline is `acoustic_technique="lbl"`, altitude 5 m,
lidar, weight — the demonstrator's configuration, not the campaign's
`fixed_155`. That is by design and is not a defect.

## Finding 1 — implementation defect, reported and NOT fixed

**A sustained current carries the vehicle off the world texture, and the
renderer aborts the entire mission.**

`S3_currents_building_and_veering` fails on all three policies with
`ValueError: requested camera footprint leaves the world texture`, raised at
`rendering/georeferenced.py:44`.

The world texture `run_one` builds is 2048 px × 0.04 m/px, centred, so it spans
±40.94 m. At the 5 m survey altitude the camera footprint is 5.77 × 5.77 m,
leaving a usable vehicle radius of **36.86 m**.

> **Corrected 2026-08-11.** This section first quoted ±20.46 m and a 16.38 m
> usable radius, taken from `WorldTexture.generate()`'s default 1024 px size.
> `run_one` passes 2048. The defect is unchanged; the exposure is about half
> what was first reported. See `STUDY3_FREEZE_AMENDMENT_V1.md`.

Truth-side positions from the failing run:

| t (s) | x (m) | y (m) | \|r\| (m) | current E/N (m·s⁻¹) |
|---|---|---|---|---|
| 60 | −2.26 | 1.58 | 2.76 | 0.06 / 0.00 |
| 140 | 11.66 | 3.56 | 12.19 | 0.14 / 0.00 |
| 180 | 20.26 | 0.82 | 20.28 | 0.14 / −0.11 |
| 240 | 37.06 | −4.30 | 37.31 | 0.24 / −0.11 |
| 244 | 38.32 | −4.64 | 38.60 | abort |

Why this is a defect rather than a domain limit: a vehicle that has drifted off
a finite mapped patch physically has *no georeferenced imagery*, which is
exactly the "optical aiding unavailable" condition the six-mode selector exists
to handle. Instead the exception propagates out of `run_one` and destroys the
run. The simulator crashes where the modelled system should simply lose a
modality.

**Exposure.** A sustained current of **0.205 m·s⁻¹ over the 180 s campaign
horizon**, or 0.061 m·s⁻¹ over 600 s, is enough to exit the map. The Part E1
generated environments permit currents to ±0.18 / ±0.20 m·s⁻¹, just under the
180 s threshold.

**Existing evidence is unaffected in fact.** The held-out block and every
scripted family set both current components to 0.0, so no drift arises. All 180
Part E1 runs completed; `run_exploratory_e1.py` wraps `run_one` in no exception
handler, so an out-of-map render would have aborted the block rather than
producing packets. Nothing already reported needs revisiting. The risk is to
future runs that use stronger or more persistent currents.

**Fixed on 2026-08-11**, on instruction, after this report was written. Leaving
the patch now yields an unavailable optical observation instead of an exception.
See `STUDY3_FREEZE_AMENDMENT_V1.md` for the behaviour-preservation evidence and
the resulting freeze-manifest drift.

## Finding 2 — behavioural, reported and NOT changed

**REACTIVE alternates between LBL and USBL at the action-hold period even when
neither acoustic service is disturbed.**

In `S1_optical_degrade_recover` the only disturbance is turbidity. Acoustic
noise stays at its 48 dB default and LBL geometry at 1.0, giving a service
response probability near 0.96. REACTIVE nonetheless changes mode 22 times, and
20 of those changes are LBL↔USBL flips. The inter-change intervals are:

```
97, 8, 8, 8, 8, 8, 56, 8, 8, 8, 8, 8, 16, 8, 88, 8, 48, 8, 32, 8, 8
```

Fifteen of twenty-one are exactly 8 s — `minimum_action_hold_s`. The policy
flips back the instant the hold expires.

Mechanism, from `discovery.py`: `take_opportunity` is strict round-robin over
the catalogue, one probe per `opportunity_period_s = 4.0`, and evidence expires
after `evidence_ttl_s = 8.0`. With the two-service catalogue `("lbl", "usbl")`
each service is revisited every 2 × 4.0 = **8.0 s, exactly equal to the evidence
lifetime**. Each service's evidence therefore expires precisely as its
replacement falls due, so typically only one service is visible at a time and
the visible one alternates. Because LBL outranks USBL in the priority order,
the selected mode alternates with it.

This is a knife-edge rather than a coding error: the TTL and the revisit period
are equal, and the design is deliberate serialized probing. But it produces
sustained chatter with no environmental cause, and it is the plausible mechanism
behind the "+9.3 mode switches, +6.3 physical interventions" cost measured in
Part E1. **Nothing was changed.** Altering either constant would be tuning a
frozen policy after seeing an outcome.

## Results

`sw` = frozen `mode_switches`; `chg` = telemetry mode changes after the first;
`rev` = changes reverting to the previous mode within 15 s (a descriptive
statistic, declared before running, not an acceptance threshold); `cov` and
`lat` = adaptation coverage and median latency under the V5 C7/C8 definition.

| Session | Policy | RMSE | Peak | Gap s | Unaided s | sw | chg | rev | Safety | cov | lat |
|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 optical degrade/recover | DEPLOY_FIXED | **0.170** | **0.811** | **14** | **362** | 0 | 0 | 0 | 0 | — | — |
| | REACTIVE | 0.197 | 0.973 | 22 | 375 | 23 | 22 | 11 | 0 | — | — |
| S2 DVL bottom then water | DEPLOY_FIXED | 0.0817 | 0.104 | 2 | 234 | 0 | 0 | 0 | 0 | — | — |
| | REACTIVE | 0.0817 | 0.104 | 2 | 232 | 17 | 16 | 6 | 0 | — | — |
| S3 currents building | all | **aborted — Finding 1** | | | | | | | | | |
| S3b currents reversing | DEPLOY_FIXED | 0.255 | 1.391 | 2 | 231 | 0 | 0 | 0 | 0 | — | — |
| | REACTIVE | 0.255 | 1.391 | 2 | 230 | 13 | 12 | 3 | 0 | — | — |
| S4 acoustic noise + LBL geom | DEPLOY_FIXED | 0.0798 | 0.214 | 2 | 266 | 0 | 0 | 0 | 0 | 0.00 | — |
| | REACTIVE | 0.0799 | 0.214 | 2 | 251 | 40 | 39 | 16 | 0 | 1.00 | 15 |
| S5 LBL loss + USBL departure | DEPLOY_FIXED | 0.0786 | **0.120** | 6 | 276 | 0 | 0 | 0 | 0 | 0.00 | — |
| | REACTIVE | 0.0810 | 0.279 | 5 | 250 | 22 | 21 | 11 | 0 | 1.00 | 7 |
| **S6 USBL departure, turbid** | DEPLOY_FIXED | 4.994 | 12.507 | 390 | 532 | 0 | 0 | 0 | **1** | 0.00 | — |
| | **REACTIVE** | **3.257** | **7.980** | **322** | **499** | 8 | 7 | 2 | **0** | 0.67 | 16 |
| S7 compound → surfacing 152 s | DEPLOY_FIXED | **0.122** | **0.359** | 14 | 95 | 0 | 0 | 0 | 0 | — | — |
| | REACTIVE | 0.129 | 0.382 | 14 | 95 | 9 | 8 | 0 | 0 | — | — |
| **S8 total loss → surfacing 319 s** | DEPLOY_FIXED | 0.800 | **1.897** | 142 | 249 | 0 | 0 | 0 | 0 | 0.00 | — |
| | **REACTIVE** | **0.491** | 2.543 | **98** | **239** | 7 | 6 | 1 | 0 | 1.00 | 7 |
| S9 compound, recovers | DEPLOY_FIXED | **0.345** | **1.423** | 83 | 453 | 0 | 0 | 0 | 0 | 0.00 | — |
| | REACTIVE | 0.418 | 1.872 | **35** | **420** | 28 | 27 | 8 | 0 | 1.00 | 8 |

DEPLOYMENT_FIXED never changes mode in any session; it holds `fixed_multimodal`
throughout, which is why its `sw`, `chg`, `rev` and adaptation coverage are all
zero by construction.

## PREDICTIVE

**PREDICTIVE is identical to REACTIVE on every collected metric in all ten
sessions** — completion, safety, RMSE, transition RMSE, peak error, unaided
time, aiding gaps, coverage, interventions, mode switches and mode sequence.

Its `trace_digest` differs, because the trace records `output.forecast.impending`
and PREDICTIVE populates it. It computes forecasts and never acts on them. This
independently reproduces the development null in
`STUDY3_DEVELOPMENT_NULL_FINDINGS.md` (`preemptive_actions == 0`) in an
interactive, non-scripted setting, which the original null could not show.

## Where REACTIVE helps, hurts, or looks wrong

**Clearly helps — both worth keeping.**

- **S6, USBL vessel departs while the water turns turbid.** The strongest case
  observed. REACTIVE cuts RMSE from 4.994 to 3.257 m, peak error from 12.51 to
  7.98 m, the longest aiding gap from 390 to 322 s, and — the substantive point
  — **DEPLOYMENT_FIXED records a safety violation and REACTIVE does not.** Its
  mode track is legible: LBL → dead reckoning while the vessel is away → USBL
  again 26 s after the vessel returns at 501 s.
- **S8, staged total loss of submerged aiding.** RMSE 0.491 vs 0.800 m and the
  longest gap 98 vs 142 s, with a clean monotone escalation
  `lbl_aided → usbl_aided → relative_dead_reckoning → terminal_degraded`, one
  change per real loss, one reversion, and surfacing at 319 s. Honest caveat:
  **REACTIVE's peak error is worse**, 2.543 vs 1.897 m, so it trades a higher
  transient for a better trajectory overall.

**Clearly hurts.**

- **S1, optical degradation and recovery.** Worse on every error measure — RMSE
  0.197 vs 0.170, peak 0.973 vs 0.811, longest gap 22 vs 14 s, unaided 375 vs
  362 s — while making 22 mode changes, 11 of which revert within 15 s. The
  disturbance is turbidity alone, so almost all of that switching is the LBL/USBL
  chatter of Finding 2.
- **S9, compound degradation with recovery.** Worse RMSE (0.418 vs 0.345) and
  peak (1.872 vs 1.423), better aiding gap (35 vs 83 s) and unaided time. A real
  trade, not a win.

**Costly with no measurable benefit.** S2, S3b and S4 end with error metrics
equal to DEPLOYMENT_FIXED to three or four significant figures while REACTIVE
makes 16, 12 and 39 mode changes. S4 is the extreme: 39 changes, 16 reversions,
and an RMSE difference of 0.0001 m.

**Looks wrong.** The Finding 2 chatter. Alternating between two healthy acoustic
services every 8 s, in a session with no acoustic disturbance at all, is not
adaptation to anything.

**Not testable, blocked.** Strong sustained currents (S3), by Finding 1.

## Recordings worth keeping

| Recording | Why |
|---|---|
| `S6_usbl_departure_under_turbidity.json` | The demonstration case. Clear separation on every measure including safety, with a legible four-mode track and a visible recovery when the vessel returns. |
| `S8_total_loss_to_surfacing.json` | The escalation-to-surfacing case. Textbook monotone mode ladder ending in a correct GPS surfacing; good for the mode-selector figure. |
| `S1_optical_degrade_recover.json` | The honest counter-case. Keep it: it shows both a loss to the fixed comparator and the chatter pathology in one run. |
| `S3_currents_building_and_veering.json` | Keep as the reproducer for Finding 1, not as a result. |
| `S4_acoustic_noise_and_lbl_geometry.json` | Optional. The clearest illustration of switching cost without benefit. |

S2, S3b, S5, S7 and S9 are worth retaining as coverage but are not
demonstration material.

## What was not done

No controller, threshold, scenario or frozen file was modified. No constant was
tuned. Both findings are reported for your decision rather than repaired, and
neither has been used to justify changing anything.
