# Study 3 — scenario-family lineage and post-freeze exploratory proposal (V1)

Status: **PROPOSAL. Nothing here has been executed.**
Date: 2026-08-11
Prepared by: agent, on instruction. Acceptance decisions belong to the researcher.

Two things are settled here and one is proposed:

1. where each of the nineteen Study 2 scenario families (`E1`–`E19`) stands with
   respect to the ten frozen Study 3 families;
2. whether the nineteen can be run under the frozen Study 3 implementation
   (**they cannot** — the reason is mechanical, and given below);
3. a small exploratory evaluation that *is* available and answers a question the
   completed held-out block genuinely did not.

Nothing in this document is held-out evidence, and nothing proposed here may be
described as held-out or confirmatory.

## 0. The two family sets are different constructs, not two versions of one

The nineteen families are defined in `scripts/run_campaign.py::scenario_family()`
as tuples of `WaterProfile`, `FaultSchedule`, `CurrentProfile`, `NoiseProfile` and
`TerrainProfile`. The ten Study 3 families are defined in
`study3/scenarios.py::FAMILIES` and realised by `physical_state()` as a
deterministic schedule over turbidity, bottom-lock and water-track probability,
acoustic response probability, ambient noise and deployed infrastructure.

The two share no code. `study3/` imports none of the Study 2 profile or schedule
builders, and `run_campaign.py` contains no reference to `study3`. Study 3 was
specified with ten families from the outset (`STUDY3_PROTOCOL.md` §3: seven
transition families plus three controls); there was never a nineteen-family
Study 3 design that was cut down to ten.

So the mapping below is a **phenomenon-level** trace — which Study 2 stressor is
or is not reachable in the Study 3 physics — not a record of families being
deleted from a Study 3 list.

## 1. Classification of all nineteen

Categories:

- **retained primary** — the phenomenon is carried by a Study 3 *primary* family;
- **retained control** — carried by a Study 3 *control* family;
- **development-only mechanism** — the physics exists in the frozen Study 3 code
  but no registered family exercises it; it is reachable only through the
  generated-environment or interactive paths;
- **merged / superseded** — the phenomenon survives inside a broader Study 3
  family or as a context attribute, with the stated loss of fidelity;
- **excluded** — not expressible under the frozen Study 3 implementation at all.

| Study 2 family | Stressor as defined in Study 2 | Class | Study 3 carrier | Reason |
|---|---|---|---|---|
| `E1_nominal` | clear water, no fault | retained control | `S3_NOMINAL` | Fault-free control, same role: confirm no policy intervenes when nothing degrades. |
| `E2_dvl_short` | 12 s bottom-lock loss, then re-latch | merged / superseded | `S3_RECOVERY` (role) | The degrade-then-restore role is kept, but `S3_RECOVERY` applies it to optical and acoustic, not the DVL. E2's second role — a false-alarm control against churn on a short dropout — is carried in Study 3 by policy hysteresis (`minimum_action_hold_s = 8.0`, `trend_confirmation_frames = 3`) asserted in mechanism tests (`test_study3_mode_stability.py`), not by a scenario. |
| `E3_dvl_long` | sustained bottom-lock loss | **retained primary** | `S3_DVL_GRADUAL` | Same end state: bottom-lock probability driven to 0.02. Onset is a ramp over 25–70% of the horizon rather than a step. |
| `E4_optical_graded` | turbidity ramp 0.20 → 1.60 | **retained primary** | `S3_OPTICAL_GRADUAL` | Direct counterpart; turbidity ramps to ~1.0 on the Study 3 normalised scale over the same fractional window. |
| `E5_optical_loss` | sustained optical blackout | merged / superseded | `S3_SUDDEN` | Study 3 has no optical-only blackout family. The step-loss phenomenon survives inside `S3_SUDDEN`, which removes optical *and* both DVL modes together at 50% of the horizon. Optical-only isolation is lost. |
| `E6_acoustic_intermittent` | duty-cycled acoustic outage, 25 s off in every 40 s | **excluded** | — | Study 3 acoustic degradation is monotone by construction (`acoustic_response_loss` is a single ramp). `physical_state` has no periodic-outage construct. Note this is the family PREDICTIVE was for, and PREDICTIVE returned a null with `preemptive_actions == 0` (`STUDY3_DEVELOPMENT_NULL_FINDINGS.md`). |
| `E7_compound` | overlapping optical + DVL + acoustic loss | merged / superseded | `S3_NO_RECOVERY` | All three modalities ramp together and never restore. Study 3's version is more severe than E7 and is a control, not a primary family. |
| `E8_turbid_dvl_loss` | turbidity past the optical limit with both DVL modes lost | merged / superseded | `S3_SUDDEN` | The optical+DVL coupling is present, but only as a simultaneous step. There is no *gradual* optical+DVL compound primary family; Study 3's two compound primaries are optical+acoustic and DVL+acoustic. This is a real reduction in coverage and is recorded as such. |
| `E9_current_unobservable` | strong current with both DVL modes lost | development-only mechanism | — | `PhysicalState` carries `current_east_mps` / `current_north_mps`, but every registered Study 3 family leaves both at 0.0. Currents are set only by `transition_driver` scenarios (`dvl_acoustic_handover`, `compound_terminal`), the generated-environment config and the interactive control window. |
| `E10_current_steady` | steady 0.12 m·s⁻¹ flow, no fault | development-only mechanism | — | Same: no registered family sets a current. |
| `E11_current_building` | current ramp to 0.22 m·s⁻¹ with rising turbidity | development-only mechanism | — | Same. Reachable in the generated-environment path, which varies turbidity and both current components as independent bounded processes. |
| `E12_current_rotating` | continuously veering flow | **excluded** | — | No rotating or time-varying-direction current construct exists in `study3/`. Transition targets and generated processes set scalar east/north components; nothing composes a rotation. |
| `E13_acoustic_noise` | independent ambient-noise ramp 40 → 70 dB | merged / superseded | any acoustic family | In `physical_state`, `acoustic_noise_db = 45 + 35·acoustic` — noise is a deterministic function of the acoustic degradation ramp, not an independent axis. Noise as an *independent* stressor exists only in the generated-environment path. |
| `E14_noisy_dvl_loss` | loud water with velocity aiding lost | merged / superseded | `S3_COMPOUND_DVL_ACOUSTIC` | The DVL ramp plus the acoustic ramp, which carries noise from 45 to 80 dB. The two are coupled rather than crossed. |
| `E15_turbid_and_noisy` | turbidity with loud water | merged / superseded | `S3_COMPOUND_OPTICAL_ACOUSTIC` | Same coupling, on the optical side. |
| `E16_featureless_plain` | terrain matching returns nothing over a flat seabed | **excluded** | — | Study 3 has no terrain modality: no terrain field in `PhysicalState`, no terrain technique in `FixedConfiguration`, no occurrence of "terrain" anywhere in `study3/`. |
| `E17_terrain_recoverable` | terrain is the only surviving absolute reference | **excluded** | — | Same. This was Study 2's decisive terrain case and it has no Study 3 counterpart. |
| `E18_vessel_departs` | USBL support vessel leaves station as turbidity rises | **retained primary** | `S3_INFRASTRUCTURE_WARNING` | `INFRASTRUCTURE_TRANSITION` withdraws USBL at 68% of the horizon. Study 2 offered terrain as the replacement modality; Study 3 has none, so the vehicle falls back to optical or dead reckoning. Same scenario, harder fallback. |
| `E19_unprepared_area` | no acoustic infrastructure deployed at all | merged / superseded | context attribute | Represented not as a family but as `INFRASTRUCTURE_FREE`, carried by `S3_OPTICAL_GRADUAL`, `S3_SUDDEN` and `S3_NO_RECOVERY`. The terrain fallback E19 relied on is gone. |

Totals: 3 retained primary, 1 retained control, 3 development-only mechanism,
8 merged / superseded, 4 excluded — nineteen.

### Study 3 families with no Study 2 ancestor

`S3_ACOUSTIC_GEOMETRY_ASYNC` (acoustic response loss deliberately desynchronised
from geometry), `S3_COMPOUND_OPTICAL_ACOUSTIC`, `S3_COMPOUND_DVL_ACOUSTIC` and
`S3_NO_RECOVERY` are new. Four of the ten Study 3 families are therefore not
descended from the nineteen at all, which is the other half of the reason the two
sets do not line up.

## 2. Can the nineteen be run under the frozen implementation? No.

Three independent blockers, each verifiable:

1. **The family list is closed.** `physical_state()` begins
   `if family not in FAMILIES: raise ValueError(f"unknown Study 3 family {family}")`.
   Passing `E1_nominal` to the Study 3 execution path raises immediately.
2. **There is no terrain modality.** `E16`, `E17`, `E18` and `E19` all depend on
   terrain-relative navigation as a technique. Adding it means adding a sensor
   model, an action-space entry and a mode to the frozen selector.
3. **The two runners are not connected.** `run_campaign.py` cannot construct a
   `Study3Policy`; running `E1`–`E19` through the Study 2 runner exercises the
   Study 2 policy stack, which is already published, and would answer nothing new
   about the six-mode selector.

Making the nineteen runnable would mean editing `scenarios.py`, `simulation.py`,
`modes.py` and `policies.py` — all of them allowlisted in
`STUDY3_FREEZE_MANIFEST_V1.json`. That breaks the freeze and invalidates the
completed held-out result. **The conditional in the instruction — "if all 19
remain valid" — is not satisfied**, so no full-family evaluation is proposed.

## 3. What is worth running instead, and what it would answer

The completed held-out block was **Part A only**: 810 executions across the ten
scripted families, every one of them a deterministic monotone schedule with
currents fixed at zero and noise pinned to the acoustic ramp. It carried no
generated-environment part.

That leaves three questions the held-out did not touch:

- **Stochastic, non-monotone environments.** Every held-out episode degraded
  along a fixed ramp. `environment_generator.py` produces bounded
  Ornstein–Uhlenbeck-style processes with hazard-driven availability, so
  modalities fail *and recover* at random times. Nothing outside the development
  seed band has tested the selector against that.
- **Adaptation coverage.** C7 is a Part B metric. The 59.8% figure carried as a
  standing limitation in `STUDY3_FREEZE_DECISION_V1.md` rests entirely on twelve
  environment seeds at development root 31,900,000. The held-out block produced
  no adaptation-coverage measurement at all. This is the largest unexamined
  number in the study.
- **Independent currents and independent acoustic noise.** These are exactly the
  `E9`–`E11` and `E13` phenomena classified above as development-only. The
  generated-environment config varies `current_east_mps`, `current_north_mps`,
  `acoustic_noise_db` and `lbl_geometry_scale` as independent processes. They have
  never been exercised against the six-mode selector outside development.

### Proposed exploratory evaluation

Nothing frozen is modified. `run_one` already accepts an
`environment_realization` that overrides the family schedule entirely, and Part B
passes the config name in place of a family, so no new family is registered and
`scenarios.py` is not touched.

| | Part E1 — generated environments | Part E2 — scripted transitions *(optional)* |
|---|---|---|
| Content | `moderate_severe_variable_multimodal`, the existing config file unmodified | the three `standard_transition_scenarios()` |
| Seeds | 60 environment seeds | 20 seeds × 3 scenarios |
| Policies | FIXED, DEPLOYMENT_FIXED, REACTIVE — frozen, shared realisation per seed | same |
| Executions | 60 × 3 = **180** | 60 × 3 = **180** |

Total **360 executions**, or 180 if Part E2 is cut. At the held-out throughput of
810 runs in 554 s on four workers this is roughly four minutes.

Sixty seeds is five times the twelve V5 Part B used. V5 yielded 87 classifiable
post-launch changes of the truth-side best viable mode from those twelve; the
episode count should scale roughly with seeds, though the exact number depends on
the realisations drawn and cannot be predicted.

**Seeds.** A fresh exploratory band, disjoint from every spent block:

| Band | Use | Status |
|---|---|---|
| 20.0 / 20.4 / 20.8 M | Study 2 | spent |
| 22.1–22.3 M | platform_v2 | spent |
| 31.x M | Study 3 development | spent |
| 32,000,000 | Study 3 held-out | **spent, once, irreversibly** |
| **33,000,000** | Part E1, proposed | unused |
| **33,100,000** | Part E2, proposed | unused |

To be registered in `STUDY3_SEED_REGISTRY.json` under a new `exploratory` key —
never under `held_out`, and not as a development root.

**Metrics.** The existing set only: the twelve outcome metrics the held-out
analyser already reports, plus the corrected adaptation metrics
(`mode_switches`, `physical_interventions`, `unnecessary_interventions`,
adaptation coverage and latency) as defined for V5. No new metric is introduced.

**Reporting.** Exploratory, post-freeze, single execution per seed, reported
regardless of direction. It is not held-out, not confirmatory, and cannot be used
to revise the held-out result or the freeze decision. If it disagrees with V5 it
is reported as disagreeing.

**No acceptance threshold is proposed and no verdict will be assigned.** The run
reports numbers with intervals. Whether any of them is scientifically sufficient
is the researcher's decision.

### What this cannot recover

`E6` (intermittent acoustic duty cycle), `E12` (rotating current) and the terrain
families `E16`–`E17` remain unreachable under the freeze. The terrain gap is the
substantive one: Study 2 added terrain-relative navigation precisely so that a
technique with no failure mode was not the only escape, and Study 3's action space
has nothing equivalent. That belongs in the limitations, not in an exploratory
run.
