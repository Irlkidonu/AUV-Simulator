# Study 3 freeze amendment V1 — out-of-map robustness fix

Status: **RECORDED, NOT APPLIED.** The freeze manifest is deliberately left
unmodified, so `verify_lock` reports the drift instead of hiding it.
Date: 2026-08-11. Prepared by: agent, on instruction.

## What changed

Two files allowlisted in `STUDY3_FREEZE_MANIFEST_V1.json` were modified to
repair the out-of-map defect reported in `STUDY3_INTERACTIVE_TESTING_V1.md`.

| File | Frozen SHA-256 | Current SHA-256 |
|---|---|---|
| `study3/simulation.py` | `52b729ab7e61e776…6cf5fb2f` | `84582110 22c57b91…db0df766` |
| `rendering/georeferenced.py` | `74041166 00f8adc8…7d7adca5` | `3e6df3b4 39cee2fa…afa50dac` |

`verify_lock` now reports both as drifted. **That is correct and intended.**
Re-baselining the manifest would silently redefine what "frozen" means, and that
is a researcher decision, not an agent one.

## Why the change was justified

The frozen implementation aborted the entire mission with
`ValueError: requested camera footprint leaves the world texture` whenever the
vehicle drifted off the georeferenced patch. Physically the vehicle has merely
left the surveyed area and has no map imagery, which is a modality loss the
six-mode selector already handles. The simulator crashed where the modelled
system should have degraded.

## Why it does not invalidate the held-out result

The fix is **behaviour-preserving for every render that stays on the patch**.
The changed control flow is entered only on the exception path that previously
terminated the run; a query render that succeeds follows exactly the prior code
path with the same values.

Evidence, all gathered after the change:

| Check | Result |
|---|---|
| Part E1 packets re-derived and compared on `trace_digest` | **180 / 180 identical** |
| Part E1 packets compared on `overall_rmse_m` | **180 / 180 identical** |
| Interactive replays re-run across three policies | **27 / 27 identical** |
| Interactive runs that previously aborted | 3, now complete |
| Golden pre-fix digest, captured from a worktree at the pre-fix commit | reproduced by the fixed code |
| Full regression suite | **419 passed, 0 failed** |

The held-out block itself was **not** re-run and root 32,000,000 was not
accessed. It does not need to be: every scripted Study 3 family sets both
current components to 0.0, so no held-out run could have left the patch, and the
only behaviour this change alters is the off-patch path.

## Correction to a previously reported figure

`STUDY3_INTERACTIVE_TESTING_V1.md` originally quoted a usable radius of 16.38 m
and an exit threshold of 0.097 m·s⁻¹ over 180 s. Those were computed from
`WorldTexture.generate()`'s **default** 1024 px size. `run_one` builds the world
with `WorldTexture.generate(2048, .04, …)`, giving ±40.94 m and a usable radius
of **36.86 m** at 5 m altitude, so the exit threshold is **0.205 m·s⁻¹ over 180 s**
and 0.061 m·s⁻¹ over 600 s. The defect is real and the direction of the argument
is unchanged, but the exposure is roughly half what was first reported. That
document has been corrected.

## What the researcher must decide

1. Whether to re-baseline `STUDY3_FREEZE_MANIFEST_V1.json` onto the fixed
   implementation, or to keep the drift visible as it is now.
2. Whether the held-out provenance record should cite this amendment. It
   currently records the pre-fix commit `41af86d5`, which remains accurate.

Nothing further is proposed and nothing was re-baselined.
