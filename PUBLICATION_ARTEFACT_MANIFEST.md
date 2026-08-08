# Publication artefact manifest

Every result artefact supporting the manuscript, with its provenance and the
claims it carries. Every hash and every number below was recomputed from a
**clean clone** of this branch, not from a working tree.

Source commit for the published tree: `8db709d`
Freeze record commit (the source that produced the Study 2 campaigns):
`16a091ada410cf45574caddd2edfdda3313a141f`, frozen `2026-08-05T06:06:18Z`,
46 files, isolation tests `173 passed`.

Verify everything in one command:

```bash
cd src/uuv_mode_aware_navigation/results && sha256sum -c ARTEFACT_SHA256SUMS
```

---

## 1. Artefacts

### Study 1 — initial implementation, 15 families

| | |
|---|---|
| **File** | `results/campaign_v5.csv` |
| **Campaign** | Study 1 development |
| **Command** | `python3 scripts/run_campaign.py --seeds 10 --root 20000000 --out results/campaign_v5.csv` |
| **Content** | 1200 runs = 15 families × 10 seeds × 8 policies |
| **Seed block** | development root 20,000,000; seeds 20001000–20001009 |
| **SHA-256** | `f609d5cbcace228ef9845996528eab778bf41d96800e4edbcf82eb7d6a3b74ae` |
| **Supports** | Table `tab:aggregate` (all 8 policies); the withdrawn 14-of-15 development finding; F1 development verdict; **and it is the source of the normalisation constants** — `DEVELOPMENT_NORMALISERS.json` declares this exact hash as its `source_sha256` |

| | |
|---|---|
| **File** | `results/held_out.csv` |
| **Campaign** | Study 1 held-out |
| **Command** | `python3 scripts/run_campaign.py --seeds 20 --root 20400000 --out results/held_out.csv` |
| **Content** | 2400 runs = 15 families × 20 seeds × 8 policies |
| **Seed block** | held-out root 20,400,000; seeds 20401000–20401019 |
| **SHA-256** | `5095d7338b8f8608a0b54f7df7793aa6d59c9df8cf90a389ee5c0494ca01d75d` |
| **Supports** | Study 1 held-out J (fixed 1.554, proposed 2.173); F1 replication; F4 row of `tab:falsification` — E8 A1 64.063 m failing 18/20 against proposed 0.103 m failing 0/20; E7 compound A1 75.23 / proposed 58.44 / DR 74.60 |

| | |
|---|---|
| **Files** | `results/static_sweep_development_v5.csv`, `results/static_sweep_held_out.csv` |
| **Content** | 16,200 and 32,400 runs; 108 configurations |
| **SHA-256** | `55ac407c872a0a0ac383b71cd78124906181ddec12a81450925c9d36b3812393`, `67ce041d753a15079cafb23e830e7de77cd08123240b7f4bedb9dbf54c1b693c` |
| **Supports** | Study 1 baseline selection; COMPARATOR_SPEC R8 (complete sweep published, best to worst) |

### Study 2 — corrected implementation, 19 families *(the final system)*

| | |
|---|---|
| **File** | `results/campaign_v7.csv` |
| **Campaign** | Study 2 development |
| **Command** | `python3 scripts/run_campaign.py --seeds 10 --root 20000000 --out results/campaign_v7.csv` |
| **Content** | 1520 runs = 19 families × 10 seeds × 8 policies |
| **Seed block** | development root 20,000,000; seeds 20001000–20001009 |
| **SHA-256** | `cb9b8d71139c6449c7660cfe63984772efd79aedf79896bf2c747d28d3ff865e` |
| **Supports** | Table `tab:dev2`, all 8 policies; the 0.001 development gap (*J* 0.017 against 0.016); the statement that the development result did not replicate |

| | |
|---|---|
| **File** | `results/held_out_2.csv` |
| **Campaign** | **Study 2 held-out — every claim about the final system** |
| **Command** | `python3 scripts/run_campaign.py --seeds 20 --root 20800000 --fixed-config 'lidar+terrain_relative@1.0m/0.25mps/weight/continue' --out results/held_out_2.csv` (gated on a verified freeze record; executed once) |
| **Content** | 3040 runs = 19 families × 20 seeds × 8 policies |
| **Seed block** | held-out root 20,800,000; seeds 20801000–20801019. Disjoint from development — verified, overlap 0 |
| **SHA-256** | `76c19def0b37b70d981c77b2392da590b12e11813b29be15a1ab9ecb67e03c5c` |
| **Independently attested** | this hash appears in **`freeze_record.json`** (`held_out_output_sha256`) and in **`held_out_2.log`**, both written at execution time |
| **Executed** | `2026-08-05T07:05:48Z`, once; block marked spent |
| **Supports** | Table `tab:heldout2`; abstract *J* = 0.081 against 0.017; F1 triggered; the 33× ablation factor; the 254× unprepared-area factor; Table `tab:secondary` (speed, duration, coverage, energy); the two `E18_vessel_departs` non-completions; per-family discrimination counts |

| | |
|---|---|
| **File** | `results/static_sweep_v7.csv` |
| **Campaign** | Study 2 development configuration sweep |
| **Content** | 27,360 runs = 144 configurations × 19 families × 10 seeds |
| **Seed block** | development root 20,000,000 |
| **SHA-256** | `d4534bd5d9f22f2bb90a9b22fb26e2959b772b253e138b17d6898118e27f681c` |
| **Supports** | **The claim that C1 was selected on development data**, not on the block it was later tested against. Winner: `lidar+terrain_relative@1.0m/0.25mps/weight/continue`. This artefact is what makes that claim checkable rather than asserted |

| | |
|---|---|
| **File** | `results/static_sweep_held_out_2.csv` |
| **Campaign** | Study 2 held-out configuration sweep |
| **Content** | 54,720 runs = 144 configurations × 19 families × 20 seeds |
| **Seed block** | held-out root 20,800,000 |
| **SHA-256** | `041a2e552a6d8a59d373fc7403b2601655115f4c55b042d01b22884b45a8e783` |
| **Supports** | "An exhaustive sweep of all 144 configurations over the held-out block returns that same configuration as its winner" — confirmed: same winner as the development sweep, and the same top three in the same order |

### Metadata

| File | SHA-256 | Role |
|---|---|---|
| `results/DEVELOPMENT_NORMALISERS.json` | `108134a30785ebde47ebdb59a392261d680c23f465ae20fd9f06648157b427e4` | Normalisation constants for *J*, derived from `campaign_v5.csv`. Its own provenance note states these are **not** pre-registered constants and must not be described as such |
| `results/PRE_CAMPAIGN_BASELINE.sha256` | `d8fce5b6976bf274866342ec1e12cffa8209c74afee83eee97383ce413dff7ff` | Pre-campaign integrity baseline |
| `freeze_record.json` | *(tracked in package root)* | 46 digests, freeze time, held-out execution record |
| `results/campaign_v5.log`, `campaign_v7.log`, `held_out.log`, `held_out_2.log`, `heldout_sweep_partB.log` | see `ARTEFACT_SHA256SUMS` | Execution logs |
| `results/ARTEFACT_SHA256SUMS` | *(the checksum file itself)* | All 15 artefacts |

---

## 2. Manuscript claims mapped to artefacts

Every aggregate *J* in the manuscript is `analysis.aggregate_outcome` — per-family
equal weighting — evaluated with the **development normalisers**
(`DEVELOPMENT_NORMALISERS.json`), not with normalisers re-derived on the set
being reported. Reproduced from the clean clone: **26 of 26 J values matched, 0
mismatches.**

| Claim | Artefact | Verified |
|---|---|---|
| `tab:aggregate` — 8 policies, Study 1 dev | `campaign_v5.csv` | 8/8 exact |
| Study 1 held-out 1.554 / 2.173 | `held_out.csv` | exact |
| `tab:dev2` — 8 policies, Study 2 dev | `campaign_v7.csv` | 8/8 exact |
| `tab:heldout2` — 8 policies, Study 2 held-out | `held_out_2.csv` | 8/8 exact |
| Abstract: *J* 0.081 vs 0.017, F1 triggered both blocks | `held_out_2.csv`, `held_out.csv` | exact |
| 33× ablation factor (2.668 / 0.081) | `held_out_2.csv` | exact |
| 254× unprepared area | `held_out_2.csv` | A1 25.438 m, proposed 0.0969 m — see §4.2 |
| Speed 0.251 / 0.495 m/s | `held_out_2.csv` | exact |
| Duration 379.5 / 192.3 s | `held_out_2.csv` | exact |
| Path length 95.3 / 95.2 m | `held_out_2.csv` | exact |
| Swath per second 5.65× | `held_out_2.csv` | 5.65× |
| Energy 17,076 J (45 W × 379.5 s) | `held_out_2.csv` | 17,076 J |
| Two non-completions, both `E18_vessel_departs` | `held_out_2.csv` | 2/380, both E18 |
| F4: E8 held-out 64.06 m failing 90% vs 0.103 m failing none | `held_out.csv` | 18/20 vs 0/20 |
| F4: E7 dev A1 74.63 / A2 69.38 / proposed 50.38 / DR 73.63 | `campaign_v5.csv` | exact |
| F4: E7 held-out 75.23 / 58.44 / DR 74.60 | `held_out.csv` | exact |
| A1 within 1.00 m and 0.63 m of dead reckoning | `campaign_v5.csv`, `held_out.csv` | 74.63−73.63, 75.23−74.60 |
| C1 selected on development, confirmed on held-out | `static_sweep_v7.csv`, `static_sweep_held_out_2.csv` | same winner, both |
| Held-out block executed once | `freeze_record.json`, `held_out_2.log` | `held_out_executed: true` |
| Development and held-out seeds disjoint | both | overlap 0 |

---

## 3. Reproducing the tables

```bash
python3 experiments/analyse_held_out.py \
    --held-out    src/uuv_mode_aware_navigation/results/held_out_2.csv \
    --development src/uuv_mode_aware_navigation/results/campaign_v7.csv
```

**Read the `J_dev` column, not `J_self`.** The script prints both. `J_self`
re-derives normalisers on the set being reported; `J_dev` uses the development
constants, and `J_dev` is what the manuscript reports. The script's own docstring
explains why both exist. Reading `J_self` gives 0.190 against 0.104 and appears
to contradict the paper; it does not.

---

## 4. Known discrepancies, stated rather than left to be found

### 4.1 The freeze record does not verify against the published tree

`python3 scripts/freeze.py --verify` exits 1 and reports 20 differences against
the published tree. This is expected and its scope is bounded:

**Campaign-critical modules** — the import closure of `campaign.py` is twelve
modules: `acoustics`, `availability`, `campaign`, `comparators`, `environment`,
`estimator`, `imaging`, `manager`, `mission`, `modes`, `optics`, `sensors`.

- **10 of 12 match the freeze record byte for byte.**
- `modes.py` and `optics.py` differ **on one line each**, inside the module
  docstring:

  ```
  -Reference implementation of ``projects/paper2/method/MODE_MANAGER_SPEC.md``
  +Reference implementation of ``method/MODE_MANAGER_SPEC.md``
  ```

  The specification path was rewritten when this repository was extracted from
  the development workspace, where the specs sit under `projects/paper2/method/`.
  Line 3 of both files is inside the module docstring. No executable statement,
  constant or default differs. `diff` output for both files is exactly the one
  line shown.

**Everything else** — the remaining 18 differences are demonstrator, packaging
and scenery files added or changed after the freeze: `launch/*`, `package.xml`,
`setup.py`, `nodes/*`, `worlds/mode_aware_survey.sdf`, `scripts/make_*.py`,
`scripts/capture_demonstrator_figure.py`, `test/test_demonstrator_scene.py`,
`seabed.py`. **None is in the campaign import closure.** `seabed.py` is reached
only from `scripts/make_seabed.py` and a demonstrator test.

The campaigns were run at commit `16a091a`, before any of this. The published
tree carries the interactive demonstrator, which the paper reports produces no
quantitative result.

### 4.2 The 254× figure is computed from rounded values

`25.44 / 0.10 = 254`. The unrounded values are 25.43801 and 0.09691, giving
**262.5×**. The manuscript's figure is self-consistent with the two rounded
numbers it prints alongside, and it **understates** the effect. No correction is
proposed; it is recorded here so a reader recomputing it is not surprised.

### 4.3 `campaign_final.csv` is not a paper artefact

A file of that name exists in the development workspace and is **not** the Study 1
development campaign despite the name. Two of its cells coincide with published
values, which makes it easy to mistake for the source. The Study 1 development
artefact is `campaign_v5.csv`, identified by matching all eight policies against
`tab:aggregate`. `campaign_final.csv` is not published and supports no claim.

---

## 5. What is deliberately not published

`development.csv`, `campaign_108.csv`, `campaign_v2.csv`, `campaign_v4.csv`,
`campaign_v6_superseded.csv`, `campaign_final.csv`, `proposed_altfix.csv`,
`static_sweep_v6_superseded.csv` — intermediate development runs against
implementations that no longer exist. None supports a claim in the manuscript.
The two campaigns the paper reports are published in full, together with both
configuration sweeps.
