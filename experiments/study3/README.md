# Study 3 — evidence index

Study 3 is the principal final evaluation: an observable-evidence navigation
manager that selects among six navigation modes, compared against fixed
configurations under scripted degradations and under stochastic changing
environments.

**Study 3 is frozen and closed.** Both held-out blocks are spent, and the
runners and locks refuse re-execution.

---

## Start here

| File | What it is |
|---|---|
| `final_tables/` | Every final number as CSV/JSON — start here for the reported results |
| `final_tables/final_decision_record.json` | The predeclared decision rules and their evaluation |
| `STUDY3_HELDOUT_V2_DESIGN.json` | The final design and decision rules, frozen **before** authorization |
| `STUDY3_HELDOUT_V2_PROVENANCE.json` | What was executed, and two disclosed irregularities |
| `STUDY3_MODE_ARCHITECTURE.md` | The six modes and the observable-evidence mechanism |
| `STUDY3_CORRECTION_SPECIFICATION_V1.md` | The two controller corrections, written before implementation |

---

## The two held-out blocks — never pool them

Study 3 has two held-out evaluations. They measure **different controllers** and
are kept separate deliberately.

| Block | Root | Packets | Controller | Directory |
|---|---|---|---|---|
| Final | 36,000,000 | 2010 (810 scripted + 1200 generated) | corrected | `redesign_results/heldout_v2/` |
| Original | 32,000,000 | 810 (scripted only) | **pre-correction** | `redesign_results/heldout/` |

The original block is retained because it is the provenance of the
pre-correction system and because its result is reported in full. It was not
revised and not re-run after the controller was corrected.

### Final block structure

* **Part A — scripted families.** 7 primary × 30 seeds + 3 control × 20 seeds,
  × 3 policies = 810 runs. A robustness and reproducibility evaluation; it was
  predeclared as *not* required to show superiority.
* **Part B — generated changing environments.** 400 environment seeds × 3
  policies = 1200 runs. This is where the primary claim is decided.

Analyses are stored separately and are never pooled:
`redesign_results/heldout_v2_analysis_scripted.json` and
`.../heldout_v2_analysis_generated.json`.

---

## Verifying the evidence yourself

```bash
# every packet checksum, both blocks, plus the frozen file manifests
python3 experiments/study3/verify_lock.py       # original block
python3 experiments/study3/verify_lock_v2.py    # final block
```

Both report `held-out output already present` — that is the one-shot guard
correctly reporting a spent block, not a failure. Everything above that line
must pass.

```bash
# regenerate every final table from the immutable packets
python3 experiments/study3/build_final_tables.py
```

That script re-verifies every packet checksum as it runs, then rewrites
`final_tables/`. Compare against `FINAL_TABLES_SHA256SUMS.txt`.

---

## Other evidence in this directory

| Path | Content |
|---|---|
| `redesign_results/exploratory_e1/` | Post-freeze exploratory block, root 33,000,000, pre-correction controller |
| `redesign_results/final_development_v6/` | Final development evaluation of the corrected controller, root 35,000,000 |
| `redesign_results/final_validation_v5/` | Development validation campaign, root 31,900,000 |
| `interactive_sessions/` | Recorded interactive disturbance sequences and their replay results |
| `examples/` | The generated-environment configuration used by Part B |
| `STUDY3_SEED_REGISTRY.json` | Which seed band was used for what |

Development evidence is retained because it is the provenance of the final
result. It is **not** the final result and should not be cited as such.

---

## Null and adverse findings — reported, not hidden

* `STUDY3_DEVELOPMENT_NULL_FINDINGS.md` — ROBUST_FUSION and PREDICTIVE nulls.
* `STUDY3_SWITCHING_AND_PREDICTIVE_INVESTIGATION_V1.md` — why unnecessary mode
  switching occurred, and why PREDICTIVE's pre-emption gate was unreachable.
* `STUDY3_EXPLORATORY_E1_RESULTS_V1.md` — exploratory block, reported regardless
  of direction.
* `STUDY3_FAMILY_LINEAGE_AND_EXPLORATORY_PROPOSAL_V1.md` — what Study 3 does
  **not** cover, including terrain-relative navigation.
* `STUDY3_INTERACTIVE_TESTING_V1.md` — interactive testing, including cases where
  adaptation hurt.

Headline adverse results from the final block: PREDICTIVE takes pre-emptive
actions but shows no error benefit over REACTIVE and is worse on aiding
continuity; Part A shows no superiority over deployment-informed FIXED; and
absolute mission completion in generated environments is below 0.44 for every
policy.

