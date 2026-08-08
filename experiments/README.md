# Analysis scripts

Read-only analysis of campaign outputs. Nothing here re-runs a campaign or
writes to a result file.

These live outside the package deliberately. `src/uuv_mode_aware_navigation/scripts/`
is covered by the freeze record, so adding an analysis script there after the
freeze would make `freeze.py --verify` report the tree as modified — which is
exactly the signal that has to stay meaningful. Analysing a result is not part of
the source that produced it.

| Script | Purpose |
|---|---|
| `analyse_campaign.py` | Everything needed to judge a development campaign, in one pass |
| `analyse_held_out.py` | The held-out comparison: aggregate *J*, falsification conditions, per-family breakdown |
| `heldout_sweep.py` | The configuration sweep over the held-out scenarios |

## Reproducing the reported tables

```bash
python3 experiments/analyse_held_out.py \
    --held-out    src/uuv_mode_aware_navigation/results/held_out_2.csv \
    --development src/uuv_mode_aware_navigation/results/campaign_v7.csv
```

**Read the `J_dev` column, not `J_self`.** The script prints both. `J_self`
re-derives the normalisation constants on the set being reported; `J_dev` uses
the development constants from `results/DEVELOPMENT_NORMALISERS.json`, and
`J_dev` is what the paper reports. The script's own docstring explains why both
exist.

`PUBLICATION_ARTEFACT_MANIFEST.md` in the repository root maps every number in
the paper to the artefact and command that produced it.
