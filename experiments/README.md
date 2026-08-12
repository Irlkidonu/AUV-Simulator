# Experiments and evidence

| Directory | Contents |
|---|---|
| `study3/` | **The principal final evaluation.** Mode-aware navigation against fixed comparators, including both held-out blocks. Start at `study3/README.md`. |
| `platform_v2/` | Sensing and platform characterisation spikes: the optical front end, terrain-relative navigation, active recovery and system integration. Checksums in `platform_v2/SHA256SUMS`. |
| `analyse_campaign.py`, `analyse_held_out.py`, `heldout_sweep.py` | Study 1–2 analysis entry points. See `../RESULTS.md`. |

Result packets are immutable and self-checksummed: each carries a
`packet_sha256` over its own canonical content, so any edit is detectable. The
verification commands are in `../README.md`.
