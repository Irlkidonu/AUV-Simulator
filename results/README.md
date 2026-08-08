# Results

The campaign outputs are in
[`../src/uuv_mode_aware_navigation/results/`](../src/uuv_mode_aware_navigation/results/),
alongside the freeze record and the normalisation constants.

Every artefact — its generating command, campaign, seed block, SHA-256 and the
claims it supports — is listed in
[`../PUBLICATION_ARTEFACT_MANIFEST.md`](../PUBLICATION_ARTEFACT_MANIFEST.md).

To check the files against their recorded hashes:

```bash
cd src/uuv_mode_aware_navigation/results && sha256sum -c ARTEFACT_SHA256SUMS
```

Two campaigns are published. Study 1 (`campaign_v5.csv`, `held_out.csv`)
evaluated the initial implementation over fifteen scenario families. Study 2
(`campaign_v7.csv`, `held_out_2.csv`) is the final implementation over nineteen
families, and every claim about the system as it stands rests on its held-out
block. Both configuration sweeps are included.
