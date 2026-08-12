# Study 3 interactive recordings

Immutable disturbance recordings from interactive DEVELOPMENT sessions. Each
file is a truth-side disturbance script that can be replayed exactly; none is a
campaign result and none is held-out evidence.

Recordings are never edited. A recording carries its own `sha256` field over its
canonical body, and `SHA256SUMS` additionally fixes the file as stored.

| Recording | Seed | Referenced by |
|---|---|---|
| `study3_disturbance_seed_31895000.json` | 31,895,000 | `../STUDY3_INTERACTIVE_SESSION_31895000_DIAGNOSIS.md`, `../STUDY3_TERMINAL_SAFETY_PRECEDENCE_MECHANISM.md` |

Verify:

```bash
cd experiments/study3/recordings && sha256sum -c SHA256SUMS
```

The embedded record checksum is checked independently by the interactive loader
(`study3/interactive.py`), which refuses a record whose canonical body does not
hash to its stored `sha256`.
