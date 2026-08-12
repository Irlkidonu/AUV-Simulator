# AUV-Simulator benchmark registry

Two benchmark identities coexist in this repository.

## `study2_legacy_v1.0`

The published Study 2 numerical identity. Its scenarios, positional seeds,
shared sensor RNG stream, abstract optical fix, gradient TRN abstraction,
first-order vehicle, manager/costs, metrics and result artefacts are immutable.
Fidelity overrides are refused.

## `platform_v2_dev`

The development identity for improved platform components. It uses new seeds and
explicit fidelity selections. Results from this identity never replace or
silently update Study 2 evidence.

Every new result must record its benchmark identifier and resolved fidelity
configuration.

