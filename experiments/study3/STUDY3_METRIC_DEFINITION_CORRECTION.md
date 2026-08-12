# Study 3 metric-definition correction

Date: 2026-08-11. Scope: analysis/instrumentation only.

The previous development implementation formed one action-change tuple from
speed, altitude, optical channel, acoustic technique, mission action **and the
logical navigation-mode label**. Consequently, a pure mode-label transition
could increment both `mode_switches` and, in nominal conditions,
`unnecessary_interventions` even when no command or sensor configuration
changed.

The corrected definitions are:

- `mode_switches`: changes in the logical `navigation_mode` state;
- `physical_interventions`: changes in speed, altitude, optical channel,
  acoustic technique, or mission action;
- `unnecessary_interventions`: entries into a non-nominal physical action or
  configuration while the development scenario is nominal.

Fusion-mode changes remain estimator treatments rather than physical
interventions, consistently with the registered ROBUST_FUSION comparator.
The correction does not change policy outputs, scenario physics, thresholds,
trajectories, or trace digests.

Existing development packets retain their original values. Their
`mode_switches` and `unnecessary_interventions` fields use the former mixed
definition and are not directly comparable to newly generated packets on those
two fields. No frozen Study-2 packet or evidence file was rewritten.

`CHANGE_HEADING` remains available as generic platform-v2 vocabulary but is
explicitly excluded from the current Study-3 action set: no scientifically
justified selector branch or heading-command consequence has been implemented.
