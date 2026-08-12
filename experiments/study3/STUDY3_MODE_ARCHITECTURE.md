# Study 3 observable multimodal navigation-mode architecture

Status: **IMPLEMENTED AND MECHANISM-TESTED; NO SCIENTIFIC CAMPAIGN RUN**  
Date: 2026-08-11

## Decision chain

The REACTIVE primary path is now:

> sensor/service observations -> probabilistic capability inference ->
> viable-mode determination -> navigation-mode selection -> technique/fusion/
> mission behavior -> fallback recovery only when no adequate absolute mode is
> viable.

The selector accepts only optical/velocity posterior probabilities, DVL
bottom-lock/water-track observations, technique-specific acoustic service
responses, and the existing terminal-safety decision. Its call signature has
no scenario, family, turbidity, infrastructure truth, true pose or future event.

## Finite mode set

The six modes below are the smallest set with distinct navigation behavior in
the actual simulator. Separate LBL-with-DVL and LBL-without-DVL combinatorial
modes are unnecessary: both use the same absolute source and record the
currently viable velocity source inside the mode decision. Single-beacon
ranging has no absolute mode because it does not observe horizontal position.

| Mode | Required observable capability | Aiding used | Fusion/navigation behavior | Entry | Exit/hysteresis | Uncertainty/safety |
|---|---|---|---|---|---|---|
| `OPTICAL_DVL` | optical posterior at existing usability boundary; bottom lock observed | P5-v4, bottom-lock DVL, IMU/depth | nominal fixed fusion; normal survey | optical and bottom lock usable; no responding absolute acoustic service with priority | immediate if a requirement is lost; minimum existing action-hold interval for switching between still-viable modes | absolute optical correction plus velocity aiding; no recovery |
| `OPTICAL_NO_BOTTOM_LOCK` | optical usable; bottom lock absent | P5-v4, water-track if observed, otherwise inertial/depth | optical-dominant navigation; nominal altitude and mission continue | optical usable without bottom lock | optical loss, bottom-lock restoration, or responding LBL/USBL | covariance grows faster between optical fixes; no recovery merely because DVL changed mode |
| `LBL_AIDED` | responding, position-observing LBL service | LBL fixes, available P5-v4, available DVL/IMU/depth | select LBL technique; nominal fixed fusion; continue mission | observable LBL response with valid geometry/propagation | immediate on lost/unusable LBL response; stable while service remains viable | LBL supplies horizontal absolute aiding; recovery suppressed |
| `USBL_AIDED` | responding, position-observing USBL service | USBL fixes, available P5-v4, available DVL/IMU/depth | select USBL technique; nominal fixed fusion; continue mission | observable USBL response | immediate on vessel/service/geometry loss | USBL supplies horizontal absolute aiding; recovery suppressed |
| `RELATIVE_DEAD_RECKONING` | no observable horizontal absolute fix; terminal boundary not reached | bottom-/water-track DVL if usable, otherwise IMU/depth | explicit degraded mode; continue with DVL+IMU while velocity aiding is viable; recovery only if velocity aiding also becomes inadequate | optical below boundary and no responding position-observing LBL/USBL | exit immediately when optical/LBL/USBL becomes observably viable; terminal safety may supersede | covariance/unaided time grows; spatial recovery is secondary to viable relative navigation |
| `TERMINAL_DEGRADED` | existing terminal safety condition | safety sensors only | surface/terminal safety action | existing terminal mission decision | terminal | no claim of submerged recoverability |

## Technique-specific acoustic evidence

`AcousticSignal` now carries a tuple of observable service evidence for each
responding deployed service: technique name, response status, whether it
observes position, DOP, sigma and age. The sensor generator derives these from
the existing propagation and geometry models. The policy never receives the
truth-side infrastructure context.

The mode selector therefore distinguishes:

- responding LBL with full position observability;
- responding USBL with full position observability;
- a single beacon that responds but remains range-only;
- silence/unusable geometry, which cannot make a mode viable.

## Behavioral integration

- A selected LBL/USBL mode changes the technique interrogated on the next
  cycle, which changes subsequent acoustic packets and estimator updates.
- `OPTICAL_NO_BOTTOM_LOCK` suppresses generic altitude/spatial recovery while
  healthy P5-v4 remains an absolute source.
- A viable optical or acoustic absolute mode forces normal mission continuation
  and nominal altitude; return/abort/hold are not used as substitutes.
- `RELATIVE_DEAD_RECKONING` is explicitly the point where the existing bounded
  recovery controller may act.
- `TERMINAL_DEGRADED` maps directly to the existing surface safety action.
- The selected `fixed_155` covariance-weighting behavior remains nominal unless
  existing observable innovation evidence calls for robust weighting.
- PREDICTIVE uses the same mode selector but prediction does not enter its
  primary viability logic; it remains secondary infrastructure.

## Deterministic mechanism verification

Tests demonstrate:

1. optical loss plus observable LBL response: `OPTICAL_DVL -> LBL_AIDED`;
2. DVL bottom-lock loss with healthy P5-v4:
   `OPTICAL_DVL -> OPTICAL_NO_BOTTOM_LOCK`, with unchanged altitude and
   `continue` rather than recovery;
3. observable LBL service changes actual acoustic fixes and subsequent aiding
   gaps relative to FIXED;
4. `USBL_AIDED -> OPTICAL_DVL` after USBL response disappears while optical
   remains healthy;
5. `RELATIVE_DEAD_RECKONING -> OPTICAL_DVL` when optical capability recovers;
6. range-only single beacon cannot create an absolute mode;
7. no absolute aid enters `RELATIVE_DEAD_RECKONING`, and the existing terminal
   condition enters `TERMINAL_DEGRADED`;
8. nominal LBL operation remains stable, continues the mission and does not
   invoke altitude/spatial recovery;
9. the selector interface contains no hidden truth/scenario argument.

## Remaining gaps before another DEVELOPMENT comparison

- The existing probabilistic filter still reports aggregate acoustic belief to
  legacy recovery code; the mode selector itself uses technique-specific
  evidence. A future cleanup may expose per-technique calibrated posteriors,
  but this is not required to distinguish current service viability.
- Service evidence is response-level rather than a temporally calibrated
  per-technique probability. Current minimum-hold behavior prevents chatter,
  but calibration should be assessed in development evidence rather than
  assumed.
- `RELATIVE_DEAD_RECKONING` fallback behavior retains the previously documented
  recovery limitations. Those fallbacks are now correctly secondary but are
  not thereby proven effective.
- Mission-mode dwell, transition counts, selected velocity source and fallback
  entry reasons must be included in the next development analysis.
- The new architecture has mechanism tests only. It has not been tuned or
  evaluated in a new scientific campaign, and no success claim is made yet.

P5-v4, `fixed_155`, scenarios, thresholds, corrected infrastructure physics,
previous evidence and held-out root `32,000,000` remain unchanged.
