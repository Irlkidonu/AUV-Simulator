# P5-v2 terminal execution failure

Date: 2026-08-10  
Status: **EXECUTION FAIL — no scientific feasibility result**

The one permitted frozen invocation terminated after 6.7 seconds while
processing the deterministic manifest. OpenCV returned a degenerate similarity
transform with zero estimated scale for one pair. The frozen implementation
attempted to invert that scale while converting the transform to a camera
position and raised `ZeroDivisionError` in `_centre_from_params`.

No `result.json` or verified per-pair scoring packets were written. Consequently
the interruption cannot use the protocol's same-root verified-packet resume
provision. Adding the missing degenerate-transform rejection would modify the
frozen implementation, and executing that modification would be a second
invocation. Both are prohibited.

This record is not evidence that the optical method is scientifically
infeasible. It is evidence that the frozen P5-v2 execution implementation was
not robust to a degenerate estimator return. P5-v1 remains the only completed
optical feasibility result and remains FAIL. Optical-localizer development is
closed for this submission cycle.

Frozen files, manifest, thresholds and prior evidence remain unchanged.
