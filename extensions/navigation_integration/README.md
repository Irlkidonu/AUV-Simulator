# Optional fiducial/DVL integration mechanisms

This opt-in extension contains two reusable interface mechanisms developed for
an external localization integration:

- image-derived mapped multi-marker ArUco pose and covariance estimation;
- explicit navigation/body/DVL frame conversion, lever-arm handling, and
  acquisition-time rate gating.

The extension does not alter the simulator baseline, launch files, worlds, or
default behavior. It contains no scientific results, generated artifacts,
plots, or frozen estimator configuration. Consumers may import the
pure Python modules directly and adapt their outputs to their own ROS interface.

Run the bounded extension tests with:

```bash
python3 -m pytest extensions/navigation_integration/test -q
```
