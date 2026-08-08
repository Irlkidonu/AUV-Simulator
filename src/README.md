# Paper 2 software

`uuv_mode_aware_navigation/` — the ROS 2 package for this study. Physics,
estimator, mode manager, campaign runner and tests.

See [the package README](uuv_mode_aware_navigation/README.md) for run
instructions, or [the project README](../README.md) for an overview of the whole
project.

It consumes no code from Paper 1 and does not modify Paper 1's frozen estimator.
The two projects share only the workspace they sit in; this package builds into
its own isolated overlay (`.build`, `.install`)
so that building or testing here cannot affect anything else.
