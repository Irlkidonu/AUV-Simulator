#!/usr/bin/env python3
"""P5-v4.1 development: require bidirectionally consistent ratio matches."""

from __future__ import annotations

import importlib.util
from pathlib import Path


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("p5_v4_frozen_dependency", HERE / "run.py")
V4 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V4)


class MutualMatcher:
    def __init__(self, matcher):
        self.matcher = matcher

    def knnMatch(self, first, second, k=2):
        forward = self.matcher.knnMatch(first, second, k=k)
        reverse = self.matcher.knnMatch(second, first, k=k)
        reverse_good = {
            pair[0].queryIdx: pair[0].trainIdx
            for pair in reverse if len(pair) == 2 and pair[0].distance < .80 * pair[1].distance
        }
        return [pair for pair in forward if len(pair) == 2
                and pair[0].distance < .80 * pair[1].distance
                and reverse_good.get(pair[0].trainIdx) == pair[0].queryIdx]


class CvProxy:
    def __init__(self, wrapped):
        self.wrapped = wrapped

    def __getattr__(self, name):
        return getattr(self.wrapped, name)

    def BFMatcher(self, *args, **kwargs):
        return MutualMatcher(self.wrapped.BFMatcher(*args, **kwargs))


V4.BASE.cv2 = CvProxy(V4.BASE.cv2)


if __name__ == "__main__":
    raise SystemExit(V4.main())
