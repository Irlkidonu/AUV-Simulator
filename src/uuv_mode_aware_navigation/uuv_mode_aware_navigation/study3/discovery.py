"""Serialized, observable acoustic-service discovery for Study 3."""
from __future__ import annotations
from dataclasses import dataclass,replace

from ..platform_v2 import AcousticServiceEvidence


@dataclass(frozen=True)
class PendingProbe:
    service:str
    completion_time_s:float
    evidence:AcousticServiceEvidence


class SerializedServiceDiscovery:
    """Round-robin one catalogued service per acoustic opportunity.

    The catalogue is legitimate pre-mission deployment information. Current
    quality is absent until a probe completes, and stale evidence expires after
    two ordinary acoustic opportunities. The scheduler has no scenario/fault
    input and cannot probe services outside the preloaded catalogue.
    """
    def __init__(self,catalogue,opportunity_period_s=4.0,evidence_ttl_s=8.0):
        if opportunity_period_s<=0 or evidence_ttl_s<=0:raise ValueError("invalid discovery timing")
        self.catalogue=tuple(sorted(set(catalogue)))
        self.opportunity_period_s=float(opportunity_period_s)
        self.evidence_ttl_s=float(evidence_ttl_s)
        self._next_opportunity_s=0.0;self._cursor=0;self._pending=[];self._latest={}

    def take_opportunity(self,time_s):
        if not self.catalogue or time_s+1e-9<self._next_opportunity_s:return None
        service=self.catalogue[self._cursor%len(self.catalogue)];self._cursor+=1
        self._next_opportunity_s=float(time_s)+self.opportunity_period_s
        return service

    def submit(self,probe:PendingProbe):
        if probe.service not in self.catalogue:raise ValueError("probe outside preloaded catalogue")
        self._pending.append(probe)

    def observe(self,time_s):
        ready=[p for p in self._pending if p.completion_time_s<=time_s+1e-9]
        self._pending=[p for p in self._pending if p.completion_time_s>time_s+1e-9]
        for p in sorted(ready,key=lambda x:x.completion_time_s):
            self._latest[p.service]=(p.completion_time_s,p.evidence)
        visible=[]
        for name,(stamp,evidence) in sorted(self._latest.items()):
            age=max(0.0,float(time_s)-stamp)
            if age<=self.evidence_ttl_s:visible.append(replace(evidence,age_s=age))
        return tuple(visible)
