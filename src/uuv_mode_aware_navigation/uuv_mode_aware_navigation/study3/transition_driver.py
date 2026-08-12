"""Configurable truth-side within-mission transition driver for Study 3.

The driver is owned by the simulator.  Policies receive only measurements
generated from :class:`PhysicalState`; no phase label, target, transition time,
or future state crosses the policy boundary.
"""
from __future__ import annotations

from dataclasses import asdict,dataclass,fields,replace
import json
from pathlib import Path

from .scenarios import InfrastructureContext,PhysicalState


@dataclass(frozen=True)
class TransitionTarget:
    """Physical values reached at the end of a phase.

    Numeric values ramp linearly. Deployed services and technique-specific
    response maps change at phase entry, representing asset handover/departure.
    """
    turbidity:float|None=None
    dvl_lock_probability:float|None=None
    dvl_water_track_probability:float|None=None
    acoustic_response_probability:float|None=None
    acoustic_noise_db:float|None=None
    vessel_offset_m:float|None=None
    current_east_mps:float|None=None
    current_north_mps:float|None=None
    dvl_noise_scale:float|None=None
    imu_drift_mps2:float|None=None
    lbl_geometry_scale:float|None=None
    deployed_acoustic_services:tuple[str,...]|None=None
    service_response_probability:tuple[tuple[str,float],...]|None=None


@dataclass(frozen=True)
class TransitionPhase:
    name:str
    start_s:float
    end_s:float
    target:TransitionTarget

    def __post_init__(self):
        if self.start_s<0 or self.end_s<=self.start_s:
            raise ValueError(f"invalid phase interval {self.name}")


@dataclass(frozen=True)
class ModeExpectation:
    """Evaluator-only expected viable mode after a physical transition."""
    after_s:float
    expected_mode:str
    rationale:str


@dataclass(frozen=True)
class TransitionScenario:
    name:str
    horizon_s:float
    initial:PhysicalState
    phases:tuple[TransitionPhase,...]
    service_catalogue:tuple[str,...]
    expectations:tuple[ModeExpectation,...]=()

    def __post_init__(self):
        if self.horizon_s<=0:raise ValueError("horizon must be positive")
        previous=-1.0
        allowed={"single_beacon","lbl","usbl"}
        for phase in self.phases:
            if phase.start_s<previous or phase.end_s>self.horizon_s:
                raise ValueError("phases must be ordered, non-overlapping and inside horizon")
            previous=phase.end_s
        if not set(self.service_catalogue)<=allowed:raise ValueError("unknown catalogue service")
        deployed=set(self.initial.deployed_acoustic_services)
        for phase in self.phases:
            if phase.target.deployed_acoustic_services is not None:
                deployed.update(phase.target.deployed_acoustic_services)
        if not deployed<=set(self.service_catalogue):
            raise ValueError("every potentially deployed service must be predeclared in catalogue")

    def state_at(self,time_s:float)->PhysicalState:
        """Return hidden physical state without exposing schedule to a policy."""
        t=min(max(float(time_s),0.0),self.horizon_s);state=self.initial
        for phase in self.phases:
            if t<phase.start_s:break
            start=state
            fraction=min(max((t-phase.start_s)/(phase.end_s-phase.start_s),0.0),1.0)
            updates={}
            for item in fields(TransitionTarget):
                value=getattr(phase.target,item.name)
                if value is None:continue
                if item.name in {"deployed_acoustic_services","service_response_probability"}:
                    updates[item.name]=(frozenset(value) if item.name=="deployed_acoustic_services" else tuple(value))
                else:
                    updates[item.name]=getattr(start,item.name)+fraction*(value-getattr(start,item.name))
            state=replace(start,**updates)
            if t<phase.end_s:break
        degraded=(state!=self.initial)
        return replace(state,infrastructure_available=bool(state.deployed_acoustic_services),
                       degradation_active=degraded)

    @classmethod
    def from_dict(cls,record:dict):
        initial=dict(record["initial"])
        initial["infrastructure_context"]=InfrastructureContext(initial["infrastructure_context"])
        initial["deployed_acoustic_services"]=frozenset(initial.get("deployed_acoustic_services",()))
        initial["service_response_probability"]=tuple(
            (str(k),float(v)) for k,v in initial.get("service_response_probability",()))
        phases=[]
        for p in record.get("phases",()):
            target=dict(p["target"])
            if "deployed_acoustic_services" in target:
                target["deployed_acoustic_services"]=tuple(target["deployed_acoustic_services"])
            if "service_response_probability" in target:
                target["service_response_probability"]=tuple(
                    (str(k),float(v)) for k,v in target["service_response_probability"])
            phases.append(TransitionPhase(p["name"],float(p["start_s"]),float(p["end_s"]),
                                          TransitionTarget(**target)))
        expectations=tuple(ModeExpectation(**x) for x in record.get("expectations",()))
        return cls(record["name"],float(record["horizon_s"]),PhysicalState(**initial),
                   tuple(phases),tuple(record.get("service_catalogue",())),expectations)

    def to_dict(self):
        record=asdict(self)
        record["initial"]["infrastructure_context"]=self.initial.infrastructure_context.value
        record["initial"]["deployed_acoustic_services"]=sorted(self.initial.deployed_acoustic_services)
        return record


def load_transition_scenario(path)->TransitionScenario:
    return TransitionScenario.from_dict(json.loads(Path(path).read_text()))


def truth_side_best_viable_mode(state:PhysicalState)->str:
    """Evaluator-only physical viability classification for mechanism tests.

    This function is never imported by the policy path.  Its conservative
    cutoffs describe whether a physical source can plausibly support a mode;
    policy decisions must independently infer the same condition from packets.
    """
    response=dict(state.service_response_probability)
    acoustic_ok=state.acoustic_noise_db<=65.
    lbl=("lbl" in state.deployed_acoustic_services and acoustic_ok and
         response.get("lbl",state.acoustic_response_probability)>=.5 and
         state.lbl_geometry_scale>=.35)
    usbl=("usbl" in state.deployed_acoustic_services and acoustic_ok and
          response.get("usbl",state.acoustic_response_probability)>=.5)
    optical=state.turbidity<=.35
    bottom=state.dvl_lock_probability>=.5
    water=state.dvl_water_track_probability>=.5
    if lbl:return "lbl_aided"
    if usbl:return "usbl_aided"
    if optical and bottom:return "optical_dvl"
    if optical:return "optical_no_bottom_lock"
    if bottom or water:return "relative_dead_reckoning"
    return "terminal_degraded"


def deployment_informed_transition_configuration(fixed,scenario:TransitionScenario):
    """Choose one launch-time acoustic technique without future knowledge.

    The full catalogue is legitimate service identity, but only infrastructure
    declared deployed at launch can determine the fixed choice.  The returned
    object never adapts later. All non-acoustic fixed settings are preserved.
    """
    deployed=scenario.initial.deployed_acoustic_services
    technique=("lbl" if "lbl" in deployed else "usbl" if "usbl" in deployed else
               "single_beacon" if "single_beacon" in deployed else "none")
    return replace(fixed,acoustic_technique=technique)


def standard_transition_scenarios()->dict[str,TransitionScenario]:
    """Physically interpretable DEVELOPMENT mechanisms, not campaign roots."""
    base=PhysicalState(.05,.98,.85,1.0,45.,True,5.,False,
        InfrastructureContext.INFRASTRUCTURE_TRANSITION,frozenset({"lbl"}),
        0.,0.,1.,0.,1.,(("lbl",1.0),("usbl",1.0)))
    return {
      "optical_lbl_recovery":TransitionScenario("optical_lbl_recovery",120.,base,(
        TransitionPhase("visibility_loss",24.,48.,TransitionTarget(turbidity=.85)),
        TransitionPhase("lbl_geometry_loss_and_usbl_arrival",52.,64.,TransitionTarget(
            acoustic_noise_db=48.,lbl_geometry_scale=.18,
            deployed_acoustic_services=("lbl","usbl"),
            service_response_probability=(("lbl",.05),("usbl",1.0)))),
        TransitionPhase("optical_recovery",78.,98.,TransitionTarget(
            turbidity=.05,acoustic_noise_db=48.,
            service_response_probability=(("lbl",.05),("usbl",.05)))),
      ),("lbl","usbl"),(
        ModeExpectation(0.,"lbl_aided","initial deployed LBL is observable"),
        ModeExpectation(52.,"usbl_aided","LBL response is lost while USBL support arrives"),
        ModeExpectation(91.,"optical_dvl","visibility becomes physically viable again"))),
      "dvl_acoustic_handover":TransitionScenario("dvl_acoustic_handover",120.,replace(base,
          deployed_acoustic_services=frozenset({"usbl"}),
          service_response_probability=(("usbl",1.0),)),(
        TransitionPhase("bottom_lock_loss",24.,44.,TransitionTarget(
            dvl_lock_probability=.03,dvl_water_track_probability=.80,current_north_mps=.10)),
        TransitionPhase("usbl_departure_lbl_entry",54.,66.,TransitionTarget(
            deployed_acoustic_services=("lbl",),vessel_offset_m=85.,
            service_response_probability=(("usbl",.02),("lbl",1.0)))),
        TransitionPhase("bottom_lock_recovery",82.,102.,TransitionTarget(
            dvl_lock_probability=.98,current_north_mps=.01)),
      ),("usbl","lbl"),(
        ModeExpectation(0.,"usbl_aided","surface support initially responds"),
        ModeExpectation(54.,"lbl_aided","USBL departs and deployed LBL responds"),
        ModeExpectation(106.,"lbl_aided","bottom lock recovers while LBL remains usable"))),
      "compound_terminal":TransitionScenario("compound_terminal",110.,replace(base,
          deployed_acoustic_services=frozenset({"usbl"}),
          service_response_probability=(("usbl",1.0),)),(
        TransitionPhase("compound_loss",25.,55.,TransitionTarget(turbidity=.95,
            dvl_lock_probability=.02,dvl_water_track_probability=.02,
            deployed_acoustic_services=(),service_response_probability=(("usbl",.02),),
            current_north_mps=.16,imu_drift_mps2=.004)),
      ),("usbl",),(
        ModeExpectation(0.,"usbl_aided","initial USBL support responds"),
        ModeExpectation(41.,"terminal_degraded","no submerged horizontal mode remains"))),
    }
