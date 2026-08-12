"""Observable-only multimodal navigation-mode selection for Study 3."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math


class NavigationMode(Enum):
    OPTICAL_DVL="optical_dvl"
    OPTICAL_NO_BOTTOM_LOCK="optical_no_bottom_lock"
    LBL_AIDED="lbl_aided"
    USBL_AIDED="usbl_aided"
    RELATIVE_DEAD_RECKONING="relative_dead_reckoning"
    TERMINAL_DEGRADED="terminal_degraded"


@dataclass(frozen=True)
class ModeDecision:
    mode:NavigationMode
    reason:str
    absolute_source:str|None
    velocity_source:str
    fallback_required:bool


class ObservableModeSelector:
    """Select the smallest behaviorally distinct viable navigation mode.

    Inputs are posterior capability probabilities and service responses. No
    scenario identity, infrastructure truth, true pose, or future schedule is
    accepted by this interface.
    """
    #: A change of acoustic mode costs one probe opportunity: the newly selected
    #: service must be interrogated before it yields a fix, and the incumbent's
    #: next scheduled fix is forgone meanwhile. A Gaussian fix of standard
    #: deviation sigma carries information 1/sigma^2, so the candidate must
    #: carry at least twice the information of the fix it displaces to repay
    #: that opportunity: 1/s_c^2 >= 2/s_i^2, i.e. s_c <= s_i/sqrt(2). Derived
    #: from the sensing model and the probe schedule; not fitted to any result.
    SWITCH_INFORMATION_RATIO=2.0

    def __init__(self,boundary=.35,minimum_hold_s=8.):
        self.boundary=float(boundary);self.minimum_hold_s=float(minimum_hold_s)
        self.mode=None;self.last_change_s=float("-inf")
        #: Last *positive* observation per acoustic service: name -> (sigma_m, dop).
        #: Absence of a fresh probe is not evidence of absence; only a completed
        #: probe that did not answer is, and that erases the entry at once.
        self._viable={}

    def _observe_services(self,time_s,services):
        """Separate a refuted service from a merely un-refreshed one.

        Round-robin probing re-visits every catalogued service each revisit
        interval, so a retained entry is replaced by a positive or a negative
        observation within one interval. Retention needs no timeout of its own.
        """
        for evidence in services:
            if evidence.responding and evidence.gives_position:
                self._viable[evidence.name]=(float(evidence.sigma_m),float(evidence.dop),
                                             float(time_s)-float(evidence.age_s))
            elif evidence.responding or not (math.isfinite(evidence.dop)
                                             and math.isfinite(evidence.sigma_m)):
                # Either the service answered but cannot give a position, or the
                # probe came back with no usable geometry at all -- infinite DOP
                # and sigma, which is what a withdrawn or unusable service
                # returns. Both are evidence of absence: refute at once.
                self._viable.pop(evidence.name,None)
            # Otherwise the probe found valid geometry and merely lost the
            # packet. A stochastic drop is not evidence that the service is
            # gone, so the last positive observation stands.

    def _preferred_absolute(self,time_s,viable):
        """Smallest observable position uncertainty, with incumbent retention.

        A candidate may only displace the incumbent on evidence that is at
        least as recent as the incumbent's. Comparing a stale reading against a
        freshly confirmed one would act on older information than the vehicle
        already holds, which is the same error as treating silence as loss.
        """
        incumbent=("lbl" if self.mode is NavigationMode.LBL_AIDED else
                   "usbl" if self.mode is NavigationMode.USBL_AIDED else None)
        age=lambda name:float(time_s)-viable[name][2]
        best=min(viable,key=lambda n:(viable[n][0],viable[n][1],n))
        if incumbent is None or incumbent not in viable or best==incumbent:
            return best
        margin=viable[incumbent][0]/math.sqrt(self.SWITCH_INFORMATION_RATIO)
        if viable[best][0]<=margin and age(best)<=age(incumbent):
            return best
        return incumbent

    def _candidate(self,time_s,optical_probability,velocity_probability,dvl_bottom_lock,
                   dvl_water_track,services,terminal):
        if terminal:
            return ModeDecision(NavigationMode.TERMINAL_DEGRADED,"terminal_safety_boundary",
                                None,"inertial",True)
        absolute=dict(self._viable)
        velocity="bottom_lock_dvl" if dvl_bottom_lock else "water_track_dvl" if dvl_water_track else "inertial"
        if absolute:
            preferred=self._preferred_absolute(time_s,absolute)
            if preferred=="lbl":
                return ModeDecision(NavigationMode.LBL_AIDED,"observable_lbl_fix","lbl",velocity,False)
            if preferred=="usbl":
                return ModeDecision(NavigationMode.USBL_AIDED,"observable_usbl_fix","usbl",velocity,False)
        optical=optical_probability>=self.boundary
        velocity_ok=velocity_probability>=self.boundary and (dvl_bottom_lock or dvl_water_track)
        if optical and dvl_bottom_lock:
            return ModeDecision(NavigationMode.OPTICAL_DVL,"optical_and_bottom_lock_usable","optical","bottom_lock_dvl",False)
        if optical:
            return ModeDecision(NavigationMode.OPTICAL_NO_BOTTOM_LOCK,"optical_usable_without_bottom_lock",
                                "optical",velocity,False)
        return ModeDecision(NavigationMode.RELATIVE_DEAD_RECKONING,
                            "no_observable_horizontal_absolute_fix",None,
                            velocity if velocity_ok else "inertial",not velocity_ok)

    def select(self,time_s,*,optical_probability,velocity_probability,dvl_bottom_lock,
               dvl_water_track,services,terminal=False):
        self._observe_services(time_s,services)
        candidate=self._candidate(time_s,optical_probability,velocity_probability,dvl_bottom_lock,
                                  dvl_water_track,services,terminal)
        # Safety is immediate. Otherwise hold a still-viable selected absolute
        # mode to prevent chatter; loss of its required evidence exits at once.
        if self.mode is not None and candidate.mode is not self.mode:
            old_still_viable=(
                (self.mode is NavigationMode.OPTICAL_DVL and optical_probability>=self.boundary and dvl_bottom_lock) or
                (self.mode is NavigationMode.OPTICAL_NO_BOTTOM_LOCK and optical_probability>=self.boundary) or
                (self.mode is NavigationMode.LBL_AIDED and "lbl" in self._viable) or
                (self.mode is NavigationMode.USBL_AIDED and "usbl" in self._viable))
            if (not terminal and old_still_viable and
                    time_s-self.last_change_s<self.minimum_hold_s):
                return ModeDecision(self.mode,"minimum_mode_hold",self.mode.value.split("_")[0]
                                    if self.mode in {NavigationMode.LBL_AIDED,NavigationMode.USBL_AIDED} else
                                    "optical" if self.mode in {NavigationMode.OPTICAL_DVL,NavigationMode.OPTICAL_NO_BOTTOM_LOCK} else None,
                                    candidate.velocity_source,self.mode in {NavigationMode.RELATIVE_DEAD_RECKONING,NavigationMode.TERMINAL_DEGRADED})
        if candidate.mode is not self.mode:
            self.mode=candidate.mode;self.last_change_s=float(time_s)
        return candidate
