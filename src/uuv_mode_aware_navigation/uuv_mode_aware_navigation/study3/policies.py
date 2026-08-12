"""Fair Study 3 wrappers: one shared implementation, forecast-only contrast."""

from __future__ import annotations

from dataclasses import dataclass,replace
from enum import Enum
import math

from ..estimator import FusionMode
from ..capability.prediction import OpticalEvidenceForecaster,OpticalEvidenceForecasterConfig
from ..platform_v2 import PlatformStepInput,PlatformV2Coordinator
from ..recovery import RecoveryAction,RecoveryDecision
from .modes import NavigationMode,ObservableModeSelector


# Recovery actions with executable consequences in the current Study-3 loop.
# ``CHANGE_HEADING`` remains a generic platform-v2 enum value, but Study 3 has
# neither a justified selection rule nor heading-command semantics for it and
# therefore does not claim it as part of its action set.
STUDY3_RECOVERY_ACTIONS=frozenset({
    "continue","lower_altitude","reduce_speed","reposition_for_acoustics",
    "hold_for_fix","surface_for_gps",
})

# One complete acoustic-evidence lifetime and two ordinary 4 s optical/probing
# opportunities. This is derived from the sensor interface timing, not from a
# campaign or interactive outcome.
TERMINAL_LOSS_CONFIRMATION_S=8.0


@dataclass
class TerminalSafetyPrecedence:
    """Observable-only confirmation of complete, unsafe navigation loss."""
    viability_boundary:float
    critical_uncertainty_m2:float
    confirmation_s:float=TERMINAL_LOSS_CONFIRMATION_S
    loss_duration_s:float=0.0

    def update(self,dt_s:float,*,optical_evidence:bool,
               position_acoustic_evidence:bool,dvl_bottom_lock:bool,
               dvl_water_track:bool,capability_probability,
               uncertainty_m2:float)->bool:
        navigation_probabilities=(capability_probability["optical"],
            capability_probability["acoustic"],capability_probability["velocity"])
        complete_unsafe_loss=(
            not optical_evidence and not position_acoustic_evidence and
            not dvl_bottom_lock and not dvl_water_track and
            all(value<self.viability_boundary for value in navigation_probabilities) and
            uncertainty_m2>=self.critical_uncertainty_m2)
        self.loss_duration_s=(self.loss_duration_s+max(0.0,float(dt_s))
                              if complete_unsafe_loss else 0.0)
        return self.loss_duration_s>=self.confirmation_s


class PolicyKind(Enum):
    FIXED="fixed";DEPLOYMENT_FIXED="deployment_fixed";ROBUST_FUSION="robust_fusion";REACTIVE="reactive";PREDICTIVE="predictive"


@dataclass(frozen=True)
class FixedConfiguration:
    optical_channel: str="camera_offaxis"
    altitude_m: float=3.0
    speed_mps: float=.5
    acoustic_technique: str="single_beacon"
    fusion_mode: str="gate"
    prediction_horizon_s: float=10.0
    optical_quality_floor: float=.25
    usable_probability_boundary: float=.35
    trend_confirmation_frames: int=3
    minimum_cumulative_quality_decline: float=.18
    # One P5 observation arrives every 4 s in Study 3.  Eight seconds therefore
    # gives a recovery action two image opportunities before its benefit is
    # assessed.  A 48 s cooldown bounds low-altitude duty to 8/(8+48)=14.3%,
    # below the 16.5% maximum at 2 m implied by the declared survey target.
    recovery_dwell_s: float=8.0
    recovery_cooldown_s: float=48.0
    minimum_action_hold_s: float=8.0
    recovery_altitude_floor_m: float=2.0
    optical_evidence_window: int=4
    optical_evidence_horizon_s: float=16.0
    optical_evidence_score_floor: float=.40
    optical_evidence_decline_quorum: int=2
    optical_evidence_minimum_slope: float=.001


def deployment_informed_fixed_configuration(fixed:FixedConfiguration,catalogue)->FixedConfiguration:
    """Choose the one fixed acoustic technique known to be deployed at launch.

    This construction consumes only the pre-mission service catalogue.  It
    leaves every other locked fixed_155 setting unchanged and supplies no
    run-time adaptation or knowledge of later service loss/recovery.
    """
    services=frozenset(catalogue)
    if len(services)>1:
        raise ValueError("Study 3 deployment catalogue must identify at most one acoustic service")
    technique=next(iter(services),"none")
    if technique not in {"none","single_beacon","lbl","usbl"}:
        raise ValueError(f"unknown preloaded acoustic service {technique}")
    return replace(fixed,acoustic_technique=technique)


@dataclass(frozen=True)
class Study3Action:
    speed_mps: float
    altitude_m: float
    mission_action: str
    optical_channel: str
    acoustic_technique: str
    fusion_mode: str
    preemptive: bool
    navigation_mode: str="fixed_multimodal"


class Study3Policy:
    """Policies share sensing, belief, recovery and safety code.

    REACTIVE differs from PREDICTIVE only by replacing the observable trend
    supplied to prediction/recovery with zero. FIXED overrides normal selector
    outputs but retains the same terminal safety action.
    """
    def __init__(self,kind: PolicyKind,fixed=FixedConfiguration(),coordinator=None):
        self.kind=kind;self.fixed=fixed
        self.coordinator=coordinator or PlatformV2Coordinator()
        self._channel=fixed.optical_channel
        self._acoustic=fixed.acoustic_technique
        self._fusion=fixed.fusion_mode
        self._preempt_until_s=-1.0
        self._recovery_until_s=-1.0
        self._recovery_cooldown_until_s=-1.0
        self._recovery_preemptive=False
        self._altitude_trial_active=False
        self._altitude_trial_failed=False
        self._last_action=None
        self._last_action_change_s=-math.inf
        self._optical_forecaster=OpticalEvidenceForecaster(OpticalEvidenceForecasterConfig(
            fixed.optical_evidence_window,fixed.optical_evidence_horizon_s,
            fixed.optical_evidence_score_floor,fixed.optical_evidence_decline_quorum,
            fixed.optical_evidence_minimum_slope))
        self.last_optical_evidence_forecast=None
        self.mode_selector=ObservableModeSelector(fixed.usable_probability_boundary,
                                                  fixed.minimum_action_hold_s)
        self.last_mode_decision=None
        self._terminal_committed=False
        self._terminal_safety=TerminalSafetyPrecedence(
            fixed.usable_probability_boundary,
            self.coordinator.recovery.critical_covariance_m2)
        self.coordinator.predictor=replace(
            self.coordinator.predictor,horizon_s=fixed.prediction_horizon_s,
            optical_quality_floor=fixed.optical_quality_floor)
        self.coordinator.recovery=replace(
            self.coordinator.recovery,horizon_s=fixed.prediction_horizon_s,
            quality_floor=fixed.optical_quality_floor)

    @property
    def estimator(self):return self.coordinator.estimator

    def step(self,observable: PlatformStepInput):
        # Close a bounded optical altitude trial using only its subsequently
        # observed outcome. A failed trial is not repeated until a real optical
        # fix supplies materially new evidence.
        if self._altitude_trial_active and observable.time_s>=self._recovery_until_s:
            self._altitude_trial_failed=not observable.optical.available
            self._altitude_trial_active=False
        if observable.optical.available:self._altitude_trial_failed=False
        optical_evidence=self._optical_forecaster.observe(observable.time_s,observable.optical)
        self.last_optical_evidence_forecast=optical_evidence
        # Raw image-quality slope is deliberately excluded from both wrappers.
        # Only PREDICTIVE may consume the normalized P5 evidence forecast below.
        predictive_input=replace(observable,optical_quality_trend_per_s=0.0)
        supplied=(replace(observable,optical_quality_trend_per_s=0.0,
                          dvl=replace(observable.dvl,lock_probability_trend_per_s=0.0))
                  if self.kind is not PolicyKind.PREDICTIVE else predictive_input)
        output=self.coordinator.step(supplied)
        if self.kind is PolicyKind.PREDICTIVE and optical_evidence.warning:
            times=dict(output.forecast.time_to_loss_s);times["optical"]=optical_evidence.time_to_loss_s
            probability=dict(output.forecast.probability);probability["optical"]=optical_evidence.health_score
            output=replace(output,forecast=replace(output.forecast,probability=probability,
                impending=output.forecast.impending|frozenset({"optical"}),time_to_loss_s=times))
        position_acoustic_evidence=bool(output.acoustic_update_accepted or any(
            item.responding and item.gives_position
            for item in observable.acoustic.service_evidence))
        uncertainty_m2=float(sum(self.estimator.P[index,index] for index in range(3)))
        terminal_precedence=self._terminal_safety.update(observable.dt_s,
            optical_evidence=observable.optical.available,
            position_acoustic_evidence=position_acoustic_evidence,
            dvl_bottom_lock=observable.dvl.bottom_lock,
            dvl_water_track=observable.dvl.water_track,
            capability_probability=output.belief.usable_probability,
            uncertainty_m2=uncertainty_m2)
        if terminal_precedence:
            output=replace(output,
                recovery=RecoveryDecision(RecoveryAction.SURFACE_FOR_GPS,
                    "complete_navigation_loss_critical_uncertainty",False),
                mission_action="surface_for_gps")
        safety=output.mission_action=="surface_for_gps"
        self._terminal_committed=bool(self._terminal_committed or safety)
        mode=self.mode_selector.select(observable.time_s,
            optical_probability=output.belief.usable_probability["optical"],
            velocity_probability=output.belief.usable_probability["velocity"],
            dvl_bottom_lock=observable.dvl.bottom_lock,
            dvl_water_track=observable.dvl.water_track,
            services=observable.acoustic.service_evidence,terminal=self._terminal_committed)
        self.last_mode_decision=mode
        mission=output.mission_action
        # Generic platform-v2 may propose return/abort actions that are outside
        # the declared Study-3 action set.  They must not leak back into this
        # experiment as an undeclared spatial-recovery controller.
        if mission not in STUDY3_RECOVERY_ACTIONS:
            mission="hold_for_fix"
        # Waiting for the next routine acoustic packet is not a recovery action
        # while either optical or velocity aiding is currently usable.  Without
        # this reachability guard the generic mission selector holds forever in
        # nominal operation, so the logged policy never reaches the vehicle.
        if (mission=="hold_for_fix" and
            max(output.belief.usable_probability["optical"],
                output.belief.usable_probability["velocity"])>=.5):
            mission="continue"
        if self.kind in {PolicyKind.FIXED,PolicyKind.DEPLOYMENT_FIXED,PolicyKind.ROBUST_FUSION}:
            fusion=("weight" if self.kind is PolicyKind.ROBUST_FUSION
                    else self.fixed.fusion_mode)
            action=Study3Action(self.fixed.speed_mps,self.fixed.altitude_m,
                                "surface_for_gps" if self._terminal_committed else "continue",
                                self.fixed.optical_channel,self.fixed.acoustic_technique,
                                fusion,False,"fixed_multimodal")
        else:
            # Both adaptive policies use the same present-capability rules.  The
            # predictive policy may apply them to the frozen forecast too; no
            # scenario identifier, physical fault state or future time enters.
            usable=output.belief.usable_probability
            impending=(output.forecast.impending
                       if self.kind is PolicyKind.PREDICTIVE else frozenset())
            boundary=self.fixed.usable_probability_boundary
            optical_now=usable["optical"]<boundary
            acoustic_now=usable["acoustic"]<boundary
            velocity_now=usable["velocity"]<boundary
            optical_bad=(optical_now or
                         (self.kind is PolicyKind.PREDICTIVE and optical_evidence.warning))
            acoustic_bad=acoustic_now or "acoustic" in impending
            velocity_bad=velocity_now or "velocity" in impending
            # A weak modality alone is not a reason to abandon the nominal
            # survey geometry when another absolute aid is healthy. Recovery is
            # warranted for velocity loss or simultaneous/predicted loss of the
            # two absolute-aiding paths.
            trigger=(velocity_bad or (optical_bad and acoustic_bad))
            # Mode selection is primary. Recovery is reachable only when no
            # viable horizontal absolute mode remains.
            trigger=bool(trigger and mode.fallback_required and
                         mode.mode is NavigationMode.RELATIVE_DEAD_RECKONING)
            current_loss=(velocity_now or (optical_now and acoustic_now))
            # A capability is *predicted* to be lost only when it has not yet
            # reached its floor and the observable trend projects that it will
            # within the horizon. `impending` cannot express this: its
            # time_to_loss is 0.0 for capabilities already at the floor, so it
            # is non-empty almost always and conflates prediction with
            # detection. The strictly-positive test is the discriminator.
            horizon=self.fixed.prediction_horizon_s
            predicted={name for name,value in output.forecast.time_to_loss_s.items()
                       if 0.<value<=horizon}
            if optical_evidence.warning:predicted.add("optical")
            # Pre-emption is warranted for a capability the *currently selected*
            # mode depends on. Requiring RELATIVE_DEAD_RECKONING first made the
            # gate unreachable: that mode already implies optical loss, so the
            # branch could only open once there was nothing left to pre-empt.
            depends={"velocity"}
            if mode.absolute_source in {"lbl","usbl"}:depends.add("acoustic")
            elif mode.absolute_source=="optical":depends.add("optical")
            predicted_trigger=bool(self.kind is PolicyKind.PREDICTIVE
                                   and (predicted&depends) and not current_loss
                                   and not trigger)
            if predicted_trigger:
                # A predicted loss opens the same bounded recovery episode a
                # realised loss would, subject to the same cooldown below.
                trigger=True
                self._preempt_until_s=max(self._preempt_until_s,
                                          observable.time_s+self.fixed.prediction_horizon_s)
            if self.kind is PolicyKind.PREDICTIVE and observable.time_s<self._preempt_until_s:
                trigger=True
            # Recovery is an experiment with a finite observation window, not
            # a new permanent survey mode.  Repeated low-altitude requests were
            # previously accepted every tick, consuming the survey swath while
            # noisy beliefs also caused rapid action chatter.  Start one shared
            # bounded episode only when the cooldown permits it.
            altitude_retry_blocked=(self._altitude_trial_failed and
                                    output.recovery.action.value=="lower_altitude")
            if (trigger and not altitude_retry_blocked and
                    observable.time_s>=self._recovery_cooldown_until_s
                    and observable.time_s>=self._recovery_until_s):
                self._recovery_until_s=observable.time_s+self.fixed.recovery_dwell_s
                self._recovery_cooldown_until_s=(self._recovery_until_s+
                                                 self.fixed.recovery_cooldown_s)
                self._recovery_preemptive=predicted_trigger
                self._altitude_trial_active=(output.recovery.action.value=="lower_altitude")
            # A pre-emptive episode must be able to act while the mode is still
            # viable; requiring fallback_required here was the second gate that
            # made pre-emption unreachable. False for REACTIVE, which never sets
            # _recovery_preemptive, so REACTIVE behaviour is unchanged.
            recovery_active=(observable.time_s<self._recovery_until_s and
                             (mode.fallback_required or self._recovery_preemptive))
            # Return to the development-selected fixed channel when healthy;
            # hard-coding camera_offaxis here silently weakened both adaptive
            # policies after FIXED selected lidar.
            self._channel=("lidar" if optical_bad and self.fixed.optical_channel!="lidar"
                           else self.fixed.optical_channel)
            # The wrappers may react to technique-specific service handshakes,
            # never to the truth-side deployment label. Prefer the selected
            # technique when it responds; otherwise use an actually observed
            # absolute service. A restricted single beacon remains range-only.
            services=observable.acoustic.observable_services
            if mode.absolute_source in {"lbl","usbl"}:
                self._acoustic=mode.absolute_source
            self._fusion=("weight" if observable.innovation_exceedance_rate>.20
                          else self.fixed.fusion_mode)
            selected_speed=(output.selected_speed_mps
                            if output.recovery.action.value=="reduce_speed"
                            else self.fixed.speed_mps)
            selected_altitude=(max(self.fixed.recovery_altitude_floor_m,
                                   output.selected_altitude_m)
                               if recovery_active else self.fixed.altitude_m)
            action=Study3Action(selected_speed,selected_altitude,
                                ("surface_for_gps" if mode.mode is NavigationMode.TERMINAL_DEGRADED
                                 else "continue" if not mode.fallback_required else mission),self._channel,
                                self._acoustic,self._fusion,
                                bool(self.kind is PolicyKind.PREDICTIVE and recovery_active
                                     and self._recovery_preemptive),mode.mode.value)
            # Hysteresis is identical for REACTIVE and PREDICTIVE.  Terminal
            # safety remains immediate; ordinary selector changes persist long
            # enough for two image observations before another change.
            if (self._last_action is not None and not self._terminal_committed and
                    observable.time_s-self._last_action_change_s<self.fixed.minimum_action_hold_s):
                action=self._last_action
        if self._last_action is None or action!=self._last_action:
            self._last_action_change_s=observable.time_s
        self._last_action=action
        self.estimator.filter.fusion=FusionMode(action.fusion_mode)
        return action,output
