"""Truth-separated, image-driven closed-loop Study 3 development simulator."""
from __future__ import annotations

from dataclasses import asdict,dataclass,replace
import hashlib,math,time
import numpy as np

from ..acoustics import ACOUSTIC_TECHNIQUES,NoiseState
from ..acoustics_v2 import (AcousticPacketModel,AcousticWorldGeometry,
                            GeometryAwareFix,geometry_aware_fix)
from ..delayed_estimator import FixedLagNavigationFilter
from ..imaging import analyse_image
from ..localization import P5V4CapabilityAdapter,P5V4ImageLocalizer
from ..optics import CAMERA_COAXIAL,CAMERA_OFFAXIS,LIDAR,WaterState
from ..platform_v2 import (AcousticServiceEvidence,AcousticSignal,DVLSignal,
                           PlatformStepInput,PlatformV2Coordinator)
from ..rendering import CameraPose,FootprintOutsideWorld,GeoreferencedRenderer,WorldTexture
from .policies import FixedConfiguration,PolicyKind,Study3Policy
from .scenarios import deployed_acoustic_services,physical_state
from .discovery import PendingProbe,SerializedServiceDiscovery
from .telemetry import ModeTelemetry
from .transition_driver import TransitionScenario
from .environment_generator import EnvironmentRealization

CHANNELS={x.name:x for x in (CAMERA_COAXIAL,CAMERA_OFFAXIS,LIDAR)}
TECHNIQUES={x.name:x for x in ACOUSTIC_TECHNIQUES}
FORBIDDEN_POLICY_KEYS=frozenset({"truth","true_pose","family","scenario","turbidity",
                                 "fault_time","degradation_active","future"})


def navigation_velocity(action,estimated_xy,last_good_xy):
    """Translate a mission action into a horizontal command using onboard state.

    The physical plant applies the returned command to truth, but truth is not
    an input to this guidance function.  Abort/hold stop the current survey leg;
    return guidance points toward the last accepted estimated absolute fix.
    """
    if action.mission_action in {"hold_for_fix","surface_for_gps"}:
        return np.zeros(2)
    speed=.15*action.speed_mps
    if action.mission_action=="abort_leg":
        # Terminate the present along-track leg and divert onto the next survey
        # line. This is physically distinct from silently continuing +x and
        # avoids pretending that an abort means an indefinite stationary hold.
        return np.array([0.0,.15*min(action.speed_mps,.25)])
    if action.mission_action=="return_to_last_good_fix" and last_good_xy is not None:
        delta=np.asarray(last_good_xy,dtype=float)-np.asarray(estimated_xy,dtype=float)
        norm=float(np.linalg.norm(delta))
        return np.zeros(2) if norm<1e-9 else speed*delta/norm
    return np.array([speed,0.0])


def stream_seed(root:int,family:str,index:int,stream:str)->int:
    value=f"{root}:{family}:{index}:{stream}".encode()
    return int.from_bytes(hashlib.sha256(value).digest()[:8],"big")%(2**32)


@dataclass(frozen=True)
class RunResult:
    family:str;index:int;policy:str;completed:bool;safety_violation:bool
    rmse_transition_m:float;peak_error_m:float;unaided_time_s:float
    mission_duration_s:float;unnecessary_interventions:int;preemptive_actions:int
    optical_fixes:int;acoustic_fixes:int;runtime_s:float;trace_digest:str
    mean_policy_runtime_ms:float
    longest_unaided_gap_s:float=0.0
    recovery_time_s:float=math.inf
    capability_preserved:bool=False
    survey_coverage_fraction:float=0.0
    optical_forecast_episodes:int=0
    true_optical_forecast_episodes:int=0
    false_optical_forecast_episodes:int=0
    optical_prediction_lead_s:float=math.nan
    overall_rmse_m:float=math.nan
    mode_switches:int=0
    optical_channels_used:tuple[str,...]=()
    acoustic_techniques_used:tuple[str,...]=()
    fusion_modes_used:tuple[str,...]=()
    mission_actions_used:tuple[str,...]=()
    recovery_action_counts:tuple[tuple[str,int],...]=()
    navigation_modes_used:tuple[str,...]=()
    #: Analysis-only mode telemetry. Observational; no policy reads these and
    #: none participates in trace_digest, completion, or any primary outcome.
    mode_telemetry:dict|None=None
    #: Changes to commands/configuration actually applied to the vehicle.
    #: Added after the historical ``mode_switches`` field was found to mix
    #: logical mode labels with physical interventions.
    physical_interventions:int=0
    surfaced_for_gps:bool=False
    gps_reacquired:bool=False
    post_gps_mission_rule:str="not_applicable"
    gps_pre_reset_error_m:float=math.nan
    gps_post_reset_error_m:float=math.nan


def _quality(image)->float:
    f=analyse_image(image)
    # Pixel-only diagnostic. Absolute scene modulation prevents a smooth,
    # backscatter-dominated frame from masquerading as high quality merely
    # because it is also low noise.
    absolute=np.clip(f.structure_absolute/.12,0,1)
    contrast=np.clip(f.structure_contrast/.12,0,1)
    return float(math.sqrt(absolute*contrast))


def assert_observation_clean(record):
    def walk(value,path=""):
        if isinstance(value,dict):
            for key,item in value.items():
                if key.lower() in FORBIDDEN_POLICY_KEYS:raise AssertionError(f"truth leakage at {path}{key}")
                walk(item,path+key+".")
        elif isinstance(value,(list,tuple)): 
            for i,item in enumerate(value):walk(item,f"{path}{i}.")
    walk(record)


def run_one(root:int,family:str,index:int,kind:PolicyKind,
            fixed:FixedConfiguration=FixedConfiguration(),horizon_s:float=120.0,
            dt_s:float=2.0,image_period_s:float=4.0,keep_trace:bool=False,
            redesign_version:int=1,policy_factory=None,
            transition_scenario:TransitionScenario|None=None,
            environment_realization:EnvironmentRealization|None=None):
    """Execute one paired-policy member using common pre-action streams."""
    if transition_scenario is not None and environment_realization is not None:
        raise ValueError("scripted transition and generated environment are mutually exclusive")
    if transition_scenario is not None:
        if abs(horizon_s-transition_scenario.horizon_s)>1e-9:
            raise ValueError("run horizon must match transition scenario horizon")
    if environment_realization is not None:
        if abs(horizon_s-environment_realization.horizon_s)>1e-9 or abs(dt_s-environment_realization.dt_s)>1e-9:
            raise ValueError("run timing must match generated environment realization")
    started=time.perf_counter();rng=np.random.default_rng(stream_seed(root,family,index,"sensor"))
    dvl_rng=np.random.default_rng(stream_seed(root,family,index,"dvl"))
    water_track_rng=np.random.default_rng(stream_seed(root,family,index,"dvl_water_track"))
    acoustic_rng=np.random.default_rng(stream_seed(root,family,index,"acoustic"))
    world=WorldTexture.generate(2048,.04,stream_seed(root,family,index,"texture"))
    renderer=GeoreferencedRenderer(world,sensor_seed=stream_seed(root,family,index,"camera"),
                                   add_sensor_noise=False)
    localizer=P5V4ImageLocalizer();adapter=P5V4CapabilityAdapter()
    initial=(-7.0,float(rng.normal(0,.15)),-3.0)
    estimator=FixedLagNavigationFilter(initial_position=initial,fixed_lag_s=15)
    # `policy_factory` exists so a mechanism test can substitute an ablation
    # wrapper. It defaults to the real policy, so ordinary runs are unchanged.
    policy=(policy_factory or Study3Policy)(kind,fixed,PlatformV2Coordinator(estimator=estimator))
    true=np.array(initial,dtype=float);yaw=0.0;action_speed=fixed.speed_mps
    action_alt=fixed.altitude_m;channel=fixed.optical_channel;technique=fixed.acoustic_technique
    mission_progress_m=0.0
    generated_catalogue=(environment_realization.service_catalogue
                         if environment_realization is not None else ())
    acoustic_geometry_family=((transition_scenario is not None and
        "lbl" in transition_scenario.service_catalogue) or "lbl" in generated_catalogue or
        family in {"S3_ACOUSTIC_GEOMETRY_ASYNC",
        "S3_COMPOUND_OPTICAL_ACOUSTIC","S3_COMPOUND_DVL_ACOUSTIC"} or (
        redesign_version>=2 and family in {"S3_OPTICAL_GRADUAL","S3_DVL_GRADUAL","S3_RECOVERY"}))
    lbl_points=(((-10,-4,0),(-4,-4,0),(-4,4,0),(-10,4,0)) if acoustic_geometry_family
                else ((-8,-8,0),(8,-8,0),(8,8,0),(-8,8,0)))
    geometry=AcousticWorldGeometry(lbl_points,(0,0,0),(0,0,0))
    service_catalogue=(frozenset(environment_realization.service_catalogue)
                       if environment_realization is not None else
                       frozenset(transition_scenario.service_catalogue)
                       if transition_scenario is not None else
                       deployed_acoustic_services(family,0.0,horizon_s))
    discovery=SerializedServiceDiscovery(service_catalogue,opportunity_period_s=4.0,
                                         evidence_ttl_s=8.0)
    packets=[];last_optical=None;last_quality=1.0;last_lock=1.0;last_optical_t=-math.inf
    errors=[];transition_errors=[];unaided=0.;optical_fixes=acoustic_fixes=0
    current_gap=longest_gap=0.;restoration_time=None;recovery_time=math.inf
    unnecessary=preemptive=0;policy_times=[];trace=[];last_abs=-math.inf
    nominal_action=(fixed.speed_mps,fixed.altitude_m,fixed.optical_channel,
                    fixed.acoustic_technique,"continue")
    previous_action=nominal_action
    previous_navigation_mode="fixed_multimodal"
    commanded_velocity=np.array([.15*fixed.speed_mps,0.0])
    last_good_estimated_xy=None
    optical_channels={channel};acoustic_techniques={technique}
    fusion_modes={fixed.fusion_mode};mission_actions={"continue"}
    navigation_modes={"fixed_multimodal"}
    recovery_counts={};mode_switches=physical_interventions=0
    telemetry=ModeTelemetry()
    previous_optical_warning=False;optical_forecasts=true_optical_forecasts=false_optical_forecasts=0
    first_true_optical_warning=None;first_degraded_optical_loss=None
    surfacing=False;surfaced_for_gps=False;gps_reacquired=False
    gps_pre_reset_error=gps_post_reset_error=math.nan
    steps=int(round(horizon_s/dt_s))
    for step in range(steps+1):
        now=step*dt_s
        physical=(environment_realization.physical_state(
                      step,altitude_m=-true[2],position_xy=true[:2])
                  if environment_realization is not None else
                  transition_scenario.state_at(now) if transition_scenario is not None else
                  physical_state(family,now,horizon_s))
        estimator.predict(np.array([physical.imu_drift_mps2,0.,0.]),dt_s if step else 0.)
        # DVL is a measurement generated from hidden state and a paired stream.
        # A declared hardware crashout suppresses returns absolutely. Retain
        # the legacy draws (and consume would-have-been measurement noise) so
        # recovery resumes on the same paired random stream; only the failed
        # interval changes.
        legacy_effective_lock=float(np.clip(
            physical.dvl_lock_probability+.30*(3.0+true[2]),.02,1.0))
        locked_draw=bool(dvl_rng.random()<legacy_effective_lock)
        water_draw=bool(water_track_rng.random()<physical.dvl_water_track_probability)
        forced_dvl_loss=bool(physical.dvl_forced_unavailable)
        effective_lock=0.0 if forced_dvl_loss else legacy_effective_lock
        locked=bool(locked_draw and not forced_dvl_loss)
        water_track=bool(water_draw and not forced_dvl_loss)
        if forced_dvl_loss and locked_draw:
            dvl_rng.normal(0,.015*physical.dvl_noise_scale,3)
        if forced_dvl_loss and water_draw:
            water_track_rng.normal(0,.025*physical.dvl_noise_scale,3)
        if locked:
            velocity=np.r_[commanded_velocity,0.0]+dvl_rng.normal(0,.015*physical.dvl_noise_scale,3)
            estimator.update_velocity(velocity)
        if water_track:
            # Water track measures velocity relative to the water, not ground.
            water_velocity=np.r_[commanded_velocity,0.0]+water_track_rng.normal(
                0,.025*physical.dvl_noise_scale,3)
            estimator.update_water_velocity(water_velocity)
        estimator.update_depth(true[2]+rng.normal(0,.015))
        # Actual image path; reference is the surveyed map at the estimated pose.
        if step==0 or now-last_optical_t>=image_period_s-1e-9:
            water=WaterState.from_turbidity(physical.turbidity)
            config=CHANNELS[channel]
            estimate=estimator.position
            # Altitude is a direct onboard altimeter observation, not truth pose.
            # Using estimator depth here confounds map registration with vertical
            # covariance and can spuriously change reference scale.
            ref_pose=CameraPose(float(estimate[0]),float(estimate[1]),float(-true[2]),yaw)
            query_pose=CameraPose(float(true[0]),float(true[1]),float(-true[2]),yaw)
            try:
                query=renderer.render(query_pose,water,config)
            except FootprintOutsideWorld:
                # The vehicle has left the surveyed patch.  There is no
                # georeferenced imagery out here, so map-based optical aiding is
                # simply unavailable -- the same condition a failed registration
                # produces below, and one the mode selector already handles.  It
                # is not an error, and it must not abort the mission.
                query=None;quality=0.
            else:
                quality=_quality(query)
            if query is None:
                fix=None;record={"localization_success":False}
            else:
                try:
                    reference=renderer.render(ref_pose,WaterState.from_turbidity(0.0),config)
                    fix=localizer.localize(reference,query,ref_pose)
                    record=fix.capability_record()
                except ValueError:
                    fix=None;record={"localization_success":False}
            last_optical=adapter.observe(record,quality,0.)
            last_optical_t=now
            if last_optical.available and fix is not None and fix.estimated_pose is not None:
                # P5-v4 is a horizontal fix; retain the pressure-filter depth so
                # a fictitious zero-variance vertical optical observation cannot
                # dominate the joint NIS gate.
                outcome=estimator.update_position(np.array([fix.estimated_pose.x_m,fix.estimated_pose.y_m,estimator.position[2]]),last_optical.sigma_m)
                if outcome.accepted:
                    optical_fixes+=1;last_abs=now
                    last_good_estimated_xy=estimator.position[:2].copy()
        else:
            quality=last_quality
            last_optical=type(last_optical)(last_optical.available,last_optical.quality,last_optical.sigma_m,
                now-last_optical_t,last_optical.inliers,last_optical.inlier_fraction,
                last_optical.reprojection_px,last_optical.ambiguity_ratio,last_optical.reason,
                last_optical.keypoints_a,last_optical.keypoints_b,last_optical.matches)
        # One serialized acoustic opportunity is shared by service discovery and
        # positioning.  Catalogue identity is preloaded, but current quality is
        # unavailable until the corresponding probe response actually arrives.
        tech=TECHNIQUES.get(technique)
        # Deployment is distinct from physical usability.  A technique cannot
        # produce a fix unless its own required asset exists in this family;
        # geometry/range/noise are then evaluated independently below.
        infra=bool(tech and (tech.infrastructure=="none" or
                   technique in physical.deployed_acoustic_services))
        lbl=np.asarray(geometry.lbl_transponders_m,dtype=float)
        centre=np.mean(lbl,axis=0)
        scaled_lbl=tuple(map(tuple,centre+(lbl-centre)*physical.lbl_geometry_scale))
        moving_geometry=AcousticWorldGeometry(scaled_lbl,geometry.single_beacon_m,
            (physical.vessel_offset_m,0,0),(-.15,0,0))
        service_evidence=discovery.observe(now)
        gfix=None;probe_name=discovery.take_opportunity(now)
        if probe_name is not None:
            probe_tech=TECHNIQUES[probe_name]
            if probe_name in physical.deployed_acoustic_services:
                gfix=geometry_aware_fix(probe_tech,true,moving_geometry,
                                        NoiseState(physical.acoustic_noise_db),now)
                if gfix.available and acoustic_rng.random()>physical.response_probability(probe_name):
                    gfix=replace(gfix,available=False,covariance_m2=None,
                                 reason="interference_response_loss")
            else:
                gfix=GeometryAwareFix(False,probe_name,None,math.inf,math.inf,-math.inf,
                                      "service_not_responding")
            if gfix.available:
                packet=AcousticPacketModel(.08,.02,.05).generate(
                    now,gfix.slant_range_m,probe_tech,acoustic_rng)
                sigma=math.sqrt(float(np.max(np.linalg.eigvalsh(gfix.covariance_m2))))
                evidence=AcousticServiceEvidence(probe_name,not packet.dropped,
                    probe_tech.gives_position,gfix.dop,sigma,0.)
                discovery.submit(PendingProbe(probe_name,packet.arrival_time_s,evidence))
                # The probe can also yield a position update only when the
                # currently selected technique is the probed service.
                if technique==probe_name and probe_tech.gives_position:
                    measured=true+acoustic_rng.normal(0,sigma,3);measured[2]=estimator.position[2]
                    packets.append((packet,measured,gfix,sigma))
            else:
                timeout=now+probe_tech.fix_period_s
                discovery.submit(PendingProbe(probe_name,timeout,
                    AcousticServiceEvidence(probe_name,False,probe_tech.gives_position,
                                            math.inf,math.inf,0.)))
        service_evidence=discovery.observe(now)
        arrived=[p for p in packets if p[0].arrival_time_s<=now]
        packets=[p for p in packets if p[0].arrival_time_s>now]
        if arrived:
            packet,measured,afix,sigma=arrived[-1]
            acoustic=AcousticSignal(True,packet.validity_time_s,packet.arrival_time_s,measured,
                np.eye(3)*sigma**2,afix.dop,infra,4.,packet.dropped,
                service_catalogue,tuple(service_evidence))
        else:
            acoustic=AcousticSignal(False,max(0.,now-4),now,None,None,
                gfix.dop if gfix else math.inf,infra,4.,False,
                service_catalogue,tuple(service_evidence))
        optical_trend=(last_optical.quality-last_quality)/max(dt_s,1e-9)
        dvl_trend=(effective_lock-last_lock)/max(dt_s,1e-9)
        observation=PlatformStepInput(now,dt_s,last_optical,optical_trend,
            DVLSignal(locked,water_track,.0,effective_lock,dvl_trend),acoustic,
            0.,0.,-true[2],action_speed,action_speed*.01)
        serialized=asdict(observation);assert_observation_clean(serialized)
        tick=time.perf_counter();new_action,output=policy.step(observation)
        policy_times.append(time.perf_counter()-tick)
        optical_warning=bool(policy.last_optical_evidence_forecast and
                             policy.last_optical_evidence_forecast.warning and
                             kind is PolicyKind.PREDICTIVE)
        if optical_warning and not previous_optical_warning:
            optical_forecasts+=1
            future_step=min(steps,step+int(round(fixed.optical_evidence_horizon_s/dt_s)))
            future=(environment_realization.physical_state(
                        future_step,altitude_m=-true[2],position_xy=true[:2])
                    if environment_realization is not None else
                    transition_scenario.state_at(min(horizon_s,now+fixed.optical_evidence_horizon_s))
                    if transition_scenario is not None else
                    physical_state(family,min(horizon_s,now+fixed.optical_evidence_horizon_s),horizon_s))
            true_warning=future.turbidity>physical.turbidity+.02
            true_optical_forecasts+=int(true_warning);false_optical_forecasts+=int(not true_warning)
            if true_warning and first_true_optical_warning is None:first_true_optical_warning=now
        if (first_degraded_optical_loss is None and not last_optical.available and
                physical.turbidity>.02):first_degraded_optical_loss=now
        previous_optical_warning=optical_warning
        if output.acoustic_update_accepted:
            acoustic_fixes+=1;last_abs=now
            last_good_estimated_xy=estimator.position[:2].copy()
        aided=now-last_abs<=dt_s
        if not aided:unaided+=dt_s;current_gap+=dt_s;longest_gap=max(longest_gap,current_gap)
        else:
            if restoration_time is not None and not math.isfinite(recovery_time):recovery_time=now-restoration_time
            current_gap=0.
        # Intervention metrics describe vehicle/navigation actions. Estimator
        # fusion changes are the ROBUST_FUSION treatment itself, not physical
        # interventions, and returning to nominal closes an intervention rather
        # than starting an unnecessary new one.
        # Keep logical state transitions separate from commands/configuration
        # that can change sensing, fusion-independent vehicle behaviour, or
        # mission execution.  A renamed/reclassified mode is not by itself a
        # physical intervention.
        action_tuple=(new_action.speed_mps,new_action.altitude_m,new_action.optical_channel,
                      new_action.acoustic_technique,new_action.mission_action)
        intervention=action_tuple!=previous_action
        physical_interventions+=int(intervention)
        mode_switches+=int(new_action.navigation_mode!=previous_navigation_mode)
        optical_channels.add(new_action.optical_channel)
        acoustic_techniques.add(new_action.acoustic_technique)
        fusion_modes.add(new_action.fusion_mode)
        mission_actions.add(new_action.mission_action)
        navigation_modes.add(new_action.navigation_mode)
        recovery_name=output.recovery.action.value
        recovery_counts[recovery_name]=recovery_counts.get(recovery_name,0)+1
        # Analysis-only telemetry. Fed after the action is fixed and outside the
        # timed policy region, and never written into `trace`, so it cannot
        # affect a decision, a timing measurement or the trace digest.
        if policy.last_mode_decision is not None:
            recovery_executed=(
                (recovery_name=="lower_altitude" and
                 new_action.altitude_m!=fixed.altitude_m) or
                (recovery_name=="reduce_speed" and
                 new_action.speed_mps!=fixed.speed_mps) or
                (recovery_name=="reposition_for_acoustics") or
                (recovery_name=="hold_for_fix" and
                 new_action.mission_action=="hold_for_fix") or
                (recovery_name=="surface_for_gps" and
                 new_action.mission_action=="surface_for_gps"))
            telemetry.observe(now,policy.last_mode_decision,dt_s,aided=aided,
                              recovery_action=recovery_name,
                              recovery_executed=recovery_executed)
        entering_non_nominal=intervention and action_tuple!=nominal_action
        if entering_non_nominal and not physical.degradation_active:unnecessary+=1
        # Count entry into a real non-nominal vehicle action, not a forecast
        # label, estimator-only change, or return-to-nominal transition.
        preemptive+=int(entering_non_nominal and new_action.preemptive and
                        not physical.degradation_active)
        # Action is physically applied before the next observation.
        action_speed=new_action.speed_mps
        action_alt=max(1.,min(5.,new_action.altitude_m));channel=new_action.optical_channel;technique=new_action.acoustic_technique
        commanded_velocity=navigation_velocity(new_action,estimator.position[:2],last_good_estimated_xy)
        surfacing=bool(surfacing or new_action.mission_action=="surface_for_gps")
        # Compact headless mission scale: commanded speed changes trajectory
        # progress but the 120 s transition horizon remains fully exercised.
        true[:2]+=commanded_velocity*dt_s
        true[:2]+=np.array([physical.current_east_mps,physical.current_north_mps])*dt_s
        if surfacing:
            true[2]=min(0.0,true[2]+.5*dt_s)
        else:
            true[2]+=np.clip(-action_alt-true[2],-.25,.25)
        if redesign_version>=2:
            # Unmodelled cross-current becomes consequential when bottom-track
            # velocity aiding weakens. This is a physical vehicle/environment
            # interaction, not injected estimator noise.
            true[1]+=.04*(1.0-effective_lock)*dt_s
        # Survey-line coverage, rather than kinematic speed alone, determines
        # mission completion. Swath is proportional to altitude; operating low
        # improves sensing but needs more track length for equal area coverage.
        if new_action.mission_action in {"continue","abort_leg"}:
            mission_progress_m+=float(np.linalg.norm(commanded_velocity))*dt_s*(-true[2]/3.0)
        if output.recovery.action.value=="reposition_for_acoustics":true[1]+=-np.sign(true[1] or 1)*.15
        error=float(np.linalg.norm(estimator.position[:2]-true[:2]));errors.append(error)
        if physical.degradation_active:transition_errors.append(error)
        trace.append((round(now,3),round(quality,6),last_optical.reason,locked,round(float(output.belief.usable_probability["optical"]),6),
                      tuple(sorted(output.forecast.impending)),asdict(new_action),round(float(true[2]),4),round(error,5)))
        last_quality=last_optical.quality;last_lock=effective_lock
        previous_action=action_tuple
        previous_navigation_mode=new_action.navigation_mode
        if family=="S3_RECOVERY" and physical.degradation_active:restoration_time=None
        elif family=="S3_RECOVERY" and step>steps//2 and restoration_time is None:restoration_time=now
        if surfacing and true[2]>=-1e-9:
            surfaced_for_gps=True
            # GPS is a surface-only sensor measurement.  Truth generates the
            # noisy packet, but only the measurement crosses into the estimator.
            gps=np.r_[true[:2]+rng.normal(0.,1.5,2),0.0]
            gps_pre_reset_error=float(np.linalg.norm(estimator.position[:2]-true[:2]))
            estimator.reinitialize_position(gps,1.5)
            gps_post_reset_error=float(np.linalg.norm(estimator.position[:2]-true[:2]))
            gps_reacquired=True
            # Registered correction-cycle mission rule: safety surfacing aborts
            # the survey after estimator recovery; submerged mission resumption
            # is not claimed or simulated.
            break
    digest=hashlib.sha256(repr(trace).encode()).hexdigest()
    completion_target=(8.49 if redesign_version<2 else .63*.15*.75*horizon_s)
    completed=bool(mission_progress_m>=completion_target and
                   (not trace or trace[-1][6]["mission_action"]!="surface_for_gps"))
    result=RunResult(family,index,kind.value,completed,bool(max(errors,default=0)>8),
        float(np.sqrt(np.mean(np.square(transition_errors or errors)))),max(errors,default=0.),unaided,
        trace[-1][0] if trace else 0.,unnecessary,preemptive,optical_fixes,acoustic_fixes,
        time.perf_counter()-started,hashlib.sha256(repr(trace).encode()).hexdigest(),
        1000*float(np.mean(policy_times)),longest_gap,recovery_time,
        # Three missed 4 s absolute-aiding opportunities is the declared
        # preservation boundary; a single fix somewhere in the run is not.
        bool(longest_gap<=12.0 and (optical_fixes>0 or acoustic_fixes>0)),
        float(min(1.0,mission_progress_m/max(completion_target,1e-9))),
        optical_forecasts,true_optical_forecasts,false_optical_forecasts,
        (float(first_degraded_optical_loss-first_true_optical_warning)
         if first_degraded_optical_loss is not None and first_true_optical_warning is not None
         else math.nan),float(np.sqrt(np.mean(np.square(errors)))),mode_switches,
        tuple(sorted(optical_channels)),tuple(sorted(acoustic_techniques)),
        tuple(sorted(fusion_modes)),tuple(sorted(mission_actions)),
        tuple(sorted(recovery_counts.items())),tuple(sorted(navigation_modes)),
        telemetry.as_record(),physical_interventions,surfaced_for_gps,gps_reacquired,
        "terminate_after_gps_reacquisition" if gps_reacquired else "not_applicable",
        gps_pre_reset_error,gps_post_reset_error)
    return (result,trace) if keep_trace else result
