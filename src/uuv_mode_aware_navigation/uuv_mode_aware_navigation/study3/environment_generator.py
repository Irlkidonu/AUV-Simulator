"""Seeded, policy-independent physical environment processes for Study 3."""
from __future__ import annotations

from dataclasses import asdict,dataclass,replace
import hashlib,json,math
from pathlib import Path
import numpy as np

from .scenarios import InfrastructureContext,PhysicalState


@dataclass(frozen=True)
class BoundedProcess:
    minimum:float
    maximum:float
    mean:float
    correlation_s:float
    stationary_sigma:float
    initial:float|None=None

    def __post_init__(self):
        if not self.minimum<=self.mean<=self.maximum:raise ValueError("process mean outside bounds")
        if self.correlation_s<=0 or self.stationary_sigma<0:raise ValueError("invalid process dynamics")


@dataclass(frozen=True)
class AvailabilityProcess:
    initial_available:bool
    failure_hazard_per_s:float
    recovery_hazard_per_s:float

    def __post_init__(self):
        if self.failure_hazard_per_s<0 or self.recovery_hazard_per_s<0:
            raise ValueError("availability hazards must be nonnegative")


@dataclass(frozen=True)
class EnvironmentConfig:
    """Ranges and difficulty, without event times or policy information."""
    name:str
    turbidity:BoundedProcess
    current_east_mps:BoundedProcess
    current_north_mps:BoundedProcess
    acoustic_noise_db:BoundedProcess
    lbl_geometry_scale:BoundedProcess
    water_track_probability:BoundedProcess
    dvl_health:AvailabilityProcess
    optical_health:AvailabilityProcess
    acoustic_health:AvailabilityProcess
    lbl_infrastructure:AvailabilityProcess
    usbl_infrastructure:AvailabilityProcess
    single_beacon_deployed:bool=False
    bottom_lock_reference_altitude_m:float=4.5
    bottom_lock_transition_width_m:float=.55
    bottom_lock_nominal_probability:float=.98
    acoustic_noise_midpoint_db:float=64.
    acoustic_noise_width_db:float=5.
    usbl_nominal_offset_m:float=8.

    def __post_init__(self):
        if self.bottom_lock_transition_width_m<=0 or self.acoustic_noise_width_db<=0:
            raise ValueError("transition widths must be positive")
        if not 0<=self.bottom_lock_nominal_probability<=1:
            raise ValueError("invalid nominal bottom-lock probability")

    @classmethod
    def from_dict(cls,record):
        value=dict(record)
        for key in ("turbidity","current_east_mps","current_north_mps","acoustic_noise_db",
                    "lbl_geometry_scale","water_track_probability"):
            value[key]=BoundedProcess(**value[key])
        for key in ("dvl_health","optical_health","acoustic_health","lbl_infrastructure",
                    "usbl_infrastructure"):
            value[key]=AvailabilityProcess(**value[key])
        return cls(**value)


@dataclass(frozen=True)
class LatentEnvironmentFrame:
    turbidity:float
    current_east_mps:float
    current_north_mps:float
    acoustic_noise_db:float
    lbl_geometry_scale:float
    water_track_probability:float
    dvl_healthy:bool
    optical_healthy:bool
    acoustic_healthy:bool
    lbl_deployed:bool
    usbl_deployed:bool


@dataclass(frozen=True)
class EnvironmentRealization:
    config:EnvironmentConfig
    seed:int
    dt_s:float
    frames:tuple[LatentEnvironmentFrame,...]
    digest:str

    @property
    def horizon_s(self):return (len(self.frames)-1)*self.dt_s

    @property
    def service_catalogue(self):
        result=[]
        if self.config.single_beacon_deployed:result.append("single_beacon")
        # Installed service identity is legitimate pre-mission knowledge even
        # if its current asset state later becomes unavailable.
        if (self.config.lbl_infrastructure.initial_available or
            self.config.lbl_infrastructure.recovery_hazard_per_s>0):result.append("lbl")
        if (self.config.usbl_infrastructure.initial_available or
            self.config.usbl_infrastructure.recovery_hazard_per_s>0):result.append("usbl")
        return tuple(result)

    def frame_at(self,step:int):return self.frames[min(max(int(step),0),len(self.frames)-1)]

    def physical_state(self,step:int,*,altitude_m:float,position_xy=(0.,0.))->PhysicalState:
        """Map latent environment and vehicle geometry into sensor capability."""
        frame=self.frame_at(step);cfg=self.config
        altitude=max(0.,float(altitude_m))
        sigmoid=lambda x:1./(1.+math.exp(-max(-60.,min(60.,x))))
        bottom_geometry=sigmoid((cfg.bottom_lock_reference_altitude_m-altitude)/
                                cfg.bottom_lock_transition_width_m)
        bottom=(cfg.bottom_lock_nominal_probability*bottom_geometry if frame.dvl_healthy else .01)
        water=(frame.water_track_probability if frame.dvl_healthy else .01)
        turbidity=min(1.,frame.turbidity+(0.55 if not frame.optical_healthy else 0.))
        noise_factor=sigmoid((cfg.acoustic_noise_midpoint_db-frame.acoustic_noise_db)/
                             cfg.acoustic_noise_width_db)
        acoustic_response=(noise_factor if frame.acoustic_healthy else .01)
        services=set()
        if cfg.single_beacon_deployed:services.add("single_beacon")
        if frame.lbl_deployed:services.add("lbl")
        if frame.usbl_deployed:services.add("usbl")
        response=[]
        for name in sorted(set(self.service_catalogue)):
            deployed=name in services
            geometry=(frame.lbl_geometry_scale if name=="lbl" else 1.)
            response.append((name,float(np.clip(acoustic_response*geometry if deployed else .01,.01,1.))))
        position=np.asarray(position_xy,dtype=float)
        # The support asset follows a separate nominal track. Relative range is
        # evaluated later by the acoustic geometry model, not disclosed here.
        vessel_offset=cfg.usbl_nominal_offset_m+.08*float(np.linalg.norm(position))
        degraded=(turbidity>.35 or bottom<.5 or acoustic_response<.5 or
                  abs(frame.current_east_mps)>.08 or abs(frame.current_north_mps)>.08)
        return PhysicalState(turbidity,float(np.clip(bottom,.01,1.)),float(np.clip(water,.01,1.)),
            float(np.clip(acoustic_response,.01,1.)),frame.acoustic_noise_db,bool(services),
            vessel_offset,degraded,InfrastructureContext.INFRASTRUCTURE_TRANSITION,
            frozenset(services),frame.current_east_mps,frame.current_north_mps,
            1.0 if frame.dvl_healthy else 3.0,0.0,frame.lbl_geometry_scale,tuple(response))


def _bounded_series(config:BoundedProcess,rng,steps,dt_s):
    phi=math.exp(-dt_s/config.correlation_s)
    innovation=config.stationary_sigma*math.sqrt(max(0.,1.-phi*phi))
    value=config.mean if config.initial is None else config.initial;result=[]
    for _ in range(steps):
        result.append(float(np.clip(value,config.minimum,config.maximum)))
        value=config.mean+phi*(value-config.mean)+innovation*rng.normal()
    return result


def _availability_series(config:AvailabilityProcess,rng,steps,dt_s):
    available=config.initial_available;result=[]
    fail=1.-math.exp(-config.failure_hazard_per_s*dt_s)
    recover=1.-math.exp(-config.recovery_hazard_per_s*dt_s)
    for _ in range(steps):
        result.append(available)
        if available and rng.random()<fail:available=False
        elif not available and rng.random()<recover:available=True
    return result


def generate_environment(config:EnvironmentConfig,seed:int,horizon_s:float,dt_s:float)->EnvironmentRealization:
    """Generate the immutable exogenous realization before either policy runs."""
    if horizon_s<=0 or dt_s<=0:raise ValueError("invalid realization duration")
    steps=int(round(horizon_s/dt_s))+1
    # Named child streams keep each physical process stable if an unrelated
    # process is added later and prevent policy/run ordering from consuming RNG.
    def child(name):
        raw=hashlib.sha256(f"{int(seed)}:{name}".encode()).digest()[:8]
        return np.random.default_rng(int.from_bytes(raw,"big"))
    continuous={name:_bounded_series(getattr(config,name),child(name),steps,dt_s)
      for name in ("turbidity","current_east_mps","current_north_mps","acoustic_noise_db",
                   "lbl_geometry_scale","water_track_probability")}
    discrete={name:_availability_series(getattr(config,name),child(name),steps,dt_s)
      for name in ("dvl_health","optical_health","acoustic_health","lbl_infrastructure",
                   "usbl_infrastructure")}
    frames=tuple(LatentEnvironmentFrame(*(continuous[name][i] for name in
        ("turbidity","current_east_mps","current_north_mps","acoustic_noise_db",
         "lbl_geometry_scale","water_track_probability")),*(discrete[name][i] for name in
        ("dvl_health","optical_health","acoustic_health","lbl_infrastructure",
         "usbl_infrastructure"))) for i in range(steps))
    payload={"config":asdict(config),"seed":int(seed),"dt_s":float(dt_s),
             "frames":[asdict(x) for x in frames]}
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    return EnvironmentRealization(config,int(seed),float(dt_s),frames,digest)


def load_environment_config(path)->EnvironmentConfig:
    return EnvironmentConfig.from_dict(json.loads(Path(path).read_text()))


def deployment_informed_environment_configuration(fixed,realization:EnvironmentRealization):
    """Select one fixed launch technique from initially deployed assets only."""
    initial=realization.physical_state(0,altitude_m=fixed.altitude_m,position_xy=(0.,0.))
    deployed=initial.deployed_acoustic_services
    technique=("lbl" if "lbl" in deployed else "usbl" if "usbl" in deployed else
               "single_beacon" if "single_beacon" in deployed else "none")
    return replace(fixed,acoustic_technique=technique)
