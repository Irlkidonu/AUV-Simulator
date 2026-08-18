"""Generate the physics world SDF from the package's YAML configuration.

The configuration is the single source of truth; the SDF is a build product,
checked in so it can be reviewed, hashed and recorded in run provenance.
``test_world_builder.py`` regenerates it and requires a byte-for-byte match, so
the two cannot drift: editing the SDF by hand fails the suite.

Why generate rather than hand-maintain: the physics constants have to be
*discoverable* -- mass, inertia, damping and thruster geometry belong in one
readable file, not scattered through 650 lines of markup where a reviewer
cannot see them together. Generation is what keeps the readable copy
authoritative instead of decorative.

Pure Python and PyYAML. No ROS, no Gazebo -- building a world must not require
the simulator that runs it.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import yaml

__all__ = ["load_config", "build_world_sdf", "write_world", "PACKAGE_ROOT",
           "CONFIG_DIR", "WORLD_PATH", "WORLD_NAME"]

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
WORLD_DIR = PACKAGE_ROOT / "worlds"
WORLD_NAME = "auv_sim_physics_base"
WORLD_PATH = WORLD_DIR / f"{WORLD_NAME}.sdf"
VALIDATED_WORLD_NAME = "auv_sim_physics_validated"
VALIDATED_WORLD_PATH = WORLD_DIR / f"{VALIDATED_WORLD_NAME}.sdf"

I = "  "                     # one indent level


#: The eight-segment annular collar. Inner radius 0.24 m lets the 0.46 x 0.18 m
#: hull enter the throat; a solid cylinder would seal it. Segments are boxes
#: because DART resolves box-box contact stably at a 1 ms step, and the collar's
#: job is to stop a laterally offset vehicle, not to look like a torus.
#: Sensors added to the dynamic vehicle by correction C7. Transcribed from the
#: frozen kinematic reference (underwater_docking.sdf), not re-chosen.
SENSOR_SUITE = {
    "fls": {"type": "gpu_lidar", "pose": [0.24, 0.0, 0.0, 0.0, 0.0, 0.0],
            "update_rate_hz": 10, "topic": "/bluerov2_phys/fls/raw",
            "samples": 128, "min_angle_rad": -0.52, "max_angle_rad": 0.52,
            "range_min_m": 0.2, "range_max_m": 12.0, "range_resolution_m": 0.01,
            "frame_id": "bluerov2_phys/base_link/fls"},
    "imu": {"type": "imu", "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "update_rate_hz": 100, "topic": "/bluerov2_phys/imu",
            "frame_id": "bluerov2_phys/base_link/imu"},
    "dvl": {"type": "custom", "gz_type": "dvl",
            "pose": [0.0, 0.0, -0.09, 0.0, 0.0, 0.0],
            "update_rate_hz": 10, "topic": "/bluerov2_phys/dvl",
            "beam_tilt_deg": 30, "beam_aperture_deg": 2,
            "beam_rotations_deg": [45, 135, 225, 315], "noise_stddev_mps": 0.0005,
            "frame_id": "bluerov2_phys/base_link/dvl"},
}

COLLAR_SEGMENTS = 8
COLLAR_INNER_R = 0.24
COLLAR_OUTER_R = 0.30


def load_config(validated: bool = False) -> dict:
    """Load the REFERENCE configuration, or the VALIDATED one.

    The three YAML files are the inherited model and are never edited after M2.
    ``validated=True`` applies ``corrections.yaml`` on top, so the difference
    between the two states is exactly that file and nothing else.
    """
    config = {name: yaml.safe_load((CONFIG_DIR / f"{name}.yaml").read_text())
              for name in ("physics", "vehicle_bluerov2_phys", "dock_station")}
    if not validated:
        return config

    corrections = yaml.safe_load((CONFIG_DIR / "corrections.yaml").read_text())
    applied = {name: entry for name, entry in corrections["corrections"].items()
               if entry.get("status") == "applied"}

    if "C1_system_neutral_buoyancy" in applied:
        change = applied["C1_system_neutral_buoyancy"]["change"]
        config["vehicle_bluerov2_phys"]["base_link"]["mass_kg"] = change["to"]
        derived = config["vehicle_bluerov2_phys"]["derived_buoyancy"]
        derived["net_weight_in_water_N"] = 0.0
        derived["note"] = "neutral by construction after correction C1"

    if "C2_dock_collision_geometry" in applied:
        config["dock_station"]["collision"] = {"present": True,
                                               "source": "correction C2"}

    if "C6_validated_timestep" in applied:
        step = applied["C6_validated_timestep"]["change"]["to"]
        config["physics"]["engine"]["max_step_size_s"] = step
        # The profile name is what Gazebo prints on startup; leaving it at
        # "1ms" while running 0.5 ms would misreport the solver in every log.
        config["physics"]["engine"]["profile_name"] = f"{step * 1000:g}ms"

    if "C7_sensor_suite_on_the_dynamic_vehicle" in applied:
        config["vehicle_bluerov2_phys"]["sensors"].update(SENSOR_SUITE)
        # The sensors need their world systems; gz-sim-sensors-system alone
        # renders cameras and lidars but does not populate IMU or DVL.
        config["physics"]["systems"] += [
            {"filename": "gz-sim-imu-system",
             "name": "gz::sim::systems::Imu"},
            {"filename": "gz-sim-dvl-system",
             "name": "gz::sim::systems::DopplerVelocityLogSystem"},
        ]

    if "C4_thruster_visual_geometry" in applied:
        config["vehicle_bluerov2_phys"]["cosmetic_vectored_visuals"] = False

    config["_validated"] = True
    config["_corrections"] = sorted(applied)
    return config


def _dock_collision(dock: dict) -> list[str]:
    """Convex collision primitives mirroring the visual docking geometry."""
    out = []
    funnel = dock["links"]["funnel"]["visuals"]
    out.append(f'{I*3}<link name="funnel_collision">')
    for plate in funnel:
        out += [
            f'{I*4}<collision name="col_{plate["name"]}">',
            f'{I*5}<pose>{_pose(plate["pose"])}</pose>',
            _geometry({"shape": "box", "size": plate["size"]}, I * 5),
            f"{I*4}</collision>",
        ]
    out.append(f"{I*3}</link>")

    # Collar: an annulus approximated by boxes on a ring.
    #
    # The segment pose applies roll = angle about X, so the box's local +y axis
    # maps to the RADIAL direction and local +z to the TANGENTIAL direction.
    # The size vector must therefore be [axial, radial, tangential] =
    # [0.06, thickness, width]. Supplying it as [0.06, width, thickness]
    # transposes the two and emits a throat of inner radius
    #   mid - width/2 = 0.27 - 0.111 = 0.159 m
    # instead of the intended
    #   mid - thickness/2 = 0.27 - 0.03 = 0.240 m,
    # which is narrower than the hull's 0.1749 m half-diagonal and makes centred
    # entry physically impossible. That was the v2.0.0 defect; see correction C2
    # and the P14a-d throat-entry tests that now pin this.
    import math
    thickness = COLLAR_OUTER_R - COLLAR_INNER_R
    mid = 0.5 * (COLLAR_INNER_R + COLLAR_OUTER_R)
    width = 2.0 * math.pi * mid / COLLAR_SEGMENTS
    out.append(f'{I*3}<link name="collar_collision">')
    for index in range(COLLAR_SEGMENTS):
        angle = 2.0 * math.pi * index / COLLAR_SEGMENTS
        y, z = mid * math.cos(angle), mid * math.sin(angle)
        out += [
            f'{I*4}<collision name="col_collar_{index}">',
            f"{I*5}<pose>-0.02 {y:.6g} {z:.6g} {angle:.6g} 0 0</pose>",
            _geometry({"shape": "box",
                       "size": [0.06, thickness, width * 1.05]}, I * 5),
            f"{I*4}</collision>",
        ]
    out.append(f"{I*3}</link>")

    support = {v["name"]: v for v in dock["links"]["support"]["visuals"]}
    post, plate = support["post"], support["baseplate"]
    out += [
        f'{I*3}<link name="support_collision">',
        f'{I*4}<collision name="col_post">',
        f'{I*5}<pose>{_pose(post["pose"])}</pose>',
        _geometry(post, I * 5),
        f"{I*4}</collision>",
        f'{I*4}<collision name="col_baseplate">',
        f'{I*5}<pose>{_pose(plate["pose"])}</pose>',
        _geometry({"shape": "box", "size": plate["size"]}, I * 5),
        f"{I*4}</collision>",
        f"{I*3}</link>",
    ]
    return out


def _pose(values) -> str:
    return " ".join(f"{float(v):g}" for v in values)


def _rgba(values) -> str:
    return " ".join(f"{float(v):g}" for v in values)


def _geometry(spec: dict, indent: str) -> str:
    shape = spec["shape"]
    if shape == "box":
        inner = f"<box><size>{_pose(spec['size'] if 'size' in spec else spec['size_m'])}</size></box>"
    elif shape == "sphere":
        inner = f"<sphere><radius>{spec['radius']:g}</radius></sphere>"
    elif shape == "cylinder":
        inner = (f"<cylinder><radius>{spec['radius']:g}</radius>"
                 f"<length>{spec['length']:g}</length></cylinder>")
    elif shape == "plane":
        inner = (f"<plane><normal>{_pose(spec['normal'])}</normal>"
                 f"<size>{_pose(spec['size'])}</size></plane>")
    else:                                                    # pragma: no cover
        raise ValueError(f"unsupported shape: {shape}")
    return f"{indent}<geometry>{inner}</geometry>"


def _material(indent: str, ambient="0.5 0.5 0.5 1", diffuse="0.6 0.6 0.6 1",
              emissive: str | None = None) -> str:
    parts = [f"<ambient>{ambient}</ambient>", f"<diffuse>{diffuse}</diffuse>"]
    if emissive:
        parts.append(f"<emissive>{emissive}</emissive>")
    return f"{indent}<material>{''.join(parts)}</material>"


# --- world sections ---------------------------------------------------------

def _header(physics: dict, validated: bool = False) -> list[str]:
    engine = physics["engine"]
    name = VALIDATED_WORLD_NAME if validated else WORLD_NAME
    return [
        '<?xml version="1.0" ?>',
        # A double hyphen is illegal inside an XML comment. libsdformat's parser
        # tolerates it and reports the file valid; strict parsers do not, so the
        # separators here are single hyphens throughout.
        "<!--",
        f"  {name}.sdf : GENERATED. Do not edit by hand.",
        "",
        "  Built from config/physics.yaml, config/vehicle_bluerov2_phys.yaml and",
        "  config/dock_station.yaml by uuv_sim_physics.world_builder. Edit the YAML",
        "  and regenerate; test_world_builder.py fails if this file is edited",
        "  directly.",
        "",
        "  Physics parameters are reproduced verbatim from the frozen reference",
        "  src/uuv_adaptive_nav/worlds/underwater_docking_physics.sdf. No value has",
        "  been corrected. Known inconsistencies are recorded in the YAML under",
        "  known_discrepancies and are addressed at M2.5, not here.",
        "-->",
        # The gz: namespace is only needed by the DVL's <gz:dvl> block, which
        # exists solely in the validated world. Declaring it unconditionally
        # would change the REFERENCE world's bytes for no reason.
        ('<sdf version="1.9" xmlns:gz="http://gazebosim.org/schema">'
         if validated else '<sdf version="1.9">'),
        f'{I}<world name="{name}">',
        "",
        f'{I*2}<physics name="{engine["profile_name"]}" type="{engine["type"]}">',
        f'{I*3}<max_step_size>{engine["max_step_size_s"]:g}</max_step_size>',
        f'{I*3}<real_time_factor>{engine["real_time_factor"]:g}</real_time_factor>',
        f"{I*2}</physics>",
        f'{I*2}<gravity>{_pose(physics["gravity_mps2"])}</gravity>',
        "",
    ]


def _systems(physics: dict) -> list[str]:
    out = []
    for system in physics["systems"]:
        head = (f'{I*2}<plugin filename="{system["filename"]}" '
                f'name="{system["name"]}"')
        params = system.get("params")
        if not params:
            out.append(head + "/>")
            continue
        out.append(head + ">")
        for key, value in params.items():
            out.append(f"{I*3}<{key}>{value}</{key}>")
        out.append(f"{I*2}</plugin>")
    return out + [""]


def _scene_and_lights(physics: dict) -> list[str]:
    scene = physics["scene"]
    out = [f"{I*2}<scene>",
           f'{I*3}<ambient>{_rgba(scene["ambient"])}</ambient>',
           f'{I*3}<background>{_rgba(scene["background"])}</background>',
           f'{I*3}<shadows>{str(scene["shadows"]).lower()}</shadows>',
           f"{I*2}</scene>", ""]
    for light in physics["lights"]:
        out += [
            f'{I*2}<light type="{light["type"]}" name="{light["name"]}">',
            f'{I*3}<cast_shadows>{str(light["cast_shadows"]).lower()}</cast_shadows>',
            f'{I*3}<pose>{_pose(light["pose"])}</pose>',
            f'{I*3}<diffuse>{_rgba(light["diffuse"])}</diffuse>',
            f'{I*3}<specular>{_rgba(light["specular"])}</specular>',
            f'{I*3}<direction>{_pose(light["direction"])}</direction>',
            f'{I*3}<intensity>{light["intensity"]:g}</intensity>',
            f"{I*2}</light>", "",
        ]
    return out


def _seabed(physics: dict) -> list[str]:
    bed = physics["seabed"]
    plane = {"shape": "plane", "normal": bed["normal"], "size": bed["plane_size_m"]}
    return [
        f'{I*2}<model name="seabed">',
        f"{I*3}<static>true</static>",
        f'{I*3}<pose>{_pose(bed["pose"])}</pose>',
        f'{I*3}<link name="link">',
        f'{I*4}<collision name="col">',
        _geometry(plane, I * 5),
        f"{I*4}</collision>",
        f'{I*4}<visual name="sand">',
        _geometry(plane, I * 5),
        _material(I * 5, "0.55 0.5 0.38 1", "0.6 0.52 0.4 1"),
        f"{I*4}</visual>",
        f"{I*3}</link>",
        f"{I*2}</model>", "",
    ]


def _rocks(physics: dict) -> list[str]:
    out = []
    for rock in physics["rocks"]:
        spec = dict(rock)
        out += [
            f'{I*2}<model name="{spec["name"]}">',
            f"{I*3}<static>true</static>",
            f'{I*3}<pose>{_pose(spec["pose"])}</pose>',
            f'{I*3}<link name="link">',
            f'{I*4}<collision name="c">',
            _geometry(spec, I * 5),
            f"{I*4}</collision>",
            f'{I*4}<visual name="v">',
            _geometry(spec, I * 5),
            _material(I * 5, "0.30 0.29 0.27 1", "0.38 0.36 0.33 1"),
            f"{I*4}</visual>",
            f"{I*3}</link>",
            f"{I*2}</model>",
        ]
    return out + [""]


def _dock(dock: dict) -> list[str]:
    out = [
        f'{I*2}<!-- Docking station. NOTE: no collision geometry, reproduced from',
        f"{I*2}     the reference. See dock_station.yaml known_discrepancies. -->",
        f'{I*2}<model name="{dock["name"]}">',
        f'{I*3}<static>{str(dock["static"]).lower()}</static>',
        f'{I*3}<pose>{_pose(dock["pose"])}</pose>',
    ]
    palette = {
        "support": ("0.20 0.20 0.22 1", "0.30 0.30 0.32 1", None),
        "collar": ("0.90 0.35 0 1", "1 0.40 0 1", "0.55 0.18 0 1"),
        "funnel": ("0.78 0.66 0.08 1", "0.98 0.85 0.12 1", None),
        "led_markers": ("0.1 1 0.2 1", "0.15 1 0.25 1", "0.1 1 0.2 1"),
        "guidance_lasers": ("0.1 1 0.2 1", "0.1 1 0.2 1", "0.15 1 0.25 1"),
        "beacons": ("0.9 0.9 1 1", "1 1 1 1", "0.8 0.8 1 1"),
    }
    for link_name, link in dock["links"].items():
        ambient, diffuse, emissive = palette[link_name]
        out.append(f'{I*3}<link name="{link_name}">')
        for visual in link["visuals"]:
            spec = dict(visual)
            if link_name == "led_markers":
                spec.update(shape="sphere", radius=link["radius"])
            out += [
                f'{I*4}<visual name="{spec["name"]}">',
                f'{I*5}<pose>{_pose(spec["pose"])}</pose>',
                _geometry(spec, I * 5),
                _material(I * 5, ambient, diffuse, emissive),
                f"{I*4}</visual>",
            ]
        out.append(f"{I*3}</link>")
    if dock.get("collision", {}).get("present"):
        out += _dock_collision(dock)
    return out + [f"{I*2}</model>", ""]


def _extra_sensors(sensors: dict) -> list[str]:
    """FLS, IMU and DVL, emitted only when correction C7 is applied."""
    out = []
    fls = sensors.get("fls")
    if fls:
        out += [
            f'{I*4}<sensor name="fls" type="gpu_lidar">',
            f"{I*5}<always_on>1</always_on>",
            f'{I*5}<update_rate>{fls["update_rate_hz"]:g}</update_rate>',
            f'{I*5}<topic>{fls["topic"]}</topic>',
            f'{I*5}<pose>{_pose(fls["pose"])}</pose>',
            f"{I*5}<lidar><scan><horizontal>",
            f'{I*6}<samples>{fls["samples"]}</samples><resolution>1</resolution>',
            f'{I*6}<min_angle>{fls["min_angle_rad"]:g}</min_angle>'
            f'<max_angle>{fls["max_angle_rad"]:g}</max_angle>',
            f"{I*5}</horizontal><vertical>",
            f"{I*6}<samples>1</samples><resolution>1</resolution>"
            f"<min_angle>0</min_angle><max_angle>0</max_angle>",
            f"{I*5}</vertical></scan>",
            f'{I*5}<range><min>{fls["range_min_m"]:g}</min>'
            f'<max>{fls["range_max_m"]:g}</max>'
            f'<resolution>{fls["range_resolution_m"]:g}</resolution></range>',
            f"{I*5}</lidar>",
            f"{I*4}</sensor>",
        ]
    imu = sensors.get("imu")
    if imu:
        out += [
            f'{I*4}<sensor name="imu" type="imu">',
            f"{I*5}<always_on>1</always_on>",
            f'{I*5}<update_rate>{imu["update_rate_hz"]:g}</update_rate>',
            f'{I*5}<topic>{imu["topic"]}</topic>',
            f'{I*5}<pose>{_pose(imu["pose"])}</pose>',
            f"{I*4}</sensor>",
        ]
    dvl = sensors.get("dvl")
    if dvl:
        beams = "".join(
            f'<beam id="{index + 1}"><aperture>{dvl["beam_aperture_deg"]}</aperture>'
            f"<rotation>{rotation}</rotation>"
            f'<tilt>{dvl["beam_tilt_deg"]}</tilt></beam>'
            for index, rotation in enumerate(dvl["beam_rotations_deg"]))
        out += [
            f'{I*4}<sensor name="dvl" type="custom" gz:type="dvl">',
            f"{I*5}<always_on>1</always_on>",
            f'{I*5}<update_rate>{dvl["update_rate_hz"]:g}</update_rate>',
            f'{I*5}<topic>{dvl["topic"]}</topic>',
            f'{I*5}<pose>{_pose(dvl["pose"])}</pose>',
            f"{I*5}<gz:dvl>",
            f'{I*6}<arrangement degrees="true">{beams}</arrangement>',
            f"{I*6}<tracking><bottom_mode><when>best</when>",
            f'{I*7}<noise type="gaussian">'
            f'<stddev>{dvl["noise_stddev_mps"]:g}</stddev></noise>',
            f"{I*7}<visualize>false</visualize>",
            f"{I*6}</bottom_mode></tracking>",
            f"{I*5}</gz:dvl>",
            f"{I*4}</sensor>",
        ]
    return out


def _vehicle(vehicle: dict) -> list[str]:
    base = vehicle["base_link"]
    inertia = base["inertia_kgm2"]
    hydro = vehicle["hydrodynamics"]
    thrusters = vehicle["thrusters"]
    camera = vehicle["sensors"]["camera"]

    out = [
        f'{I*2}<model name="{vehicle["name"]}">',
        f'{I*3}<pose>{_pose(vehicle["spawn_pose"])}</pose>',
        f'{I*3}<self_collide>{str(vehicle["self_collide"]).lower()}</self_collide>',
        "",
        f'{I*3}<link name="base_link">',
        f"{I*4}<inertial>",
        f'{I*5}<pose>{_pose(list(base["center_of_mass_m"]) + [0, 0, 0])}</pose>',
        f'{I*5}<mass>{base["mass_kg"]:g}</mass>',
        f"{I*5}<inertia>",
        f'{I*6}<ixx>{inertia["ixx"]:g}</ixx><iyy>{inertia["iyy"]:g}</iyy>'
        f'<izz>{inertia["izz"]:g}</izz>',
        f'{I*6}<ixy>{inertia["ixy"]:g}</ixy><ixz>{inertia["ixz"]:g}</ixz>'
        f'<iyz>{inertia["iyz"]:g}</iyz>',
        f"{I*5}</inertia>",
        f"{I*4}</inertial>",
        f'{I*4}<collision name="{base["collision"]["name"]}">',
        f'{I*5}<pose>{_pose(base["collision"]["pose"])}</pose>',
        _geometry({"shape": "box", "size": base["collision"]["size_m"]}, I * 5),
        f"{I*4}</collision>",
        f'{I*4}<visual name="body">',
        _geometry({"shape": "box", "size": base["visual_body"]["size_m"]}, I * 5),
        _material(I * 5, "0.15 0.15 0.75 1", "0.18 0.18 0.85 1"),
        f"{I*4}</visual>",
    ]

    for light in vehicle["headlights"]:
        out += [
            f'{I*4}<light name="{light["name"]}" type="spot">',
            f'{I*5}<pose>{_pose(light["pose"])}</pose>',
            f"{I*5}<cast_shadows>false</cast_shadows>",
            f"{I*5}<diffuse>1 0.96 0.85 1</diffuse>",
            f"{I*5}<specular>0.3 0.3 0.25 1</specular>",
            f'{I*5}<attenuation><range>{light["range_m"]:g}</range><linear>0.15</linear>'
            f"<constant>0.25</constant><quadratic>0.02</quadratic></attenuation>",
            f"{I*5}<direction>1 0 0</direction>",
            f"{I*5}<spot><inner_angle>0.3</inner_angle><outer_angle>0.6</outer_angle>"
            f"<falloff>1</falloff></spot>",
            f"{I*4}</light>",
        ]

    out += [
        f'{I*4}<sensor name="camera" type="camera">',
        f"{I*5}<always_on>1</always_on>",
        f'{I*5}<update_rate>{camera["update_rate_hz"]:g}</update_rate>',
        f"{I*5}<visualize>true</visualize>",
        f'{I*5}<topic>{camera["topic"]}</topic>',
        f'{I*5}<pose>{_pose(camera["pose"])}</pose>',
        f"{I*5}<camera>",
        f'{I*6}<horizontal_fov>{camera["horizontal_fov_rad"]:g}</horizontal_fov>',
        f'{I*6}<image><width>{camera["width"]}</width>'
        f'<height>{camera["height"]}</height>'
        f'<format>{camera["format"]}</format></image>',
        f'{I*6}<clip><near>{camera["near_clip_m"]:g}</near>'
        f'<far>{camera["far_clip_m"]:g}</far></clip>',
        f"{I*5}</camera>",
        f"{I*4}</sensor>",
    ]
    out += _extra_sensors(vehicle["sensors"])
    out += [f"{I*3}</link>", ""]

    inertia_p = thrusters["propeller_inertia_kgm2"]
    for unit in thrusters["units"]:
        out += [
            f'{I*3}<link name="{unit["name"]}">',
            f'{I*4}<pose>{_pose(list(unit["position_m"]) + [0, 0, 0])}</pose>',
            f'{I*4}<inertial><mass>{unit["mass_kg"]:g}</mass><inertia>'
            f"<ixx>{inertia_p:g}</ixx><iyy>{inertia_p:g}</iyy><izz>{inertia_p:g}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>",
            f'{I*4}<visual name="v">',
            _geometry({"shape": "cylinder", "radius": 0.03, "length": 0.05}, I * 5),
            _material(I * 5, "0.9 0.5 0.1 1", "1 0.6 0.1 1"),
            f"{I*4}</visual>",
            f"{I*3}</link>",
            f'{I*3}<joint name="{unit["joint"]}" type="revolute">',
            f"{I*4}<parent>base_link</parent><child>{unit['name']}</child>",
            f'{I*4}<axis><xyz>{_pose(unit["axis"])}</xyz>'
            f"<limit><lower>-1e16</lower><upper>1e16</upper></limit>"
            f'<dynamics><damping>{thrusters["joint_damping"]:g}</damping></dynamics></axis>',
            f"{I*3}</joint>",
        ]

    added, linear, quad = (hydro["added_mass"], hydro["linear_damping"],
                           hydro["quadratic_damping"])
    out += [
        "",
        f'{I*3}<plugin filename="gz-sim-hydrodynamics-system" '
        f'name="gz::sim::systems::Hydrodynamics">',
        f'{I*4}<link_name>{hydro["link_name"]}</link_name>',
    ]
    for group in (added, linear, quad):
        out.append(I * 4 + "".join(f"<{k}>{v:g}</{k}>" for k, v in group.items()))
    out.append(f"{I*3}</plugin>")

    common = thrusters["common"]
    for unit in thrusters["units"]:
        out += [
            f'{I*3}<plugin filename="gz-sim-thruster-system" '
            f'name="gz::sim::systems::Thruster">',
            f'{I*4}<namespace>{common["namespace"]}</namespace>'
            f'<joint_name>{unit["joint"]}</joint_name>',
            f'{I*4}<thrust_coefficient>{common["thrust_coefficient"]:g}</thrust_coefficient>'
            f'<fluid_density>{common["fluid_density_kgm3"]:g}</fluid_density>',
            f'{I*4}<propeller_diameter>{common["propeller_diameter_m"]:g}</propeller_diameter>',
            f'{I*4}<max_thrust_cmd>{common["max_thrust_cmd_N"]:g}</max_thrust_cmd>'
            f'<min_thrust_cmd>{common["min_thrust_cmd_N"]:g}</min_thrust_cmd>',
            f"{I*3}</plugin>",
        ]
    return out + [f"{I*2}</model>", ""]


def build_world_sdf(validated: bool = False) -> str:
    config = load_config(validated)
    physics = config["physics"]
    lines = (_header(physics, validated) + _systems(physics) + _scene_and_lights(physics)
             + _seabed(physics) + _rocks(physics)
             + _dock(config["dock_station"])
             + _vehicle(config["vehicle_bluerov2_phys"])
             + [f"{I}</world>", "</sdf>"])
    return "\n".join(lines) + "\n"


def write_world(path: Path | None = None, validated: bool = False) -> Path:
    target = Path(path) if path else (VALIDATED_WORLD_PATH if validated else WORLD_PATH)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(build_world_sdf(validated))
    return target


def config_digest() -> str:
    """One hash over every configuration file, for run provenance."""
    digest = hashlib.sha256()
    for name in ("physics", "vehicle_bluerov2_phys", "dock_station", "corrections"):
        digest.update((CONFIG_DIR / f"{name}.yaml").read_bytes())
    return digest.hexdigest()


if __name__ == "__main__":
    for flag in (False, True):
        written = write_world(validated=flag)
        label = "VALIDATED" if flag else "REFERENCE"
        print(f"{label:9s} {written.name}  "
              f"sha256 {hashlib.sha256(written.read_bytes()).hexdigest()}")
    print(f"config    sha256 {config_digest()}")
