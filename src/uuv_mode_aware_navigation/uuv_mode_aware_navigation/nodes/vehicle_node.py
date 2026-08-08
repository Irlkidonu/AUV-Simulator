#!/usr/bin/env python3
"""Drives the survey and publishes the observables the manager consumes.

Runs the same estimator, guidance and mission classes as the headless campaign.
Gazebo supplies the vehicle's true motion and the camera view; this node closes
the loop by
  * feeding sensor readings into :class:`NavigationFilter`,
  * asking :class:`Guidance` -- using **only** the estimate -- where to go next,
  * applying the manager's commanded altitude and speed,
  * publishing the observables and a truth-side error for the display.

Rule N1 is preserved: the commanded velocity is computed from
``estimator.position``. Ground truth arrives from Gazebo odometry and is used
for scoring and for generating sensor measurements, never for control.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, Float32, String

from ..estimator import FilterConfig, NavigationFilter
from ..mission import Guidance, SurveyMission, Vehicle
from ..optics import CONFIGURATIONS, WaterState
from ..seabed import depth_at as seabed_depth_at
from ..sensors import (
    BeaconGeometry, FaultKind, FaultSchedule, FaultWindow, SensorNoise,
    SensorSuite,
)


class VehicleNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle")
        self.declare_parameter("turbidity_c", 0.2)
        self.declare_parameter("seed", 20_000_700)
        self.declare_parameter("rate_hz", 10.0)
        #: True ocean current, world frame (m/s). Settable live:
        #: ``ros2 param set /vehicle current_mps "[0.2, -0.1, 0.0]"``.
        #: This is evaluator-side truth. It reaches the sensor layer and nothing
        #: else; the vehicle must infer the flow from the two DVL modes.
        self.declare_parameter("current_mps", [0.05, -0.02, 0.0])

        self._mission = SurveyMission()
        self._guidance = Guidance(self._mission)
        self._estimator = NavigationFilter(
            FilterConfig(), initial_position=self._mission.waypoints[0]
        )
        self._sensors = SensorSuite(
            schedule=FaultSchedule(),
            noise=SensorNoise(),
            beacon=BeaconGeometry(),
            seed=int(self.get_parameter("seed").value),
        )
        self._channels = {c.name: c for c in CONFIGURATIONS}

        # Where the world file spawns the vehicle. Gazebo's odometry is relative
        # to this pose, so it is the offset that turns odometry into world
        # coordinates. Declared as a parameter rather than hard-coded because it
        # is a property of the world file, and a world that moves the vehicle
        # without updating this would reintroduce exactly the silent offset this
        # exists to remove.
        self.declare_parameter("spawn_position_m", [-10.0, -9.0, -17.0])
        self._spawn_position = np.array(
            self.get_parameter("spawn_position_m").value, dtype=float
        )

        #: The demonstrator integrates its own motion with the same kinematics
        #: the campaign uses, and Gazebo renders the result.
        #:
        #: It previously took truth from Gazebo odometry and drove the model
        #: through the VelocityControl plugin. That plugin does not move this
        #: vehicle vertically: commanding {x: 0.2, z: 0.3} over gz transport
        #: yields x = 0.19999999999953 and z = 0.0 exactly, and with buoyancy
        #: disabled the vehicle does not even sink, so the vertical degree of
        #: freedom is held at zero rather than merely unresponsive. Altitude is
        #: the strongest action the manager has, so a demonstrator wired that way
        #: cannot show the method working at all.
        #:
        #: Owning the motion here also removes a whole class of boundary defect
        #: -- frame conventions, spawn offsets, differentiation intervals -- that
        #: produced three separate bugs while this node was being brought up, and
        #: makes the demonstrator show the same dynamics the reported numbers
        #: come from rather than a second, differently-wrong vehicle.
        self._vehicle = Vehicle(
            self._mission.waypoints[0],
            current_mps=[float(v) for v in self.get_parameter("current_mps").value],
        )
        self._truth = self._vehicle.position.copy()
        self._velocity = np.zeros(3)
        self._acceleration = np.zeros(3)
        #: Stamp of the previous odometry message, for differentiating velocity
        #: against the interval that actually elapsed between samples.
        self._last_odom_time: float | None = None
        self._t = 0.0

        # Manager commands.
        self._commanded_altitude = 3.0
        self._commanded_speed = 0.5
        self._channel = "camera_offaxis"
        self._action = "continue"

        self.create_subscription(
            Odometry, "/model/bluerov2/odometry", self._on_odometry, 10
        )
        self.create_subscription(
            Float32, "/uuv/commanded_altitude", self._on_altitude_cmd, 10
        )
        self.create_subscription(
            Float32, "/uuv/commanded_speed", self._on_speed_cmd, 10
        )
        self.create_subscription(
            String, "/uuv/optical_channel", self._on_channel, 10
        )
        self.create_subscription(
            String, "/uuv/mission_action", self._on_action, 10
        )

        # Gazebo is told where the vehicle IS, not how fast to move it. See
        # :meth:`_render_pose`. The service is bridged from gz transport by the
        # launch file; the world name has to match the one in the SDF.
        self._model_name = str(self.declare_parameter(
            "gz_model_name", "bluerov2").value)
        self._set_pose = self.create_client(
            SetEntityPose,
            f"/world/{self.declare_parameter('gz_world', 'mode_aware_survey').value}"
            "/set_pose")
        self._pose_pending = None

        # --- interactive control -------------------------------------------
        # None of this exists in the campaign. The headless runner constructs
        # Vehicle and Guidance directly and never subscribes to a topic, so an
        # interactive session cannot alter a reported number. What the two share
        # is everything beneath the steering: the same sensor models, the same
        # estimator, the same manager.
        self._manual = False
        self._teleop = np.zeros(3)
        self._heading = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._body_rates = np.zeros(3)
        self._surge_cmd = 0.0
        self._sway_cmd = 0.0
        self._heave_cmd = 0.0
        self._last_attitude_t: float | None = None
        self._forced_channel = None
        # "no_map" is not a sensor fault: it removes the prior bathymetry that
        # terrain-relative navigation matches against, which is a property of
        # the deployment rather than of any instrument.
        self._faults = {"dvl": False, "acoustic": False, "vessel_gone": False,
                        "optical": False, "no_map": False}
        self.create_subscription(Twist, "/uuv/teleop_cmd", self._on_teleop, 10)
        self.create_subscription(
            String, "/uuv/control_mode",
            lambda m: setattr(self, "_manual", m.data == "manual"), 10)
        self.create_subscription(
            String, "/uuv/force_channel",
            lambda m: setattr(self, "_forced_channel", m.data or None), 10)
        self.create_subscription(
            Float32, "/uuv/set_turbidity",
            lambda m: self.set_parameters([
                Parameter("turbidity_c", Parameter.Type.DOUBLE, float(m.data))
            ]), 10)
        self.create_subscription(String, "/uuv/inject_fault", self._on_fault, 10)
        self.create_subscription(Bool, "/uuv/reset", self._on_reset, 10)
        self._attitude = self.create_publisher(Vector3, "/uuv/attitude_rpy", 10)
        # Ocean current, settable while running. The campaign drives this from
        # the scenario's current profile; here a driver can turn it up and feel
        # the vehicle crab off the survey line.
        self.create_subscription(Vector3, "/uuv/set_current", self._on_current, 10)
        self.create_subscription(
            Bool, "/uuv/level_attitude", self._on_level, 10)
        self._current_speed = self.create_publisher(
            Float32, "/uuv/current_speed", 10)
        # Truth, for scenery that has to react to where the vehicle is. This is
        # evaluator-side and demonstrator-only: no decision-making component
        # subscribes to it, and the campaign publishes nothing at all.
        self._true_pos = self.create_publisher(Vector3, "/uuv/true_position", 10)
        self._altitude_pub = self.create_publisher(Float32, "/uuv/altitude", 10)
        self._turbidity_pub = self.create_publisher(Float32, "/uuv/turbidity_c", 10)
        self._lock_pub = self.create_publisher(Bool, "/uuv/dvl_bottom_lock", 10)
        self._optical_pub = self.create_publisher(Bool, "/uuv/optical_available", 10)
        self._acoustic_pub = self.create_publisher(
            Float32, "/uuv/acoustic_fix_age", 10
        )
        self._cov_pub = self.create_publisher(
            Float32, "/uuv/position_covariance_trace", 10
        )
        # Truth-side, for the display only. Never consumed by a decision.
        self._error_pub = self.create_publisher(Float32, "/uuv/position_error", 10)
        #: Estimated current speed and how well it is known. Both are observables
        #: the mode manager may use; both come from the filter, not the parameter.
        self._current_pub = self.create_publisher(Float32, "/uuv/current_speed", 10)
        self._current_cov_pub = self.create_publisher(
            Float32, "/uuv/current_covariance", 10
        )
        self._water_track_pub = self.create_publisher(
            Bool, "/uuv/dvl_water_track", 10
        )
        #: Truth-side, for the display only: how far the estimated flow is from
        #: the commanded one. Never reaches a decision.
        self._current_error_pub = self.create_publisher(
            Float32, "/uuv/current_error", 10
        )
        self._waypoint_pub = self.create_publisher(Float32, "/uuv/waypoint_index", 10)

        self._acoustic_age = 0.0
        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._step)

        # The picture is refreshed far more often than the state is stepped.
        #
        # Stepping runs at rate_hz, which is the rate the estimator and the
        # manager are specified at and must not be changed to suit the display.
        # Writing the model's pose only that often, though, moves it in visible
        # jumps: Gazebo draws at screen rate and simply holds the last pose in
        # between, so ten steps a second look like ten frames a second however
        # fast the renderer is running. The velocity plugin this replaced hid
        # that by interpolating at the physics rate, which is the one thing it
        # did well.
        #
        # So the pose is written at render_hz from the last stepped state
        # carried forward along the velocity and body rates it was stepped with.
        # That is display interpolation and nothing more: it is never fed back,
        # no sensor or estimator reads it, and the truth remains whatever _step
        # integrated. See :meth:`_render_pose`.
        self.declare_parameter("render_hz", 50.0)
        render_hz = float(self.get_parameter("render_hz").value)
        self._render_dt = 1.0 / render_hz
        self._truth_stamp: float | None = None
        self.create_timer(self._render_dt, self._render_pose)
        self.get_logger().info("vehicle active")

    # -- inputs ---------------------------------------------------------------
    def _on_teleop(self, msg: Twist) -> None:
        """Record the commanded body rates and surge. Integration happens in
        :meth:`_step`.

        Attitude used to be integrated here, on message arrival, against the
        node's own timer period. That is wrong twice over: the sender's rate is
        not the node's rate, so a panel publishing at 20 Hz against a 10 Hz dt
        turned every commanded yaw into twice the rotation; and the heading then
        depended on how fast commands happened to arrive rather than on how long
        the operator held the key. Holding two keys for a few seconds was enough
        to leave the vehicle pointing somewhere the driver had not asked for,
        with forward no longer forward.

        Attitude does not redirect translation. Surge and sway are square to the
        heading and heave is straight up, whatever the vehicle is tilted to, so
        pitching the nose down does not turn forward travel into a descent. That
        is a deliberate choice rather than an omission: a driver who tilts to
        look at something on the seabed should not find that every subsequent
        press of "forward" has quietly become a dive. Depth is on its own two
        keys, and nothing else touches it.

        What attitude does do is aim: the camera and the lamps are mounted on
        the body, so pitch and roll decide what the optical channel is looking
        at, and the rendered model tilts with them.

        A caveat worth stating: the sensor models in this study are functions of
        altitude, water state and configuration, not of attitude. Tilting the
        vehicle therefore does not by itself change what the Doppler log or the
        camera can do. Flying down until the altitude closes does.
        """
        self._surge_cmd = float(msg.linear.x)
        self._sway_cmd = float(msg.linear.y)
        self._heave_cmd = float(msg.linear.z)
        self._body_rates = np.array(
            [float(msg.angular.x), float(msg.angular.y), float(msg.angular.z)],
            dtype=float,
        )

    def _integrate_attitude(self) -> None:
        """Advance attitude by the held rates over one control period."""
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self._last_attitude_t if self._last_attitude_t else self._dt
        self._last_attitude_t = now
        # A stall or a resumed pause can hand back a large interval; one step
        # should never rotate the vehicle by more than a few control periods.
        dt = float(min(max(dt, 0.0), 4.0 * self._dt))
        self._roll += float(self._body_rates[0]) * dt
        self._pitch += float(self._body_rates[1]) * dt
        self._heading += float(self._body_rates[2]) * dt
        # Pitch is clamped rather than wrapped: past vertical the surge axis
        # would invert and the controls would read as reversed.
        self._pitch = float(np.clip(self._pitch, -1.4, 1.4))
        self._roll = (self._roll + np.pi) % (2.0 * np.pi) - np.pi
        self._heading = (self._heading + np.pi) % (2.0 * np.pi) - np.pi
        # Quadcopter mapping, in the vehicle's own frame.
        #
        #   surge   forward and back along the nose
        #   sway    left and right, square to the nose, WITHOUT turning
        #   heave   up and down
        #   yaw     turn on the spot
        #
        # Translation and rotation are separate commands, which is the whole
        # point: an earlier version put yaw on the left and right arrows, so
        # every attempt to sidestep turned the vehicle instead and "forward"
        # quietly became a different direction. A hovering vehicle with vectored
        # thrusters does not work that way and neither does a drone.
        #
        # Left of a heading t is (t + 90 deg), so sway contributes
        # (-sin t, cos t) and surge contributes (cos t, sin t).
        surge = self._surge_cmd
        sway = self._sway_cmd
        self._teleop = np.array([
            surge * np.cos(self._heading) - sway * np.sin(self._heading),
            surge * np.sin(self._heading) + sway * np.cos(self._heading),
            self._heave_cmd,
        ], dtype=float)

    def _render_pose(self) -> None:
        """Place the Gazebo model exactly where the vehicle actually is.

        Gazebo renders this study; it does not simulate it. The true state is
        integrated in :meth:`_step` with the same kinematics the campaign uses,
        and the model on screen is a picture of that state.

        It used to be driven by commanded velocity through VelocityControl,
        which made Gazebo a SECOND integrator running beside this one. That
        cannot work here, for three separate reasons, all measured:

        1. The two integrate in different time bases. Nothing bridges /clock and
           use_sim_time is never set, so this node advances on the wall clock
           while Gazebo advances on the sim clock -- and the sim clock was
           running at a real-time factor of 0.14 to 0.26 on the machine this was
           found on. The rendered vehicle therefore covered roughly a fifth of
           the true distance, and the gap grew for as long as the session ran.
           Heading was the damaging part: it drifted without bound, so the model
           pointed one way while the heading that decides where the arrow keys
           push had gone somewhere else. Flying by what is on screen then puts
           the vehicle anywhere but where the driver asked, which is exactly the
           fault this replaces.

        2. VelocityControl reads its linear vector in the MODEL'S frame, and the
           velocity here is in the world frame. Below a heading of zero the two
           agree, so this hid until the vehicle turned.

        3. VelocityControl never moved this model vertically at all -- measured,
           and recorded against the model in the world file. Altitude is the
           strongest action the manager has, so the one action most worth
           demonstrating was the one that could not be seen.

        Writing the pose removes all three at once: there is no second
        integrator to disagree, a pose is unambiguously world-framed, and every
        axis is set rather than commanded. The cost is that the model no longer
        collides with anything, which is right for a kinematic study whose
        vehicle model has no contact dynamics to begin with.

        The call is asynchronous and at most one is in flight. A dropped frame
        costs a stale picture for one step; blocking the timer would stall the
        estimator, which is the thing that must not miss a step.
        """
        if not self._set_pose.service_is_ready():
            return
        if self._pose_pending is not None and not self._pose_pending.done():
            return

        # Carry the last stepped state forward to now. The horizon is capped at
        # one step: beyond that the state is about to be replaced anyway, and an
        # uncapped extrapolation would let the picture run away from the truth
        # if a step were ever missed.
        lead = 0.0
        if self._truth_stamp is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            lead = float(min(max(now - self._truth_stamp, 0.0), self._dt))
        position = self._truth + self._velocity * lead
        roll = self._roll + float(self._body_rates[0]) * lead
        pitch = self._pitch + float(self._body_rates[1]) * lead
        heading = self._heading + float(self._body_rates[2]) * lead

        req = SetEntityPose.Request()
        req.entity.name = self._model_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = float(position[0])
        req.pose.position.y = float(position[1])
        req.pose.position.z = float(position[2])
        (req.pose.orientation.x, req.pose.orientation.y,
         req.pose.orientation.z, req.pose.orientation.w) = self._quaternion(
            roll, pitch, heading)
        self._pose_pending = self._set_pose.call_async(req)

    @staticmethod
    def _quaternion(roll: float, pitch: float, heading: float) -> tuple:
        """An attitude as (x, y, z, w), Z-Y-X like the SDF pose it sets."""
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cy, sy = np.cos(heading * 0.5), np.sin(heading * 0.5)
        return (
            float(sr * cp * cy - cr * sp * sy),
            float(cr * sp * cy + sr * cp * sy),
            float(cr * cp * sy - sr * sp * cy),
            float(cr * cp * cy + sr * sp * sy),
        )

    #: Keyboard fault names to the injectable kinds they stand for. "dvl" breaks
    #: both Doppler modes together, because a driver asking for "no DVL" means
    #: no velocity aiding of any kind, not the loss of one mode with the other
    #: still answering.
    _FAULT_KINDS = {
        "dvl": (FaultKind.DVL_BOTTOM_LOCK_LOSS, FaultKind.DVL_WATER_TRACK_LOSS),
        "acoustic": (FaultKind.ACOUSTIC_OUTAGE,),
        "vessel_gone": (FaultKind.SURFACE_ASSET_LOSS,),
        "optical": (FaultKind.OPTICAL_BLACKOUT,),
    }

    def _on_level(self, msg: Bool) -> None:
        """Bring roll and pitch back to zero, leaving heading and position be.

        Six axes make it easy to end up tilted at an angle that is hard to undo
        by eye, and there is no restoring moment here to do it for you: the
        vehicle holds whatever attitude it was left in.
        """
        if msg.data:
            self._roll = 0.0
            self._pitch = 0.0

    def _on_current(self, msg: Vector3) -> None:
        self.set_parameters([Parameter(
            "current_mps", Parameter.Type.DOUBLE_ARRAY,
            [float(msg.x), float(msg.y), float(msg.z)])])
        # Rebuild the vehicle's flow field. Vehicle holds the current it was
        # constructed with, so setting the parameter alone would change what the
        # estimator is told and not what the water is doing.
        self._vehicle.current = np.array(
            [float(msg.x), float(msg.y), float(msg.z)], dtype=float)

    def _on_fault(self, msg: String) -> None:
        name, _, state = msg.data.partition(":")
        if name not in self._faults:
            return
        self._faults[name] = state == "on"
        self._apply_faults()
        self.get_logger().info(
            f"{name} -> {'failed' if self._faults[name] else 'healthy'}"
        )

    def _apply_faults(self) -> None:
        """Rebuild the sensor suite's schedule from the live toggles.

        A held-down fault is expressed as a window opening at t=0 and running
        far past any session, which is what the sensor layer already understands.
        The suite's schedule is a plain attribute, so this swaps it in place: no
        sensor, estimator or manager code is aware that a human rather than a
        scenario decided when the instrument stopped answering.
        """
        windows = []
        for name, active in self._faults.items():
            if not active:
                continue
            for kind in self._FAULT_KINDS.get(name, ()):
                windows.append(FaultWindow(kind=kind, start_s=0.0,
                                           duration_s=1.0e9))
        self._sensors.schedule = FaultSchedule(windows=tuple(windows))

    def _on_reset(self, msg: Bool) -> None:
        if not msg.data:
            return
        start = np.asarray(self._mission.waypoints[0], dtype=float)
        self._vehicle = Vehicle(
            start,
            current_mps=[float(v) for v in self.get_parameter("current_mps").value],
        )
        self._truth = self._vehicle.position.copy()
        self._velocity = np.zeros(3)
        self._acceleration = np.zeros(3)
        self._estimator = NavigationFilter(FilterConfig(), initial_position=start)
        self._guidance = Guidance(self._mission)
        self._heading = self._pitch = self._roll = 0.0
        self._body_rates = np.zeros(3)
        self.get_logger().info("reset to the first waypoint")

    def _on_odometry(self, msg: Odometry) -> None:
        """Ignored for truth. Retained so the topic is still consumed.

        Gazebo renders this demonstrator; it does not simulate it. Truth comes
        from :class:`Vehicle`, integrated here with the same kinematics the
        reported campaign uses, because the velocity-control plugin cannot move
        the vehicle vertically and altitude is the manager's strongest action.

        Kept as a subscription rather than deleted so that a future world which
        does simulate the vehicle faithfully -- thrusters and hydrodynamics
        rather than commanded velocity -- has an obvious place to reconnect.
        """
        return

    def _on_altitude_cmd(self, m: Float32) -> None:
        self._commanded_altitude = float(m.data)

    def _on_speed_cmd(self, m: Float32) -> None:
        self._commanded_speed = float(m.data)

    def _on_channel(self, m: String) -> None:
        self._channel = str(m.data)

    def _on_action(self, m: String) -> None:
        self._action = str(m.data)

    # -- one control cycle ----------------------------------------------------
    def _step(self) -> None:
        turbidity = float(self.get_parameter("turbidity_c").value)
        water = WaterState(c=turbidity)
        # Altitude above the seabed actually under the vehicle, not above a
        # nominal plane. The demonstrator's floor has relief, and an altimeter
        # reading a constant would drive optical availability and terrain
        # matching from a surface that is not the one on screen.
        altitude = float(self._truth[2] - seabed_depth_at(
            self._truth[0], self._truth[1]))
        # A channel forced from the keyboard overrides the manager's choice, so
        # a driver can hold the coaxial camera in water where the manager would
        # have moved off it and watch the fix rate collapse.
        selected = self._forced_channel or self._channel
        config = self._channels.get(selected, CONFIGURATIONS[1])

        current = np.array(
            [float(v) for v in self.get_parameter("current_mps").value], dtype=float
        )

        reading = self._sensors.sample(
            t=self._t,
            true_position=self._truth,
            true_velocity=self._velocity,
            true_accel=self._acceleration,
            altitude_m=max(altitude, 0.05),
            water=water,
            config=config,
            true_current_mps=current,
        )

        # --- estimation, exactly as in the campaign ---
        self._estimator.predict(reading.accel_mps2, self._dt)
        if reading.dvl_velocity_mps is not None:
            self._estimator.update_velocity(reading.dvl_velocity_mps)
        else:
            self._estimator.note_aiding_outage()
        if reading.dvl_water_velocity_mps is not None:
            self._estimator.update_water_velocity(reading.dvl_water_velocity_mps)
        if reading.depth_m is not None:
            self._estimator.update_depth(reading.depth_m)
        if reading.optical_position_m is not None:
            self._estimator.update_position(
                reading.optical_position_m, reading.optical_sigma_m
            )
        if reading.acoustic_range_m is not None:
            self._estimator.update_acoustic_range(
                reading.acoustic_range_m, self._sensors.beacon.position
            )
            self._acoustic_age = 0.0
        else:
            self._acoustic_age += self._dt

        # --- guidance: estimate only (rule N1) ---
        # The current fed forward is the filter's estimate, never the parameter
        # above. The parameter is the true flow and reaches the sensor layer
        # only, exactly as the scenario's current does in the campaign.
        command = self._guidance.command(
            self._estimator.position, self._commanded_speed,
            self._commanded_altitude,
            current_estimate_mps=self._estimator.current,
        )
        if self._action != "continue":
            command = np.zeros(3)
        if self._manual:
            self._integrate_attitude()
            # Hand control. The estimator, the sensor models and the manager
            # all keep running and keep reporting, which is the point: a driver
            # can fly into turbid water and watch the mode escalate and the
            # covariance grow while the vehicle does not correct itself.
            command = self._teleop.copy()

        # Advance the true state here, with the same kinematics the campaign
        # uses. The command is still published so Gazebo animates the model and
        # the camera view is genuinely re-rendered as the vehicle moves, but the
        # rendered pose is a consequence of this integration rather than the
        # source of it.
        self._acceleration = self._vehicle.step(command, self._dt)
        self._truth = self._vehicle.position.copy()
        self._velocity = self._vehicle.velocity.copy()

        # Stamp the state so the render timer knows how far to carry it
        # forward. It writes the pose; this only produces it.
        self._truth_stamp = self.get_clock().now().nanoseconds * 1e-9

        self._current_speed.publish(Float32(
            data=float(np.linalg.norm(current))))

        truth = Vector3()
        truth.x, truth.y, truth.z = (float(v) for v in self._truth)
        self._true_pos.publish(truth)

        rpy = Vector3()
        rpy.x, rpy.y, rpy.z = (float(self._roll), float(self._pitch),
                               float(self._heading))
        self._attitude.publish(rpy)

        # --- observables ---
        self._altitude_pub.publish(Float32(data=float(max(altitude, 0.05))))
        self._turbidity_pub.publish(Float32(data=turbidity))
        self._lock_pub.publish(Bool(data=bool(reading.dvl_bottom_lock)))
        self._optical_pub.publish(
            Bool(data=reading.optical_position_m is not None)
        )
        self._acoustic_pub.publish(Float32(data=float(self._acoustic_age)))
        self._cov_pub.publish(
            Float32(data=float(self._estimator.position_covariance_trace))
        )
        self._water_track_pub.publish(
            Bool(data=reading.dvl_water_velocity_mps is not None)
        )
        self._current_pub.publish(
            Float32(data=float(np.linalg.norm(self._estimator.current)))
        )
        self._current_cov_pub.publish(
            Float32(data=float(self._estimator.current_covariance_trace))
        )
        self._current_error_pub.publish(
            Float32(data=float(np.linalg.norm(current - self._estimator.current)))
        )
        # Truth-side diagnostics for the display.
        self._error_pub.publish(
            Float32(data=float(
                np.linalg.norm(self._truth - self._estimator.position)
            ))
        )
        self._waypoint_pub.publish(Float32(data=float(self._guidance.index)))
        self._t += self._dt


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
