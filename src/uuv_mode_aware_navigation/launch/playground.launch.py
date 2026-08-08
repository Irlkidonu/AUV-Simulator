"""The interactive environment: fly it yourself, break it, watch it respond.

    ros2 launch uuv_mode_aware_navigation playground.launch.py
    ros2 launch uuv_mode_aware_navigation playground.launch.py scenario:=E7
    ros2 launch uuv_mode_aware_navigation playground.launch.py scenario:=E19 turbidity_c:=1.4

Then, in a second terminal, take the controls:

    ros2 run uuv_mode_aware_navigation teleop

This differs from ``demo.launch.py`` in one respect: it adds a scenario director,
so the run is one of the campaign's nineteen failure families rather than flat
water, and it accepts keyboard control. Everything below the steering is the
same software the campaign runs -- the same propagation model, sensor models,
estimator and mode manager.

Nothing launched here writes a result file, and the campaign does not go through
Gazebo at all, so a session cannot affect a reported number.

Scenario families
-----------------

  E1  nominal                E8  turbid + DVL loss     E15 turbid and noisy
  E2  short DVL loss         E9  current unobservable  E16 featureless plain
  E3  long DVL loss          E10 steady current        E17 terrain recoverable
  E4  graded turbidity       E11 building current      E18 vessel departs
  E5  optical loss           E12 rotating current      E19 unprepared area
  E6  acoustic intermittent  E13 acoustic noise
  E7  compound               E14 noisy + DVL loss
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable,
                            Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uuv_mode_aware_navigation.mission import SurveyMission

#: The <world name> inside the SDF, which is also the file's stem. Gazebo names
#: its services after it, so the two have to agree.
WORLD_NAME = "mode_aware_survey"


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("uuv_mode_aware_navigation"))
    world = share / "worlds" / f"{WORLD_NAME}.sdf"
    models = share / "models"

    spawn = [float(v) for v in SurveyMission().waypoints[0]]

    turbidity = LaunchConfiguration("turbidity_c")
    scenario = LaunchConfiguration("scenario")
    camera = LaunchConfiguration("camera_view")
    hud = LaunchConfiguration("hud")

    gz = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "1", str(world)],
        output="screen",
    )

    return LaunchDescription([
        # Gazebo resolves model:// against this. Without it the rock meshes
        # silently fall back to nothing and the seabed is bare.
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share.parent)),
        # Offscreen camera rendering picks an EGL vendor, and on a machine with
        # both NVIDIA and Mesa installed it can pick Mesa, fail to create a
        # screen, and fall back to software rendering for every camera frame.
        # Naming the NVIDIA vendor removes several hundred warnings a minute and
        # puts the camera back on the GPU. Harmless where NVIDIA is absent: the
        # file simply does not exist and the loader carries on.
        SetEnvironmentVariable(
            "__EGL_VENDOR_LIBRARY_FILENAMES",
            "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),

        DeclareLaunchArgument(
            "scenario", default_value="E1_nominal",
            description="scenario family to replay, e.g. E7 or E7_compound",
        ),
        DeclareLaunchArgument(
            "turbidity_c", default_value="0.2",
            description="starting beam attenuation coefficient c, in m^-1; the "
                        "scenario director overrides this as the run proceeds",
        ),
        DeclareLaunchArgument(
            "camera_view", default_value="false",
            description="open the degraded camera feed in a SEPARATE window; "
                        "off by default because the Gazebo window now shows it "
                        "docked beside the 3D view",
        ),
        DeclareLaunchArgument(
            "hud", default_value="false",
            description="also run the terminal status display; the graphical "
                        "panel shows the same state and is on by default",
        ),
        DeclareLaunchArgument(
            "panel", default_value="true",
            description="open the graphical control panel",
        ),

        # DART is the declared physics engine (worlds/mode_aware_survey.sdf,
        # <physics type="dart">), but it does not move the vehicle: the vehicle
        # node writes the model's pose from the true state it integrates. The
        # note on the bluerov2 model in the world file records why, and what
        # commanding velocity got wrong. Scenery still moves under the physics
        # engine, which is what the fish and weed plugins below are for.
        gz,

        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="gz_bridge", output="screen",
            arguments=[
                "/paper2/camera@sensor_msgs/msg/Image[gz.msgs.Image",
                "/model/bluerov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                # The vehicle's pose, written by the vehicle node each step.
                # Gazebo offers no topic for placing a model, only this service.
                f"/world/{WORLD_NAME}/set_pose"
                "@ros_gz_interfaces/srv/SetEntityPose",
                # Degraded camera frame back out to gz, so the Gazebo window can
                # show it docked beside the 3D view instead of opening a second
                # window for it.
                "/uuv/camera_degraded@sensor_msgs/msg/Image]gz.msgs.Image",
            ] + [
                # One command topic per fish, ROS to Gazebo. The school node
                # decides what each should do; Gazebo moves it.
                f"/model/fish_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
                for i in range(10)
            ] + [
                f"/model/jelly_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
                for i in range(12)
            ] + [
                f"/model/weed_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
                for i in range(24)
            ] + [
            ],
            remappings=[("/paper2/camera", "/camera/image_raw")],
        ),

        Node(
            package="uuv_mode_aware_navigation", executable="water_column",
            name="water_column", output="screen",
            parameters=[{"turbidity_c": turbidity}],
        ),
        Node(
            package="uuv_mode_aware_navigation", executable="optical_feedback",
            name="optical_feedback", output="screen",
            parameters=[{"model_path": str(models / "optical_feedback.json")}],
        ),
        Node(
            package="uuv_mode_aware_navigation", executable="mode_manager",
            name="mode_manager", output="screen",
            parameters=[{
                "availability_model_path": str(models / "availability.json"),
                "decision_period_s": 0.5,
            }],
        ),
        Node(
            package="uuv_mode_aware_navigation", executable="vehicle",
            name="vehicle", output="screen",
            parameters=[{
                "turbidity_c": turbidity,
                "spawn_position_m": spawn,
            }],
        ),
        Node(
            package="uuv_mode_aware_navigation", executable="fish_school",
            name="fish_school", output="screen",
            parameters=[{"count": 10}],
        ),
        Node(
            package="uuv_mode_aware_navigation", executable="scenario_director",
            name="scenario_director", output="screen",
            parameters=[{"scenario": scenario}],
        ),
        Node(
            package="uuv_mode_aware_navigation", executable="status_display",
            name="status_display", output="screen",
            condition=IfCondition(hud),
        ),

        Node(
            package="uuv_mode_aware_navigation", executable="control_panel",
            name="control_panel", output="screen",
            condition=IfCondition(LaunchConfiguration("panel")),
        ),

        # Closing Gazebo ends the session. Without this the image viewer and the
        # nodes outlive the simulator they were watching, leaving a frozen
        # window and a set of processes that have to be found and killed by
        # hand.
        RegisterEventHandler(
            OnProcessExit(target_action=gz, on_exit=[Shutdown(
                reason="Gazebo closed")])
        ),

        # The vehicle's own view after the propagation model has degraded it,
        # rather than the clean render -- which is the point of showing it. This
        # is the display-encoded copy; the estimator reads the radiometric one
        # (/uuv/camera_radiance), both produced from the same degraded frame.
        Node(
            package="image_tools", executable="showimage",
            name="camera_view", output="log",
            remappings=[("image", "/uuv/camera_degraded")],
            condition=IfCondition(camera),
        ),
    ])
