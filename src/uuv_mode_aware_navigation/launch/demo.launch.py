"""One-command demonstration: Gazebo, the water column, and the manager.

    ros2 launch uuv_mode_aware_navigation demo.launch.py

What runs, and why each piece is there:

  gz sim               renders the survey area and moves the vehicle
  ros_gz_bridge        camera frames and odometry into ROS, velocity out
  water_column         degrades the rendered frame through the propagation model
  optical_feedback     estimates water condition from that frame, pixels only
  mode_manager         the method itself, unchanged from the campaign
  vehicle              estimator + guidance; commands the vehicle from the estimate
  status_display       live view of the mode, the evidence, and the decision

Change the water and watch the system respond:

    ros2 param set /vehicle turbidity_c 1.6
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable,
                            Shutdown)
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

    # Three places have to agree about where the vehicle starts: the SDF pose,
    # the vehicle node's estimator, and the mission's first waypoint. When they
    # did not, the demonstrator reported the disagreement as position error for
    # the whole run and looked like an estimator defect. The mission is the
    # authority -- the waypoints define the survey -- so it is read here and
    # pushed into the node, rather than each end carrying its own literal.
    spawn = [float(v) for v in SurveyMission().waypoints[0]]

    turbidity = LaunchConfiguration("turbidity_c")
    headless = LaunchConfiguration("headless")

    gz = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "1", str(world)],
        output="screen",
    )

    return LaunchDescription([
        # Gazebo resolves model:// against this. Without it every mesh in the
        # world fails to load and the server exits before the scene appears,
        # which is what the documented command used to do.
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share.parent)),
        # Offscreen camera rendering can pick the Mesa EGL vendor on a machine
        # that also has NVIDIA, fail, and fall back to software for every frame.
        # Naming the NVIDIA vendor is harmless where it is absent.
        SetEnvironmentVariable(
            "__EGL_VENDOR_LIBRARY_FILENAMES",
            "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),

        DeclareLaunchArgument(
            "turbidity_c", default_value="0.2",
            description="beam attenuation coefficient c, in m^-1",
        ),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="run Gazebo without its GUI",
        ),

        gz,

        # Camera and odometry in; the vehicle's pose out.
        #
        # The pose is a service rather than a topic because that is the only way
        # Gazebo offers to place a model. The vehicle node owns the true state
        # and writes it straight onto the model each step; see the note on the
        # bluerov2 model in the world file for why it is not commanded velocity.
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="gz_bridge", output="screen",
            arguments=[
                "/paper2/camera@sensor_msgs/msg/Image[gz.msgs.Image",
                "/model/bluerov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                f"/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
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
            package="uuv_mode_aware_navigation", executable="status_display",
            name="status_display", output="screen",
        ),

        # Closing Gazebo ends the session rather than leaving orphaned nodes.
        RegisterEventHandler(
            OnProcessExit(target_action=gz,
                          on_exit=[Shutdown(reason="Gazebo closed")])
        ),
    ])
