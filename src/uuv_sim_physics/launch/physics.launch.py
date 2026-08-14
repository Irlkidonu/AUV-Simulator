"""Launch the physics world on the pinned Gazebo stack.

The pin is enforced *before* the simulator starts: ``toolchain.verify()`` runs
inside an ``OpaqueFunction``, so a mismatched stack aborts the launch rather
than producing a run that looks fine and used the wrong solver. The executable
is addressed by absolute path and the environment is set explicitly -- ``gz``
from PATH is never consulted.

    ros2 launch uuv_sim_physics physics.launch.py
    ros2 launch uuv_sim_physics physics.launch.py gui:=true
    ros2 launch uuv_sim_physics physics.launch.py iterations:=2000
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from uuv_sim_physics import toolchain, world_builder


def _launch_setup(context, *args, **kwargs):
    # Fails closed. A ToolchainError here stops the launch before Gazebo runs.
    stack = toolchain.verify()

    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true")
    iterations = LaunchConfiguration("iterations").perform(context).strip()
    world = Path(LaunchConfiguration("world").perform(context))

    if not world.is_file():
        raise FileNotFoundError(f"world not found: {world}")

    print(f"[uuv_sim_physics] gz-sim {stack['gz_sim_version']} "
          f"({stack['gz_executable']})")
    print(f"[uuv_sim_physics] DART {stack['dart_version']}, world {world.name}")

    command = [str(toolchain.GZ_EXECUTABLE), "sim", "-r", "-v", "3"]
    if not gui:
        command.append("-s")
    if iterations:
        command += ["--iterations", iterations]
    command.append(str(world))

    return [ExecuteProcess(cmd=command, output="screen",
                           additional_env=toolchain.environment())]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="run the Gazebo GUI as well as the server"),
        DeclareLaunchArgument("iterations", default_value="",
                              description="stop after N steps; empty runs until closed"),
        DeclareLaunchArgument("world", default_value=str(world_builder.WORLD_PATH),
                              description="world SDF to load"),
        OpaqueFunction(function=_launch_setup),
    ])
