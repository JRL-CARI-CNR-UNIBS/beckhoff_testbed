# Copyright 2025 CARI-JRL
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Import launch system core modules
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,       # Used to declare launch arguments (like command-line args)
    OpaqueFunction,              # Used to evaluate launch setup at runtime (for advanced substitutions)
    IncludeLaunchDescription,    # Allows inclusion of other launch files
    ExecuteProcess               # Allows execution of shell commands/processes
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Substitutions allow dynamic values to be evaluated at launch time
from launch.substitutions import (
    LaunchConfiguration,         # Refers to the value of a launch argument
    PathJoinSubstitution,        # Joins paths at runtime
    Command,                     # Allows calling external commands and getting their output
    FindExecutable,              # Finds a given executable (e.g., xacro)
    NotSubstitution,             # Logical NOT substitution
    OrSubstitution               # Logical OR substitution
)

# ROS 2-specific launch tools
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

# MoveIt utility to generate robot description configs
from moveit_configs_utils import MoveItConfigsBuilder

# Python helper to get package paths
from ament_index_python.packages import get_package_share_directory

import os

def launch_setup(context, *args, **kwargs):
    """
    This function dynamically sets up the launch configuration using runtime context.
    OpaqueFunction allows this function to evaluate LaunchConfigurations at runtime.
    """

    # Read the 'ethercat_fake' argument from the launch context
    ethercat_fake = LaunchConfiguration('ethercat_fake')

    # Robot description created dynamically using xacro and a parameter from the launch argument
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]),  # xacro executable
                " ",
                PathJoinSubstitution([
                    FindPackageShare('beckhoff_testbed_description'),
                    "urdf",
                    "main.xacro"                                       # Path to your xacro file
                ]),
                " ", "use_ethercat_fake_hardware:='", ethercat_fake.perform(context), "'",  # Pass whether fake EtherCAT hardware should be used
            ]),
            value_type=str
        )
    }

    # Load controller configuration YAML
    controller_config = PathJoinSubstitution([
        FindPackageShare("beckhoff_testbed_description"),
        "config",
        "ros2_controller_config.yaml"
    ])

    # Load EtherCAT-related utility configuration
    ethercat_utils_config = PathJoinSubstitution([
        FindPackageShare('beckhoff_testbed_description'),
        "config",
        "ethercat_utils_config.yaml"
    ])

    # Main controller manager node, runs the hardware interface and controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",  # Print output to both stdout and stderr
        parameters=[robot_description, controller_config],  # Provide robot description and controller configs
        arguments=["--ros-args", "--log-level", "info"],    # Optional: log level
    )

    # EtherCAT utility node that manages the communication with the brushless motor driver (slave)
    brushless_utils_node = Node(
        name="brushless_utils",
        package="ethercat_utils",
        executable="cia402_slave_manager",                  # Custom node handling CiA402 profiles
        parameters=[ethercat_utils_config]
    )

    # Controller spawner for publishing joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",                      # Name of the controller to spawn
            "--controller-manager", "/controller_manager"
        ]
    )

    # Controller spawner for digital I/O interface
    digital_io_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "digital_io_controller",
            "--controller-manager", "/controller_manager"
        ]
    )

    # Controller spawner for the actual brushless motor controller
    brushless_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "brushless_controller",
            "--controller-manager", "/controller_manager"
        ]
    )

    # Collect all the nodes and spawners to be launched
    what_to_launch = [
        control_node,
        brushless_utils_node,
        joint_state_broadcaster_spawner,
        digital_io_controller_spawner,
        brushless_controller_spawner,
    ]

    return what_to_launch


def generate_launch_description():
    """
    Main function that gets called when this launch file is executed.
    It declares launch arguments and returns the full launch description.
    """

    # Declare arguments available to the user (e.g., via command line)
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'ethercat_fake',              # Launch arg name
        default_value='false'         # Default value: assume real EtherCAT hardware
    ))

    # Combine the launch arguments and the dynamic setup function
    return LaunchDescription(
        launch_arguments + [
            OpaqueFunction(function=launch_setup)  # Setup is delayed until launch time
        ]
    )
