# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config', 'slam.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_slam.rviz']
    )
    
    # lc = LaunchContext()
    # ros_distro = EnvironmentVariable('ROS_DISTRO')
    # slam_param_name = 'slam_params_file'
    # if ros_distro.perform(lc) == 'foxy': 
    #     slam_param_name = 'params_file'

    robot_ns = os.getenv('ROBOT_NAMESPACE')
    if robot_ns is None:
        robot_ns = ""

    if robot_ns != "":
        remappings = [
            ('/tf', '/'+robot_ns + '/tf'),
            ('/tf_static', '/'+robot_ns + '/tf_static'),
            ('/scan', '/' + robot_ns + '/scan'),
            ('/map', '/' + robot_ns + '/map'),
            ('/map_metadata', '/' + robot_ns + '/map_metadata')
        ]
        slam_param_substitutions = {
            'map_frame': robot_ns + '_map',
            'odom_frame': robot_ns + '_odom',
            'base_frame': robot_ns + '_base_footprint'
            }

        slam_config = RewrittenYaml(
                source_file=slam_config_path,
                root_key=robot_ns,
                param_rewrites=slam_param_substitutions,
                convert_types=True)
    else:
        slam_config = slam_config_path
        remappings=[]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),
        Node(
            parameters=[
                slam_config,
                {'use_sim_time': LaunchConfiguration("sim")}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=remappings
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])


# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     slam_params_file = LaunchConfiguration('slam_params_file')

#     declare_use_sim_time_argument = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description='Use simulation/Gazebo clock')
#     declare_slam_params_file_cmd = DeclareLaunchArgument(
#         'slam_params_file',
#         default_value=os.path.join(get_package_share_directory("slam_toolbox"),
#                                    'config', 'mapper_params_online_async.yaml'),
#         description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

#     start_async_slam_toolbox_node = Node(
#         parameters=[
#           slam_params_file,
#           {'use_sim_time': use_sim_time}
#         ],
#         package='slam_toolbox',
#         executable='async_slam_toolbox_node',
#         name='slam_toolbox',
#         output='screen')

#     ld = LaunchDescription()

#     ld.add_action(declare_use_sim_time_argument)
#     ld.add_action(declare_slam_params_file_cmd)
#     ld.add_action(start_async_slam_toolbox_node)

#     return ld