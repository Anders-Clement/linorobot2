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
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

MAP_NAME='home' #change to the name of your own map here

def generate_launch_description():
    #depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config', 'navigation.yaml']
    )

    robot_ns = os.environ.get('ROBOT_NAMESPACE')
    if robot_ns is None:
        robot_ns = ""

    if robot_ns != "":
        use_namespace = 'true'
        nav2_config = nav2_config_path
        nav2_param_substitutions = {
            # 'base_frame_id': robot_ns + '_base_footprint',
            # 'global_frame_id': robot_ns + '_map',
            # 'odom_frame_id': robot_ns + '_odom',
            # 'robot_base_frame': robot_ns + '_base_link',
            # 'odom_topic': robot_ns + '/odom',
            # 'frame_id': robot_ns + '_map',
            }

        nav2_config = RewrittenYaml(
                source_file=nav2_config_path,
                root_key=robot_ns,
                param_rewrites=nav2_param_substitutions,
                #convert_types=True
                )

        # # also replace keys with same name but different values
        # lc = LaunchContext()
        # nav2_config = nav2_config.perform(lc)
        # import yaml
        # nav2_config_yaml = yaml.safe_load(open(nav2_config,'r'))
        # nav2_config_yaml[robot_ns]['bt_navigator']['ros__parameters']['global_frame'] = robot_ns + '_map'
        # nav2_config_yaml[robot_ns]['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = robot_ns + '_odom'
        # nav2_config_yaml[robot_ns]['global_costmap']['global_costmap']['ros__parameters']['global_frame'] = robot_ns + '_map'
        # nav2_config_yaml[robot_ns]['recoveries_server']['ros__parameters']['global_frame'] = robot_ns + '_odom'
        # # save the yaml back to the temp yaml file given by RewrittenYaml
        # with open(nav2_config, 'w') as output_yaml:
        #     yaml.dump(nav2_config_yaml, output_yaml)
        # nav2_config = PathJoinSubstitution([nav2_config])
        
    else:
        nav2_config = nav2_config_path
        use_namespace = 'false'

    #print('-------------------', nav2_config, use_namespace)

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

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'namespace' : robot_ns,
                'use_namespace' : use_namespace,
                'use_composition' : 'False',
                'params_file': nav2_config
            }.items()
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