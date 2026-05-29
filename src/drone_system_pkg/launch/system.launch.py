# Application nodes only. Start PX4, Gazebo, and MAVROS separately (see README).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

try:
    from drone_system.field_layout import bird_count as _layout_bird_count
    _DEFAULT_BIRD_COUNT = str(_layout_bird_count())
except Exception:
    _DEFAULT_BIRD_COUNT = '3'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    bird_count = LaunchConfiguration('bird_count')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true when using Gazebo / PX4 SITL clock',
        ),
        DeclareLaunchArgument(
            'bird_count',
            default_value=_DEFAULT_BIRD_COUNT,
            description='Simulated birds (default from field_layout.yaml)',
        ),
        Node(
            package='drone_system_pkg',
            executable='birds_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'bird_count': ParameterValue(bird_count, value_type=int)},
                {'frame_id': 'map'},
                {'pose_topic': '/mavros/local_position/pose'},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='obstacles_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'frame_id': 'map'},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='perception_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='centralized_coordinator_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'birds_topic': '/birds/positions'},
                {'bird_status_topic': '/birds/status'},
                {'drone_ids': ['drone_1']},
                {'drone_pose_topics': ['/mavros/local_position/pose']},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='coordinator_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'drone_id': 'drone_1'},
                {'pose_topic': '/mavros/local_position/pose'},
                {'setpoint_topic': '/mavros/setpoint_position/local'},
                {'birds_topic': '/birds/positions'},
                {'auto_offboard': True},
                {'map_frame': 'map'},
                {'target_lock_s': 2.0},
                {'switch_margin_m': 2.0},
                {'enable_low_battery_return': False},
                {'battery_drain_per_tick': 0.0},
                {'return_home_when_no_targets': True},
                {'obstacles_topic': '/obstacles/positions'},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='visualization_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'pose_topic': '/mavros/local_position/pose'},
                {'birds_topic': '/birds/positions'},
                {'obstacles_topic': '/obstacles/positions'},
                {'map_frame': 'map'},
            ],
        ),
    ])
