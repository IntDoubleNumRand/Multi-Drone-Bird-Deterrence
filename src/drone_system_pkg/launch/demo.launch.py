# Same stack as system.launch.py; use scripts/launch_demo.sh (often with field_layout_demo.yaml).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from drone_system.launch_params import (
    CHASED_MASK_INPUT_TOPICS,
    DRONE_IDS,
    DRONE_POSE_TOPICS,
)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    bird_count = LaunchConfiguration('bird_count')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bird_count', default_value='3'),
        Node(
            package='drone_system_pkg',
            executable='birds_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'bird_count': ParameterValue(bird_count, value_type=int)},
                {'frame_id': 'map'},
                {'drone_pose_topics': DRONE_POSE_TOPICS},
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
                {'drone_ids': DRONE_IDS},
                {'drone_pose_topics': DRONE_POSE_TOPICS},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='chased_mask_aggregator_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'input_topics': CHASED_MASK_INPUT_TOPICS},
                {'bird_status_topic': '/birds/status'},
                {'output_topic': '/birds/chased_mask'},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='coordinator_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'drone_id': 'drone_1'},
                {'pose_topic': '/drone_1/local_position/pose'},
                {'setpoint_topic': '/drone_1/setpoint_position/local'},
                {'mavros_state_topic': '/drone_1/state'},
                {'battery_topic': '/drone_1/battery'},
                {'mavros_arm_service': '/drone_1/cmd/arming'},
                {'mavros_set_mode_service': '/drone_1/set_mode'},
                {'chased_mask_topic': '/birds/chased_mask/drone_1'},
                {'target_topic': '/coordinator/drone_1/target_index'},
                {'z_state_topic': '/drone/drone_1/z_state'},
                {'birds_topic': '/birds/positions'},
                {'home_x': -5.0},
                {'home_y': -10.0},
                {'auto_offboard': True},
                {'use_local_assignment_fallback': False},
                {'map_frame': 'map'},
                {'chase_standoff_m': 1.5},
                {'setpoint_max_step_m': 0.0},
                {'setpoint_max_z_step_m': 0.0},
                {'patrol_advance_m': 0.5},
                {'demo_post_chase_home_s': 0.0},
                {'enable_low_battery_return': False},
                {'battery_drain_per_tick': 0.0},
                {'obstacles_topic': '/obstacles/positions'},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='coordinator_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'drone_id': 'drone_2'},
                {'pose_topic': '/drone_2/local_position/pose'},
                {'setpoint_topic': '/drone_2/setpoint_position/local'},
                {'mavros_state_topic': '/drone_2/state'},
                {'battery_topic': '/drone_2/battery'},
                {'mavros_arm_service': '/drone_2/cmd/arming'},
                {'mavros_set_mode_service': '/drone_2/set_mode'},
                {'chased_mask_topic': '/birds/chased_mask/drone_2'},
                {'target_topic': '/coordinator/drone_2/target_index'},
                {'z_state_topic': '/drone/drone_2/z_state'},
                {'birds_topic': '/birds/positions'},
                {'home_x': 5.0},
                {'home_y': 10.0},
                {'auto_offboard': True},
                {'use_local_assignment_fallback': False},
                {'map_frame': 'map'},
                {'chase_standoff_m': 1.5},
                {'setpoint_max_step_m': 0.0},
                {'setpoint_max_z_step_m': 0.0},
                {'patrol_advance_m': 0.5},
                {'demo_post_chase_home_s': 0.0},
                {'enable_low_battery_return': False},
                {'battery_drain_per_tick': 0.0},
                {'obstacles_topic': '/obstacles/positions'},
            ],
        ),
        Node(
            package='drone_system_pkg',
            executable='visualization_node',
            output='screen',
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'pose_topic': '/drone_1/local_position/pose'},
                {'drone_ids': DRONE_IDS},
                {'drone_pose_topics': DRONE_POSE_TOPICS},
                {'target_topics': ['/coordinator/drone_1/target_index', '/coordinator/drone_2/target_index']},
                {'z_state_topics': ['/drone/drone_1/z_state', '/drone/drone_2/z_state']},
                {'birds_topic': '/birds/positions'},
                {'obstacles_topic': '/obstacles/positions'},
                {'map_frame': 'map'},
            ],
        ),
    ])
