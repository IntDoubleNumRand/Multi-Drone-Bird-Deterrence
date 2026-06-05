# Shared multi-drone defaults for ROS launch files (importable as drone_system.launch_params).

DRONE_IDS = ['drone_1', 'drone_2']

DRONE_POSE_TOPICS = [
    '/drone_1/local_position/pose',
    '/drone_2/local_position/pose',
]

CHASED_MASK_INPUT_TOPICS = [
    '/birds/chased_mask/drone_1',
    '/birds/chased_mask/drone_2',
]
