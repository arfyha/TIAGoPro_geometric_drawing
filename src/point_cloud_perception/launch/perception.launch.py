from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='point_cloud_perception',
        executable='pre_process',
        name='pre_process',
        parameters=[{
            'cloud_topic': '/head_front_camera/depth/color/points',
            'world_frame': 'base_footprint',
            'voxel_leaf_size': 0.01,
            'x_filter_min': -0.7,
            'x_filter_max': 5.0,
            'y_filter_min': -1.2,
            'y_filter_max': 1.2,
            'z_filter_min': -0.1,
            'z_filter_max': 1.8 
        }],
        output='screen',
    ),
        Node(
            package='point_cloud_perception',
            executable='euclidean_cluster',
            name='euclidean_cluster',
            parameters=[{
                'cloud_topic': '/head_front_camera/depth/color/points', #pre_process_filtered_cloud
                'world_frame': 'base_footprint',
                'cluster_tolerance': 0.05,
                'min_cluster_size': 100,
                'max_cluster_size': 10000,
            }],
            output='screen',
        )
    ])
