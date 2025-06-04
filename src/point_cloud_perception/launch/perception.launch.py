from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_cloud_perception',
            executable='pre_process',
            name='pre_process_node',
            parameters=[{
                'cloud_topic': '/head_front_camera/depth/color/points',
                'world_frame': 'base_footprint',
                'voxel_leaf_size': 0.01,
                'x_filter_min': -0.7,
                'x_filter_max': 3.0,
                'y_filter_min': -1.2,
                'y_filter_max': 1.2,
                'z_filter_min': 0.1,
                'z_filter_max': 1.8,
                'nr_k': 100,
                'stddev_mult': 0.5,
            }],
            output='screen',
        ),
        Node(
            package='point_cloud_perception',
            executable='euclidean_cluster',
            name='euclidean_cluster_node',
            parameters=[{
                'cloud_topic': 'pre_process_filtered_cloud',
                'world_frame': 'base_footprint',
                'cluster_tolerance': 0.05,
                'min_cluster_size': 500,
                'max_cluster_size': 1000000,
                'index': 0,
            }],
            output='screen',
        ),
        Node(
            package='point_cloud_perception',
            executable='plane_segmentation',
            name='plane_segmentation_node',
            parameters=[{
                'cloud_topic': 'euclidean_cluster_cloud',
                'world_frame': 'base_footprint',
                'threshold': 0.004,
            }],
            output='screen',
        ),
    ])
