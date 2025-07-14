from launch import LaunchDescription
from launch_ros.actions import Node

# -----------------------------------------------------------------------------
# Launch file for starting the point cloud perception pipeline nodes
# -----------------------------------------------------------------------------
def generate_launch_description():
    """
    Generates the launch description for the point cloud perception pipeline.

    This launch file starts:
    - pre_process_node: Filters and transforms the raw point cloud data.
    - euclidean_cluster_node: Performs Euclidean clustering to segment objects.
    - plane_segmentation_node: Segments the largest plane (e.g., whiteboard) from clusters.

    Each node is launched with its parameters and output shown on the screen.
    """
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
                'nr_k': 200,
                'stddev_mult': 2.0,
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
                'cluster_tolerance': 0.18,
                'min_cluster_dev': 4.0,
                'index': 0,
            }],
            output='screen',
        ),
        Node(
            package='point_cloud_perception',
            executable='plane_segmentation',
            name='plane_segmentation_node',
            parameters=[{
                'cloud_topic': 'whiteboard_cluster_cloud',
                'world_frame': 'base_footprint',
                'threshold': 0.0001,
            }],
            output='screen',
        ),
    ])
