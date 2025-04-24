#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>


class VoxelFilterNode : public rclcpp::Node
{
public:
    VoxelFilterNode() : Node("voxel_filter_node", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true))
    {

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);

        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->declare_parameter<std::string>("cloud_topic", "/head_front_camera/depth/color/points");
        this->declare_parameter<std::string>("world_frame", "base_footprint");
        this->declare_parameter<std::string>("camera_frame", "head_front_camera_depth_optical_frame");
        this->declare_parameter<float>("voxel_leaf_size", 0.05f);

        cloud_topic = this->get_parameter("cloud_topic").as_string();
        world_frame = this->get_parameter("world_frame").as_string();
        camera_frame = this->get_parameter("camera_frame").as_string();
        voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();

        /*
         * SET UP SUBSCRIBER
         */
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cloud_topic,
                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                        .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
                        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                        .keep_last(300),
                    std::bind(&VoxelFilterNode::pointCloudCallback, this, std::placeholders::_1)
                );

        /*
         * SET UP TF. Optional for transforming between coordinate frames
         *          You need to create a static transform publisher to use this
         */
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Voxel Filter Node started.");
    }

private:

    /*
     * Subscriber and Publisher declaration
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    std::string camera_frame;

    float voxel_leaf_size;

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
    {

        // Transform for pointcloud in world frame
        geometry_msgs::msg::TransformStamped stransform;

        try {
            stransform = tf_buffer_->lookupTransform(world_frame, point_cloud_msg->header.frame_id,
                tf2::TimePointZero, tf2::durationFromSec(3));
        }
        catch(const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(world_frame, stransform, *point_cloud_msg, transformed_cloud);

        // Convert ROS2 msg to PCL PointCloud
        pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(transformed_cloud, *pcl_cloud);

        // VoxelGrid filter
        pcl::PCLPointCloud2::Ptr pcl_filtered(new pcl::PCLPointCloud2());
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
        voxel_filter.setInputCloud(pcl_cloud);
        voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_filter.filter(*pcl_filtered);

        // Convert back to ROS2 PointCloud2
        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(*pcl_filtered, output);
        output.header.stamp = point_cloud_msg->header.stamp;
        output.header.frame_id = world_frame;

        pub_->publish(output);
    }

    
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelFilterNode>());
    rclcpp::shutdown();
    return 0;
}
