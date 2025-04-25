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
#include <pcl/filters/statistical_outlier_removal.h>


class VoxelFilterNode : public rclcpp::Node
{
public:
    VoxelFilterNode() : Node("voxel_filter_node", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true))
    {

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_filtered_cloud", 10);

        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->declare_parameter<std::string>("cloud_topic", "/head_front_camera/depth/color/points");
        this->declare_parameter<std::string>("world_frame", "base_footprint");
        this->declare_parameter<float>("voxel_leaf_size", 0.02f);

        cloud_topic = this->get_parameter("cloud_topic").as_string();
        world_frame = this->get_parameter("world_frame").as_string();
        voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();

        /*
         * SET UP SUBSCRIBER
         */

         rclcpp::QoS qos_settings = rclcpp::SensorDataQoS()
            .keep_last(5)  // Reduced from 300
            .best_effort();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cloud_topic,
                    qos_settings,
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
        geometry_msgs::msg::TransformStamped transform;

        try {
            transform = tf_buffer_->lookupTransform(world_frame, point_cloud_msg->header.frame_id,
                tf2::TimePointZero, tf2::durationFromSec(0.1));
        }
        catch(const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(world_frame, transform, *point_cloud_msg, transformed_cloud);

        // Convert ROS2 msg to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_cloud, *cloud);

        cloud = preprocess(cloud);

        // Convert back to ROS2 PointCloud2
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.stamp = this->now();
        output.header.frame_id = world_frame;

        pub_->publish(output);
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Downsample
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(point_cloud);
        voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel.filter(*filtered);
    
        // Remove noise
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(filtered);
        sor.setMeanK(50); // Neighbor points to analyze
        sor.setStddevMulThresh(1.0); // Remove outliers
        sor.filter(*filtered);
    
        // Crop X-axis
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0, 2.0);
        pass.filter(*filtered);
    
        return filtered;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelFilterNode>());
    rclcpp::shutdown();
    return 0;
}
