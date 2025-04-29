#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/convert.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/transform_datatypes.hpp>
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
        /*
        * SET UP PUBLISHERS
        */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_filtered_cloud", 10);
        crop_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/crop_filtered_cloud", 10);
        sor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sor_filtered_cloud", 10);
        pre_process_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pre_process_filtered_cloud", 10);

        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->declare_parameter<std::string>("cloud_topic", "/head_front_camera/depth/color/points");
        this->declare_parameter<std::string>("world_frame", "base_footprint");
        this->declare_parameter<double>("voxel_leaf_size", 0.01);
        this->declare_parameter<double>("x_filter_min", -0.7);
        this->declare_parameter<double>("x_filter_max", 2.0);
        this->declare_parameter<double>("y_filter_min", -1.2);
        this->declare_parameter<double>("y_filter_max", 1.2);
        this->declare_parameter<double>("z_filter_min", 0.0);
        this->declare_parameter<double>("z_filter_max", 1.8);

        cloud_topic = this->get_parameter("cloud_topic").as_string();
        world_frame = this->get_parameter("world_frame").as_string();
        voxel_leaf_size = float(this->get_parameter("voxel_leaf_size").as_double());
        x_filter_min = this->get_parameter("x_filter_min").as_double();
        x_filter_max = this->get_parameter("x_filter_max").as_double();
        y_filter_min = this->get_parameter("y_filter_min").as_double();
        y_filter_max = this->get_parameter("y_filter_max").as_double();
        z_filter_min = this->get_parameter("z_filter_min").as_double();
        z_filter_max = this->get_parameter("z_filter_max").as_double();

        /*
         * SET UP SUBSCRIBER
         */
        RCLCPP_INFO(this->get_logger(), "Setting up subscriber");

         rclcpp::QoS qos_settings = rclcpp::SensorDataQoS()
            .keep_last(5)
            .best_effort();

        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cloud_topic,
                    qos_settings,
                    std::bind(&VoxelFilterNode::pointCloudCallback, this, std::placeholders::_1)
                );

        /*
         * SET UP TF.
         * 
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
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crop_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_process_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;

    float voxel_leaf_size;
    float x_filter_min, x_filter_max;
    float y_filter_min, y_filter_max;
    float z_filter_min, z_filter_max;

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
            return;
        }
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(world_frame, transform, *point_cloud_msg, transformed_cloud);

        // Convert ROS2 msg to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_cloud, *cloud);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud = voxel_filter(cloud);
        this->publishPointCloud(voxel_pub_, *voxel_cloud);
        //RCLCPP_INFO(this->get_logger(), "Voxel Filtered Cloud published.");

        pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud = crop_box_filter(cloud);
        this->publishPointCloud(crop_pub_, *crop_cloud);
        //RCLCPP_INFO(this->get_logger(), "Crop Box Filtered Cloud published.");

        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud = sor_filter(cloud);
        this->publishPointCloud(sor_pub_, *sor_cloud);
        //RCLCPP_INFO(this->get_logger(), "Statistical Outlier Removal Filtered Cloud published.");

        pcl::PointCloud<pcl::PointXYZ>::Ptr pre_processed_cloud = sor_filter(crop_box_filter(voxel_filter(cloud)));
        this->publishPointCloud(pre_process_pub_, *pre_processed_cloud);
        //RCLCPP_INFO(this->get_logger(), "Pre Processed Cloud published.");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Downsample
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(point_cloud);
        voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel.filter(*filtered);

        return filtered;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_box_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Crop
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(point_cloud);
        Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
        Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
        crop.setMin(min_point);
        crop.setMax(max_point);
        crop.filter(*filtered);

        return filtered;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Remove noise
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(point_cloud);
        sor.setMeanK(50); // Neighbor points to analyze
        sor.setStddevMulThresh(1.0); // Remove outliers
        sor.filter(*filtered);

        return filtered;
    }

    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
        pcl::PointCloud<pcl::PointXYZ> &point_cloud) 
    {
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(point_cloud, *pc2_cloud);
    pc2_cloud->header.frame_id = world_frame;
    pc2_cloud->header.stamp = this->get_clock()->now();
    publisher->publish(*pc2_cloud);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelFilterNode>());
    rclcpp::shutdown();
    return 0;
}
