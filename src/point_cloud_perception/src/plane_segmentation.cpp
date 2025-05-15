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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


class PlaneSegmentationNode : public rclcpp::Node
{
public:
    PlaneSegmentationNode() : Node("plane_segmentation", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
    {
        /*
        * SET UP PUBLISHERS
        */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        plane_seg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_out_cloud", 10);
        plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/whiteboard_cloud", 10);

        /*
         * SET UP PARAMETERS
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        cloud_topic = this->get_or_create_parameter<std::string>("cloud_topic", "/pre_process_filtered_cloud");
        world_frame = this->get_or_create_parameter<std::string>("world_frame", "base_footprint");

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
                    std::bind(&PlaneSegmentationNode::pointCloudCallback, this, std::placeholders::_1)
                );

        RCLCPP_INFO(this->get_logger(), "Plane Segmentation Node started.");
    }

private:

    /*
     * Subscriber and Publisher declaration
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_seg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
    {
        // Convert ROS2 msg to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_out (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_whiteboard (new pcl::PointCloud<pcl::PointXYZ> ());

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (200);
        seg.setDistanceThreshold (0.004);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.") ;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices(inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_whiteboard);
        //RCLCPP_INFO(this->get_logger(), "PointCloud2 representing the planar component: '%lu' data points.", cloud_plane->points.size());
        
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_filtered_out);

        this->publishPointCloud(plane_pub_, *cloud_whiteboard);
        this->publishPointCloud(plane_seg_pub_, *cloud_filtered_out);
        
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

    template<typename T>
    T get_or_create_parameter(const std::string & name, const T & default_value)
    {
        T value;
        if (!this->has_parameter(name)) {
            this->declare_parameter<T>(name, default_value);
            value = default_value;
        } else {
            value = this->get_parameter(name).get_parameter_value().get<T>();
        }
        return value;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}
