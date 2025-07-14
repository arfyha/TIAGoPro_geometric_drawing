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

/**
 * @brief Node for preprocessing point clouds in ROS 2.
 *
 * This node subscribes to a raw point cloud topic, transforms the cloud to a target frame,
 * applies voxel grid, crop box, and statistical outlier removal filters, and publishes
 * the results to various topics.
 */
class PreProcessNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor. Sets up publishers, parameters, subscriber, and TF2.
     */
    PreProcessNode() : Node("pre_process_node", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
    {
        /*
        * SET UP PUBLISHERS
        */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_filtered_cloud", 10);
        crop_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/crop_filtered_cloud", 10);
        pre_process_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pre_process_filtered_cloud", 10);
        org_cloud_oub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/original_cloud", 10);
        trans_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transformed_cloud", 10);

        /*
         * SET UP PARAMETERS
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        cloud_topic = this->get_or_create_parameter<std::string>("cloud_topic", "/head_front_camera/depth/color/points");
        world_frame = this->get_or_create_parameter<std::string>("world_frame", "base_footprint");
        voxel_leaf_size = float(this->get_or_create_parameter<double>("voxel_leaf_size", 0.01));
        x_filter_min = this->get_or_create_parameter<double>("x_filter_min", -0.7);
        x_filter_max = this->get_or_create_parameter<double>("x_filter_max", 3.0);
        y_filter_min = this->get_or_create_parameter<double>("y_filter_min", -1.2);
        y_filter_max = this->get_or_create_parameter<double>("y_filter_max", -1.2);
        z_filter_min = this->get_or_create_parameter<double>("z_filter_min", 0.1);
        z_filter_max = this->get_or_create_parameter<double>("z_filter_max", 1.8);
        nr_k = this->get_or_create_parameter<int>("nr_k", 50);
        stddev_mult = this->get_or_create_parameter<double>("stddev_mult", 1.0);

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
                    std::bind(&PreProcessNode::pointCloudCallback, this, std::placeholders::_1)
                );

        /*
         * SET UP TF2.
         * 
         */
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Pre Process Node started.");
    }

private:

    /*
     * Subscriber and Publisher declaration
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crop_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_process_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr org_cloud_oub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trans_cloud_pub_;


    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;

    float voxel_leaf_size;
    float x_filter_min, x_filter_max;
    float y_filter_min, y_filter_max;
    float z_filter_min, z_filter_max;
    int nr_k;
    double stddev_mult;

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;


    /**
     * @brief Callback for incoming point cloud messages.
     * 
     * Transforms the point cloud to the target frame, applies filters, and publishes results.
     * 
     * @param point_cloud_msg The incoming point cloud message.
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
    {
        // Publishing orginal cloud with differnt header (just for visualization)
        pcl::PointCloud<pcl::PointXYZ>::Ptr org_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*point_cloud_msg, *org_cloud);
        this->publishPointCloud(org_cloud_oub, *org_cloud);
        //RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", org_cloud->size());

        // Update parameters at runtime (for dynamic reconfigure)
        voxel_leaf_size = float(this->get_parameter("voxel_leaf_size").get_parameter_value().get<double>());
        x_filter_min = this->get_parameter("x_filter_min").get_parameter_value().get<double>();
        x_filter_max = this->get_parameter("x_filter_max").get_parameter_value().get<double>();
        y_filter_min = this->get_parameter("y_filter_min").get_parameter_value().get<double>();
        y_filter_max = this->get_parameter("y_filter_max").get_parameter_value().get<double>();
        z_filter_min = this->get_parameter("z_filter_min").get_parameter_value().get<double>();
        z_filter_max = this->get_parameter("z_filter_max").get_parameter_value().get<double>();
        nr_k = this->get_parameter("nr_k").get_parameter_value().get<int>();
        stddev_mult = this->get_parameter("stddev_mult").get_parameter_value().get<double>();
        
        //RCLCPP_INFO(this->get_logger(), "Voxel leaf size=%f", voxel_leaf_size);
        //RCLCPP_INFO(this->get_logger(), "Crop box paramter:(%f, %f, %f, %f, %f, %f)", x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max);
        //RCLCPP_INFO(this->get_logger(), "Number of neighbors=%d, Standard deviotion multiplier=%f", nr_k, stddev_mult);

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

        // Convert transformed cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_cloud, *cloud);
        //RCLCPP_INFO(this->get_logger(), "Transformed point cloud with %zu points", cloud->size());
        this->publishPointCloud(trans_cloud_pub_, *cloud);
        
        // Apply voxel grid filter and publish
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud = voxel_filter(cloud);
        this->publishPointCloud(voxel_pub_, *voxel_cloud);
        //RCLCPP_INFO(this->get_logger(), "Voxel filtered point cloud with %zu points", voxel_cloud->size());

        // Apply crop box filter and publish
        pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud = crop_box_filter(voxel_cloud);
        this->publishPointCloud(crop_pub_, *crop_cloud);
        //RCLCPP_INFO(this->get_logger(), "Cropped point cloud with %zu points", crop_cloud->size());

        // Apply statistical outlier removal and publish pre-processed cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pre_processed_cloud = sor_filter(crop_box_filter(voxel_filter(cloud)));
        this->publishPointCloud(pre_process_pub_, *pre_processed_cloud);
        //RCLCPP_INFO(this->get_logger(), "Pre-processed point cloud with %zu points", pre_processed_cloud->size());
    }

    /**
     * @brief Applies a voxel grid filter to downsample the point cloud.
     * @param point_cloud Input point cloud.
     * @return Filtered point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Downsample
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(point_cloud);
        voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel.filter(*filtered);

        return filtered;
    }

    /**
     * @brief Applies a crop box filter to the point cloud.
     * @param point_cloud Input point cloud.
     * @return Filtered point cloud.
     */
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

    /**
     * @brief Applies a statistical outlier removal filter to the point cloud.
     * @param point_cloud Input point cloud.
     * @return Filtered point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Remove noise
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(point_cloud);
        sor.setMeanK(nr_k); // Neighbor points to analyze
        sor.setStddevMulThresh(stddev_mult); // Remove outliers
        sor.filter(*filtered);

        return filtered;
    }

    /**
     * @brief Publishes a PCL point cloud as a ROS 2 PointCloud2 message.
     * @param publisher The publisher to use.
     * @param point_cloud The point cloud to publish.
     */
    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
        pcl::PointCloud<pcl::PointXYZ> &point_cloud) 
    {
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(point_cloud, *pc2_cloud);
    pc2_cloud->header.frame_id = world_frame;
    pc2_cloud->header.stamp = this->get_clock()->now();
    publisher->publish(*pc2_cloud);
    }

    /**
     * @brief Utility for parameter handling: declares or gets a parameter.
     * @tparam T Parameter type.
     * @param name Parameter name.
     * @param default_value Default value if parameter is not set.
     * @return Parameter value.
     */
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

/**
 * @brief Main function. Initializes ROS 2, spins the node, and shuts down.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreProcessNode>());
    rclcpp::shutdown();
    return 0;
}
