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

/**
 * @brief Node for performing Euclidean clustering on point clouds in ROS 2.
 *
 * This node subscribes to a filtered point cloud topic, performs Euclidean cluster extraction,
 * identifies the cluster in front of the robot (y ≈ 0), and publishes the clusters as separate topics.
 */
class EuclideanClusterNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor. Sets up publishers, parameters, subscriber, and logs startup.
     */
    EuclideanClusterNode() : Node("euclidean_cluster_node", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
    {
        /*
        * SET UP PUBLISHERS
        */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        euclidean_cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/euclidean_cluster_cloud", 10);
        whiteboard_cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/whiteboard_cluster_cloud", 10);

        /*
         * SET UP PARAMETERS
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        cloud_topic = this->get_or_create_parameter<std::string>("cloud_topic", "/voxel_filtered_cloud");
        world_frame = this->get_or_create_parameter<std::string>("world_frame", "base_footprint");
        cluster_tolerance = this->get_or_create_parameter<double>("cluster_tolerance", 0.2);
        min_cluster_dev = this->get_or_create_parameter<double>("min_cluster_dev", 3.0);
        index = size_t(this->get_or_create_parameter<int>("index", 0));

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
                    std::bind(&EuclideanClusterNode::pointCloudCallback, this, std::placeholders::_1)
                );

        RCLCPP_INFO(this->get_logger(), "Euclidean Cluser Node started.");
    }

private:

    /*
     * Subscriber and Publisher declaration
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr euclidean_cluster_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr whiteboard_cluster_pub_;


    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    double cluster_tolerance;
    double min_cluster_dev ;
    std::size_t index;

    /**
     * @brief Callback for incoming point cloud messages.
     *
     * Performs Euclidean cluster extraction, finds the cluster with y ≈ 0, and publishes clusters.
     * @param point_cloud_msg The incoming point cloud message.
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
    {
        // Update parameters at runtime
        cluster_tolerance = this->get_parameter("cluster_tolerance").get_parameter_value().get<double>();
        min_cluster_dev = (this->get_parameter("min_cluster_dev").get_parameter_value().get<double>());
        index = size_t(this->get_parameter("index").get_parameter_value().get<int>());
        
        // Validate min_cluster_dev parameter
        if(min_cluster_dev < 1.0 || min_cluster_dev > 100.0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid min_cluster_dev value: %f. It should be between 1 and 100.", min_cluster_dev);
            min_cluster_dev = 10.0; // Reset to default value
        }

        //RCLCPP_INFO(this->get_logger(), "maximum distance threshold: %f", cluster_tolerance);
        //RCLCPP_INFO(this->get_logger(), "Divisor: %f", min_cluster_dev);

        // Convert ROS2 msg to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        // Set up Euclidean cluster extraction
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (cluster_tolerance);
        pcl::uindex_t min_cluster_size = cloud->points.size() / min_cluster_dev;
        pcl::uindex_t max_cluster_size = cloud->points.size();
        //RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d, max_cluster_size: %d", min_cluster_size, max_cluster_size);
        ec.setMinClusterSize (min_cluster_size);
        ec.setMaxClusterSize (max_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        // Store clusters and find the cluster with y ≈ 0 (in front of robot)
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
        int whiteboard_index = -1;
        int i = 0;

        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

            for (const auto& idx : cluster.indices) {
                cloud_cluster->points.push_back((*cloud)[idx]);
            }

            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);

            // Identify cluster containing points with y ≈ 0
            for (auto& point : cloud_cluster->points) {
                if (std::abs(point.y) < 0.05) {
                    whiteboard_index = i;
                }
            }
            i++;
        }
        // Publish selected clusters
        if (clusters.empty()) return;
        //RCLCPP_INFO(this->get_logger(), "Number clusters '%lu', whiteboard index '%d'", clusters.size(), whiteboard_index);        if (clusters.empty()) return;
        for (size_t i = 0; i < clusters.size(); ++i) {
            //RCLCPP_INFO(this->get_logger(), "Cluster %zu has '%lu' points", i, clusters.at(i)->points.size());
        }
        this->publishPointCloud(euclidean_cluster_pub_, *clusters.at(index));
        if (whiteboard_index > -1) this->publishPointCloud(whiteboard_cluster_pub_, *clusters.at(whiteboard_index));
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
    rclcpp::spin(std::make_shared<EuclideanClusterNode>());
    rclcpp::shutdown();
    return 0;
}
