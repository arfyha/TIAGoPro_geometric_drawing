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


class EuclideanClusterNode : public rclcpp::Node
{
public:
    EuclideanClusterNode() : Node("euclidean_cluster_node", rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true))
    {
        /*
        * SET UP PUBLISHERS
        */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        euclidean_cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/euclidean_cluster_cloud", 10);

        /*
         * SET UP PARAMETERS
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->declare_parameter<std::string>("cloud_topic", "/pre_process_filtered_cloud");
        this->declare_parameter<std::string>("world_frame", "base_footprint");
        this->declare_parameter<double>("cluster_tolerance", 0.01);
        this->declare_parameter<int>("min_cluster_size", 1);
        this->declare_parameter<int>("max_cluster_size", 10000000);

        cloud_topic = this->get_parameter("cloud_topic").as_string();
        world_frame = this->get_parameter("world_frame").as_string();
        cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size = this->get_parameter("max_cluster_size").as_int();

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

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    double cluster_tolerance;
    pcl::uindex_t min_cluster_size;
    pcl::uindex_t max_cluster_size;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
    {
        // Convert ROS2 msg to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (cluster_tolerance);
        ec.setMinClusterSize (min_cluster_size);
        ec.setMaxClusterSize (max_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> pc2_clusters;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;

        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

            for (const auto& idx : cluster.indices) {
                cloud_cluster->points.push_back((*cloud)[idx]);
            }

            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            RCLCPP_INFO(this->get_logger(), "Cluster has '%lu' points", cloud_cluster->points.size());
            clusters.push_back(cloud_cluster);
            sensor_msgs::msg::PointCloud2::SharedPtr tempROSMsg(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
            pc2_clusters.push_back(tempROSMsg);

        }
        RCLCPP_INFO(this->get_logger(), "Largest cluster has '%lu' points", clusters.at(0)->points.size());
        this->publishPointCloud(euclidean_cluster_pub_, *clusters.at(0));
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
    rclcpp::spin(std::make_shared<EuclideanClusterNode>());
    rclcpp::shutdown();
    return 0;
}
