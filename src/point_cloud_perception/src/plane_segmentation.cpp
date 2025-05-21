#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
#include <pcl/features/normal_3d.h>

#include <visualization_msgs/msg/marker_array.hpp>


class PlaneSegmentationNode : public rclcpp::Node
{
public:
    PlaneSegmentationNode() : Node("plane_segmentation_node", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
    {
        /*
        * SET UP PUBLISHERS
        */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        plane_seg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_out_cloud", 10);
        plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/whiteboard_cloud", 10);
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/whiteboard_pose", 10);

        /*
         * SET UP PARAMETERS
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        cloud_topic = this->get_or_create_parameter<std::string>("cloud_topic", "/pre_process_filtered_cloud");
        world_frame = this->get_or_create_parameter<std::string>("world_frame", "base_footprint");
        radius = this->get_or_create_parameter<double>("radius", 0.05);
        max_iterations = this->get_or_create_parameter<int>("max_iterations", 2000);
        threshold = this->get_or_create_parameter<double>("threshold", 0.004);

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

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "Plane Segmentation Node started.");
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /*
     * Subscriber and Publisher declaration
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_seg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    double radius;
    int max_iterations;
    double threshold;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
    {
        radius = this->get_parameter("radius").get_parameter_value().get<double>();
        max_iterations = this->get_parameter("max_iterations").get_parameter_value().get<int>();
        threshold = this->get_parameter("threshold").get_parameter_value().get<double>();

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
        seg.setMaxIterations (max_iterations);
        seg.setDistanceThreshold (threshold);

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

        computeWhiteboardFeatures(cloud_whiteboard);

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

    void computeWhiteboardFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

        // Create NormalEstimation object
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(radius);
        ne.compute(*normals);

        Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
        for (const auto &normal : *normals) {
            avg_normal += Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z);
        }
        avg_normal = avg_normal.normalized(); // Unit vector
        if (avg_normal[0] > 0) {
            avg_normal = -avg_normal;
        }

        // Assume the whiteboard's normal is its "up" (Z) direction
        Eigen::Vector3f whiteboard_normal = avg_normal;
        Eigen::Vector3f world_z(0, 0, 1);

        Eigen::Vector3f rotation_axis = world_z.cross(whiteboard_normal);
        float rotation_angle = std::acos(world_z.dot(whiteboard_normal)/
                                          (world_z.norm() * whiteboard_normal.norm()));
        Eigen::Quaternionf quat(Eigen::AngleAxisf(rotation_angle, rotation_axis.normalized()));
        quat = quat.normalized();

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        Eigen::Vector4f min_pt, max_pt;
        Eigen::Vector3f center;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        center = (max_pt.head<3>() + min_pt.head<3>()) / 2;

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->get_clock()->now();
        pose_array.header.frame_id = world_frame;

        geometry_msgs::msg::Pose pose_whiteboard;
        pose_whiteboard.position.x = centroid[0];
        pose_whiteboard.position.y = centroid[1];
        pose_whiteboard.position.z = centroid[2];
        pose_whiteboard.orientation.w = quat.w();
        pose_whiteboard.orientation.x = quat.x();
        pose_whiteboard.orientation.y = quat.y();
        pose_whiteboard.orientation.z = quat.z();
        pose_array.poses.push_back(pose_whiteboard);

        geometry_msgs::msg::Pose pose_min_pt;
        pose_min_pt.position.x = min_pt[0];
        pose_min_pt.position.y = min_pt[1];
        pose_min_pt.position.z = min_pt[2];
        pose_array.poses.push_back(pose_min_pt);

        geometry_msgs::msg::Pose pose_max_pt;
        pose_max_pt.position.x = max_pt[0];
        pose_max_pt.position.y = max_pt[1];
        pose_max_pt.position.z = max_pt[2];
        pose_array.poses.push_back(pose_max_pt);

        geometry_msgs::msg::Pose pose_bb_center;
        pose_bb_center.position.x = center[0];
        pose_bb_center.position.y = center[1];
        pose_bb_center.position.z = center[2];
        pose_array.poses.push_back(pose_bb_center);

        pose_array_pub_->publish(pose_array);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}
