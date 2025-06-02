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

#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/io.h>

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
        //chull_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/whiteboard_chull", 10);

        /*
         * SET UP PARAMETERS
         */
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        cloud_topic = this->get_or_create_parameter<std::string>("cloud_topic", "/pre_process_filtered_cloud");
        world_frame = this->get_or_create_parameter<std::string>("world_frame", "base_footprint");
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
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr chull_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    double threshold;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
    {
        threshold = this->get_parameter("threshold").get_parameter_value().get<double>();

        // Convert ROS2 msg to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr whiteboard_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        std::vector<int> inliers;
        ransac.setDistanceThreshold (threshold);
        ransac.computeModel();
        ransac.getInliers(inliers);
        Eigen::VectorXf model_coefficients;
        ransac.getModelCoefficients(model_coefficients);
        RCLCPP_INFO(this->get_logger(), "Model coefficients: [%f, %f, %f, %f]", 
            model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3]);
        Eigen::VectorXf optimzed_model_coefficients;
        model_p->optimizeModelCoefficients(inliers, model_coefficients, optimzed_model_coefficients);
        RCLCPP_INFO(this->get_logger(), "Optimized model coefficients: [%f, %f, %f, %f]", 
            optimzed_model_coefficients[0], optimzed_model_coefficients[1], optimzed_model_coefficients[2], optimzed_model_coefficients[3]);

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
        inliers_ptr->indices = inliers;
        extract.setIndices(inliers_ptr);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*whiteboard_cloud);
        RCLCPP_INFO(this->get_logger(), "PointCloud2 representing the planar component: '%lu' data points.", whiteboard_cloud->points.size());
        
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*filtered_out_cloud);

        // Create a Concave Hull representation of the projected inliers
        /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConcaveHull<pcl::PointXYZ> chull;
        chull.setInputCloud (whiteboard_cloud);
        chull.setAlpha (0.1);
        chull.reconstruct (*cloud_hull);*/

        computeWhiteboardFeatures(whiteboard_cloud, optimzed_model_coefficients);

        this->publishPointCloud(plane_pub_, *whiteboard_cloud);
        this->publishPointCloud(plane_seg_pub_, *filtered_out_cloud);
        //this->publishPointCloud(chull_pub_, *cloud_hull);
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

    void computeWhiteboardFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                   const Eigen::VectorXf & model_coefficients){

        Eigen::Vector3f whiteboard_normal = Eigen::Vector3f(model_coefficients[0], model_coefficients[1], model_coefficients[2]);
        whiteboard_normal = whiteboard_normal.normalized(); // Unit vector
        if (whiteboard_normal[0] > 0) {
            whiteboard_normal = -whiteboard_normal;
        }
        
        // If the whiteboard is horizontal, ensure avg_normal points upwards (near (0,0,1))
        if (std::abs(whiteboard_normal[0]) < 0.3 && std::abs(whiteboard_normal[1]) < 0.3) {
            // Normal is close to vertical, check direction
            if (whiteboard_normal[2] < 0) {
            whiteboard_normal = -whiteboard_normal;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Average normal: [%f, %f, %f]", whiteboard_normal[0], whiteboard_normal[1], whiteboard_normal[2]);

        // Assume the whiteboard's normal is its "up" (Z) direction
        Eigen::Vector3f world_z(0, 0, 1);

        Eigen::Vector3f rotation_axis = world_z.cross(whiteboard_normal);
        RCLCPP_INFO(this->get_logger(), "Rotation axis: [%f, %f, %f]", rotation_axis[0], rotation_axis[1], rotation_axis[2]);
        float rotation_angle = std::acos(world_z.dot(whiteboard_normal) / (world_z.norm() * whiteboard_normal.norm()));
        RCLCPP_INFO(this->get_logger(), "Rotation angle: %f", rotation_angle);
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
