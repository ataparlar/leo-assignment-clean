#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

using namespace std::chrono_literals;



class MainClass : public rclcpp::Node
{
public:
    std::string pcd_file1;
    std::string pcd_file2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_aligned;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pc_publisher_error;

    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointSource = pcl::PointXYZ;
    using PointTarget = pcl::PointXYZ;
    using PointT = pcl::PointXYZ;
    



    pcl::PointCloud<pcl::PointXYZ>::Ptr file_loader(std::string file)
    // Loads a .pcd file and returns a cloud
    {
        this->declare_parameter<std::string>(file, "default"); //"pcd_file1"

        this->get_parameter(file, pcd_file1); //"pcd_file1"

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile<pcl::PointXYZ>(this->pcd_file1, *cloud);

        return cloud;
    };






    MainClass()
        : Node("point_cloud_registering")
    {
        // publishers

        pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points1", 10);
        pc_publisher_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points2", 10);
        pc_publisher_aligned = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_aligned", 10);
        pc_publisher_error = this->create_publisher<std_msgs::msg::Float64>("/points_error", 1);

        sensor_msgs::msg::PointCloud2 ros_cloud1;
        sensor_msgs::msg::PointCloud2 ros_cloud2;
        sensor_msgs::msg::PointCloud2 aligned_ros;
        std_msgs::msg::Float64 score;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1 = file_loader("pcd_file1");
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2 = file_loader("pcd_file2");

        pcl::toROSMsg(*pcl_cloud1, ros_cloud1);
        pcl::toROSMsg(*pcl_cloud2, ros_cloud2);

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded "
                                                   << pcl_cloud1->width * pcl_cloud1->height
                                                   << " data points from 1.pcd with the following fields: ");
        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded "
                                                   << pcl_cloud2->width * pcl_cloud2->height
                                                   << " data points from 2.pcd with the following fields: ");






        // gicp part

        //pcl::PointCloud<PointSource>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointCloud<PointSource>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

        cloud1 = pcl_cloud1;
        cloud2 = pcl_cloud2;

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

        gicp.setTransformationEpsilon(1e-8);
        gicp.setMaximumIterations(5);

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        gicp.setInputSource(cloud2);
        gicp.setInputTarget(cloud1);
        gicp.align(*aligned_cloud);

        Eigen::Matrix4f src2tgt   = gicp.getFinalTransformation();
        score.data                = gicp.getFitnessScore();
        bool is_converged         = gicp.hasConverged();

        RCLCPP_INFO_STREAM(this->get_logger(), "transform matrix: " << "\n" << src2tgt << "\n"
                                            << "fitness score: " << score.data << "\n"
                                            << "is converged: " << is_converged << "\n");

        pcl::toROSMsg(*aligned_cloud, aligned_ros);

        pcl::io::savePCDFileASCII("test_pcd.pcd", *aligned_cloud);




        while (rclcpp::ok())
        {
            ros_cloud1.header.frame_id = "map";
            ros_cloud1.header.stamp = this->get_clock()->now();

            ros_cloud2.header.frame_id = "map";
            ros_cloud2.header.stamp = this->get_clock()->now();

            aligned_ros.header.frame_id = "map";
            aligned_ros.header.stamp = this->get_clock()->now();

            pc_publisher_->publish(ros_cloud1);
            pc_publisher_2->publish(ros_cloud2);
            pc_publisher_aligned->publish(aligned_ros);
            pc_publisher_error->publish(score);
        }

    };
};




int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainClass>());

    rclcpp::shutdown();

    return (0);
}