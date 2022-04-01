#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <kml/base/file.h>
#include <kml/engine.h>
#include <kml/dom.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>

typedef std::vector<kmldom::LineStringPtr> LineStringVector;

class KmlClass : public rclcpp::Node
{
public:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    nav_msgs::msg::Path kml_path;
    geometry_msgs::msg::PoseStamped kml_pose;
    geometry_msgs::msg::PoseArray pose_array;

    std::string kmlfile;
    std::string file_data;
    kmlbase::File kmlbase;
    std::string kml;
    std::string errors;

    kmlengine::KmlFilePtr kml_file;
    kmldom::FeaturePtr root;
    kmldom::KmlDomType type;
    kmldom::ContainerPtr cont;
    kmldom::FeaturePtr inner_feature;
    kmldom::PlacemarkPtr placemark;
    kmldom::GeometryPtr geometry;
    kmldom::LineStringPtr line_string;
    kmldom::CoordinatesPtr coordinates;

    LineStringVector line_string_vector;

    GeographicLib::LocalCartesian locart;
    double x, y, z;
    double init_x, init_y, init_z;

    void line_string_extract(const kmlengine::KmlFilePtr &kml_file, LineStringVector *line_string_vector)
    // Gets the string line in the kml file and extracts the data that needed.
    {
        // make a placemark for get the data
        placemark = kmldom::AsPlacemark(kml_file->GetObjectById("0C11468A89207A6E17BE"));

        // get the geometry
        geometry = placemark->get_geometry();

        // make a string the geometry
        line_string = kmldom::AsLineString(geometry);

        // push
        line_string_vector->push_back(line_string);
    }

    KmlClass()
        : Node("kml_parser")
    {
        // publishers
        path_publisher = this->create_publisher<nav_msgs::msg::Path>("/kml_path", 1);

        this->declare_parameter<std::string>("kml_file", "default");

        this->get_parameter("kml_file", kmlfile);
        // RCLCPP_INFO_STREAM(this->get_logger(), kmlfile.size );

        if (!kmlbase::File::ReadFileToString(kmlfile, &file_data))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), kmlfile << " read failed");
        }

        // get the file
        kml_file = kmlengine::KmlFile::CreateFromParse(file_data, &errors);
        if (!kml_file)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), errors);
        }

        line_string_extract(kml_file, &line_string_vector);

        GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

        //[line_string_vector.size()];
        kml_path.poses = std::vector<geometry_msgs::msg::PoseStamped>();
        int array_count = 0;

        coordinates = line_string_vector[0]->get_coordinates();
        kmlbase::Vec3 coord = coordinates->get_coordinates_array_at(0);
        GeographicLib::LocalCartesian locart(coord.get_latitude(), coord.get_longitude(), coord.get_altitude(), earth);
                RCLCPP_INFO_STREAM(this->get_logger(), "Lat: " << coord.get_latitude()
                                                               << " Lng: " << coord.get_longitude());

        for (uint i = 0; i < line_string_vector.size(); i++)
        {
            line_string = line_string_vector[i];
            coordinates = line_string->get_coordinates();
            for (uint j = 0; j < coordinates->get_coordinates_array_size(); j++)
            {
                kmlbase::Vec3 coord = coordinates->get_coordinates_array_at(j);
                // RCLCPP_INFO_STREAM(this->get_logger(), "Lat: " << coord.get_latitude()
                //                                                << " Lng: " << coord.get_longitude());

                locart.Forward(coord.get_latitude(), coord.get_longitude(), coord.get_altitude(), x, y, z);
                // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << j << " " << x
                //                                              << " y: " << y
                //                                              << " z: " << z);
                // RCLCPP_INFO_STREAM(this->get_logger(), "init_x: " << init_x
                //                                                   << " init_y: " << init_y
                //                                                   << " init_z: " << init_z);
                kml_pose.pose.position.x = x;
                kml_pose.pose.position.y = y;
                kml_pose.pose.position.z = z;

                kml_pose.pose.orientation.x = 0;
                kml_pose.pose.orientation.y = 0;
                kml_pose.pose.orientation.z = 0;
                kml_pose.pose.orientation.w = 1;

                kml_path.poses.push_back(kml_pose);
                array_count++;
            }
        }

        while (rclcpp::ok())
        {
            for (geometry_msgs::msg::PoseStamped &pose : kml_path.poses)
            {
                pose.header.frame_id = "map";
                pose.header.stamp = this->get_clock()->now();
            }
            kml_path.header.frame_id = "map";
            kml_path.header.stamp = this->get_clock()->now();

            path_publisher->publish(kml_path);
        }
    };
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KmlClass>());

    rclcpp::shutdown();

    return (0);
}