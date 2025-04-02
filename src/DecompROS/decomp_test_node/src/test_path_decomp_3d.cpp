#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

vec_Vec3f obs;
vec_Vec3f path;
bool flag_path, flag_map;

ros::Publisher cloud_pub;
ros::Publisher path_pub;
ros::Publisher es_pub;
ros::Publisher poly_pub;

ros::Subscriber map_sub;
ros::Subscriber jps_sub;

void MapCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    obs.resize(cloud.points.size());

    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        obs[i](0) = cloud.points[i].x;
        obs[i](1) = cloud.points[i].y;
        obs[i](2) = cloud.points[i].z;
    }

    // sensor_msgs::convertPointCloudToPointCloud2(*msg, cloud2);
    // ROS_INFO("NUMBER OF CLOUD2 IS %ld", cloud.points.size());

    flag_map = true;
}

void JPSCallback(const visualization_msgs::Marker::ConstPtr &markerMsg)
{
    path.resize(markerMsg->points.size());

    for (int i = 0; i < int(markerMsg->points.size()); i++)
    {
        path[i](0) = markerMsg->points[i].x;
        path[i](1) = markerMsg->points[i].y;
        path[i](2) = markerMsg->points[i].z;
    }
    ROS_INFO("GET JPS PATH !!!!!!!!!!!!!!!!!!!!!!");

    nav_msgs::Path path_msg = DecompROS::vec_to_path(path);
    path_msg.header.frame_id = "world";
    path_pub.publish(path_msg);

    flag_path = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");

    path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
    es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);

    map_sub = nh.subscribe("/random_complex/global_map", 1, MapCallback);
    jps_sub = nh.subscribe("/demo_node/grid_path_vis", 1, JPSCallback);

    flag_map = false;
    flag_path = false;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (flag_map && flag_path)
        {
            // Using ellipsoid decomposition
            ROS_INFO("SC START!!!!!!!!!!!!!!!!!!!!!!");

            EllipsoidDecomp3D decomp_util;
            decomp_util.set_obs(obs);
            decomp_util.set_local_bbox(Vec3f(1, 2, 1));
            decomp_util.dilate(path); // Set max iteration number of 10, do fix the path

            // Publish visualization msgs
            decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
            es_msg.header.frame_id = "world";
            es_pub.publish(es_msg);

            decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
            poly_msg.header.frame_id = "world";
            poly_pub.publish(poly_msg);

            // Convert to inequality constraints Ax < b
            auto polys = decomp_util.get_polyhedrons();
            for (size_t i = 0; i < path.size() - 1; i++)
            {
                const auto pt_inside = (path[i] + path[i + 1]) / 2;
                LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
                printf("i: %zu\n", i);
                std::cout << "A: " << cs.A() << std::endl;
                std::cout << "b: " << cs.b() << std::endl;
                std::cout << "point: " << path[i].transpose();
                if (cs.inside(path[i]))
                    std::cout << " is inside!" << std::endl;
                else
                    std::cout << " is outside!" << std::endl;

                std::cout << "point: " << path[i + 1].transpose();
                if (cs.inside(path[i + 1]))
                    std::cout << " is inside!" << std::endl;
                else
                    std::cout << " is outside!" << std::endl;
            }
            // flag_map = false;
            flag_path = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
