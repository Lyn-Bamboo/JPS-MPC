#include "gurobi_c++.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include "Eigen/Dense"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>

// MPC config
int N = 7;
double h = 0.1;
double vsamp = 3.5;
double g = 9.81;

double posmin[3] = {-10, -10, 0};
double posmax[3] = {10, 10, 5};
double vmax = 100;
double amax[3] = {2 * g, 2 * g, g};
double jmax = 40;
double Rx = 200, RN = 100, Ru = 0.01, D = 1;
double thresh_dist = 0.4;
double eps = 0.2;

ros::Subscriber mpc_path_sub, jps_path_sub;

ros::Publisher mpc_path_pub;

std::vector<Eigen::Vector3d> ref_nodes;

// 记录
std::vector<Eigen::Vector3d> pos_nodes;
std::vector<Eigen::Vector3d> v_nodes;

// SC
vec_Vec3f obs;
vec_Vec3f jps_path;
bool flag_jps_path, flag_map;

ros::Subscriber map_sub;

ros::Publisher jps_path_pub;
ros::Publisher es_pub;
ros::Publisher poly_pub;

std::vector<MatDNf<3>> AVec;
std::vector<VecDf> bVec;

bool check_arrived(double *pos, double *goal);
bool check_update(double *pos, double *ref);
bool check_update(double *pos_0, double *ref_0, double *pos_n, double *ref_n);

// Set Obs
void MapCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

// Get JPS and Ref
void JPSPathCallback(const visualization_msgs::Marker::ConstPtr &markerMsg);
std::vector<Eigen::Vector3d> GetRefPath(vec_Vec3f jps_nodes);

// MPCSolver
void MPCSolver(std::vector<Eigen::Vector3d> ref_nodes);
void visPosNodes(std::vector<Eigen::Vector3d> nodes);
void MpcPathCallback(const visualization_msgs::Marker::ConstPtr &markerMsg);

// Set Obs
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

void JPSPathCallback(const visualization_msgs::Marker::ConstPtr &markerMsg)
{
    // Get JPS
    jps_path.resize(markerMsg->points.size());
    for (int i = 0; i < int(markerMsg->points.size()); i++)
    {
        jps_path[i](0) = markerMsg->points[i].x;
        jps_path[i](1) = markerMsg->points[i].y;
        jps_path[i](2) = markerMsg->points[i].z;
    }
    ROS_INFO("NUMBER OF JPS POINTS IS %ld", jps_path.size());

    nav_msgs::Path jps_path_msg = DecompROS::vec_to_path(jps_path);
    jps_path_msg.header.frame_id = "world";
    jps_path_pub.publish(jps_path_msg);

    // Get Ref
    ref_nodes = GetRefPath(jps_path);

    flag_jps_path = true;
}

std::vector<Eigen::Vector3d> GetRefPath(vec_Vec3f jps_nodes)
{
    std::vector<Eigen::Vector3d> ref_nodes;
    Eigen::Vector3d start, end, coord;

    for (int i = 0; i < int(jps_nodes.size() - 1); ++i)
    {
        start = jps_nodes[i];
        end = jps_nodes[i + 1];
        double dx = end(0) - start(0);
        double dy = end(1) - start(1);
        double dz = end(2) - start(2);
        double segment_length = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));

        // 计算分段上等间距采样点数量
        int num_samples = std::max(1, static_cast<int>(segment_length / (vsamp * h)));

        // 计算分段上等间距采样点
        for (int j = 0; j <= num_samples; ++j)
        {
            coord(0) = start(0) + dx * j / num_samples;
            coord(1) = start(1) + dy * j / num_samples;
            coord(2) = start(2) + dz * j / num_samples;
            ref_nodes.push_back(coord);
        }
    }
    ref_nodes.push_back(jps_nodes.back());

    return ref_nodes;
}

bool check_arrived(double *pos, double *goal)
{
    double dist = std::sqrt(std::pow(pos[0] - goal[0], 2) + std::pow(pos[1] - goal[1], 2) + std::pow(pos[2] - goal[2], 2));
    // std::cout << "goal_dist = " << dist << std::endl;
    return (dist < eps);
}

bool check_update(double *pos, double *ref)
{
    double dist = std::sqrt(std::pow(pos[0] - ref[0], 2) + std::pow(pos[1] - ref[1], 2) + std::pow(pos[2] - ref[2], 2));
    return (dist < thresh_dist);
}
bool check_update(double *pos_0, double *ref_0, double *pos_n, double *ref_n)
{
    double dist1 = std::sqrt(std::pow(pos_0[0] - ref_0[0], 2) + std::pow(pos_0[1] - ref_0[1], 2) + std::pow(pos_0[2] - ref_0[2], 2));
    double dist2 = std::sqrt(std::pow(pos_n[0] - ref_n[0], 2) + std::pow(pos_n[1] - ref_n[1], 2) + std::pow(pos_n[2] - ref_n[2], 2));
    return (dist1 < thresh_dist || dist2 < thresh_dist);
}

void MPCSolver(std::vector<Eigen::Vector3d> ref_nodes)
{
    // 初始化起止点、参考点
    double pos[3] = {0, 0, 0};
    double v[3] = {0, 0, 0};
    double a[3] = {0, 0, 0};
    double jerk[3] = {0, 0, 0};
    double goal[3] = {3, 3, 1};

    double posN[N][3] = {0};
    double vN[N][3] = {0};
    double aN[N][3] = {0};
    double jerkN[N][3] = {0};

    pos_nodes.clear();
    v_nodes.clear();

    int refNum = ref_nodes.size();
    double refposN[N][3];
    for (int i = 0; i < N; i++)
    {
        refposN[i][0] = ref_nodes[i](0);
        refposN[i][1] = ref_nodes[i](1);
        refposN[i][2] = ref_nodes[i](2);
    }
    pos[0] = ref_nodes[0](0);
    pos[1] = ref_nodes[0](1);
    pos[2] = ref_nodes[0](2);
    goal[0] = ref_nodes[refNum - 1](0);
    goal[1] = ref_nodes[refNum - 1](1);
    goal[2] = ref_nodes[refNum - 1](2);

    // Creat SC
    // Using ellipsoid decomposition
    ROS_INFO("SC START!!!!!!!!!!!!!!!!!!!!!!");

    EllipsoidDecomp3D decomp_util;
    decomp_util.set_obs(obs);
    decomp_util.set_local_bbox(Vec3f(1, 2, 1));
    decomp_util.dilate(jps_path); // Set max iteration number of 10, do fix the path

    // Publish visualization msgs
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
    es_msg.header.frame_id = "world";
    es_pub.publish(es_msg);

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
    poly_msg.header.frame_id = "world";
    poly_pub.publish(poly_msg);

    // Convert to inequality constraints Ax < b
    AVec.clear();
    bVec.clear();
    auto polys = decomp_util.get_polyhedrons();
    for (size_t i = 0; i < jps_path.size() - 1; i++)
    {
        const auto pt_inside = (jps_path[i] + jps_path[i + 1]) / 2;
        LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
        AVec.push_back(cs.A());
        bVec.push_back(cs.b());
        // printf("i: %zu\n", i);
        // std::cout << "A: " << cs.A() << std::endl;
        // std::cout << "b: " << cs.b() << std::endl;
        // std::cout << "point: " << jps_path[i].transpose();
        // if (cs.inside(jps_path[i]))
        //     std::cout << " is inside!" << std::endl;
        // else
        //     std::cout << " is outside!" << std::endl;

        // std::cout << "point: " << jps_path[i + 1].transpose();
        // if (cs.inside(jps_path[i + 1]))
        //     std::cout << " is inside!" << std::endl;
        // else
        //     std::cout << " is outside!" << std::endl;
    }
    ROS_INFO("AVec SIZE IS %ld", AVec.size());

    int sc_current_idx = 0;
    int scNum = polys.size();

    ROS_INFO("SC NUM IS %d", scNum);

    // MPC
    int num = 0, iteration = 0, fail = 0;
    while (!check_arrived(pos, goal) && num < 100)
    {

        // std::cout << "---------------------------------------------------------------------------------------" << std::endl;
        // ROS_INFO("NUM = %d, ITERATION = %d", num, iteration);

        // Solve MPC Problem
        try
        {

            // Create an environment
            GRBEnv env = GRBEnv(true);
            env.set("LogFile", "mpc1.log");
            env.start();

            // Create an empty model
            GRBModel model = GRBModel(env);
            model.getEnv().set(GRB_IntParam_OutputFlag, 0);

            // Create variables
            GRBVar posNvar[N][3];
            GRBVar vNvar[N][3];
            GRBVar aNvar[N][3];
            GRBVar jerkNvar[N][3];
            GRBVar m[N - 1][scNum];
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    posNvar[i][j] = model.addVar(posmin[j], posmax[j], 0.0, GRB_CONTINUOUS, "posNvar_" + std::to_string(i) + "_" + std::to_string(j));
                    vNvar[i][j] = model.addVar(-vmax, vmax, 0.0, GRB_CONTINUOUS, "vNvar_" + std::to_string(i) + "_" + std::to_string(j));
                    aNvar[i][j] = model.addVar(-amax[j], amax[j], 0.0, GRB_CONTINUOUS, "aNvar_" + std::to_string(i) + "_" + std::to_string(j));
                    jerkNvar[i][j] = model.addVar(-jmax, jmax, 0.0, GRB_CONTINUOUS, "jerkNvar_" + std::to_string(i) + "_" + std::to_string(j));
                }
            }
            for (int i = 0; i < N - 1; i++)
            {
                for (int j = 0; j < scNum; j++)
                {
                    m[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "m_" + std::to_string(i) + "_" + std::to_string(j));
                }
            }

            // Set objective: obj = Jx + Ju + JN;
            // 设置目标函数
            GRBQuadExpr obj = 0;
            GRBQuadExpr Jx = 0;
            GRBQuadExpr Ju = 0;
            GRBQuadExpr JN = 0;
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Jx += (posNvar[i][j] - refposN[i][j]) * Rx * (posNvar[i][j] - refposN[i][j]);
                    Ju += jerkNvar[i][j] * Ru * jerkNvar[i][j];
                }
            }
            for (int j = 0; j < 3; j++)
            {
                JN += (posNvar[N - 1][j] - refposN[N - 1][j]) * RN * (posNvar[N - 1][j] - refposN[N - 1][j]);
            }
            // 设置为简化的目标函数
            obj = Jx + Ju + JN;
            model.setObjective(obj, GRB_MINIMIZE);

            // Add Constrains 1
            // Xk+1 = A*Xk + B
            // N = 1
            for (int j = 0; j < 3; j++)
            {
                model.addConstr(posNvar[0][j] == pos[j] + h * v[j]);
                // model.addConstr(vNvar[0][j] == v[j] + h * (a[j] - D * v[j]));
                model.addConstr(vNvar[0][j] == v[j] + h * a[j]);
                model.addConstr(aNvar[0][j] == a[j] + h * jerk[j]);
            }
            // N > 1
            for (int i = 1; i < N; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    model.addConstr(posNvar[i][j] == posNvar[i - 1][j] + h * vNvar[i - 1][j]);
                    // model.addConstr(vNvar[i][j] == vNvar[i-1][j] + h * (aNvar[i-1][j] - D * vNvar[i-1][j]));
                    model.addConstr(vNvar[i][j] == vNvar[i - 1][j] + h * aNvar[i - 1][j]);
                    model.addConstr(aNvar[i][j] == aNvar[i - 1][j] + h * jerkNvar[i - 1][j]);
                }
            }

            // Add Constrains 2
            // Ai·pk<ci; Ai·pk+1<ci;
            // LinearConstraint3D cs(pos_nodes.back(), polys[sc_current_idx].hyperplanes());
            for (int i = 0; i < N - 1; i++)
            {
                GRBLinExpr expr = 0;
                for (int j = 0; j < scNum; j++)
                {
                    for (int k = 0; k < bVec[j].size(); k++)
                    {
                        model.addGenConstrIndicator(m[i][j], 1,
                                                    posNvar[i][0] * AVec[j](k, 0) + posNvar[i][1] * AVec[j](k, 1) + posNvar[i][2] * AVec[j](k, 2) <= bVec[j](k));
                        model.addGenConstrIndicator(m[i][j], 1,
                                                    posNvar[i + 1][0] * AVec[j](k, 0) + posNvar[i + 1][1] * AVec[j](k, 1) + posNvar[i + 1][2] * AVec[j](k, 2) <= bVec[j](k));
                    }
                    expr += m[i][j];
                }
                model.addConstr(expr >= 1);
            }

            // Optimize model
            model.optimize();

            // // print value of variables
            std::cout << "NUM = " << num << "  ITERATION = " << iteration << std::endl;
            // for (int i = 0; i < scNum; i++)
            // {
            //     std::cout << m[0][i].get(GRB_StringAttr_VarName) << " "
            //               << m[0][i].get(GRB_DoubleAttr_X) << std::endl;
            // }

            // std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl
            //           << std::endl
            //           << std::endl
            //           << std::endl;

            // Update
            // update x0
            double pos_n[3];
            for (int j = 0; j < 3; j++)
            {
                pos[j] = posNvar[0][j].get(GRB_DoubleAttr_X);
                v[j] = vNvar[0][j].get(GRB_DoubleAttr_X);
                a[j] = aNvar[0][j].get(GRB_DoubleAttr_X);
                jerk[j] = jerkNvar[0][j].get(GRB_DoubleAttr_X);
                pos_n[j] = posNvar[N - 1][j].get(GRB_DoubleAttr_X);
            }
            // whether close enough
            // if (check_update(pos, refposN[0]))
            if (check_update(pos, refposN[0]), pos_n, refposN[N - 1])
            {
                fail = 0;
                iteration++;
                // update reference
                for (int i = 0; i < N; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (i + iteration < refNum)
                        {
                            refposN[i][j] = ref_nodes[i + iteration](j);
                        }
                        else
                        {
                            refposN[i][j] = ref_nodes[refNum - 1](j);
                        }
                    }
                }
                // update xN
                for (int i = 0; i < N; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        posN[i][j] = posNvar[i][j].get(GRB_DoubleAttr_X);
                        vN[i][j] = vNvar[i][j].get(GRB_DoubleAttr_X);
                        aN[i][j] = aNvar[i][j].get(GRB_DoubleAttr_X);
                        jerkN[i][j] = jerkNvar[i][j].get(GRB_DoubleAttr_X);
                    }
                }
            }
            else // update x0 = x1...
            {
                fail++;
                if (fail >= N)
                {
                    ROS_INFO("Solution Failure");
                    break;
                }
                ROS_INFO("Fail %d times", fail);
                for (int i = 0; i < 3; i++)
                {
                    pos[i] = posN[fail][i];
                    v[i] = vN[fail][i];
                    a[i] = aN[fail][i];
                    jerk[i] = jerkN[fail][i];
                }
            }

            // Record
            Eigen::Vector3d coord;
            coord(0) = pos[0];
            coord(1) = pos[1];
            coord(2) = pos[2];
            pos_nodes.push_back(coord);
            num++;
        }
        catch (GRBException e)
        {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
        }
        catch (...)
        {
            std::cout << "Exception during optimization" << std::endl;
        }

        // ROS_INFO("NUM = %d, ITERATION = %d", num, iteration);
        // ROS_INFO("x = %f, y = %f, z = %f", pos[0], pos[1], pos[2]);
        // ROS_INFO("vx = %f, vy = %f, vz = %f", v[0], v[1], v[2]);
        // ROS_INFO("ax = %f, ay = %f, az = %f", a[0], a[1], a[2]);
        // ROS_INFO("jx = %f, jy = %f, jz = %f", jerk[0], jerk[1], jerk[2]);
    }

    if (check_arrived(pos, goal))
    {
        ROS_INFO("Arrive Goal Successfully !!!");
    }
    else
    {
        ROS_INFO("Planning Failure !");
    }

    std::cout << "ref_pos:" << refNum << std::endl;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < refNum; j++)
        {
            std::cout << ref_nodes[j](i) << "  ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    std::cout << "mpc_pos:" << int(pos_nodes.size()) << std::endl;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < int(pos_nodes.size()); j++)
        {
            std::cout << pos_nodes[j](i) << "  ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    visPosNodes(pos_nodes);
}

void visPosNodes(std::vector<Eigen::Vector3d> nodes)
{

    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "mpc/mpc_path_nodes";
    // node_vis.type = visualization_msgs::Marker::SPHERE;
    node_vis.type = visualization_msgs::Marker::LINE_STRIP;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = 0.1;
    // node_vis.scale.y = 0.1;
    // node_vis.scale.z = 0.1;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        // std::cout << "i = " << i << ": x:" << pt.x << "  y:" << pt.y << " z:" << pt.z << std::endl;

        node_vis.points.push_back(pt);
    }

    mpc_path_pub.publish(node_vis);
}

void MpcPathCallback(const visualization_msgs::Marker::ConstPtr &markerMsg)
{
    ROS_INFO("!!! Received Marker message !!!  points num = %ld", markerMsg->points.size());
}

int main(int argc, char *argv[])
{
    // 创建ros节点
    ros::init(argc, argv, "mpc");
    ros::NodeHandle nh("~");

    map_sub = nh.subscribe("/random_complex/global_map", 1, MapCallback);
    mpc_path_sub = nh.subscribe("mpc_path_vis", 10, MpcPathCallback);
    jps_path_sub = nh.subscribe("/demo_node/grid_path_vis", 10, JPSPathCallback);

    jps_path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
    es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    mpc_path_pub = nh.advertise<visualization_msgs::Marker>("mpc_path_vis", 1);

    flag_map = false;
    flag_jps_path = false;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (flag_map && flag_jps_path)
        {
            MPCSolver(ref_nodes);
            flag_jps_path = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
