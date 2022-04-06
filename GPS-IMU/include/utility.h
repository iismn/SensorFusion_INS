#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <utility>
#include <functional>

using namespace std;

class ParamServer
{
public:

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    //Topics
    string IMU_topic;
    string GPS_topic;

    //Frames
    string baselinkFrame;
    string odometryFrame;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // Sensor RESET
    double IMU_HZ;              // IMU _ TIME DATA dt
    double GPS_HZ;              // GPS _ TIME DATA dt
    double Covariance_GAIN;     // Covariance GAIN
    int FILTER_UNSYNC;          // GPS _ TIME DATA FILTER
    double ALT_RATIO;
    ParamServer()
    {
        nh.param<std::string>("AGV_GPSwINS/IMU_topic", IMU_topic, "imu/data");
        nh.param<std::string>("AGV_GPSwINS/GPS_topic", GPS_topic, "gps/fix");

        nh.param<std::string>("AGV_GPSwINS/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("AGV_GPSwINS/odometryFrame", odometryFrame, "odom");

        nh.param<float>("AGV_GPSwINS/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("AGV_GPSwINS/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("AGV_GPSwINS/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("AGV_GPSwINS/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("AGV_GPSwINS/imuGravity", imuGravity, 9.80511);
        nh.param<float>("AGV_GPSwINS/imuRPYWeight", imuRPYWeight, 0.01);

        nh.param<double>("AGV_GPSwINS/IMU_HZ", IMU_HZ, 100);
        nh.param<double>("AGV_GPSwINS/GPS_HZ", GPS_HZ, 1);
        nh.param<int>("AGV_GPSwINS/FILTER_UNSYNC", FILTER_UNSYNC, 1);
        nh.param<double>("AGV_GPSwINS/ALTITUDE_RATIO", ALT_RATIO, 1);
        nh.param<double>("AGV_GPSwINS/Covariance_GAIN", Covariance_GAIN, 1);

        nh.param<vector<double>>("AGV_GPSwINS/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("AGV_GPSwINS/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("AGV_GPSwINS/extrinsicTrans", extTransV, vector<double>());

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


#endif
