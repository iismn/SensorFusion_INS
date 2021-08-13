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

typedef pcl::PointXYZI PointTypeI;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointXYZ PointType_NI;

enum class SensorType { VELODYNE, OUSTER };

class ParamServer
{
public:

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic_L;
    string pointCloudTopic_M;
    string pointCloudTopic_R;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

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

    // voxel filter paprams
    float odometryLeafSize;
    float mappingLeafSize ;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;


    // global map visualization radius
    float TransformationEpsilon;
    float Resolution;
    float MaximumIterations;
    float MaxCorrespondenceDistance;


    // Vehicle parameters
    int DESKEW_FLAG;
    int MULTI_LIDAR;
    int ONLINE_CALIBRATION;
    float xT, yT, zT, rollT, pitchT, yawT;
    float xR, yR, zR, rollR, pitchR, yawR;
    float xL, yL, zL, rollL, pitchL, yawL;


    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "pharos");

        nh.param<std::string>("VGICP_SAM/pointCloudTopic_L", pointCloudTopic_L, "vlp_l/velodyne_points");
        nh.param<std::string>("VGICP_SAM/pointCloudTopic_M", pointCloudTopic_M, "vlp_t/velodyne_points");
        nh.param<std::string>("VGICP_SAM/pointCloudTopic_R", pointCloudTopic_R, "vlp_r/velodyne_points");

        nh.param<std::string>("VGICP_SAM/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("VGICP_SAM/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("VGICP_SAM/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("VGICP_SAM/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("VGICP_SAM/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("VGICP_SAM/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("VGICP_SAM/mapFrame", mapFrame, "map");

        nh.param<bool>("VGICP_SAM/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("VGICP_SAM/useGpsElevation", useGpsElevation, false);
        nh.param<float>("VGICP_SAM/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("VGICP_SAM/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("VGICP_SAM/savePCD", savePCD, false);
        nh.param<std::string>("VGICP_SAM/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        nh.param<std::string>("VGICP_SAM/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("VGICP_SAM/N_SCAN", N_SCAN, 16);
        nh.param<int>("VGICP_SAM/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("VGICP_SAM/downsampleRate", downsampleRate, 1);
        nh.param<float>("VGICP_SAM/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("VGICP_SAM/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("VGICP_SAM/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("VGICP_SAM/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("VGICP_SAM/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("VGICP_SAM/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("VGICP_SAM/imuGravity", imuGravity, 9.80511);
        nh.param<float>("VGICP_SAM/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("VGICP_SAM/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("VGICP_SAM/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("VGICP_SAM/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("VGICP_SAM/odometryLeafSize", odometryLeafSize, 0.2);
        nh.param<float>("VGICP_SAM/mappingLeafSize", mappingLeafSize, 0.2);

        nh.param<float>("VGICP_SAM/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("VGICP_SAM/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("VGICP_SAM/numberOfCores", numberOfCores, 8);
        nh.param<double>("VGICP_SAM/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("VGICP_SAM/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("VGICP_SAM/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("VGICP_SAM/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("VGICP_SAM/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("VGICP_SAM/loopClosureEnableFlag", loopClosureEnableFlag, true);
        nh.param<float>("VGICP_SAM/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("VGICP_SAM/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("VGICP_SAM/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("VGICP_SAM/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("VGICP_SAM/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("VGICP_SAM/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("VGICP_SAM/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("VGICP_SAM/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("VGICP_SAM/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        nh.param<float>("VGICP_SAM/TransformationEpsilon", TransformationEpsilon, 0.1);
        nh.param<float>("VGICP_SAM/Resolution", Resolution, 0.5);
        nh.param<float>("VGICP_SAM/MaximumIterations", MaximumIterations, 30);
        nh.param<float>("VGICP_SAM/MaxCorrespondenceDistance", MaxCorrespondenceDistance, 1.0);

        nh.param<int>("/VGICP_SAM/MultiLidarEnableFlag", MULTI_LIDAR, 0);
        nh.param<int>("/VGICP_SAM/MultiLidarOnlineCalibration", ONLINE_CALIBRATION, 0);

        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_LEFT/xL" , xL, 1.43);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_LEFT/yL" , yL, 0.457);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_LEFT/zL" , zL, 0.9195);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_LEFT/rollL" , rollL, -14.89);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_LEFT/pitchL" , pitchL, -0.617389);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_LEFT/yawL" , yawL, 0.353);

        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_TOP/xT" , xT, 1.43);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_TOP/yT" , yT, 0.0);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_TOP/zT" , zT, 1.1);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_TOP/rollT" , rollT, 0.0);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_TOP/pitchT" , pitchT, 0.0);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_TOP/yawT" , yawT, 0.0);

        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_RIGHT/xR" , xR, 1.43);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_RIGHT/yR" , yR, -0.453);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_RIGHT/zR" , zR, 0.9316);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_RIGHT/rollR" , rollR, 15.36);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_RIGHT/pitchR" , pitchR, 0.117);
        nh.param<float>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/VLP16_RIGHT/yawR" , yawR, 0.3054);

        nh.param<int>("/PHAROS_AGV/SENSOR_CONFIG/LiDAR/DESKEW_FLAG", DESKEW_FLAG, 0);

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


sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointTypeI>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

sensor_msgs::PointCloud2 publishPose3D(ros::Publisher *thisPub, pcl::PointCloud<PointTypeI>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

nav_msgs::Odometry publishPose6D(ros::Publisher *thisPub, nav_msgs::Odometry KFodometry, ros::Time thisStamp, std::string thisFrame)
{
    KFodometry.header.stamp = thisStamp;
    KFodometry.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(KFodometry);
    return KFodometry;
}

sensor_msgs::PointCloud2 publishCloud_NI(ros::Publisher *thisPub, pcl::PointCloud<PointType_NI>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointTypeI p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointTypeI p1, PointTypeI p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
