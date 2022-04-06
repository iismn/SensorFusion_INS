#include "utility.h"
#include "navsat_conversions.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
using namespace gtsam;
using namespace std;

using gtsam::symbol_shorthand::X; // Pose       (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Velocity   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias       (ax,ay,az,gx,gy,gz)

class SensorFusion_GPSINS : public ParamServer
{

public:
    std::mutex mtx;

    // GTSAM OPTIMIZER
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Values LM_CurrentEstimate;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_Nominal;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_Reset;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::Pose3 gpsPose;
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState ODOM_Prv;
    gtsam::imuBias::ConstantBias BIAS_Prv;

    gtsam::Pose3 gps2IMU = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 IMU2gps = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));
    gtsam::Pose3 GPS_Pose3;

    // DATA CONATINER
    std::deque<nav_msgs::Odometry> gpsQueue;
    std::deque<nav_msgs::Odometry> imuQueue;
    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    // Pre-INTG PARAM
    bool gpsInitialized = false;
    bool systemInitialized = false;
    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    double utm_x_init = 0;
    double utm_y_init = 0;
    double utm_z_init = 0;

    int key = 1;

    double UPDATED_qX = 0;
    double UPDATED_qY = 0;
    double UPDATED_qZ = 0;
    double UPDATED_qW = 0;

    double currentCorrectionTime_Prev = 0;

    // ROS
    nav_msgs::Odometry  INS_Odometry;
    nav_msgs::Odometry  GPS_Odometry;
    nav_msgs::Odometry  INT_Odometry;
    nav_msgs::Path      INS_Path;
    nav_msgs::Path      GPS_Path;

    ros::Subscriber subIMU;
    ros::Subscriber subGPS;
    ros::Publisher  pubINS;
    ros::Publisher  pubGPS;
    ros::Publisher  pubGST;
    ros::Publisher  pubINSPath;
    ros::Publisher  pubGTPath;

    // --------------------------------------------------------------------------------------------------------------------------------------------
    SensorFusion_GPSINS(){

        subIMU =  nh.subscribe<sensor_msgs::Imu>(IMU_topic, 2000, &SensorFusion_GPSINS::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS =  nh.subscribe<sensor_msgs::NavSatFix> (GPS_topic, 200, &SensorFusion_GPSINS::gpsHandler, this, ros::TransportHints().tcpNoDelay());

        pubINS =  nh.advertise<nav_msgs::Odometry>("ublox/ins/odom", 1);
        pubGPS = nh.advertise<nav_msgs::Odometry>("ublox/fix/odom", 1);
        pubGST = nh.advertise<nav_msgs::Odometry>("ublox/fix/init", 1);
        pubINSPath = nh.advertise<nav_msgs::Path>("ublox/ins/fused_path", 1);
        pubGTPath = nh.advertise<nav_msgs::Path>("ublox/ins/gps_path", 1);

        boost::shared_ptr<gtsam::PreintegrationParams> preIntgParam = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        preIntgParam->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2);    // ACC white noise
        preIntgParam->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2);    // Gyro white noise
        preIntgParam->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);           // ERR c- Vel Pos Estimation
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());;  // Initial Bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                              // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);                                                             // 1e-2 ~ 1e-3
        correctionNoise_Nominal = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise_Reset = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                  // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(preIntgParam, prior_imu_bias); // IMU Message Thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(preIntgParam, prior_imu_bias); // IMU Optimization Thread
    }
    // --------------------------------------------------------------------------------------------------------------------------------------------
    void imuHandler(const sensor_msgs::Imu::ConstPtr& IMU_RAW){

        std::lock_guard<std::mutex> lock(mtx);

        // A-1. Transformate IMU Coordinate (ENU)
        sensor_msgs::Imu IMU_CURRENT = imuConverter(*IMU_RAW);
        imuQueOpt.push_back(IMU_CURRENT);
        imuQueImu.push_back(IMU_CURRENT);

        UPDATED_qX = IMU_CURRENT.orientation.x;
        UPDATED_qY = IMU_CURRENT.orientation.y;
        UPDATED_qZ = IMU_CURRENT.orientation.z;
        UPDATED_qW = IMU_CURRENT.orientation.w;

        // RETURN (IMU not INITIALIZED)
        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&IMU_CURRENT);
        double dt = (lastImuT_imu < 0) ? (1.0 / IMU_HZ) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // A-2. GTSAM INTG Single IMU State
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(IMU_CURRENT.linear_acceleration.x, IMU_CURRENT.linear_acceleration.y, IMU_CURRENT.linear_acceleration.z),
                                                gtsam::Vector3(IMU_CURRENT.angular_velocity.x,    IMU_CURRENT.angular_velocity.y,    IMU_CURRENT.angular_velocity.z), dt);

        // A-2. GTSAM Predict IMU State (IMU Preintegration)
        gtsam::NavState ODOM_Now = imuIntegratorImu_->predict(ODOM_Prv, BIAS_Prv);

        gtsam::Pose3 IMU_Pose3 = gtsam::Pose3(ODOM_Now.quaternion(), ODOM_Now.position());
        GPS_Pose3 = IMU_Pose3.compose(gps2IMU);
        // A-4. ROS Publish GTSAM NAV STATE
        INS_Odometry.header.stamp = IMU_CURRENT.header.stamp;
        INS_Odometry.header.frame_id = odometryFrame;
        INS_Odometry.child_frame_id = "odom_imu";

        INS_Odometry.pose.pose.position.x = GPS_Pose3.translation().x();
        INS_Odometry.pose.pose.position.y = GPS_Pose3.translation().y();
        INS_Odometry.pose.pose.position.z = GPS_Pose3.translation().z();
        INS_Odometry.pose.pose.orientation.x = GPS_Pose3.rotation().toQuaternion().x();
        INS_Odometry.pose.pose.orientation.y = GPS_Pose3.rotation().toQuaternion().y();
        INS_Odometry.pose.pose.orientation.z = GPS_Pose3.rotation().toQuaternion().z();
        INS_Odometry.pose.pose.orientation.w = GPS_Pose3.rotation().toQuaternion().w();

        INS_Odometry.twist.twist.linear.x = ODOM_Now.velocity().x();
        INS_Odometry.twist.twist.linear.y = ODOM_Now.velocity().y();
        INS_Odometry.twist.twist.linear.z = ODOM_Now.velocity().z();
        INS_Odometry.twist.twist.angular.x = IMU_CURRENT.angular_velocity.x + BIAS_Prv.gyroscope().x();
        INS_Odometry.twist.twist.angular.y = IMU_CURRENT.angular_velocity.y + BIAS_Prv.gyroscope().y();
        INS_Odometry.twist.twist.angular.z = IMU_CURRENT.angular_velocity.z + BIAS_Prv.gyroscope().z();

        // pubINS.publish(INS_Odometry);


        INS_Path.header.stamp = IMU_CURRENT.header.stamp;
        INS_Path.header.frame_id = odometryFrame;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = IMU_CURRENT.header.stamp;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = GPS_Pose3.translation().x();
        pose_stamped.pose.position.y = GPS_Pose3.translation().y();
        pose_stamped.pose.position.z = GPS_Pose3.translation().z();
        pose_stamped.pose.orientation.x = GPS_Pose3.rotation().toQuaternion().x();
        pose_stamped.pose.orientation.y = GPS_Pose3.rotation().toQuaternion().y();
        pose_stamped.pose.orientation.z = GPS_Pose3.rotation().toQuaternion().z();
        pose_stamped.pose.orientation.w = GPS_Pose3.rotation().toQuaternion().w();

        INS_Path.poses.push_back(pose_stamped);

        pubINSPath.publish(INS_Path);

    }
    // --------------------------------------------------------------------------------------------------------------------------------------------
    void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& GPS_RAW){

        std::lock_guard<std::mutex> lock(mtx);
        double currentCorrectionTime = ROS_TIME(GPS_RAW);

        if (imuQueOpt.empty())
            return;




        // B-A. CONVERT LATLONG to UTM
        double utm_x = 0;
        double utm_y = 0;
        std::string utm_zone_;
        GeonavTransform::NavsatConversions::LLtoUTM(GPS_RAW->latitude, GPS_RAW->longitude, utm_y, utm_x, utm_zone_);
                                                                      //  NORTH  EAST
        // B-1. COMPILE LiDAR Odom
        float p_x = utm_x;
        float p_y = utm_y;
        float p_z = GPS_RAW->altitude*ALT_RATIO;
        float r_x = UPDATED_qX;
        float r_y = UPDATED_qY;
        float r_z = UPDATED_qZ;
        float r_w = UPDATED_qW;


        if (gpsInitialized == false){
          utm_x_init = p_x;
          utm_y_init = p_y;
          utm_z_init = p_z;
          p_x = p_x - utm_x_init;
          p_y = p_y - utm_y_init;
          p_z = p_z - utm_z_init;
          gpsInitialized = true;
        }else{
          p_x = p_x - utm_x_init;
          p_y = p_y - utm_y_init;
          p_z = p_z - utm_z_init;
        }


        GPS_Path.header.stamp = GPS_RAW->header.stamp;
        GPS_Path.header.frame_id = odometryFrame;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = GPS_RAW->header.stamp;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = p_x;
        pose_stamped.pose.position.y = p_y;
        pose_stamped.pose.position.z = p_z;
        pose_stamped.pose.orientation.x = r_x;
        pose_stamped.pose.orientation.y = r_y;
        pose_stamped.pose.orientation.z = r_z;
        pose_stamped.pose.orientation.w = r_w;

        GPS_Path.poses.push_back(pose_stamped);

        pubGTPath.publish(GPS_Path);

        // A-4. ROS Publish GTSAM NAV STATE
        GPS_Odometry.header.stamp = GPS_RAW->header.stamp;
        GPS_Odometry.header.frame_id = "odom";

        GPS_Odometry.pose.pose.position.x = p_x;
        GPS_Odometry.pose.pose.position.y = p_y;
        GPS_Odometry.pose.pose.position.z = p_z;
        GPS_Odometry.pose.pose.orientation.x = r_x;
        GPS_Odometry.pose.pose.orientation.y = r_y;
        GPS_Odometry.pose.pose.orientation.z = r_z;
        GPS_Odometry.pose.pose.orientation.w = r_w;

        // A-4B. ROS Publish INIT START POINT UTM
        INT_Odometry.header.stamp = GPS_RAW->header.stamp;
        INT_Odometry.header.frame_id = "odom";

        INT_Odometry.pose.pose.position.x = utm_x_init;
        INT_Odometry.pose.pose.position.y = utm_y_init;
        INT_Odometry.pose.pose.position.z = utm_z_init;

        // Use ENU covariance to build XYZRPY covariance
        boost::array<double, 36> covariance = {{
          GPS_RAW->position_covariance[0],
          GPS_RAW->position_covariance[1],
          GPS_RAW->position_covariance[2],
          0, 0, 0,
          GPS_RAW->position_covariance[3],
          GPS_RAW->position_covariance[4],
          GPS_RAW->position_covariance[5],
          0, 0, 0,
          GPS_RAW->position_covariance[6],
          GPS_RAW->position_covariance[7],
          GPS_RAW->position_covariance[8],
          0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0
        }};

        GPS_Odometry.pose.covariance = covariance;

        GPS_Odometry.twist.twist.linear.x = 0;
        GPS_Odometry.twist.twist.linear.y = 0;
        GPS_Odometry.twist.twist.linear.z = 0;
        GPS_Odometry.twist.twist.angular.x = 0;
        GPS_Odometry.twist.twist.angular.y = 0;
        GPS_Odometry.twist.twist.angular.z = 0;

        pubGPS.publish(GPS_Odometry);
        pubGST.publish(INT_Odometry);



        // B-!. Optimize Fail -> RESET GTSAM Optimizer
        if (systemInitialized == false)
        {   
            if (IMU_HZ <= 100){
                ROS_WARN("SAFETY FAILURE : Low IMU Hz / [AT LEAST 300]");
                return;
            }
            resetOptimization();
            currentCorrectionTime_Prev = currentCorrectionTime;

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - GPS_HZ)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // initial pose
            gtsam::Pose3 gpsPose_INIT = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));
            prevPose_ = gpsPose_INIT.compose(gps2IMU);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            systemInitialized = true;

            return;
        }
        else{

          if (currentCorrectionTime-0.2 > currentCorrectionTime_Prev && FILTER_UNSYNC == 1)
            return;


          // cout.precision(15);
          // cout << currentCorrectionTime << endl;
          // cout << "POSE3 : " << p_x << " " << p_y << " " << p_z << endl;
          gpsPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));
        }

        // B-2. RESET Graph : Optimize Performance
        if (key == 100)
        {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }

        // B-3. GTSAM IMU PREINTG + GRAPH : Integrate IMU / Optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - GPS_HZ)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / IMU_HZ) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        // B-3T. ADD IMU Factor to NonlinearFactorGraph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);//!----------------------------------------------------------------------------------------------------------

        // B-3T. ADD IMU BIAS to NonlinearFactorGraph
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));//!------------

        // B-3T. ADD GPS Factor to NonlinearFactorGraph
        gtsam::Vector Vector3(3);
        Vector3 << (float)GPS_RAW->position_covariance[0], (float)GPS_RAW->position_covariance[4], (float)GPS_RAW->position_covariance[8];
        noiseModel::Diagonal::shared_ptr correctionNoise_GPS = noiseModel::Isotropic::Sigmas(Vector3*Covariance_GAIN);

        // correctionNoise_GPS->print("GPS Noise:\n");
        gtsam::Pose3 curPose = gpsPose.compose(gps2IMU);
        // gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, correctionNoise_Nominal);
        // graphFactors.add(pose_factor);
        gtsam::GPSFactor gps_factor(X(key), gtsam::Point3(p_x, p_y, p_z), correctionNoise_GPS);
        graphFactors.add(gps_factor);

        // B-3T. ADD IMU Predicted Value
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());//!------------------------------------------------------------------------------------------
        graphValues.insert(V(key), propState_.v());//!---------------------------------------------------------------------------------------------
        graphValues.insert(B(key), prevBias_);//!--------------------------------------------------------------------------------------------------

        // B-3T. OPTIMIZER
        optimizer.update(graphFactors, graphValues);//!---------------------------------------------------------------------------------
        optimizer.update();//!----------------------------------------------------------------------------------------------------------
        graphFactors.resize(0);
        graphValues.clear();

        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();//!------------------------------------------------------------------------
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // prevBias_.print("Optimized Noise:\n");
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        // check optimization (Optimize Failuer Detection)
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        ODOM_Prv = prevState_;
        BIAS_Prv  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - GPS_HZ)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(BIAS_Prv);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / IMU_HZ) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        // !! -- PERFORMANCE OPTMIZER KEY Threshold -- !!
        ++key;
        doneFirstOpt = true;

        GPS_Odometry.pose.pose.position.x = GPS_Pose3.translation().x();
        GPS_Odometry.pose.pose.position.y = GPS_Pose3.translation().y();
        GPS_Odometry.pose.pose.position.z = GPS_Pose3.translation().z();
        GPS_Odometry.pose.pose.orientation.x = GPS_Pose3.rotation().toQuaternion().x();
        GPS_Odometry.pose.pose.orientation.y = GPS_Pose3.rotation().toQuaternion().y();
        GPS_Odometry.pose.pose.orientation.z = GPS_Pose3.rotation().toQuaternion().z();
        GPS_Odometry.pose.pose.orientation.w = GPS_Pose3.rotation().toQuaternion().w();
        pubINS.publish(GPS_Odometry);

        INS_Path.header.stamp = GPS_RAW->header.stamp;
        INS_Path.header.frame_id = odometryFrame;

        // geometry_msgs::PoseStamped pose_stamped_INS;
        // pose_stamped_INS.header.stamp = GPS_RAW->header.stamp;
        // pose_stamped_INS.header.frame_id = odometryFrame;
        // pose_stamped_INS.pose.position.x = GPS_Pose3.translation().x();
        // pose_stamped_INS.pose.position.y = GPS_Pose3.translation().y();
        // pose_stamped_INS.pose.position.z = GPS_Pose3.translation().z();
        // pose_stamped_INS.pose.orientation.x = GPS_Pose3.rotation().toQuaternion().x();
        // pose_stamped_INS.pose.orientation.y = GPS_Pose3.rotation().toQuaternion().y();
        // pose_stamped_INS.pose.orientation.z = GPS_Pose3.rotation().toQuaternion().z();
        // pose_stamped_INS.pose.orientation.w = GPS_Pose3.rotation().toQuaternion().w();
        //
        // INS_Path.poses.push_back(pose_stamped_INS);
        //
        // pubINSPath.publish(INS_Path);

        currentCorrectionTime_Prev = currentCorrectionTime;


    }
    // --------------------------------------------------------------------------------------------------------------------------------------------
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur){
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("FAILURE DETECTED : Reset System / [Large Velocity]");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("FAILURE DETECTED : Reset System / [Large Bias]");
            return true;
        }

        return false;
    }
    // --------------------------------------------------------------------------------------------------------------------------------------------
    void resetOptimization(){
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }
    // --------------------------------------------------------------------------------------------------------------------------------------------
    void resetParams(){
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "SensorFusion_GPSINS");

    SensorFusion_GPSINS SF;
    ROS_INFO("\033[1;32m----> GNSS + INS FUSION START \033[0m");

    ros::MultiThreadedSpinner spinner(6);
    spinner.spin();

    return 0;
}
