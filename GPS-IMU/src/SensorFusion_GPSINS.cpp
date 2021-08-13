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


using namespace gtsam;
using namespace std;


struct Position{
  double X;
  double Y;
  double Z;
  double Roll;
  double Pitch;
  double Yaw;
} EIGEN_ALIGN16;

typedef Position ROS_Pose6D;

class LocalMAP_OSM_Localization : public ParamServer
{

public:

    // GTSAM OPTIMIZER
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Values LM_CurrentEstimate;

    LevenbergMarquardtParams LM_params;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_Nominal;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_Reset;
    gtsam::Vector noiseModelBetweenBias;


    // DATA CONATINER
    std::deque<nav_msgs::Odometry> gpsQueue;
    std::deque<nav_msgs::Odometry> imuQueue;
    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    // Pre-INTG PARAM
    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    const double delta_t = 0;

    Eigen::MatrixXd poseCovariance;
    Eigen::Vector3f START_POINT;
    Eigen::Vector3f END_POINT;
    Eigen::Vector3f DISTANCE_POINT;

    ROS_Pose6D TEMP_POSE;

    nav_msgs::Odometry INS_Odometry;

    ros::Publisher pubCloudRegisteredRaw;
    ros::Subscriber subGPS;

    map<int, pair< ROS_Pose6D ,pcl::PointCloud<PointTypeRGB>>> DATA_CONTAINER;

    // --------------------------------------------------------------------------------------------------------------------------------------------
    SensorFusion_GPSINS()
    {
        ISAM2Params ISAM_params;
        ISAM_params.relinearizeThreshold = 0.1;
        ISAM_params.relinearizeSkip = 1;
        isam = new ISAM2(ISAM_params);

        LM_params.orderingType = Ordering::METIS;

        subIMU = nh.subscribe<vgicp_sam::cloud_info>("imu/data", 1, &mapOptimization::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS = nh.subscribe<nav_msgs::Odometry> ("gps/fix", 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());

        pubINS = nh.advertise<nav_msgs::Odometry>("ins/odom", 1);

        allocateMemory();
    }

    void allocateMemory()
    {
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization

        // VGICP-SAM : SHARED PTR BOOST
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise_Nominal = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise_Reset = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        PCL_DATA.reset(new pcl::PointCloud<PointTypeRGB>());
    };

    // --------------------------------------------------------------------------------------------------------------------------------------------
    void imuHandler(const sensor_msgs::Imu::ConstPtr& IMU_RAW){

        std::lock_guard<std::mutex> lock(mtx);

        // A-1. Transformate IMU Coordinate (ENU)
        sensor_msgs::Imu IMU_CURRENT = imuConverter(*IMU_RAW);

        imuQueOpt.push_back(IMU_CURRENT);
        imuQueImu.push_back(IMU_CURRENT);

        // RETURN (IMU not INITIALIZED)
        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&IMU_CURRENT);
        double dt = (lastImuT_imu < 0) ? (1.0 / 400.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // A-2. GTSAM INTG Single IMU State
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(IMU_CURRENT.linear_acceleration.x, IMU_CURRENT.linear_acceleration.y, IMU_CURRENT.linear_acceleration.z),
                                                gtsam::Vector3(IMU_CURRENT.angular_velocity.x,    IMU_CURRENT.angular_velocity.y,    IMU_CURRENT.angular_velocity.z), dt);

        // A-2. GTSAM Predict IMU State (IMU Preintegration)
        gtsam::NavState ODOM_Now = imuIntegratorImu_->predict(ODOM_Prv, BIAS_Prv);
        gtsam::Pose3 IMU_Pose3 = gtsam::Pose3(ODOM_Now.quaternion(), ODOM_Now.position());
        gtsam::Pose3 GPS_Pose3 = IMU_Pose3.compose(imu2Lidar);

        // A-4. ROS Publish GTSAM NAV STATE
        INS_Odometry.header.stamp = IMU_CURRENT.header.stamp;
        INS_Odometry.header.frame_id = odometryFrame;
        INS_Odometry.child_frame_id = "navsat_link";

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
        pubImuOdometry.publish(INS_Odometry);

    }

    // --------------------------------------------------------------------------------------------------------------------------------------------
    void gpsHandler(const sensor_msgs::NavSatFix:ConstPtr& GPS_RAW){

        std::lock_guard<std::mutex> lock(mtx);
        double currentCorrectionTime = ROS_TIME(GPS_RAW);

        if (imuQueOpt.empty())
            return;

        // B-A. CONVERT LATLONG to UTM
        double utm_x = 0;
        double utm_y = 0;
        std::string utm_zone_;
        NavsatConversions::LLtoUTM(GPS_RAW->latitude, GPS_RAW->longitude, utm_y, utm_x, utm_zone_);
                                                                      //  NORTH  EAST
        // B-1. COMPILE LiDAR Odom
        float p_x = utm_x;
        float p_y = utm_y;
        float p_z = GPS_RAW->altitude;
        float r_x = INS_Odometry.pose.pose.orientation.x;
        float r_y = INS_Odometry.pose.pose.orientation.y;
        float r_z = INS_Odometry.pose.pose.orientation.z;
        float r_w = INS_Odometry.pose.pose.orientation.w;

        // gtsam::Vector Vector3(3);
        // Vector3 << max(GPS_RAW->position_covariance[0], 1.0f), max(GPS_RAW->position_covariance[4], 1.0f), max(GPS_RAW->position_covariance[8], 1.0f);
        // noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
        // gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(p_x, p_y, p_z), gps_noise);
        // gtSAMgraph.add(gps_factor);

        bool degenerate = (int)INS_Odometry->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 gpsPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // B-!. Optimize Fail -> RESET GTSAM Optimizer
        if (systemInitialized == false)
        {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // initial pose
            prevPose_ = gpsPose.compose(gps2Imu);
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
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 400.0) : (imuTime - lastImuT_opt);
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
        Vector3 << max(GPS_RAW->position_covariance[0], 1.0f), max(GPS_RAW->position_covariance[4], 1.0f), max(GPS_RAW->position_covariance[8], 1.0f);
        correctionNoise_Nominal = noiseModel::Diagonal::Variances(Vector3);

        gtsam::Pose3 curPose = gpsPose.compose(gps);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise_Reset : correctionNoise_Nominal);
        graphFactors.add(pose_factor);//!----------------------------------------------------------------------------------------------------------


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

        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        // check optimization (Optimize Failuer Detection)
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 400.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        // !! -- PERFORMANCE OPTMIZER KEY Threshold -- !!
        ++key;
        doneFirstOpt = true;

    }

    // --------------------------------------------------------------------------------------------------------------------------------------------
    void publish_Odom(){

    }

    // --------------------------------------------------------------------------------------------------------------------------------------------
    gtsam::Pose3 trans2gtsamPose(Position SE3_IN)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(SE3_IN.Yaw, SE3_IN.Pitch, SE3_IN.Roll),
                                  gtsam::Point3(SE3_IN.X, SE3_IN.Y, SE3_IN.Z));
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "SensorFusion_GNSS");

    SensorFusion_GPSINS SF;
    ROS_INFO("\033[1;32m----> Local OSM Localization Start \033[0m");

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
