![header](https://capsule-render.vercel.app/api?type=rect&color=timeGradient&text=AGV%20SENSOR%20FUSION&fontSize=20)

## <div align=left>Repository Information</div>  
- Sensor Fusion with GPS, IMU with tightly-coupled method. Algorithm is performed with GTSAM 4.x and IMU Pre-integration.  
- Initial Release (v 1.0)
- Safety Rule IMU under 100Hz / KITTI, ComplexUrbanData Config Add (v 1.2)

## <div align=left>Fusion Ability</div>  
#### GPS-IMU
* Main GPS-IMU Package for Sensor Fusion
#### GPS-IMU-ODOM  
* Remained for Future Works  
#### GPS-IMU-VISION  
* Remained for Future Works  

## <div align=left>Pre Requirements</div>  
#### Ubuntu   
- Ubuntu 18.04 / 20.04
#### ROS - Robot-Operating-System  
- ROS Melodic *(with Bionic Beaver)*  
- ROS Noetic *(with Focal Fossa)*  
#### GTSAM  
- Georgia-Tech Smoothing and Mapping  
- Package Github : https://github.com/borglab/gtsam  


## <div align=left>How to Use</div> 
<pre>cd catkin_ws/  
git clone https://github.com/iismn/AGV-SENSOR-FUSION
cd .. && catkin_make</pre>

## <div align=left>Test Environment</div>
#### TEST SENSOR 
- Xsense MTI-30 AHRS IMU / 300Hz
- Ublox M8P RTK-GPS / 10Hz


## <div align=left>Reference</div>
- IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation, Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza, RSS11 [PDF](http://www.roboticsproceedings.org/rss11/p06.pdf)
