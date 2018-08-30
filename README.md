# ROS_UTILS

Some nav_msgs message type conversion tools.

## Types

1. geometry_msgs/Pose
```
# A representation of pose in free space, composed of position and orientation.
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
```

2. geometry_msgs/PoseStamped
```
# A Pose with reference coordinate frame and timestamp
std_msgs/Header header
geometry_msgs/Pose pose
```

3. geometry_msgs/PoseWithCovarianceStamped
```
# This expresses an estimated pose with a reference coordinate frame and timestamp
std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
```

4. nav_msgs/Odometry
```
# This represents an estimate of a position and velocity in free space.
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

5. nav_msgs/Path
```
# An array of poses that represents a Path for a robot to follow
std_msgs/Header header
geometry_msgs/PoseStamped[] poses
```

6. sensor_msgs/Imu

```
# Message to hold data from an IMU (Inertial Measurement Unit)
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

## ROS Nodes
- [x] odom_to_pose: `Nav_msgs/Odometry` to `geometry_msgs/PoseWithCovarianceStamped`
- [x] odom_to_path: `Nav_msgs/Odometry` to `nav_msgs/Path`
- [x] odom_to_imu: `Nav_msgs/Odometry` to `sensor_msgs/Imu`
- [x] pose_to_odom: `geometry_msgs/PoseStamped` to `Nav_msgs/Odometry`
- [x] pose_to_path: `geometry_msgs/PoseStamped` to `nav_msgs/Path`
- [x] poseWithCovariance_to_ellipsoid:
 `geometry_msgs/PoseWithCovarianceStamped` to RVIZ `visualization_msgs::Marker`