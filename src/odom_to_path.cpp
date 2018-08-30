//
// Created by hd on 8/30/18.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class OdomToPath
{
public:

    OdomToPath();

private:

    ros::NodeHandle nh_, nhp_;

    ros::Subscriber odom_sub_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_msg_;

    // helper functions

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    std::string target_frame;
};

OdomToPath::OdomToPath()
        : nhp_("~"), listener(tf_buffer)
{
    odom_sub_ = nh_.subscribe("odom", 100, &OdomToPose::odomCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 100);
    nhp_.param("target_frame", target_frame, std::string());
}

void OdomToPath::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = odom_msg->header;
    pose_msg.pose = odom_msg->pose;

    path_msg_.header.frame_id = odom_msg->header.frame_id;

    path_msg_.poses.push_back( pose_msg );

    path_pub_.publish( path_msg_ );

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_to_path");

    OdomToPath odom_to_path;

    ROS_INFO("odom_to_path node running...");

    ros::spin();

    return 0;
}
