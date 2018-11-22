#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class PathBuilder
{
  public:

    PathBuilder();

  private:
    Eigen::Quaterniond rot_g0_inverse_;
    Eigen::Vector3d offset_;
    unsigned int frame_no_;
    bool external_frame_, init_origin_;
    std::string frame_id_;
    ros::NodeHandle nh_, nhp_;

    ros::Subscriber pose_sub_;
    ros::Publisher path_pub_;

    nav_msgs::Path path_msg_;

  // helper functions

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
};

PathBuilder::PathBuilder()
  : frame_no_(0), external_frame_(false), nhp_("~")
{
  if(nhp_.getParam("frame_id", frame_id_)) {
    external_frame_ = true;
  }

  nhp_.param<bool>("init_origin", init_origin_, false);
  pose_sub_ = nh_.subscribe("pose", 100, &PathBuilder::poseCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 100);
}

void PathBuilder::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  if(external_frame_)
    path_msg_.header.frame_id = frame_id_;
  else
    path_msg_.header.frame_id = pose_msg->header.frame_id;

  geometry_msgs::PoseStamped aux;
  aux.header =pose_msg->header;
  if(external_frame_)
    aux.header.frame_id = frame_id_;

  aux.pose = pose_msg->pose.pose;
  if(init_origin_){

      Eigen::Quaterniond rot_gk_(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x,
                                 pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
      Eigen::Vector3d trans_gk_(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);

      if(frame_no_ == 0)
          rot_g0_inverse_ = rot_gk_.inverse();

      Eigen::Isometry3d gk2enu = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d fake_cam_frame_pose = Eigen::Isometry3d::Identity();
      gk2enu.linear() = rot_gk_.toRotationMatrix();
      gk2enu.translation() = trans_gk_;

      fake_cam_frame_pose = rot_g0_inverse_ * gk2enu;
      fake_cam_frame_pose.translation().z() = - fake_cam_frame_pose.translation().z();
      Eigen::Quaterniond new_rot(fake_cam_frame_pose.matrix().topLeftCorner<3,3>());

      if(frame_no_ == 0){
          offset_ = fake_cam_frame_pose.translation();

          aux.pose.position.x = 0;
          aux.pose.position.y = 0;
          aux.pose.position.z = 0;

          frame_no_ ++;
      }else{
          Eigen::Vector3d new_trans(fake_cam_frame_pose.translation() - offset_);
          aux.pose.position.x = new_trans.x();
          aux.pose.position.y = new_trans.y();
          aux.pose.position.z = new_trans.z();
      };

      aux.pose.orientation.w = new_rot.w();
      aux.pose.orientation.x = new_rot.x();
      aux.pose.orientation.y = new_rot.y();
      aux.pose.orientation.z = new_rot.z();
  }
//  else {
//      aux.pose = pose_msg->pose.pose;
//  }

  path_msg_.poses.push_back( aux );

  path_pub_.publish( path_msg_ );
}

int main(int argc, char *argv[])
{
  // Override SIGINT handler
  ros::init(argc, argv, "path_builder");

  PathBuilder path_builder;

  ROS_INFO("path_builder node running...");

  ros::spin();

  return 0;
}
 
