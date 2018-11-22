#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PathBuilder
{
  public:

    PathBuilder();

  private:

    unsigned int frame_no_;
    bool external_frame_, init_origin_;
    std::string frame_id_;
    ros::NodeHandle nh_, nhp_;
    double x, y, z;

    ros::Subscriber pose_sub_;
    ros::Publisher path_pub_;

    nav_msgs::Path path_msg_;

  // helper functions

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
};

PathBuilder::PathBuilder()
  : frame_no_(0), external_frame_(false), nhp_("~"), x(0), y(0), z(0)
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

      if(frame_no_ == 0){

          x = pose_msg->pose.pose.position.x;
          y = pose_msg->pose.pose.position.y;
          z = pose_msg->pose.pose.position.z;
          aux.pose.position.x = 0;
          aux.pose.position.y = 0;
          aux.pose.position.z = 0;

          frame_no_ ++;
      }else{
          aux.pose.position.x = pose_msg->pose.pose.position.x - x;
          aux.pose.position.y = pose_msg->pose.pose.position.y - y;
          aux.pose.position.z = pose_msg->pose.pose.position.z - z;
      };
  }

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
 
