#include <ros/ros.h>
#include <boost/thread.hpp>
#include <armadillo>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <jog_controller/twist_to_jog_frame.h>
#include "jog_msgs/JogFrame.h"

namespace jog_controller
{

TwistToJogFrame::TwistToJogFrame()
{
  ros::NodeHandle nh, pnh("~");
  jog_frame_pub_ = nh.advertise<jog_msgs::JogFrame>("jog_frame", 1);
  // set_target_frame_srv_ = nh.advertiseService("set_target_frame", setTargetFrame);
  // set_target_link_srv_ = nh.advertiseService("set_target_link", &TwistToJogFrame::setTargetLink, this);
  pnh.getParam("/jog_frame_node/group_names", group_names_);
  pnh.getParam("/jog_frame_node/link_names", link_names_);
  pnh.getParam("group_name", group_name_);
  pnh.getParam("link_name", link_name_);
  pnh.getParam("frame_id", frame_id_);
  pnh.getParam("rotate_axes", rotate_axes_);
  std::string str_rotation_matrix_;
  pnh.getParam("rotation_matrix", str_rotation_matrix_); 
  rotation_matrix_ = arma::mat(str_rotation_matrix_);
  pnh.getParam("dominant_axis_mode", dominant_axis_mode_);
  pnh.getParam("scale_linear", scale_linear_);
  pnh.getParam("scale_angular", scale_angular_);
  pnh.getParam("sub_topic", sub_topic_);
  pnh.getParam("controller_enabled", controller_enabled_);
  twist_sub_ = nh.subscribe(sub_topic_, 10, &TwistToJogFrame::twist_cb, this);
  ros::topic::waitForMessage<geometry_msgs::Twist>(sub_topic_);
}

void TwistToJogFrame::keepOnlyDominantAxis(arma::vec6 &twist_p)
{
  double min = twist_p.min();
  double max = twist_p.max();
  double min_max = max + min;
  if(min_max > 0){
    arma::uword i = twist_p.index_max();
    twist_p.zeros();
    twist_p.at(i) = max;
  } else {
    arma::uword i = twist_p.index_min();
    twist_p.zeros();
    twist_p.at(i) = min;
  }
}

void TwistToJogFrame::rotateAxes(arma::vec6 &twist_p)
{
  arma::vec linear = arma::vec3({twist_p.at(0), twist_p.at(1), twist_p.at(2)});
  arma::vec angular = arma::vec3({twist_p.at(3), twist_p.at(4), twist_p.at(5)});

  linear = arma::affmul(rotation_matrix_, linear);
  angular = arma::affmul(rotation_matrix_, angular);

  //Write in original vector
  for(arma::uword i = 0; i < 3; i++){ 
    twist_p[i] = linear[i];
    twist_p[i+3] = angular[i];
  }
}

bool TwistToJogFrame::hasDuplicate(const arma::vec6 &twist_p)
{
  arma::vec nz_tmp = arma::nonzeros(twist_p);
  arma::vec unique_tmp = arma::unique(nz_tmp);
  return size(unique_tmp) != size(nz_tmp);
}

void TwistToJogFrame::scaleCommand(arma::vec6 &twist_p)
{
  for(arma::uword i = 0; i < 3; i++)
  {
    twist_p.at(i) = twist_p.at(i) * scale_linear_;
  }
  for(arma::uword i = 3; i < 6; i++)
  {
    twist_p.at(i) = twist_p.at(i) * scale_angular_;
  }
}

void TwistToJogFrame::twist_cb(const geometry_msgs::TwistConstPtr &twist)
{
  boost::mutex::scoped_lock lock(mutex_);

  // publish
  jog_msgs::JogFrame msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  msg.group_name = group_name_;
  msg.link_name = link_name_;

  arma::vec6 v_twist = arma::vec6({twist->linear.x, twist->linear.y, twist->linear.z, twist->angular.x, twist->angular.y, twist->angular.z});

  if(dominant_axis_mode_)
  {
    if(hasDuplicate(v_twist))
    {
      v_twist.zeros(); //handling the edge case
    } else {
      keepOnlyDominantAxis(v_twist);
    }
  }
  
  if(rotate_axes_)
  {
    rotateAxes(v_twist);
  }

  scaleCommand(v_twist);

  msg.avoid_collisions = true;

  // Publish if the button is enabled
  if(controller_enabled_)
  {
    // Publish only if at least one of the commands is different from zero.
    if(arma::any(v_twist))
    {
      jog_frame_pub_.publish(msg);
    }
  }  
  }  

//callback for controller_enable service
bool TwistToJogFrame::setControllerStatus(std_msgs::Bool &status)
{
  controller_enabled_ = status.data;
  return true;
}

bool TwistToJogFrame::setTargetFrame(std_msgs::String &frame)
{
  frame_id_ = frame.data;
  return true;
}

bool TwistToJogFrame::setTargetLink(std_msgs::String &link)
{
  link_name_ = link.data;
  return true;
}

}  // namespace jog_controller

/**
 * @brief Main function of the node
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_to_jog_frame");
  jog_controller::TwistToJogFrame node;

  ros::Rate loop_rate(10);
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}