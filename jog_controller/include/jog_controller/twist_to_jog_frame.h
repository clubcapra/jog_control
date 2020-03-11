#ifndef TWIST_TO_JOG_FRAME_H
#define TWIST_TO_JOG_FRAME_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <armadillo>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>
#include "jog_msgs/JogFrame.h"
#include "jog_msgs/ControllerStatus.h"
#include "jog_msgs/GetTargetList.h"
#include "jog_msgs/SetTarget.h"

namespace jog_controller
{
class TwistToJogFrame
  {
  public:
    TwistToJogFrame();

  private:
    //ROS Publisher, Subscriber and Services
    ros::Subscriber twist_sub_;
    ros::Publisher jog_frame_pub_;
    ros::ServiceServer get_target_frame_list_srv_;
    ros::ServiceServer set_controller_status_srv_;
    ros::ServiceServer set_target_frame_srv_;
    ros::ServiceServer set_target_link_srv_;
    
    //ROS Subscriber and Service callback
    void twist_cb(const geometry_msgs::TwistConstPtr &twist);
    bool getTargetFrameList(jog_msgs::GetTargetListRequest &req, jog_msgs::GetTargetListResponse &res);
    bool setControllerStatus(jog_msgs::ControllerStatusRequest &req, jog_msgs::ControllerStatusResponse &res);
    bool setTargetFrame(jog_msgs::SetTargetRequest &target_frame, jog_msgs::SetTargetResponse &res);
    bool setTargetLink(jog_msgs::SetTargetRequest &target_link, jog_msgs::SetTargetResponse &res);

    //Member variables
    std::vector<std::string> group_names_, link_names_;
    std::string group_name_, link_name_, frame_id_, sub_topic_;
    double scale_linear_, scale_angular_;
    boost::mutex mutex_;
    bool rotate_axes_, dominant_axis_mode_, controller_enabled_;
    arma::mat rotation_matrix_;

    //Member methods
    void keepOnlyDominantAxis(arma::vec6 &twist_p);
    void rotateAxes(arma::vec6 &twist_p);
    bool hasDuplicate(const arma::vec6 &twist_p);
    void scaleCommand(arma::vec6 &twist_p);
    
  };

}  // namespace jog_controller

#endif  // TWIST_TO_JOG_FRAME_H
