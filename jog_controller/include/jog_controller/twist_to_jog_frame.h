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

namespace jog_controller
{
class TwistToJogFrame
  {
  public:
    TwistToJogFrame();
    void twist_cb(const geometry_msgs::TwistConstPtr &twist);
    bool setControllerStatus(std_msgs::Bool &status);
    bool setTargetFrame(std_msgs::String &frame);
    bool setTargetLink(std_msgs::String &link);

  protected:
    std::vector<std::string> group_names_;
    std::vector<std::string> link_names_;
    std::string group_name_;
    std::string link_name_;
    std::string frame_id_;
    // std::string target_link_id_;
    std::string sub_topic_;
    double scale_linear_;
    double scale_angular_;
    boost::mutex mutex_;
    bool rotate_axes_;
    bool dominant_axis_mode_;
    bool controller_enabled_;
    arma::mat rotation_matrix_;

    void keepOnlyDominantAxis(arma::vec6 &twist_p);
    void rotateAxes(arma::vec6 &twist_p);
    bool hasDuplicate(const arma::vec6 &twist_p);
    void scaleCommand(arma::vec6 &twist_p);
    
    ros::Subscriber twist_sub_;
    ros::Publisher jog_frame_pub_;
    ros::ServiceServer set_target_frame_srv_;
    ros::ServiceServer set_target_link_srv_;
  };

}  // namespace jog_controller

#endif  // TWIST_TO_JOG_FRAME_H
