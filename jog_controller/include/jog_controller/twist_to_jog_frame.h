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
    // std::vector<std::string> getTargetFrames();
    bool setTargetFrame(std_msgs::String &frame);
    bool setTargetLink(std_msgs::String &link);

  protected:
    std::vector<std::string> group_names_, link_names_;
    std::string group_name_, link_name_, frame_id_, sub_topic_;
    double scale_linear_, scale_angular_;
    boost::mutex mutex_;
    bool rotate_axes_, dominant_axis_mode_, controller_enabled_;
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
