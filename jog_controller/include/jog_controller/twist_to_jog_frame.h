#ifndef TWIST_TO_JOG_FRAME_H
#define TWIST_TO_JOG_FRAME_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <armadillo>
#include <geometry_msgs/Twist.h>

namespace jog_controller
{
class TwistToJogFrame
  {
  public:
    TwistToJogFrame();
    void publish(const geometry_msgs::Twist::ConstPtr &twist);
    
  protected:
    std::vector<std::string> group_name_;
    std::vector<std::string> link_name_;
    std::string frame_id_;
    std::string target_link_id_;
    std::string sub_topic_;
    std::string scale_linear_;
    std::string scale_angular_;
    boost::mutex mutex_;
    bool rotate_axes_;
    bool dominant_axis_mode_;
    bool controller_enabled_;
    ros::Publisher jog_frame_pub_;
    arma::mat33 rotation_matrix_;
    ros::ServiceServer set_target_frame_srv_;
    ros::ServiceServer set_target_link_srv_;

    void keepOnlyDominantAxis(const arma::vec6 &twist_p);
    void rotateAxes(const arma::vec6 &twist_p);
    bool hasDuplicate(const arma::vec6 const &twist_p);
    void scaleCommand(const arma::vec6 &twist_p);

    bool setControllerStatus(std_msgs::Bool &status);
    bool setTargetFrame(std_msgs::String &frame);
    bool setTargetLink(std_msgs::String &link);
  };

}  // namespace jog_controller

#endif  // TWIST_TO_JOG_FRAME_H
