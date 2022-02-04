#ifndef ROBOTONT_TELEOP_JOY_H
#define ROBOTONT_TELEOP_JOY_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace robotont
{
  class TeleopJoy
  {
  public:
    TeleopJoy();

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void timerCallback(const ros::TimerEvent&);

    ros::NodeHandle nh_;

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer timer;
    ros::Time last_joy_time;
  };
}  // namespace robotont
#endif
