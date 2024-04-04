#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.h>
using namespace std::chrono_literals;

class Robot : public rclcpp::Node
{
public:
  Robot()
      : Node("r2d2")
  {
    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
    broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        30ms, std::bind(&Robot::timer_callback, this));
  }

private:
  double degree = 3.14 / 180.0, tilt = 0.0, tinc = degree, swivel = 0.0, angle = 0.0, height = 0.0, hinc = 0.005, pi = 3.14;
  int count = 0;
  std::string child[7] = {"axis","body","head","box","leg2","leg1","rod"};

  geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    q.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    q.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    return q;
  }

  void timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    sensor_msgs::msg::JointState joint_state;
    geometry_msgs::msg::TransformStamped odom_trans;

    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "axis";
    odom_trans.child_frame_id =  this->child[count];

    joint_state.header.stamp = now;
    joint_state.name.resize(3);
    joint_state.name[0] = "swivel";
    joint_state.name[1] = "tilt";
    joint_state.name[2] = "periscope";
    
    joint_state.position.resize(3);
    joint_state.position[0] = swivel;
    joint_state.position[1] = tilt;
    joint_state.position[2] = height;

    odom_trans.transform.translation.x = cos(angle) * 2;
    odom_trans.transform.translation.y = sin(angle) * 2;
    odom_trans.transform.translation.z = 0.7;
    odom_trans.transform.rotation = euler_to_quaternion(0, 0, angle + pi / 2);
  
    
    tilt += tinc;
    if (tilt < -0.5 or tilt > 0.0)
      tinc *= -1;
    height += hinc;
    if (height > 0.2 or height < 0.0)
      hinc *= -1;
    swivel += degree;
    angle += degree / 4;
    count = (count + 1) % 7;

    joint_pub->publish(joint_state);
    broadcaster->sendTransform(odom_trans);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot>());
  rclcpp::shutdown();
  return 0;
}