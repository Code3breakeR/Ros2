#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class Robot : public rclcpp::Node
{
public:
  Robot()
      : Node("ros2talker")
  {
    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("/svaya/position_trajectory_controller/command", 1);
    // publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/svaya/position_trajectory_controller/command", 1);
    broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        30ms, std::bind(&Robot::timer_callback, this));
  }

private:
  double joint1 = 0.0, joint2 = 0.0, joint3 = 0.0, joint4 = 0.0, joint5 = 0.0, joint6 = 0.0, t = 0.005;

  int count=0;
  std::string child[8] = {"base_link", "link0", "link1", "link2", "link3", "link4", "link5", "link6"};
  void timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    sensor_msgs::msg::JointState joint_state;
    // trajectory_msgs::msg::JointTrajectory joint_state;
    geometry_msgs::msg::TransformStamped odom_trans;

    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = this->child[count];

    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.velocity.resize(6);
    joint_state.effort.resize(6);
    joint_state.header.frame_id = "camera_link";

    // joint_state.points.resize(2);

    joint_state.header.stamp = now;
    joint_state.name[0] = "joint1";
    joint_state.name[1] = "joint2";
    joint_state.name[2] = "joint3";
    joint_state.name[3] = "joint4";
    joint_state.name[4] = "joint5";
    joint_state.name[5] = "joint6";

    joint_state.position[0] = joint1;
    joint_state.position[1] = joint2;
    joint_state.position[2] = joint3;
    joint_state.position[3] = joint4;
    joint_state.position[4] = joint5;
    joint_state.position[5] = joint6;

    joint_state.velocity[0] = 5;
    joint_state.velocity[1] = 5;
    joint_state.velocity[2] = 5;
    joint_state.velocity[3] = 5;
    joint_state.velocity[4] = 5;
    joint_state.velocity[5] = 100;

    joint_state.effort[0] = 1000;
    joint_state.effort[1] = 1000;
    joint_state.effort[2] = 1000;
    joint_state.effort[3] = 1000;
    joint_state.effort[4] = 1000;
    joint_state.effort[5] = 1000;

    // trajectory_msgs::msg::JointTrajectoryPoint points_n;
    // for (size_t i = 0; i < 2; i++)
    // {
    //   points_n.positions.resize(6);
    //   points_n.positions[0] = joint1;
    //   points_n.positions[1] = joint2;
    //   points_n.positions[2] = joint3;
    //   points_n.positions[3] = joint4;
    //   points_n.positions[4] = joint5;
    //   points_n.positions[5] = joint6;
    //   joint_state.points[i] = points_n;

    //   joint1 = joint1 + t;
    //   if (joint1 > 1.57 || joint1 < -1.57)
    //   {
    //     t *= -1;
    //   }

    //   joint2++;
    //   joint3++;
    //   joint4++;
    //   joint5++;
    //   joint6++;
    // }

    joint1 = joint1 + t;
    if (joint1 > 1.57 || joint1 < -1.57)
    {
      t *= -1;
    }
    joint2++;
    joint3++;
    joint4++;
    joint5++;
    joint6++;

    count = (count + 1) % 8;

    joint_pub->publish(joint_state);
    // publisher_->publish(joint_state);
    broadcaster->sendTransform(odom_trans);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot>());
  rclcpp::shutdown();
  return 0;
}
