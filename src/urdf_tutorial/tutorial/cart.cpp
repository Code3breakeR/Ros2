#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class Robot : public rclcpp::Node
{
  public:
    Robot()
    : Node("cart"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 1);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&Robot::timer_callback, this));
    }

  private:
    double tailJoint1 = 0.0, tailJoint2 = 0.0, tailJoint3 = 0.0, FRWheelJoint = 0.0, FLWheelJoint = 0.0, RRWheelJoint = 0.0, RLWheelJoint = 0.0,t1=0.05,t2=0.05,t3=0.05;
    void timer_callback()
    {
      trajectory_msgs::msg::JointTrajectory joint_state;

      joint_state.joint_names.resize(7);
      joint_state.points.resize(2);

      joint_state.joint_names[0] = "tailJoint1";
      joint_state.joint_names[1] = "tailJoint2";
      joint_state.joint_names[2] = "tailJoint3";
      joint_state.joint_names[3] = "FRWheelJoint";
      joint_state.joint_names[4] = "FLWheelJoint";
      joint_state.joint_names[5] = "RRWheelJoint";
      joint_state.joint_names[6] = "RLWheelJoint";

      trajectory_msgs::msg::JointTrajectoryPoint points_n;
      for (size_t i = 0; i < 2; i++)
      {
         tailJoint1 = tailJoint1 + t1;
         if (tailJoint1 > 1.57 || tailJoint1 < -1.57)
         {
            t1 *= -1;
         }

         tailJoint2 = tailJoint2 + t2;
         if (tailJoint2 > 1.57 || tailJoint2 < -1.57)
         {
            t2 *= -1;
         }

         tailJoint3 = tailJoint3 + t3;
         if (tailJoint3 > 1.57 || tailJoint3 < -1.57)
         {
            t3 *= -1;
         }
         points_n.positions.resize(7);
         points_n.positions[0] = tailJoint1;
         points_n.positions[1] = tailJoint2;
         points_n.positions[2] = tailJoint3;
         points_n.positions[3] = FRWheelJoint;
         points_n.positions[4] = FLWheelJoint;
         points_n.positions[5] = RRWheelJoint;
         points_n.positions[6] = RLWheelJoint;
         joint_state.points[i] = points_n;

        FRWheelJoint++;
        FLWheelJoint++;
        RRWheelJoint++;
        RLWheelJoint++;

      } 
         
        publisher_->publish(joint_state);

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
  }