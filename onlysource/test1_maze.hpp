#ifndef TEST1_MAZE_HPP_
#define TEST1_MAZE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <stack>
#include <cmath>

class TestMaze : public rclcpp::Node
{
public:
  TestMaze();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg); // odom 콜백 함수 선언
  void save_crossroad();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;     // 퍼블리셔 선언
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;   // LaserScan 서브스크립션 선언
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;       // odom 서브스크립션 선언

  std::stack<std::pair<double, double>> crossroad_stack_;
  double current_x, current_y, current_yaw;

  double wall_distance_front_;
  double wall_distance_right_;
  double wall_distance_left_;

  double linear_velocity_;
  double angular_velocity_;

  double target_yaw_ = 0.0;
  bool target_yaw_flag = false;
  bool spinning = false;
  int visit_xy = 0;
  bool was_area = false;
  bool special_left = false;
};

#endif  // TEST1_MAZE_HPP_
