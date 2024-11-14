#include "test1_maze/test1_maze.hpp"

TestMaze::TestMaze() : Node("test1_maze_node")
{
  // 벽과의 거리 및 정면 거리 파라미터 선언
  this->declare_parameter<double>("wall_distance_front", 0.25);
  this->declare_parameter<double>("wall_distance_right", 0.3);
  this->declare_parameter<double>("wall_distance_left", 0.3);
  //this->declare_parameter<double>("wall_distance_back", 0.25);

  // 변수에 파라미터 값 저장
  this->get_parameter("wall_distance_front", wall_distance_front_);
  this->get_parameter("wall_distance_right", wall_distance_right_);
  this->get_parameter("wall_distance_left", wall_distance_left_);


  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&TestMaze::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&TestMaze::odom_callback, this, std::placeholders::_1));

  crossroad_stack_ = std::stack<std::pair<double, double>>();
}

// scan 토픽 콜백 함수
void TestMaze::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int front_index = 0;
  int right_index = 270;
  int left_index = 90;

  float front_distance = msg->ranges[front_index];
  float right_distance = msg->ranges[right_index];
  float left_distance = msg->ranges[left_index];
  float up_left_distance = msg->ranges[60];
  float down_left_distance = msg->ranges[120];
  float up_right_distance = msg->ranges[300];
  float down_right_distance = msg->ranges[240];

  if(spinning)
  return;


  if (front_distance > wall_distance_front_) {
    if(special_left) {
      target_yaw_ += 1.55;
      if(target_yaw_ > 3.14)
      target_yaw_ = -(6.2 - target_yaw_);
      special_left = false;
    }
    if(std::isinf(front_distance)) {
      if (left_distance > 0.3 && up_left_distance > 0.3 && down_left_distance > 0.3
      && right_distance < 0.3 && up_right_distance < 0.3 && down_right_distance < 0.3) {
        target_yaw_ += 1.55;
      if(target_yaw_ > 3.14)
      target_yaw_ = -(6.2 - target_yaw_);
      }
    }

    target_yaw_ += 0.00; // 현재 yaw 값 유지
    if(up_left_distance < 0.13)
    target_yaw_ += -0.02;
    else if(up_right_distance < 0.13)
    target_yaw_ += 0.02;

    target_yaw_flag = false;
    spinning = true;
  }
  // 앞에 벽이 있을 때
  else {
    if(!target_yaw_flag) {
      if (left_distance > wall_distance_left_) { // 왼쪽 벽이 없을 때
        target_yaw_ += 1.55; // 왼쪽으로 90도 회전 (라디안 값)
        if(right_distance > wall_distance_right_)
        save_crossroad();
      }
      else if (right_distance > wall_distance_right_) { // 오른쪽 벽이 없을 때
        target_yaw_ += -1.55; // 오른쪽으로 90도 회전 (라디안 값)
        if(target_yaw_ < -3.14)
        target_yaw_ = 6.2 + target_yaw_;
      }
      else { // 막다른 길일 때
        target_yaw_ += -3.10; // 오른쪽으로 180도 회전 (라디안 값)
      }
      target_yaw_flag = true;
      spinning = true;
    }
  }

  RCLCPP_INFO(this->get_logger(), "target_yaw_: %.2f", target_yaw_);
}

void TestMaze::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 로봇의 위치와 방향 추출
  double robot_x = msg->pose.pose.position.x;
  double robot_y = msg->pose.pose.position.y;
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // 위치 및 방향 정보 출력
  //RCLCPP_INFO(this->get_logger(), "로봇 위치: (%.2f, %.2f), 방향 (yaw): %.2f", robot_x, robot_y, yaw);

  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;
  current_yaw = yaw;

  geometry_msgs::msg::Twist cmd_vel;

  if (target_yaw_ > yaw + 0.06) { // 목표 yaw 값보다 작을 경우 왼쪽 회전
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.5;
  }
  else if (target_yaw_ < yaw - 0.06) { // 목표 yaw 값보다 클 경우 오른쪽 회전
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = -0.5;
  }
  else { // 목표 yaw 값에 도달하면 직진
    cmd_vel.linear.x = 0.15;
    cmd_vel.angular.z = 0.0;
    spinning = false;
  }
  cmd_vel_pub_->publish(cmd_vel);

  if (!crossroad_stack_.empty()) {
    auto top_pos = crossroad_stack_.top();
    bool is_area = ((top_pos.first - 0.06 < current_x) && (current_x < top_pos.first + 0.06) && (top_pos.second - 0.06 < current_y) && (current_y < top_pos.second + 0.06));
    if(is_area && !was_area) {
      visit_xy += 1;
      RCLCPP_INFO(this->get_logger(), "갈림길 재방문 (횟수: %d)", visit_xy);
    }
    was_area = is_area;

    if(visit_xy == 3) {
      crossroad_stack_.pop();
      //무조건 제자리 좌회전 로직
      special_left = true;
      visit_xy = 1;
    }
  }
  //RCLCPP_INFO(this->get_logger(), "선속도 : %.2f, 각속도 : %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
}

void TestMaze::save_crossroad()
{
  crossroad_stack_.push({current_x, current_y});
  visit_xy = 0;
  RCLCPP_INFO(this->get_logger(), "갈림길 저장: (%.2f, %.2f)", current_x, current_y);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestMaze>());
  rclcpp::shutdown();
  return 0;
}
