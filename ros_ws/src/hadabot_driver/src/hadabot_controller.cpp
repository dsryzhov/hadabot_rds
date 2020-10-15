#include <cstdio>
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef enum {
  LEFT,
  RIGHT
} HBSide;

#define UPDATE_DT 15ms
#define PUBLISH_DT 15ms

#define PI 3.14159265

class HadabotController : public rclcpp::Node
{
private:

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_power_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_power_left_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radps_right_sub_;

  rclcpp::TimerBase::SharedPtr update_odometry_timer_;
  rclcpp::TimerBase::SharedPtr publish_odometry_timer_;

  float wheel_radius_m_;
  float wheelbase_m_;

  float wheel_radps_left_;
  float wheel_radps_right_;
  double last_odom_update_sec;
  float max_radps; 

  nav_msgs::msg::Odometry::SharedPtr pose_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  /***************************************************************************/
  void wheel_radps(
    const std_msgs::msg::Float32::SharedPtr msg, HBSide which_side)
  {
    std::string the_side = which_side == HBSide::LEFT ? "left" : "right";

    switch(which_side) {
      case HBSide::LEFT:
      wheel_radps_left_ = msg->data;
      break;

      case HBSide::RIGHT:
      wheel_radps_right_ = msg->data;
      break;
    }
  }

  /***************************************************************************/
  void wheel_radps_left_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->wheel_radps(msg, HBSide::LEFT);
  }

  /***************************************************************************/
  void wheel_radps_right_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {  
    this->wheel_radps(msg, HBSide::RIGHT);
  }

  /***************************************************************************/
  void update_odometry() 
  {
    // Get the time delta since last update
     auto cur_time_sec = this->now().seconds();


    auto current_time = this->now(); 

    auto dt_ms = UPDATE_DT;
//    auto dt_s = dt_ms.count() / 1000.0;
    auto dt_s = cur_time_sec - last_odom_update_sec;  
    if (last_odom_update_sec == 0) { 
       last_odom_update_sec = cur_time_sec;
       return; 
    } else 
       last_odom_update_sec = cur_time_sec;


    // Compute distance traveled for each wheel
    float d_left_m = (wheel_radps_left_ * dt_s * wheel_radius_m_);
    float d_right_m = (wheel_radps_right_ * dt_s * wheel_radius_m_);

    double d_center_m = (d_right_m + d_left_m) / 2.0;
    double phi_rad = (d_right_m - d_left_m) / wheelbase_m_;

//    std::cout << d_left_m << "        " << d_right_m  << "  d_c  " << d_center_m << "   phi_rad   "  << phi_rad  << std::endl;
//    std::cout.flush();

    tf2::Quaternion q0(pose_->pose.pose.orientation.x, pose_->pose.pose.orientation.y, pose_->pose.pose.orientation.z, pose_->pose.pose.orientation.w);


    tf2::Matrix3x3 m(q0);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    



//    auto theta_rad = pose_->pose.pose.orientation.z;
//    auto theta_rad = atan2(q0.getW(), q0.getY());
    double theta_rad = yaw;
    
//    std::cout << q0.getX() << "        " << q0.getY() << "      " << q0.getZ() << "     " << q0.getW() << std::endl;
//    std::cout.flush();


    double x_m_dt = d_center_m * std::cos(theta_rad);
    double y_m_dt = d_center_m * std::sin(theta_rad);
    double theta_rad_dt = phi_rad;

    double x_m = pose_->pose.pose.position.x;
    double y_m = pose_->pose.pose.position.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_rad + theta_rad_dt);
    pose_->pose.pose.orientation.x = q.getX();
    pose_->pose.pose.orientation.y = q.getY();
    pose_->pose.pose.orientation.z = q.getZ(); 
    pose_->pose.pose.orientation.w = q.getW(); 
    pose_->pose.pose.position.x = x_m + x_m_dt;
    pose_->pose.pose.position.y = y_m + y_m_dt;
    pose_->twist.twist.linear.x = d_center_m / dt_s;
    pose_->twist.twist.angular.z = phi_rad / dt_s;
    pose_->header.stamp = current_time;
    pose_->header.frame_id = "Fixed Frame";

   std::cout <<  "x: " << pose_->pose.pose.position.x <<  "   y: " << pose_->pose.pose.position.y << "  theta : " <<  theta_rad << "   v : " << pose_->twist.twist.linear.x  << "   w: " <<  pose_->twist.twist.angular.z << std::endl; 
   std::cout.flush();

//    std::cout << "    theta : " <<  theta_rad << "   phi_rad : " << phi_rad << "   dx : " << x_m_dt << "  dy : " << y_m_dt << std::endl;
//    std::cout.flush();


  }

  /***************************************************************************/
  void publish_odometry()
  {
    odometry_pub_->publish(*pose_);

//   std::cout <<  "x: " << pose_->pose.pose.position.x <<  "   y: " << pose_->pose.pose.position.y << "   v : " << pose_->twist.twist.linear.x  << "   w: " <<  pose_->twist.twist.angular.z << std::endl; 
//   std::cout.flush();
  }

  /***************************************************************************/
  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
  {
    float v = twist_msg->linear.x;


    if (v != 0.0)  {
          if (v > 0) v = 0.33;
          else v = -0.33;
    }

	
   float w = twist_msg->angular.z;

   if (w != 0.0) {
       if (w > 0) w = 1.75*3.141596;
       else w = (-1.75)*3.141596;
   } 


    auto v_r = ((2.0 * v) + (w * wheelbase_m_)) / (2 * wheel_radius_m_);
    auto v_l = ((2.0 * v) - (w * wheelbase_m_)) / (2 * wheel_radius_m_);

    std_msgs::msg::Float32 pow_r;
    std_msgs::msg::Float32 pow_l;

//    pow_r.data = std::min(std::max(v_r, -1.0), 1.0);
 //   pow_l.data = std::min(std::max(v_l, -1.0), 1.0);

    float pw_r = v_r / max_radps; // 21 - max possible rad/s for my motors
    float pw_l = v_l / max_radps;
    if  (pw_r > 1.0) pw_r = 1.0;
    else if (pw_r < -1.0) pw_r = -1.0;
    if  (pw_l > 1.0) pw_l = 1.0;
    else if (pw_l < -1.0) pw_l = -1.0;

    pow_r.data = pw_r;
    pow_l.data = pw_l;
    wheel_power_left_pub_->publish(pow_l);
    wheel_power_right_pub_->publish(pow_r);
  }

public:
  HadabotController() : Node("hadabot_controller"), wheel_radius_m_(0.032), wheelbase_m_(0.117), wheel_radps_left_(0.0), max_radps(21.0), wheel_radps_right_(0.0), pose_(std::make_shared<nav_msgs::msg::Odometry>()), last_odom_update_sec(0)
  {
    RCLCPP_INFO(this->get_logger(), "Starting Hadabot Controller");

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/hadabot/cmd_vel", 10,
        std::bind(&HadabotController::twist_cb, this, _1));

    radps_left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hadabot/wheel_radps_left", 10,
        std::bind(&HadabotController::wheel_radps_left_cb, this, _1)
      );

    radps_right_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/hadabot/wheel_radps_right", 10,
      std::bind(&HadabotController::wheel_radps_right_cb, this, _1)
    );

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/hadabot/odom", 10);

    wheel_power_left_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/hadabot/wheel_power_left", 10);
    wheel_power_right_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/hadabot/wheel_power_right", 10);

    update_odometry_timer_ = this->create_wall_timer(
      UPDATE_DT, std::bind(&HadabotController::update_odometry, this));

    publish_odometry_timer_ = this->create_wall_timer(
      PUBLISH_DT, std::bind(&HadabotController::publish_odometry, this));

    tf2::Quaternion q;
    q.setEuler(0,0,0);
    pose_->pose.pose.orientation.x  = q.getX();
    pose_->pose.pose.orientation.y  = q.getY();
    pose_->pose.pose.orientation.z  = q.getZ();
    pose_->pose.pose.orientation.w  = q.getW();

   tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw,pitch, roll);  
    std::cout << "roll: " <<  roll << "  pitch: " << pitch << "  yaw : " << yaw << std::endl;
    std::cout.flush();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HadabotController>());
  rclcpp::shutdown();
  return 0;
}
