#include <cstdio>
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "hadabot_msgs/msg/odom2_d.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
//#include "time.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef enum {
  LEFT,
  RIGHT
} HBSide;

#define PI 3.14159265

enum EHadabotState {FORWARD, BACKWARD, LEFT_ROTATION, RIGHT_ROTATION, STOPED};

class HadabotController : public rclcpp::Node
{
private:

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_power_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_power_left_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr radps_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr radps_right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr distance_forward_sub_;

  rclcpp::Subscription<hadabot_msgs::msg::Odom2D>::SharedPtr odom2d_sub_;

  rclcpp::TimerBase::SharedPtr update_odometry_timer_;
  rclcpp::TimerBase::SharedPtr publish_odometry_timer_;

  float distance_forward;
  float wheel_radius_m_;
  float wheelbase_m_;

  float wheel_radps_left_;
  double wheel_radps_left_time;
  double wheel_radps_left_time_prev;

  float wheel_radps_right_;
  double wheel_radps_right_time;
  double wheel_radps_right_time_prev;

  double last_odom_update_sec;
  double last_lw_radps_update_sec;
  float max_radps; 

  int state;

  bool flLeftDataUpdated = false;
  bool flRightDataUpdated = false;

  hadabot_msgs::msg::Odom2D::SharedPtr odom2D;

  nav_msgs::msg::Odometry::SharedPtr pose_;
  geometry_msgs::msg::Twist::SharedPtr twist_msg_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_wh;

void odom2d_cb(const hadabot_msgs::msg::Odom2D::SharedPtr msg)
{
  odom2D->x = msg->x;
  odom2D->y = msg->y;
  odom2D->theta = msg->theta;
  odom2D->v = msg->v;
  odom2D->w = msg->w;
  odom2D->stamp.sec = msg->stamp.sec;
  odom2D->stamp.nanosec = msg->stamp.nanosec;

  update_odometry();

  if (odom2D->x != 0) {

    //std::cout << "sec: " << odom2D->stamp.sec << "  nsec: " << odom2D->stamp.nanosec << "  x: " << msg->x <<  "   y: " << msg->y << "  theta : " <<  msg->theta << "  v : " <<  msg->v << "  w : " <<  msg->w << std::endl; 
    //std::cout.flush();
  }

}
 
  void distance_forward_cb(const sensor_msgs::msg::Temperature::SharedPtr msg)
  {  
    this->distance_forward = msg->temperature;

  }

  /***************************************************************************/
  void update_odometry() 
  {
   
    // Get the time delta since last update
    auto current_time = this->now(); 

    rclcpp::Time odom_time(odom2D->stamp.sec, odom2D->stamp.nanosec, RCL_ROS_TIME);
    
    long int delta = (long int) (current_time - odom_time).nanoseconds();

    //std::cout << "delta: " << delta << std::endl;
    //std::cout.flush();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom2D->theta);
    pose_->pose.pose.orientation.x = q.getX();
    pose_->pose.pose.orientation.y = q.getY();
    pose_->pose.pose.orientation.z = q.getZ(); 
    pose_->pose.pose.orientation.w = q.getW(); 
    pose_->pose.pose.position.x = odom2D->x;
    pose_->pose.pose.position.y = odom2D->y;
    pose_->twist.twist.linear.x = odom2D->v;
    pose_->twist.twist.linear.y = 0;
    pose_->twist.twist.angular.z = odom2D->w;
    pose_->twist.twist.angular.x = 0;
    pose_->twist.twist.angular.y = 0;
    pose_->header.stamp = odom_time;
    pose_->header.frame_id = "odom";
    pose_->child_frame_id = "base_link";

    odometry_pub_->publish(*pose_);    

/*
   std::cout <<  "dt : " << dt_s;
   std::cout <<  "  dl : " << d_left_m <<  "   dr : " << d_right_m;
   std::cout << "  x: " << pose_->pose.pose.position.x <<  "   y: " << pose_->pose.pose.position.y << "  theta : " <<  theta_rad << "   v : " << pose_->twist.twist.linear.x  << "   w: " <<  pose_->twist.twist.angular.z << std::endl; 
   std::cout.flush();


*/ 

  }
 

public:
  HadabotController() : Node("hadabot_controller"), wheel_radius_m_(0.032), wheelbase_m_(0.117), 
  wheel_radps_left_(0.0), max_radps(21.0), wheel_radps_right_(0.0), 
  pose_(std::make_shared<nav_msgs::msg::Odometry>()), 
  odom2D(std::make_shared<hadabot_msgs::msg::Odom2D>()), 
  twist_msg_(std::make_shared<geometry_msgs::msg::Twist>()),
  last_odom_update_sec(0), state(STOPED)
  {
    RCLCPP_INFO(this->get_logger(), "Starting Hadabot Controller");

    distance_forward_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
        "/hadabot/distance_forward_timestamped", 10,
        std::bind(&HadabotController::distance_forward_cb, this, _1)
      );    


    odom2d_sub_ = this->create_subscription<hadabot_msgs::msg::Odom2D>(
        "/hadabot/odom2d", 10,
        std::bind(&HadabotController::odom2d_cb, this, _1)
      );        

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", 10);


  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HadabotController>());
  rclcpp::shutdown();
  return 0;
}
