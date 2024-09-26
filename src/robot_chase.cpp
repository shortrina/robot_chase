#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/* Rick = Blue robot, Morty = Red robot*/
/* Rick follows Morty (Blue follows red) */

class RobotChase : public rclcpp::Node {
public:
  RobotChase()
      : Node("robot_chase"), kp_distance_(0.3), kp_yaw_(0.7), error_yaw_(0.0),
        error_distance_(0.0), distance_(0.0), angle_(0.0), ahead_robot("morty"),
        follow_robot("rick"), base_frame("base_link") {
    // Create a transform listener and buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a publisher for the chaser's velocity commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + follow_robot + "/cmd_vel", 10);

    // Create a timer to periodically update the chase's movement
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&RobotChase::chase_callback, this));

    RCLCPP_INFO(this->get_logger(), "Create Publisher : %s",
                follow_robot.c_str());
  }

private:
  float kp_distance_;
  float kp_yaw_;
  float error_yaw_, error_distance_, distance_, angle_;
  std::string ahead_robot;
  std::string follow_robot;
  std::string base_frame;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void chase_callback() {
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
      // Look up the transform from the chaser to the target
      transform_stamped = tf_buffer_->lookupTransform(
          follow_robot + "/" + base_frame, ahead_robot + "/" + base_frame,
          tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // Extract the translation from the transform
    float dx = transform_stamped.transform.translation.x;
    float dy = transform_stamped.transform.translation.y;

    // Calculate distance and angle to the target
    distance_ = std::sqrt(dx * dx + dy * dy);
    angle_ = std::atan2(dy, dx);

    // Create and publish velocity command
    auto vel_msg = geometry_msgs::msg::Twist();
    error_distance_ =
        distance_ * kp_distance_;  // Proportional control with max speed
    error_yaw_ = angle_ * kp_yaw_; // Proportional control for rotation

    vel_msg.linear.x = error_distance_;
    vel_msg.angular.z = error_yaw_;
#if 0
        if(error_distance_ < 0.001 && error_yaw_ < 0.001)
        {
            vel_msg.linear.x = vel_msg.angular.z = 0.0;
        }
#endif
    publisher_->publish(vel_msg);
    // RCLCPP_INFO(this->get_logger(), "Published: linear: %f, angular: %f",
    // vel_msg.linear.x, vel_msg.angular.z);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}