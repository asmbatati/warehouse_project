#include "attach_shelf/pre_approach_v2.h"

PreApproach::PreApproach()
    : Node{kNodeName},
      scan_subscription_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          kScanTopicName, 1,
          [this](const std::shared_ptr<const sensor_msgs::msg::LaserScan> msg) {
            return scan_cb(msg);
          })},
      odom_subscription_{this->create_subscription<nav_msgs::msg::Odometry>(
          kOdomTopicName, 1,
          [this](const std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
            return odom_cb(msg);
          })},
      twist_publisher_{this->create_publisher<geometry_msgs::msg::Twist>(
          kVelCmdTopicName, 1)},
      client_{
          this->create_client<attach_shelf::srv::GoToLoading>(kServiceName)},
      timer_{this->create_wall_timer(kTimerPeriod,
                                     [this]() { return timer_cb(); })} {
  declare_params();
}

void PreApproach::scan_cb(
    const std::shared_ptr<const sensor_msgs::msg::LaserScan> msg) {
  const auto dist_front{msg->ranges[msg->ranges.size() / 2]};
  const auto range_min{msg->range_min};
  const auto range_max{msg->range_max};

  if (dist_front < range_min || dist_front > range_max) {
    RCLCPP_WARN(this->get_logger(),
                "Ignoring out-of-range laser measurement: %f (range_min:%f, "
                "range_max:%f)",
                dist_front, range_min, range_max);
  };

  dist_to_obstacle_current_ = dist_front;
  received_scan_ = true;
}

void PreApproach::odom_cb(
    const std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
  tf2::Quaternion q{msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};

  double pitch{};
  double roll{};
  double yaw{};

  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  received_odom_ = true;
  rotation_current_ = yaw;
}

bool PreApproach::get_params() {
  double dist_to_obstacle_final{};
  int rotation_final_degrees{};
  bool final_approach{};

  this->get_parameter(kObstacleParamName, dist_to_obstacle_final);
  this->get_parameter(kDegreesParamName, rotation_final_degrees);
  this->get_parameter("final_approach", final_approach);

  if (dist_to_obstacle_final <= 0) {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "Parameter %s must be positive, got: %f",
                     kObstacleParamName, dist_to_obstacle_final);
    return false;
  }

  dist_to_obstacle_final_ = dist_to_obstacle_final;
  rotation_final_ = rotation_final_degrees * kDegreesToRad;
  final_approach_ = final_approach;
  return true;
}

void PreApproach::declare_params() {
  rcl_interfaces::msg::ParameterDescriptor obstacle_param_description{};
  obstacle_param_description.description =
      "Distance from the obstacle (in m) to stop the robot.";
  this->declare_parameter<std::double_t>(kObstacleParamName, 0.5,
                                         obstacle_param_description);

  rcl_interfaces::msg::ParameterDescriptor degrees_param_description{};
  degrees_param_description.description =
      "Degrees to rotate the robot after stopping.";
  this->declare_parameter<int>(kDegreesParamName, 0.0,
                               degrees_param_description);

  rcl_interfaces::msg::ParameterDescriptor final_approach_param_description{};
  final_approach_param_description.description =
      "Sets whether to perform the final approach.";
  this->declare_parameter<bool>("final_approach", true,
                                final_approach_param_description);
}

void PreApproach::timer_cb() {
  geometry_msgs::msg::Twist twist{};

  switch (motion_state_) {
  case MotionState::Premotion:
    RCLCPP_INFO_ONCE(this->get_logger(), "Preparing for motion.");
    if (!received_odom_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Odometry not available.");
      break;
    }
    if (!received_scan_) {
      RCLCPP_WARN(this->get_logger(), "Laser scan info not available.");
      break;
    }
    if (!get_params()) {
      RCLCPP_ERROR(this->get_logger(), "Received invalid parameter values.");
      break;
    }
    motion_state_ = MotionState::MovingForward;
    [[fallthrough]];
  case MotionState::MovingForward:
    RCLCPP_INFO_ONCE(this->get_logger(), "Heading towards wall...");
    if (dist_to_obstacle_current_ > dist_to_obstacle_final_) {
      twist.linear.x = kForwardVel;
      break;
    }
    rotation_total_ = 0.0;
    rotation_last_ = rotation_current_;
    motion_state_ = MotionState::Rotating;
    [[fallthrough]];
  case MotionState::Rotating: {
    RCLCPP_INFO_ONCE(this->get_logger(), "Initiating %f degrees rotation.",
                     rotation_final_ / kDegreesToRad);
    const int sign{rotation_final_ >= 0 ? 1 : -1};
    rotation_total_ += std::atan2(std::sin(rotation_current_ - rotation_last_),
                                  std::cos(rotation_current_ - rotation_last_));
    if (sign * (rotation_total_ - rotation_final_) < 0) {
      twist.angular.z = sign * kAngularVel;
      rotation_last_ = rotation_current_;
      break;
    }
    motion_state_ = MotionState::FinalApproach;
    [[fallthrough]];
  }
  case MotionState::FinalApproach: {
    RCLCPP_INFO_ONCE(this->get_logger(), "Performing final approach.");
    twist_publisher_->publish(geometry_msgs::msg::Twist{});
    auto request{std::make_shared<attach_shelf::srv::GoToLoading::Request>()};
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture
        result_future{};
    std::future_status status{};
    request->attach_to_shelf = final_approach_;
    result_future = client_->async_send_request(request).share();
    status = result_future.wait_for(kFutureTimeout);
    if (status != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "%s service timed out.", kServiceName);
    }
    if (!result_future.get()->complete) {
      RCLCPP_WARN(this->get_logger(), "%s service failed.", kServiceName);
    }
  }
    motion_state_ = MotionState::Finished;
    [[fallthrough]];
  case MotionState::Finished:
    RCLCPP_INFO_ONCE(this->get_logger(), "Motion finished.");
    timer_->cancel();
  };

  twist_publisher_->publish(twist);
}

