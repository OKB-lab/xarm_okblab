#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>

using namespace std::chrono_literals;

class MoveContCartPos : public rclcpp::Node
{
public:
  MoveContCartPos() : Node("move_cont_cart_pos")
  {
    poses_.push_back(declare_parameter<std::vector<double>>(
      "pose1", {300.0, -200.0, 400.0, 0.0, 1.5708, 0.0}));
    poses_.push_back(declare_parameter<std::vector<double>>(
      "pose2", {400.0, -300.0, 400.0, 0.0, 1.5708, 0.0}));
    poses_.push_back(declare_parameter<std::vector<double>>(
      "pose3", {300.0, -400.0, 400.0, 0.0, 1.5708, 0.0}));
    poses_.push_back(declare_parameter<std::vector<double>>(
      "pose4", {200.0, -300.0, 400.0, 0.0, 1.5708, 0.0}));
    speed_ = declare_parameter<double>("speed", 100.0);
    acc_ = declare_parameter<double>("acc", 1000.0);
    mvtime_ = declare_parameter<double>("mvtime", 0.0);
    timeout_ = declare_parameter<double>("timeout", -1.0);
    radius_ = declare_parameter<double>("radius", -1.0);
    motion_type_ = declare_parameter<int>("motion_type", 0);

    for (std::size_t i = 0; i < poses_.size(); ++i) {
      if (poses_[i].size() != 6) {
        throw std::invalid_argument(
                "parameter 'pose" + std::to_string(i + 1) +
                "' must contain 6 values: [x, y, z, roll, pitch, yaw]");
      }
    }
    if (motion_type_ < 0 || motion_type_ > 2) {
      throw std::invalid_argument("parameter 'motion_type' must be 0, 1, or 2");
    }

    set_position_client_ = create_client<xarm_msgs::srv::MoveCartesian>("xarm/set_position");

    // This timer is used only to wait for the service at startup. Motion calls
    // themselves are chained from the completion callback below.
    service_wait_timer_ = create_wall_timer(
      500ms, std::bind(&MoveContCartPos::start_when_service_is_ready, this));

    RCLCPP_INFO(
      get_logger(),
      "Repeating %zu poses in order after each motion completes", poses_.size());
  }

private:
  void start_when_service_is_ready()
  {
    if (!set_position_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "xarm/set_position service is not available; waiting...");
      return;
    }

    service_wait_timer_->cancel();
    call_set_position();
  }

  void call_set_position()
  {
    if (request_in_flight_) {
      return;
    }

    if (!set_position_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "xarm/set_position service became unavailable; stopping");
      return;
    }

    const auto & pose = poses_[current_pose_index_];
    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
      request->pose.assign(pose.begin(), pose.end());
      request->speed = static_cast<float>(speed_);
      request->acc = static_cast<float>(acc_);
      request->mvtime = static_cast<float>(mvtime_);
      // The service must wait until the commanded motion has actually ended.
      request->wait = true;
      request->timeout = static_cast<float>(timeout_);
      request->radius = static_cast<float>(radius_);
      request->motion_type = static_cast<std::uint8_t>(motion_type_);

    RCLCPP_INFO(
      get_logger(),
      "Calling pose%zu: [%.3f, %.3f, %.3f, %.4f, %.4f, %.4f]",
      current_pose_index_ + 1,
      pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    request_in_flight_ = true;
    set_position_client_->async_send_request(
      request,
      [this](rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedFuture future) {
        request_in_flight_ = false;
        const auto response = future.get();
        if (response->ret != 0) {
          RCLCPP_ERROR(
            get_logger(), "xarm/set_position failed: ret=%d, message=%s",
            response->ret, response->message.c_str());
          return;
        }

        // The previous motion has completed successfully; send the next one now.
        current_pose_index_ = (current_pose_index_ + 1) % poses_.size();
        call_set_position();
      });
  }

  rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr set_position_client_;
  rclcpp::TimerBase::SharedPtr service_wait_timer_;

  std::vector<std::vector<double>> poses_;
  std::size_t current_pose_index_{0};
  double speed_;
  double acc_;
  double mvtime_;
  double timeout_;
  double radius_;
  int motion_type_;
  bool request_in_flight_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MoveContCartPos>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("move_cont_cart_pos"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
