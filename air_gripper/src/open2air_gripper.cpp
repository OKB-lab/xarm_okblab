#include <functional>
#include <memory>

#include <hand_control_interfaces/msg/move_hand.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Open2AirGripper : public rclcpp::Node
{
public:
  Open2AirGripper()
  : Node("open2air_gripper")
  {
    air_gripper_client_ = create_client<std_srvs::srv::SetBool>(
      "/airgripper_control");

    hand_control_subscription_ = create_subscription<hand_control_interfaces::msg::MoveHand>(
      "/hand_control", 10,
      std::bind(&Open2AirGripper::hand_control_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting /hand_control commands to /airgripper_control service calls");
  }

private:
  using MoveHand = hand_control_interfaces::msg::MoveHand;
  using SetBool = std_srvs::srv::SetBool;

  void hand_control_callback(const MoveHand::SharedPtr msg)
  {
    const bool is_close = msg->state == static_cast<uint8_t>('C');
    const bool is_open = msg->state == static_cast<uint8_t>('O');

    if (!is_open && !is_close) {
      RCLCPP_WARN(
        get_logger(),
        "Unsupported /hand_control state: %u (expected 'O'=79 or 'C'=67)",
        msg->state);
      return;
    }

    if (!air_gripper_client_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "/airgripper_control service is not ready; command was not sent");
      return;
    }

    auto request = std::make_shared<SetBool::Request>();
    request->data = is_close;
    const char command = is_close ? 'C' : 'O';

    air_gripper_client_->async_send_request(
      request,
      [this, command](rclcpp::Client<SetBool>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(
            get_logger(), "Air gripper command '%c' succeeded: %s",
            command, response->message.c_str());
        } else {
          RCLCPP_ERROR(
            get_logger(), "Air gripper command '%c' failed: %s",
            command, response->message.c_str());
        }
      });
  }

  rclcpp::Subscription<MoveHand>::SharedPtr hand_control_subscription_;
  rclcpp::Client<SetBool>::SharedPtr air_gripper_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Open2AirGripper>());
  rclcpp::shutdown();
  return 0;
}
