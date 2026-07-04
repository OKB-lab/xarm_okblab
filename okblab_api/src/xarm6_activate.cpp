#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/set_int16_by_id.hpp>
#include <xarm_msgs/srv/set_int16.hpp>

using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        Node_Class() : Node("move_cart_vel"){
            // Parameterの宣言
            mode_ = this->declare_parameter<int>("mode", 0);

            // Clientの作成
            motion_enable_ = this->create_client<xarm_msgs::srv::SetInt16ById>("xarm/motion_enable");
            set_mode_ = this->create_client<xarm_msgs::srv::SetInt16>("xarm/set_mode");
            set_state_ = this->create_client<xarm_msgs::srv::SetInt16>("xarm/set_state");
        }

        bool activate(){
            // Enable motion
            return call_motion_enable() && call_set_mode(mode_) && call_set_state();
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedPtr motion_enable_;
        rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_;
        rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_state_;
        int mode_;

        bool call_motion_enable(){
            auto request = std::make_shared<xarm_msgs::srv::SetInt16ById::Request>();
            request->id = 8;
            request->data = 1;

            while (!motion_enable_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the motion_enable service. Exiting.");
                return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the motion_enable service not available, waiting again...");
            }

            auto result = motion_enable_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive motion_enable response.");
                return false;
            }

            const auto response = result.get();
            if (response->ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "motion_enable failed: ret=%d, message=%s",
                    response->ret, response->message.c_str());
                return false;
            }
            return true;
        }

        bool call_set_mode(int mode){
            auto request = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
            request->data = mode;

            while (!set_mode_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the set_mode service. Exiting.");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the set_mode service not available, waiting again...");
            }

            auto result = set_mode_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive set_mode response.");
                return false;
            }

            const auto response = result.get();
            if (response->ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "set_mode failed: ret=%d, message=%s",
                    response->ret, response->message.c_str());
                return false;
            }
            return true;
        }

        bool call_set_state(){
            auto request = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
            request->data = 0;

            while (!set_state_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the set_state service. Exiting.");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the set_state service not available, waiting again...");
            }

            auto result = set_state_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive set_state response.");
                return false;
            }

            const auto response = result.get();
            if (response->ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "set_state failed: ret=%d, message=%s",
                    response->ret, response->message.c_str());
                return false;
            }
            return true;
        }

};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // Nodeを作成
    auto node = std::make_shared<Node_Class>();

    if (!node->activate()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to activate xArm.");
        rclcpp::shutdown();
        return 1;
    }

    // Nodeをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}
