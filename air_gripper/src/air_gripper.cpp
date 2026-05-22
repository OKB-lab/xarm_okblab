#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>

using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        Node_Class() : Node("air_gripper"){
            // subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            //     "turtle1/pose", 10,
            //     std::bind(&Node_Class::subscribe_callback_pose,this, std::placeholders::_1)
            // );

            server_griper_ = this->create_service<std_srvs::srv::SetBool>(
                "airgripper_control",
                std::bind(&Node_Class::gripper_control_callback, this, std::placeholders::_1, std::placeholders::_2)
            );

            client_tgpio_ = this->create_client<xarm_msgs::srv::SetDigitalIO>("xarm/set_tgpio_digital");
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_griper_;
        rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client_tgpio_;

        // Service呼び出し関数（Service呼び出し時に実行）
        void gripper_control_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response){
                if(request->data){
                    call_close_gripper_service();
                    response->success = true;
                    response->message = "Gripper closed.";
                } else {
                    call_open_gripper_service();
                    response->success = true;
                    response->message = "Gripper opened.";
                }
             }

        // // Subscriber呼び出し関数（Subscribe時に実行）
        // void subscribe_callback_pose(const turtlesim::msg::Pose::SharedPtr msg){
        //     pose.x = msg->x;
        //     pose.y = msg->y;
        // }

        // クラス内関数
        void call_open_gripper_service(){
            auto request1 = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
            request1->ionum = 1;
            request1->value = 0;
            
            auto request2 = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
            request2->ionum = 0;
            request2->value = 1;

            auto result1 = client_tgpio_->async_send_request(request1);
            auto result2 = client_tgpio_->async_send_request(request2);
        }

        void call_close_gripper_service(){
            auto request1 = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
            request1->ionum = 0;
            request1->value = 0;
            
            auto request2 = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
            request2->ionum = 1;
            request2->value = 1;

            auto result1 = client_tgpio_->async_send_request(request1);
            auto result2 = client_tgpio_->async_send_request(request2);
        }
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // Nodeを作成
    auto node = std::make_shared<Node_Class>();
    rclcpp::sleep_for(1s);

    // Nodeをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}