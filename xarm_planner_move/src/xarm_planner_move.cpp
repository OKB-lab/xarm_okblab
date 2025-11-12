#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        geometry_msgs::msg::Pose target_pose;

        // コンストラクタ
        Node_Class() : Node("xarm_planner_move"){
            client_plan_ = this->create_client<xarm_msgs::srv::PlanPose>("xarm_pose_plan");
            client_exec_ = this->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");
        }
        
        // Service呼び出し関数
        void call_service_plan(){
            // Serviceに送るRequestを作成
            std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();;
            req->target = target_pose;

            // ServiceへRequestを送信
            auto result = client_plan_->async_send_request(req);
        }

        void call_service_exec(){
            std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();;
            req->wait = true;

            auto result = client_exec_->async_send_request(req);
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr client_plan_;
        rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr client_exec_;
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // Nodeを作成
    auto node = std::make_shared<Node_Class>();
    rclcpp::sleep_for(1s);

    // 動作指令生成
    node->target_pose.position.x = 0.3;     //[m]
    node->target_pose.position.y = 0.0;       //[m]
    node->target_pose.position.z = 0.3;     //[m]
    node->target_pose.orientation.w = 0.0;    // quoternion
    node->target_pose.orientation.x = 1.0;
    node->target_pose.orientation.y = 0.0;
    node->target_pose.orientation.z = 0.0;

    // Serviceの呼び出し
    node->call_service_plan();
    rclcpp::sleep_for(5s);
    node->call_service_exec();

    // Nodeをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}