#include "xarm/wrapper/xarm_api.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        // コンストラクタ
        Node_Class() : Node("xarm_io"){
            // Publisherの作成
            pub_di_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "digital_in", 10
            );

            // 500ms周期でtimer_pub_callbackを実行するタイマーを作成
            timer_io_ = this->create_wall_timer(
                100ms, std::bind(&Node_Class::timer_pub_callback, this)
            );

            // パラメータの宣言
            this->declare_parameter("robot_ip", "192.168.1.203");

            // パラメータの取得
            this->get_parameter("robot_ip", ip);
            
            // digital input
            arm_ = new XArmAPI(ip);
            sleep_milliseconds(500);
        }
    
        // デストラクタ
        ~Node_Class(){
            delete arm_;
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_di_;
        rclcpp::TimerBase::SharedPtr timer_io_;
        XArmAPI *arm_;
        std::string ip;
        int ret;
        int digital1[8] = {0};
        int digital2[8] = {0};
        std_msgs::msg::Int32MultiArray msg_di;

        // タイマー呼び出し関数（周期的にPublish）
        void timer_pub_callback(){
            if(!arm_) return;

            ret = arm_->get_cgpio_digital(digital1, digital2);
            // printf("get_cgpio_digital, ret=%d", ret);
            // for (int i = 0; i < 8; ++i)
            //     RCLCPP_INFO(this->get_logger(), "io%d=%d", i, digital1[i]);
            //     // printf(", io%d=%d", i, digitals[i]);
            // for (int i = 0; i < 8; ++i)
            //     RCLCPP_INFO(this->get_logger(), "io%d=%d", i+8, digital2[i]);
            //     // printf(", io%d=%d", i+8, digitals2[i]);

            // メッセージの作成
            msg_di.data.resize(16);
            for (int i = 0; i < 8; ++i)
                if(digital1[i]==1)
                    msg_di.data[i] = 0;
                else
                    msg_di.data[i] = 1;
            for (int i = 0; i < 8; ++i)
                if(digital2[i]==0)
                    msg_di.data[i+8] = 0;
                else
                    msg_di.data[i+8] = 1;

            // TopicへPublish
            pub_di_->publish(msg_di);
        }
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // Nodeを作成
    auto node = std::make_shared<Node_Class>();

    // Nodeをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}