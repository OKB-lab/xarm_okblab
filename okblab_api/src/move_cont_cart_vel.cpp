#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/msg/move_velocity.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        Node_Class() : Node("move_cont_cart_vel"){
            // Parameterの宣言と取得
            kp_ = this->declare_parameter<double>("Kp", 1.0);
            R_ = this->declare_parameter<int>("R", 100);
            w_ = this->declare_parameter<double>("W", 3.14);
            center_x_ = this->declare_parameter<int>("center_x", 200);
            center_y_ = this->declare_parameter<int>("center_y", 0);
            center_z_ = this->declare_parameter<int>("center_z", 300);
            q_w_ = this->declare_parameter<double>("q_w", 1.0);
            q_x_ = this->declare_parameter<double>("q_x", 0.0);
            q_y_ = this->declare_parameter<double>("q_y", 0.0);
            q_z_ = this->declare_parameter<double>("q_z", 0.0);

            // Initialise
            tcp_position_ref_.setValue(center_x_, center_y_ + R_, center_z_);
            tcp_orientation_ref_.setValue(q_w_, q_x_, q_y_, q_z_);

            // Publisherの作成
            vc_set_cartesian_velocity_ = this->create_publisher<xarm_msgs::msg::MoveVelocity>(
                "xarm/vc_set_cartesian_velocity", 10
            );

            // Subscriberの作成
            robot_states_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
                "xarm/robot_states", 10, std::bind(&Node_Class::sub_robot_states, this, std::placeholders::_1)
            );

            // Timerの作成
            timer_pub_ = this->create_wall_timer(100ms, std::bind(&Node_Class::timer_pub_callback, this));
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Publisher<xarm_msgs::msg::MoveVelocity>::SharedPtr vc_set_cartesian_velocity_;
        rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_states_;
        rclcpp::TimerBase::SharedPtr timer_pub_;

        tf2::Vector3 tcp_position_ref_;
        tf2::Quaternion tcp_orientation_ref_;
        tf2::Vector3 tcp_position_;
        tf2::Quaternion tcp_orientation_;
        tf2::Vector3 tcp_velocity_;
        tf2::Quaternion tcp_angular_velocity_;
        
        long time_cnt_ = 0;
        bool is_subscribe_states_ = false;
        double angular_velocity_;

        double kp_;
        int R_;
        double w_;
        int center_x_;
        int center_y_;
        int center_z_;
        double q_w_;
        double q_x_;
        double q_y_;
        double q_z_;

        // Subscriberのコールバック関数
        void sub_robot_states(const xarm_msgs::msg::RobotMsg::SharedPtr msg){
            tcp_position_.setX(msg->pose[0]);
            tcp_position_.setY(msg->pose[1]);
            tcp_position_.setZ(msg->pose[2]);

            tcp_orientation_.setRPY(msg->pose[3], msg->pose[4], msg->pose[5]);

            is_subscribe_states_ = true;
        }

        // Timerのコールバック関数
        void timer_pub_callback(){
            if(is_subscribe_states_){
                // TCP position reference
                tcp_position_ref_.setX(R_ * sin(w_ * time_cnt_ * 0.1) + center_x_);
                tcp_position_ref_.setY(R_ * cos(w_ * time_cnt_ * 0.1) + center_y_);
                tcp_position_ref_.setZ(center_z_);

                // TCP veclocity
                tcp_velocity_.setX(tcp_position_ref_.getX() - tcp_position_.getX());
                tcp_velocity_.setY(tcp_position_ref_.getY() - tcp_position_.getY());
                tcp_velocity_.setZ(tcp_position_ref_.getZ() - tcp_position_.getZ());

                // TCP angular velocity
                tcp_angular_velocity_ = tcp_orientation_ref_ * tcp_orientation_.inverse();
                tcp_angular_velocity_.normalize();

                if(tcp_angular_velocity_.getW() < 0.0){
                    tcp_angular_velocity_.setX(-tcp_angular_velocity_.getX());
                    tcp_angular_velocity_.setY(-tcp_angular_velocity_.getY());
                    tcp_angular_velocity_.setZ(-tcp_angular_velocity_.getZ());
                    tcp_angular_velocity_.setW(-tcp_angular_velocity_.getW());
                }

                angular_velocity_ = 2.0 * acos(tcp_angular_velocity_.getW());
                if(angular_velocity_ > 1e-6){
                    tcp_angular_velocity_.setX(tcp_angular_velocity_.getX() / sin(angular_velocity_ / 2.0) * angular_velocity_);
                    tcp_angular_velocity_.setY(tcp_angular_velocity_.getY() / sin(angular_velocity_ / 2.0) * angular_velocity_);
                    tcp_angular_velocity_.setZ(tcp_angular_velocity_.getZ() / sin(angular_velocity_ / 2.0) * angular_velocity_);
                }

                auto msg = xarm_msgs::msg::MoveVelocity();
                    msg.speeds.resize(6);

                    msg.speeds[0] = kp_ * tcp_velocity_.getX();
                    msg.speeds[1] = kp_ * tcp_velocity_.getY();
                    msg.speeds[2] = kp_ * tcp_velocity_.getZ();
                    msg.speeds[3] = kp_ * tcp_angular_velocity_.getX();
                    msg.speeds[4] = kp_ * tcp_angular_velocity_.getY();
                    msg.speeds[5] = kp_ * tcp_angular_velocity_.getZ();
                    msg.duration = 0.5;

                vc_set_cartesian_velocity_->publish(msg);

                // time_cnt_のインクリメント
                time_cnt_++;
            }
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
