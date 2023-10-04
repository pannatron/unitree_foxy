#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "ros2_unitree_legged_msgs/msg/hand_check.hpp"  
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;
int symbol_type_global = 0;  // Global variable to store the symbol_type

void handCheckCallback(const ros2_unitree_legged_msgs::msg::HandCheck::SharedPtr msg) {
    symbol_type_global = msg->hand_type;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;

    auto node = rclcpp::Node::make_shared("node_ros2_walk_example");

    rclcpp::WallRate loop_rate(500);

    long motiontime = 0;
    char option;

    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;

    // rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub =
    //     node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);
    auto hand_check_sub = node->create_subscription<ros2_unitree_legged_msgs::msg::HandCheck>(
        "handcheck", 10, handCheckCallback);  // Create a subscription
    while (rclcpp::ok())
    {

        motiontime += 2;
        

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.level_flag = HIGHLEVEL;
        high_cmd_ros.mode = 0;
        high_cmd_ros.gait_type = 0;
        high_cmd_ros.speed_level = 0;
        high_cmd_ros.foot_raise_height = 0;
        high_cmd_ros.body_height = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yaw_speed = 0.0f;
        high_cmd_ros.reserve = 0;

       
        if (symbol_type_global >= 1 && symbol_type_global <= 8) {
            std::cout << "Received symbol_type is within the range 1-8: " << symbol_type_global << std::endl;
            if (symbol_type_global == 1){
               high_cmd_ros.mode = 5;
            }
            if (symbol_type_global == 2){
               high_cmd_ros.mode = 6;
            }
            if (symbol_type_global == 3){
               high_cmd_ros.mode = 2;
               high_cmd_ros.gait_type = 0;
               high_cmd_ros.velocity[0] = 0.0f; // -1  ~ +1
               high_cmd_ros.body_height = 0.0;
            }
            if (symbol_type_global == 4){
               high_cmd_ros.mode = 2;
               high_cmd_ros.gait_type = 1;
               high_cmd_ros.velocity[0] = 0.4f; // -1  ~ +1
               high_cmd_ros.body_height = 0.0;
            }
            if (symbol_type_global == 5){
               high_cmd_ros.mode = 2;
               high_cmd_ros.gait_type = 2;
               high_cmd_ros.velocity[0] = 0.4f; // -1  ~ +1
               high_cmd_ros.yaw_speed = 0.1;
               high_cmd_ros.foot_raise_height = 0.1;
            }
            if (symbol_type_global == 6){
               high_cmd_ros.mode = 2;
               high_cmd_ros.gait_type = 1;
               high_cmd_ros.velocity[0] = 0.0f; // -1  ~ +1
               high_cmd_ros.yaw_speed = 0.0;
               high_cmd_ros.foot_raise_height = 0.2;
            }
            if (symbol_type_global == 7){
               high_cmd_ros.mode = 2;
               high_cmd_ros.gait_type = 1;
               high_cmd_ros.velocity[1] = 0.2f; // -1  ~ +1
               high_cmd_ros.yaw_speed = 0.0;
               high_cmd_ros.foot_raise_height = 0.2;
            }
            
            if (symbol_type_global == 8){
               high_cmd_ros.mode = 2;
               high_cmd_ros.gait_type = 1;
               high_cmd_ros.velocity[1] = -0.2f; // -1  ~ +1
               high_cmd_ros.yaw_speed = 0.0;
               high_cmd_ros.foot_raise_height = 0.2;
            }
         
            
        }
        pub->publish(high_cmd_ros);

        rclcpp::spin_some(node);

        loop_rate.sleep();
    }

    return 0;
}
