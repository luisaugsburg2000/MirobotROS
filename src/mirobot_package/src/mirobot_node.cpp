#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class Mirobot : public rclcpp::Node {
public:
    Mirobot() : Node("mirobot_robot") {
        // Publisher for joint commands
        mirobot_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

        // Subscriber for sphere position
        sphere_sub = this->create_subscription<geometry_msgs::msg::Point>(
            "/sphere_position", 10,
            std::bind(&Mirobot::spherePositionCallback, this, std::placeholders::_1)
        );

        // Initialize joint message
        joint_message = sensor_msgs::msg::JointState();
        joint_message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        joint_message.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Control parameters
        step = 20;
        frequency = 0.1;
        amplitude = 1;

        // Timer to update joint state periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Mirobot::jointCallback, this)
        );
    }

    // Timer callback to update joint positions
    void jointCallback() {
        // Calculate target position and velocity
        target_position = sin(2 * M_PI * frequency * time);
        target_velocity = 2 * M_PI * frequency * cos(2 * M_PI * frequency * time);

        for (double &pos : joint_message.position) {
            pos = target_position;
        }
        for (double &vel : joint_message.velocity) {
            vel = target_velocity;
        }

        mirobot_pub->publish(joint_message);

        // Increment time
        time += step / 1000.0;
    }

private:
    // Callback to handle incoming sphere position data
    void spherePositionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received sphere position: x=%.2f, y=%.2f, z=%.2f",
                    msg->x, msg->y, msg->z);

        // Process the sphere position data as needed
        // For example, you might modify joint targets based on sphere's position.
    }

    // Member variables for joint state publishing
    double frequency, time, step, amplitude, target_position, target_velocity;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mirobot_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_message;

    // Subscriber for receiving sphere position
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sphere_sub;
};

int main(int argc, char **argv) {
    printf("hello world mirobot_package\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mirobot>());
    rclcpp::shutdown();
    return 0;
}

