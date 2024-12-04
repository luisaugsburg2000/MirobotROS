#ifndef MOVE_PROGRAM_M2_IFACE_HPP
#define MOVE_PROGRAM_M2_IFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <thread>

namespace move_program {

class m2Iface : public rclcpp::Node {
public:
    explicit m2Iface(const rclcpp::NodeOptions &options);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_moveGroupPtr;

    std::string groupName = "arm";
    std::string EE_LINK_NAME = "end_effector";
    std::string PLANNING_FRAME = "base_link";
};

} // namespace move_program

#endif // MOVE_PROGRAM_M2_IFACE_HPP
