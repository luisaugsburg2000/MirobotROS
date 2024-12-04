#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>
  (
    "move_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("move_program");

  // static const std::string PLANNING_GROUP = "arm";
  // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);



  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  bool use_sim_time = node->get_parameter("use_sim_time").as_bool();
  RCLCPP_INFO(logger, "use_sim_time: %s", use_sim_time ? "true" : "false");
  
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "arm"); // --> change "panda_arm" too "arm" 

  

  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -3.141/2); // roll pitch yaw
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

  geometry_msgs::msg::Pose GoalPose;
  GoalPose.orientation = msg_quat;
  GoalPose.position.x = 0.2;
  GoalPose.position.y = 0;
  GoalPose.position.z = 0.2;
  move_group_interface.setPoseTarget(GoalPose);
  move_group_interface.setPlanningTime(10.0);
  RCLCPP_INFO(logger, "target pose set");

  // const std::string named_goal = "arm_stretched";
  // move_group_interface.setNamedTarget(named_goal);


  // GET JOINT NAMES
  const std::vector<std::string> jointNames = move_group_interface.getJointNames();
  std::ostringstream jointNamesStream;
  for (const auto& joint : jointNames) {
    jointNamesStream << joint << ", ";
  }
  std::string jointNamesStr = "Joint Names: " + jointNamesStream.str();
  RCLCPP_INFO(logger, "%s", jointNamesStr.c_str());

  // GET END EFFECTOR LINK
  RCLCPP_INFO(logger, "%s", move_group_interface.getEndEffectorLink().c_str());

  // GET CURRENT JOINT VALUES
  const std::vector<double> jointAngles = move_group_interface.getCurrentJointValues();
  std::ostringstream jointAnglesStream;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    jointAnglesStream << jointNames[i] << ": " << jointAngles[i] << " rad, ";
  }
  std::string jointAnglesStr = "Joint Angles: " + jointAnglesStream.str();
  RCLCPP_INFO(logger, "%s", jointAnglesStr.c_str());

  // // GET RANDOM POSE 
  // geometry_msgs::msg::PoseStamped randomPose = move_group_interface.getRandomPose();
  // RCLCPP_INFO(logger, "Random Pose: Position [x: %f, y: %f, z: %f], Orientation [x: %f, y: %f, z: %f, w: %f]",
  //             randomPose.pose.position.x,
  //             randomPose.pose.position.y,
  //             randomPose.pose.position.z,
  //             randomPose.pose.orientation.x,
  //             randomPose.pose.orientation.y,
  //             randomPose.pose.orientation.z,
  //             randomPose.pose.orientation.w);


  // // CREATE A PLAN
  // moveit::planning_interface::MoveGroupInterface::Plan plan1;
  // auto const outcome = static_cast<bool>(move_group_interface.plan(plan1));
  // if(outcome){
  //   move_group_interface.execute(plan1);
  // }
  // else{
  //   RCLCPP_ERROR(logger, "Not able to plan and execute");
  // }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 



// EXAMPLE FROM THIS MOVEIT TUTORIAL:
// https://moveit.picknik.ai/humble/doc/examples/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html
// doesn't work either...

// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   // This enables loading undeclared parameters
//   // best practice would be to declare parameters in the corresponding classes
//   // and provide descriptions about expected use
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto node = rclcpp::Node::make_shared("move_program", node_options);
//   const auto& LOGGER = node->get_logger();


//   robot_model_loader::RobotModelLoader robot_model_loader(node);
//   const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//   RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());


//   moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
//   kinematic_state->setToDefaultValues();
//   const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
//   const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

//   std::vector<double> joint_values;
//   kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//   for (std::size_t i = 0; i < joint_names.size(); ++i)
//   {
//     RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//   }
// }