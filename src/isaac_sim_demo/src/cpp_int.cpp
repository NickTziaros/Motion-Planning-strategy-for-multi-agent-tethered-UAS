#include <pluginlib/class_loader.hpp>
#include <math.h>


// MoveIt
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_state/conversions.h>  
#include <moveit/move_group_interface/move_group_interface.h>

#pragma GCC diagnostic ignored "-Wunused-variable"



static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");












int main(int argc, char * argv[])
{  





  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("cpp_demo", node_options);  rclcpp::executors::SingleThreadedExecutor executor;

  namespace rvt = rviz_visual_tools;

// --------------------------------------- Robot_State && PlanningScene ----------------------------------------------------------------
  std::vector<double> joint_values;
  const std::string PLANNING_GROUP = "Group1";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));
  
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_;


  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  moveit::core::RobotState robot_state(robot_model);
  // RVT
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link","display_contacts",robot_model);
  // Init move_group for trajectory execution 
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);

  // Creating PlanningScene object
  std::shared_ptr<planning_scene::PlanningScene> planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  bool found_ik = false;
  bool isValid = false;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  
// --------------------------------------- Planning_Scene_Srv ----------------------------------------------------------------

  client_ = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  while (!client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
      RCLCPP_INFO(LOGGER, "Waiting for GetPlanningScene service...");
    }
  
  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS){
    auto response = future.get();  // Access the service response
    RCLCPP_INFO(node->get_logger(), "Received planning scene response");
    //Using planning scene msg from srv to use planning scene 
    planning_scene->usePlanningSceneMsg(response->scene);
    // Example: print the number of world objects
    size_t num_world_objects = response->scene.world.collision_objects.size();
    RCLCPP_INFO(node->get_logger(), "Number of collision objects in the scene: %ld", num_world_objects);
    // moveit::core::RobotState new_state(planning_scene->getCurrentState());
    robot_state = planning_scene->getCurrentState();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service get_planning_scene");
  }
// --------------------------------------------------- Path ------------------------------------------------------------------

  const moveit::core::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("Group1");
  const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform("Drone4");
  RCLCPP_INFO(LOGGER, "Translation: [%f, %f, %f]",end_effector_state.translation().x(), end_effector_state.translation().y(), end_effector_state.translation().z());
  
  geometry_msgs::msg::Pose ee_pose;
  ee_pose.orientation.w = 1.0;
  ee_pose.position.x = end_effector_state.translation().x();
  ee_pose.position.y = end_effector_state.translation().y();
  ee_pose.position.z = end_effector_state.translation().z();

  std::vector<geometry_msgs::msg::Pose> waypoints_vec;

  geometry_msgs::msg::Pose waypoints;
  waypoints.position.x = end_effector_state.translation().x();
  waypoints.position.y = end_effector_state.translation().y()+0.5;
  waypoints.position.z = end_effector_state.translation().z()+0.5;
  // waypoints_vec.push_back(waypoints);

  double timeout = 2.0;
  
// -------------------------------------------------- Planning ---------------------------------------------------------------

  // Initializing PlanningPipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
    new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));



  moveit::core::RobotState goal_state(robot_model);
  std::vector<double> joint_values1;
  // Set the goal state to current state in order to add or subtract from there
  goal_state = planning_scene->getCurrentState();
  // while ((!found_ik || !isValid) && rclcpp::ok())
  // {

    found_ik = goal_state.setFromIK(joint_model_group, waypoints,timeout);
    goal_state.copyJointGroupPositions(joint_model_group, joint_values1);
    goal_state.setJointGroupPositions(joint_model_group,joint_values1);
    visual_tools.publishRobotState(joint_values1,joint_model_group,rviz_visual_tools::BLUE);
    visual_tools.trigger();

    if (!found_ik ){
      RCLCPP_ERROR(LOGGER, "IK solution not found retrying ");
    }
    else
    {
      planning_interface::MotionPlanRequest req;
      req.pipeline_id = "ompl";
      req.planner_id = "RRTstar";
      req.allowed_planning_time = 5.0;
      req.max_velocity_scaling_factor = 1.0;
      req.num_planning_attempts = 10.0;
      req.group_name = "Group1";
      req.max_acceleration_scaling_factor = 1.0;
      req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
      req.workspace_parameters.min_corner.z = -10.0;
      req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
      req.workspace_parameters.max_corner.z = 10.0;
      double tolerance_below = 0.001;
      double tolerance_above = 0.001;
      planning_interface::MotionPlanResponse res;
      moveit_msgs::msg::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group,tolerance_above,tolerance_below);

      req.goal_constraints.clear();
      req.goal_constraints.push_back(joint_goal);



      if (!planning_pipeline->generatePlan(planning_scene, req, res) || res.error_code_.val != res.error_code_.SUCCESS){

          // planning_scene->isPathValid(&res.trajectory_,"Group1");
          RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        }
      else
      {
        moveit_msgs::msg::MotionPlanResponse msg;
        res.getMessage(msg);
        if (move_group_interface.execute(msg.trajectory) == true){
          RCLCPP_INFO(LOGGER, "Plan executed successfully!");
        }
      }

    }

      // Shutdown ROS 
    rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
    return 0;
}  
















    // bool isValid = planning_scene->isStateValid(goal_state,"Group1",true);	
      // RCLCPP_ERROR(LOGGER, "found_ik: %s", found_ik ? "true" : "false");
      // RCLCPP_ERROR(LOGGER, "isValid: %s", isValid ? "true" : "false");
      // if (!isValid)
      // {
      //   RCLCPP_ERROR(LOGGER, "IK solution found but not valid retrying ");
      //   // found_ik = goal_state.setFromIK(joint_model_group, waypoints,timeout);
      //   // goal_state.copyJointGroupPositions(joint_model_group, joint_values1);
      // }
      // else
      // {
      //   RCLCPP_INFO(LOGGER, "IK solution found and valid ");

        
      // }


        // }
    // }
    // else
    // {
      // RCLCPP_ERROR(LOGGER, "IK solution not found retrying ");
    // }