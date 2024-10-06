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
#include <nav_msgs/msg/path.hpp> 
#pragma GCC diagnostic ignored "-Wunused-variable"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");


void compute_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher_,std::vector<geometry_msgs::msg::PoseStamped>& waypoints){

  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  for (size_t i = 0; i < waypoints.size(); i++)
  {
    path.poses.push_back(waypoints[i]);
  }
  publisher_->publish(path);
}



int main(int argc, char * argv[])
{  
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cpp_demo", node_options);  
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

  publisher_ = node->create_publisher<nav_msgs::msg::Path>("/cartesian_path_topic", 10);
  namespace rvt = rviz_visual_tools;

// --------------------------------------- Robot_State && PlanningScene -----------------------------------------------
  std::vector<double> joint_values;
  const std::string PLANNING_GROUP = "Group1";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_;
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  moveit::core::RobotState robot_state(robot_model);
  
  // RVT
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "display_contacts", robot_model);
  
  // Init move_group for trajectory execution 
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);

  // Creating PlanningScene object
  std::shared_ptr<planning_scene::PlanningScene> planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  bool found_ik = false;
  bool isValid = false;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  
// --------------------------------------- Planning_Scene_Srv -----------------------------------------------
  client_ = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  while (!client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
      RCLCPP_INFO(LOGGER, "Waiting for GetPlanningScene service...");
  }
  
  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS){
    auto response = future.get();  // Access the service response
    RCLCPP_INFO(node->get_logger(), "Received planning scene response");
    planning_scene->usePlanningSceneMsg(response->scene);
    robot_state = planning_scene->getCurrentState();

    // Example: print the number of world objects
    size_t num_world_objects = response->scene.world.collision_objects.size();
    RCLCPP_INFO(node->get_logger(), "Number of collision objects in the scene: %ld", num_world_objects);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service get_planning_scene");
  }
// --------------------------------------------------- Path -------------------------------------------------------
  const moveit::core::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("Group1");
  const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform("Drone4");
  RCLCPP_INFO(LOGGER, "Translation: [%f, %f, %f]",end_effector_state.translation().x(), 
              end_effector_state.translation().y(), end_effector_state.translation().z());
  
  geometry_msgs::msg::Pose ee_pose;
  ee_pose.orientation.w = 0;
  ee_pose.orientation.x = 0;
  ee_pose.orientation.y = 0;
  ee_pose.orientation.z = 1.0;
  ee_pose.position.x = end_effector_state.translation().x();
  ee_pose.position.y = end_effector_state.translation().y();
  ee_pose.position.z = end_effector_state.translation().z();


  std::vector<geometry_msgs::msg::PoseStamped> waypoints_vec;

  geometry_msgs::msg::PoseStamped waypoints;

  waypoints.header.frame_id = "Drone4";
  waypoints.pose.position.x = end_effector_state.translation().x();
  waypoints.pose.position.y = end_effector_state.translation().y();
  waypoints.pose.position.z = end_effector_state.translation().z();
  waypoints.pose.orientation.w = 1.0;
  waypoints.pose.orientation.y = 1e-6;
  waypoints.pose.orientation.x = 1e-6;
  waypoints.pose.orientation.z = 1e-6;



  // std::vector<geometry_msgs::msg::Pose> waypoints_vec;

  // geometry_msgs::msg::Pose waypoints;
  // waypoints.position.x = end_effector_state.translation().x();
  // waypoints.position.y = end_effector_state.translation().y() + 0.0;
  // waypoints.position.z = end_effector_state.translation().z() + 0.5;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0.5;
  waypoints.pose.position.z += 0.4;
  waypoints_vec.push_back(waypoints);
  
  waypoints.pose.position.x += 0;
  waypoints.pose.position.y += 0.3;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);
  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0.2;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0.5;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0.5;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0.5;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints); 

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0.5;
  waypoints.pose.position.z += 0;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0.2;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0.2;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0.2;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0.2;
  waypoints_vec.push_back(waypoints);

  waypoints.pose.position.x += 0 ;
  waypoints.pose.position.y += 0;
  waypoints.pose.position.z += 0.2;
  waypoints_vec.push_back(waypoints);
  double timeout = 2.0;
  
// -------------------------------------------------- Planning ---------------------------------------------------------------
  compute_path(publisher_,waypoints_vec);
  // Initializing PlanningPipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
    new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));
  
  moveit::core::RobotState goal_state(robot_model);
  std::vector<double> joint_values1;

  for (std::size_t i = 0; i < waypoints_vec.size(); ++i){

      geometry_msgs::msg::Pose ik_pose;
      ik_pose.position.x += waypoints_vec[i].pose.position.x ;
      ik_pose.position.y += waypoints_vec[i].pose.position.y ;
      ik_pose.position.z += waypoints_vec[i].pose.position.z ;



      auto future = client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS){
        auto response = future.get();  // Access the service response
        RCLCPP_INFO(node->get_logger(), "Received planning scene response");
        planning_scene->usePlanningSceneMsg(response->scene);
        robot_state = planning_scene->getCurrentState();
        goal_state = planning_scene->getCurrentState();
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service get_planning_scene");
      }
 
      found_ik = goal_state.setFromIK(joint_model_group, ik_pose, timeout);
      goal_state.copyJointGroupPositions(joint_model_group, joint_values1);
      goal_state.setJointGroupPositions(joint_model_group, joint_values1);
      if(!planning_scene->isStateValid(goal_state,"Group1")){
        RCLCPP_ERROR(LOGGER, "Goal State is not valid");
      }
      visual_tools.publishRobotState(joint_values1, joint_model_group, rviz_visual_tools::BLUE);
      visual_tools.trigger();

      if (!found_ik ){
        RCLCPP_ERROR(LOGGER, "IK solution not found, retrying...");
      } else {
        planning_interface::MotionPlanRequest req;
        req.pipeline_id = "ompl";
        // req.planner_id = "RRTstar";
        req.planner_id = "RRT";
        req.allowed_planning_time = 10.0;
        req.max_velocity_scaling_factor = 1.0;
        req.num_planning_attempts = 10.0;
        req.group_name = "Group1";
        req.max_acceleration_scaling_factor = 1.0;
        req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -10.0;
        req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z = 10.0;
        std::vector<double> tolerance_pose(3, 0.1);
        std::vector<double> tolerance_angle(3, 0.1);
        planning_interface::MotionPlanResponse res;

        moveit_msgs::msg::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("Drone4", waypoints_vec[i]);
        
        req.goal_constraints.clear();
        req.goal_constraints.push_back(pose_goal);
        if (!planning_pipeline->generatePlan(planning_scene, req, res) || res.error_code_.val != res.error_code_.SUCCESS) {
            RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        } else {
          moveit_msgs::msg::MotionPlanResponse msg;
          visual_tools.publishTrajectoryLine(res.trajectory_,joint_model_group);
          visual_tools.trigger();

          res.getMessage(msg);
          if (move_group_interface.execute(msg.trajectory) == true) {
            RCLCPP_INFO(LOGGER, "Plan executed successfully!");
          }
        }
      }
    }
  
  // Shutdown ROS 
  rclcpp::shutdown();  
  return 0;
}
