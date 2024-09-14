#include <pluginlib/class_loader.hpp>
#include <math.h>


// MoveIt

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_state/conversions.h>  



static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");


int main(int argc, char * argv[])
{  





  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("cpp_demo", node_options);  rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(node);
  // std::thread([&executor]() { executor.spin(); }).detach();

// --------------------------------------- Robot_State && PlanningScene ----------------------------------------------------------------
  std::vector<double> joint_values;
  const std::string PLANNING_GROUP = "Group1";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_;


  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  moveit::core::RobotState robot_state(robot_model);


  // Creating PlanningScene object
  std::shared_ptr<planning_scene::PlanningScene> planning_scene = 
    std::make_shared<planning_scene::PlanningScene>(robot_model);

// --------------------------------------- Planning_Scene_Srv ----------------------------------------------------------------
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();


  client_ = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  while (!client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) 
        {
            RCLCPP_INFO(LOGGER, "Waiting for GetPlanningScene service...");
        }
  

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
          {
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






// ---------------------------------------Planning_Scene_Monitor----------------------------------------------------------------



      // Get the robot state from the locked planning scene
  const moveit::core::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("Group1");





  const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform("Drone4");
  Eigen::Isometry3d new_transform = end_effector_state;
  // Increase X by 0.1 meters
  new_transform.translation().x() += end_effector_state.translation().x()+0.1;
  
  double timeout = 0.1;
  bool found_ik = robot_state.setFromIK(joint_model_group, new_transform, timeout);
  
  // bool isValid = planning_scene->isStateValid(robot_state,"Group1");	

    // Initializing PlanningPipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
    new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));
    

  if (found_ik )
  {
    bool isValid = planning_scene->isStateValid(robot_state,"Group1");	

    if (isValid)
    {
      RCLCPP_INFO(LOGGER, "IK solution found and is valid");
    
    
    
      planning_interface::MotionPlanRequest req;
      req.pipeline_id = "ompl";
      req.planner_id = "RRTConnect";
      req.allowed_planning_time = 10.0;
      req.max_velocity_scaling_factor = 1.0;
      req.max_acceleration_scaling_factor = 1.0;
      req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
      req.workspace_parameters.min_corner.z = -10.0;
      req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
      req.workspace_parameters.max_corner.z = 10.0;
      
      planning_interface::MotionPlanResponse res;
      // geometry_msgs::msg::PoseStamped pose;
      // pose.header.frame_id = "base_link";
      // pose.pose.position.x = end_effector_state.translation().x()+0.1;
      // pose.pose.position.y = end_effector_state.translation().y()+0.1;
      // pose.pose.position.z = end_effector_state.translation().z()+0.1;
      // pose.pose.orientation.w = 1.0;

      moveit::core::RobotState goal_state(robot_model);

      req.group_name = "Group1";
      robot_state.copyJointGroupPositions(joint_model_group, joint_values);
      goal_state.setJointGroupPositions(joint_model_group, joint_values);
      moveit_msgs::msg::Constraints joint_goal =
          kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

      req.goal_constraints.clear();
      req.goal_constraints.push_back(joint_goal);



      if (!planning_pipeline->generatePlan(planning_scene, req, res) || res.error_code_.val != res.error_code_.SUCCESS)
      {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        rclcpp::shutdown();
        return -1;
      }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "IK solution found but it is not valid ");
    }
    














  }
  else

  {
    RCLCPP_ERROR(LOGGER, "Did not find IK solution");
  }
  // RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation().x()+0.1 << "\n");
  // RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");















// ---------------------------------------Planning_Req----------------------------------------------------------------


  


























// // ---------------------------------------RVT----------------------------------------------------------------
// // Visualization
//   // ^^^^^^^^^^^^^
//   // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
//   namespace rvt = rviz_visual_tools;
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "markers", psm);
  // visual_tools.enableBatchPublishing();
  // visual_tools.deleteAllMarkers();  // clear all old markers
  // visual_tools.trigger();

  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().x() = pose.pose.position.x;
  // text_pose.translation().y() = pose.pose.position.y;
  // text_pose.translation().z() = pose.pose.position.z+0.4;
  // visual_tools.publishAxis(pose.pose,rvt::XXLARGE);
  // visual_tools.publishText(text_pose, "",rvt::WHITE, rvt::XXXLARGE);
//   moveit_msgs::msg::Constraints pose_goal =
//     kinematic_constraints::constructGoalConstraints("Drone4", pose, tolerance_pose, tolerance_angle);

  // rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
  //     node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
  // moveit_msgs::msg::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  // RCLCPP_INFO(LOGGER, "Visualizing the trajectory");
  // moveit_msgs::msg::MotionPlanResponse response;
  // res.getMessage(response);

  // display_trajectory.trajectory_start = response.trajectory_start;
  // display_trajectory.trajectory.push_back(response.trajectory);
  // display_publisher->publish(display_trajectory);
  // visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  // visual_tools.trigger();






  // std::vector<double> tolerance_pose(3, 0.1);
  // std::vector<double> tolerance_angle(3, 0.1);
  // req.group_name = "Group1";
  //   moveit_msgs::msg::Constraints pose_goal =
  //       kinematic_constraints::constructGoalConstraints("Drone4", pose, tolerance_pose, tolerance_angle);
  //   req.goal_constraints.push_back(pose_goal);
  //   {
  //     planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
  //     /* Now, call the pipeline and check whether planning was successful. */
  //     /* Check that the planning was successful */
  //     if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code_.val != res.error_code_.SUCCESS)
  //     {
  //       RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
  //       rclcpp::shutdown();
  //       return -1;
  //     }
  //   }

















































// // ----------------------------------------------------------------------------------------------------------


  


      // Shutdown ROS 
    rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
    return 0;
}