
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
void printCurrentPos (moveit::planning_interface::MoveGroupInterface  &move_group);
void moveArmTo(double x, double y, double z, 
  moveit::planning_interface::MoveGroupInterface &move_group,
  moveit_visual_tools::MoveItVisualTools visual_tools);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

 printCurrentPos(move_group);

 double x, y, z;
 int userInput;
  while (userInput != 6){

    
    std::cout << "1. Show current pose\n";
    std::cout << "2. Move to\n";
    std::cout << "6. Exit \n";
    std::cin >> userInput;

  switch(userInput) {
      case 1  :
        printCurrentPos(move_group);
        break; //optional
      case 2  :
      { 
      
      printf("Enter x\n");
      std::cin >>x;
      printf("Enter y\n");
      std::cin >>y;
      printf("Enter z\n");
      std::cin >>z;
      moveArmTo(x,y,z, move_group, visual_tools);
        
        break; //optional
      }
    case 3 : 
      
      break;

    case 4 : 
    {
 
      break;
    }

    }
  }

  ros::shutdown();
  return 0;
}

void printCurrentPos (moveit::planning_interface::MoveGroupInterface &move_group){
   geometry_msgs::Point p = move_group.getCurrentPose(move_group.getEndEffectorLink().c_str()).pose.position;
  
  std::cout<< "p.x: " << p.x << std::endl;
  std::cout<< "p.y: " << p.y << std::endl;
  std::cout<< "p.z: " << p.z << std::endl;
  geometry_msgs::Quaternion q = move_group.getCurrentPose(move_group.getEndEffectorLink().c_str()).pose.orientation;
  std::cout<< "q.x: " << q.x << std::endl;
  std::cout<< "q.y: " << q.y << std::endl;
  std::cout<< "q.z: " << q.z << std::endl;
  std::cout<< "q.w: " << q.w << std::endl;
}

 void moveArmTo (double x, double y, double z, 
  moveit::planning_interface::MoveGroupInterface &move_group,
  moveit_visual_tools::MoveItVisualTools visual_tools){

  move_group.setPositionTarget(x,y,z);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


  visual_tools.prompt("Press 'next' to move");
  move_group.move();

}
