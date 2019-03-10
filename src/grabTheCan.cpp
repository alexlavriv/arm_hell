
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++
#include <iostream>
#include <string>
#include <sstream>


void pick(moveit::planning_interface::MoveGroupInterface& move_group, double *a);
void printCurrentPos (moveit::planning_interface::MoveGroupInterface  &move_group);
void closedGripper(trajectory_msgs::JointTrajectory& posture);
void openGripper(trajectory_msgs::JointTrajectory& posture);
void place(moveit::planning_interface::MoveGroupInterface& group, double *a);
double * tokenize(std::string &input);

void place(moveit::planning_interface::MoveGroupInterface& group, double *a)
{

    //float a[] = {0.659703, 0.19,  0.48};
    bool success = false;
    // BEGIN_SUB_TUTORIAL place
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in
    // verbose mode." This is a known issue and we are working on fixing it. |br|
    // Create a vector of placings to be attempted, currently only creating single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "base_footprint";
    // tf2::Quaternion orientation;
    // orientation.setRPY(0, 0, M_PI / 2);
    // place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* While placing it is the exact location of the center of the object. */



    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "base_footprint";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.1;
    place_location[0].pre_place_approach.desired_distance = 1.0;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    // place_location[0].post_place_retreat.direction.header.frame_id = "base_footprint";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = 0.0;
    place_location[0].post_place_retreat.min_distance = 0.0;
    place_location[0].post_place_retreat.desired_distance = 0.0;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table1");

    // Call place to place the object using the place locations given.
    int k=1;
    //while (!success){
        printf("%s\n","Begin placing" );
        place_location[0].place_pose.pose.position.x = a[0];
        place_location[0].place_pose.pose.position.y = a[1];
        place_location[0].place_pose.pose.position.z = a[2];


        moveit::planning_interface::MoveItErrorCode e = group.place("object", place_location);
        if (!e){

            //a[k % 3] +=0.01;

            printf("An error has occured, placing again x: %f, y: %f , z: %f\n", a[0],a[1],a[2]);
        }else{
            success = true;
        }
    //}
    // END_SUB_TUTORIAL
}

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    printf("%s\n","opening the grip" );
    posture.joint_names.resize(2);
    posture.joint_names[0] = "left_finger_joint";
    posture.joint_names[1] = "right_finger_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.08;
    posture.points[0].positions[1] = 0.08;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}
void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "left_finger_joint";
    posture.joint_names[1] = "right_finger_joint";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.02;
    posture.points[0].positions[1] = 0.02;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}


void pick(moveit::planning_interface::MoveGroupInterface& move_group, double *a)
{
    //float a[] = {0.559703, -0.37644,  0.380825};
    int k = 0;
    bool success = false;

    bool changeX = true;
//    while (!success){

        // BEGIN_SUB_TUTORIAL pick1
        // Create a vector of grasps to be attempted, currently only creating single grasp.
        // This is essentially useful when using a grasp generator to generate and test multiple grasps.
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);

        /*/*/// Setting grasp pose
        // ++++++++++++++++++++++
        // This is the pose of panda_link8. |br|
        // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
        // of the cube). |br|
        // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
        // extra padding)
        grasps[0].grasp_pose.header.frame_id = "base_footprint";
        tf2::Quaternion orientation;
        orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
        //grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        grasps[0].grasp_pose.pose.position.x = a[0];
        grasps[0].grasp_pose.pose.position.y = a[1];
        grasps[0].grasp_pose.pose.position.z = a[2];

        // Setting pre-grasp approach
        // ++++++++++++++++++++++++++
        /* Defined with respect to frame_id */
        grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
        // /* Direction is set as positive x axis */
        grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.095;
        grasps[0].pre_grasp_approach.desired_distance = 0.115;

        // // Setting post-grasp retreat
        // // ++++++++++++++++++++++++++
        // /* Defined with respect to frame_id */
        grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
        // /* Direction is set as positive z axis */
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.1;
        grasps[0].post_grasp_retreat.desired_distance = 0.25;

        // Setting posture of eef before grasp
        // +++++++++++++++++++++++++++++++++++
        openGripper(grasps[0].pre_grasp_posture);
        // END_SUB_TUTORIAL

        // BEGIN_SUB_TUTORIAL pick2
        // Setting posture of eef during grasp
        // +++++++++++++++++++++++++++++++++++
        closedGripper(grasps[0].grasp_posture);
        // END_SUB_TUTORIAL

        // BEGIN_SUB_TUTORIAL pick3
        // Set support surface as table1.
        move_group.setSupportSurfaceName("table1");
        // Call pick to pick up the object using the grasps given

        moveit::planning_interface::MoveItErrorCode e = move_group.pick("object", grasps);
        if (!e){
//            k++;
//            a[k % 3] -=0.01;

            printf("An error has occured, trying again x: %f, y: %f , z: %f\n", a[0],a[1],a[2]);
        }else{
            success = true;
        }
//    }
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_footprint";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.0;
    collision_objects[0].primitives[0].dimensions[1] = 2.0;
    collision_objects[0].primitives[0].dimensions[2] = 1.0;

    /* Define the pose of the table. */

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.827834;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = -0.340000;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;



    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[1].header.frame_id = "base_footprint";
    collision_objects[1].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.05;
    collision_objects[1].primitives[0].dimensions[1] = 0.05;
    collision_objects[1].primitives[0].dimensions[2] = 0.3;

    /* Define the pose of the object. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.578782;
    collision_objects[1].primitive_poses[0].position.y = -0.371643;
    collision_objects[1].primitive_poses[0].position.z = 0.395000;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[2].ADD;


    collision_objects[2].header.frame_id = "base_footprint";
    collision_objects[2].id = "object1";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.05;
    collision_objects[2].primitives[0].dimensions[1] = 0.05;
    collision_objects[2].primitives[0].dimensions[2] = 0.3;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.578782;
    collision_objects[2].primitive_poses[0].position.y = -0.251442;
    collision_objects[2].primitive_poses[0].position.z = 0.395000;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[3].ADD;



    planning_scene_interface.applyCollisionObjects(collision_objects);
}
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


    addCollisionObjects(planning_scene_interface);
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
    std::string location, destination, slashn;
    float a[] = {0.559703, -0.37644,  0.380825};
    float b[] = {0.659703, 0.19,  0.45};
    while (userInput != 6){


        std::cout << "1. Show current pose\n";
        std::cout << "2. Move to\n";
        std::cout << "3. Pick \n";
        std::cout << "6. Exit \n";
        std::cin >> userInput;
        std::getline(std::cin, slashn);

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
                {
                std::cout << "Enter can location in x y z format" << std::endl;
                std::getline(std::cin, location);
                double *loc_array = tokenize(location);
                pick(move_group, loc_array);

                std::cout << "Enter destination in x y z format" << std::endl;
                std::getline(std::cin, destination);
                double *dest_array = tokenize(destination);
                place(move_group, dest_array);

                break;
            }
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


    // visual_tools.prompt("Press 'next' to move");
    move_group.move();

}

double * tokenize(std::string &input){
    double * cords = new double[3];
    std::string::size_type first_size;
    std::string::size_type second_size;
    cords[0] = std::stod (input, &first_size);
    cords[1] = std::stod (input.substr(first_size), &second_size);
    cords[2] = std::stod (input.substr(first_size + second_size));

    return cords;
}
