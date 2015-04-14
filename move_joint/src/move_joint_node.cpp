#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <string>
#include <cstring>
#include <algorithm>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

using namespace std;

ros::Publisher pub_joint_cmd;
sensor_msgs::JointState joint_state;

double cmd;
int cmdMode=1;
string joint_name;
int incrKey, decrKey;


void joint_states_Callback(sensor_msgs::JointState join_state_cu)
{
  sensor_msgs::JointState q  = join_state_cu;
  joint_state = q;

}


void keyhitCallback(std_msgs::Int32 key)
{
    baxter_core_msgs::JointCommand joint_cmd;

  vector<string>::iterator index,index_cmd;
  int it,it_cmd;

  int sz = joint_state.name.size();
  int sz_cmd = joint_cmd.names.size();

  for (int i=0; i<joint_state.name.size();i++){
    if (joint_name.compare(joint_state.name[i])==0){
      it=i;
    }
  }


  if (key.data==incrKey){

    cmd = joint_state.position[it]+0.3;
  }
  else if (key.data==decrKey){
    cmd = joint_state.position[it]-0.3;
  }
  joint_cmd.mode = cmdMode;
  joint_cmd.names.push_back(joint_name);
  joint_cmd.command.push_back(cmd);
  if (key.data==decrKey || key.data==incrKey)
    pub_joint_cmd.publish(joint_cmd);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "move_joint");
  ROS_INFO("Node move_joint");

  ros::NodeHandle nh_("~");
  if (!nh_.getParam("joint_name",joint_name)){
    ROS_INFO("Couldn't find parameter: joint_name\n");
    return 1;
  }else{
    ROS_INFO("joint_name: %s\n",joint_name.c_str());
  }
  nh_.param("incr_key",incrKey,43);
  nh_.param("decr_key",decrKey,45);

  //Subscribing
  //ros::Subscriber key_hit_sub = nh_.subscribe <std_msgs::Int32> ("/key_hit", 1 ,keyhitCallback);
  ros::Subscriber key_hit_sub = nh_.subscribe <std_msgs::Int32> ("/key_typed", 1 ,keyhitCallback);
  ros::Subscriber joint_state_sub = nh_.subscribe <sensor_msgs::JointState> ("/robot/joint_states",1,joint_states_Callback);

  //Publishing
  pub_joint_cmd = nh_.advertise <baxter_core_msgs::JointCommand> ("/joint_command", 1);

  ros::Rate loop_rate(100);


  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("right_arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End-Effector frame: %s", group.getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);


  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // Now that we have a plan we can visualize it in Rviz.  This is not
  // necessary because the group.plan() call we made above did this
  // automatically.  But explicitly publishing plans is useful in cases that we
  // want to visualize a previously created plan.
  //if (1)
  //{
    ROS_INFO("Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
  //}

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot*/
  group.move();

  //move_group_interface::MoveGroup group("right_arm");
  //group.setRandomTarget();
  //group.move();

  int count = 0;

  while(ros::ok())
  {
	  //group.setRandomTarget();
	  //group.move();
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
