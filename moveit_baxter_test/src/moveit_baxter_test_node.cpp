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

int main(int argc, char **argv)
{

	ros::init(argc, argv, "moveit_baxter_test");
	ROS_INFO("Node moveit_baxter_test");

	ros::NodeHandle nh_("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();


	/* This sleep is ONLY to allow Rviz to come up */
	//sleep(60.0);


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
	target_pose1.orientation.x = 0.462111;
	target_pose1.orientation.y = 0.580032;	
	target_pose1.orientation.z = 0.504934;
	target_pose1.orientation.w = 0.44165;
	target_pose1.position.x = 0.58289;//0.28;
	target_pose1.position.y = -0.58289;//-0.7;
	target_pose1.position.z = 0.411985;//1.0;
	group.setPoseTarget(target_pose1);


	// Now, we call the planner to compute the plan
	// and visualize it.
	// Note that we are just planning, not asking move_group
	// to actually move the robot.
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	//sleep(10.0);
	if (success == true)
	{
		group.move();
	}
	//sleep(10.0);
	// Visualizing plans
	// ^^^^^^^^^^^^^^^^^
	// Now that we have a plan we can visualize it in Rviz.  This is not
	// necessary because the group.plan() call we made above did this
	// automatically.  But explicitly publishing plans is useful in cases that we
	// want to visualize a previously created plan.
	if (1)
	{
		ROS_INFO("Visualizing plan 1 (again)");
		display_trajectory.trajectory_start = my_plan.start_state_;
		display_trajectory.trajectory.push_back(my_plan.trajectory_);
		display_publisher.publish(display_trajectory);
		/* Sleep to give Rviz time to visualize the plan. */
		//sleep(10.0);
	}

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
	//group.move();

	//move_group_interface::MoveGroup group("right_arm");
	//group.setRandomTarget();
	//group.move();

	// Planning to a joint-space goal
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Let's set a joint space goal and move towards it.  This will replace the
	// pose target we set above.
	//
	// First get the current set of joint values for the group.
	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),
							group_variable_values);

	// Now, let's modify one of the joints, plan to the new joint
	// space goal and visualize the plan.
	group_variable_values[0] = -1.0;
	group.setJointValueTarget(group_variable_values);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	//sleep(10.0);


	// Planning with Path Constraints
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Path constraints can easily be specified for a link on the robot.
	// Let's specify a path constraint and a pose goal for our group.
	// First define the path constraint.
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "right_w0";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	group.setPathConstraints(test_constraints);

	// We will reuse the old goal that we had and plan to it.
	// Note that this will only work if the current state already
	// satisfies the path constraints. So, we need to set the start
	// state to a new pose.
	robot_state::RobotState start_state(*group.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.y = -0.05;
	start_pose2.position.z = 0.8;
	const robot_state::JointModelGroup *joint_model_group =
	start_state.getJointModelGroup(group.getName());
	start_state.setFromIK(joint_model_group, start_pose2);
	group.setStartState(start_state);

	// Now we will plan to the earlier pose target from the new
	// start state that we have just created.
	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	//sleep(10.0);

	// When done with the path constraint be sure to clear it.
	group.clearPathConstraints();

	// Cartesian Paths
	// ^^^^^^^^^^^^^^^
	// You can plan a cartesian path directly by specifying a list of waypoints
	// for the end-effector to go through. Note that we are starting
	// from the new start state above.  The initial pose (start state) does not
	// need to be added to the waypoint list.
	std::vector<geometry_msgs::Pose> waypoints;

	geometry_msgs::Pose target_pose3 = target_pose1;//start_pose2;
	target_pose3.position.x += 0.2;
	target_pose3.position.z += 0.2;
	waypoints.push_back(target_pose3);  // up and out

	target_pose3.position.y -= 0.2;
	waypoints.push_back(target_pose3);  // left

	target_pose3.position.z -= 0.2;
	target_pose3.position.y += 0.2;
	target_pose3.position.x -= 0.2;
	waypoints.push_back(target_pose3);  // down and right (back to start)

	// We want the cartesian path to be interpolated at a resolution of 1 cm
	// which is why we will specify 0.01 as the max step in cartesian
	// translation.  We will specify the jump threshold as 0.0, effectively
	// disabling it.
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group.computeCartesianPath(waypoints,
												   0.01,  // eef_step
												   0.0,   // jump_threshold
												   trajectory);

	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	/* Sleep to give Rviz time to visualize the plan. */
	//sleep(15.0);

	return 0;
}
