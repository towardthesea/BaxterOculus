#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <baxter_core_msgs/JointCommand.h>

#include <iostream>

// Moveit include
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#include "moveit_baxter.h"

using namespace std ;
//using namespace moveit_baxter;
//using namespace Eigen ;

//needed messages
geometry_msgs::Pose oculusPose;
geometry_msgs::Pose desiredPose;
geometry_msgs::Pose initialPose;

bool messageRecieved = false;
ros::Publisher pub_joint_cmd;

// Quaternion angles for rotations
tf::Quaternion rotQdesired,rotQinitial,rotQoculus,rotQ180,rotQ90_y;
//geometry_msgs::Quaternion rotQdesired,rotQinitial,rotQoculus;

// Callblacks

void poseCall(geometry_msgs::Pose Pose){

    oculusPose = Pose;
    messageRecieved = true;
    //ROS_INFO("Received new pose!!!");

}

int main(int argc, char** argv){

    //init
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Moveit group
    moveit::planning_interface::MoveGroup group("right_arm");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    pub_joint_cmd = nh.advertise <baxter_core_msgs::JointCommand> ("/robot/limb/right/joint_command", 1);
    //moveit_baxter baxter_controller;

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to deal directly with the world.
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("End-Effector frame: %s", group.getEndEffectorLink().c_str());

    //publisher /
    //needed objects



    //Initial calibration===================================================================
    //sleep(10.0);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    //listener.lookupTransform(group.getPlanningFrame().c_str(), group.getEndEffectorLink().c_str(),
    //        ros::Time(0), transform);
    ROS_INFO("Starting Calibration, please wait");
    // Put the robot in the initial condition //
        // use moveit here to send the robot arm in the initial condition //
        // position: x = 0.58289 ; y = -0.58289 ; z = 0.411985 orientation:
    initialPose.position.x = 0.58289;
    initialPose.position.y = -0.58289;
    initialPose.position.z = 0.411985;


    initialPose.orientation.x = 0.462111;
    initialPose.orientation.y = 0.580032;
    initialPose.orientation.z = 0.504934;
    initialPose.orientation.w = 0.44165;


    rotQinitial.setRotation(tf::Vector3(initialPose.orientation.x,
    		initialPose.orientation.y,initialPose.orientation.z),initialPose.orientation.w);

    group.setPoseTarget(initialPose);
    ROS_INFO("check");
    bool success = group.plan(my_plan);

    ROS_INFO("check 2");
	//ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	//Sleep to give Rviz time to visualize the plan.
	//sleep(5.0);
	if (success == true)
	{
		group.move();
		ROS_INFO("Move End-effector now");
	}
	else ROS_INFO("Can not move End-effector");
	//sleep(5.0);
    ROS_INFO("End of calibration phase, control loop is starting. Have Fun!!");

    //==========================================================================

    //Subscibers
    ros::Subscriber subOculus = nh.subscribe("/delta_pose",1,poseCall);

    ros::Rate rate(10);

    while(ros::ok()){

    	/*ROS_INFO("CHECK");
    					baxter_core_msgs::JointCommand joint_cmd;
    					joint_cmd.names.push_back("right_w2");
    					joint_cmd.command.push_back(M_PI/2);
    					pub_joint_cmd.publish(joint_cmd);*/

        //ros::spinOnce() ;
/*        if(messageRecieved==false) {
            ROS_INFO("Waiting for the message");
        }
        else if (messageRecieved==true){*/
        	ROS_INFO("Message received");

			double roll, pitch, yaw;

            //Get the Oculus rotation

            //rotQoculus = oculusPose.orientation;
            //rotQoculus.setRotation(tf::Vector3(oculusPose.orientation.x,
            //		oculusPose.orientation.y,oculusPose.orientation.z),oculusPose.orientation.w);
            //rotQoculus.setRPY(oculusPose.orientation.z,oculusPose.orientation.x,oculusPose.orientation.y);
			//rotQoculus.setRPY(oculusPose.orientation.y,oculusPose.orientation.z,oculusPose.orientation.x);
			//if (oculusPose.orientation.x<=0.15 && oculusPose.orientation.x >=-0.15){

									//roll								pitch							yaw
				rotQoculus.setRPY(-oculusPose.orientation.x,-oculusPose.orientation.y,oculusPose.orientation.z);

				rotQ90_y.setRPY(0.0,M_PI/2.0,0.0);

				//move it commands

				tf::Matrix3x3 m(rotQoculus);
				m.getRPY(roll,pitch,yaw);
				ROS_INFO("End-effector angle %f %f %f \n",roll*180.0/M_PI,
										pitch*180.0/M_PI, yaw*180.0/M_PI);

				//calculate desired pose
				//rotQdesired = rotQinitial *= rotQoculus;
				//rotQdesired =  rotQoculus *= rotQinitial;
				rotQdesired =  rotQoculus *= rotQ90_y;

				//desiredPose.orientation = rotQdesired;
				desiredPose.orientation.x = rotQdesired.getX();
				desiredPose.orientation.y = rotQdesired.getY();
				desiredPose.orientation.z = rotQdesired.getZ();
				desiredPose.orientation.w = rotQdesired.getW();
				desiredPose.position = initialPose.position;



				//ROS_INFO("End-effector angle %f %f %f %f\n",desiredPose.orientation.x,
				//		desiredPose.orientation.y,desiredPose.orientation.z,
				//		desiredPose.orientation.w);

				//m(rotQdesired);
				//m.getRPY(roll,pitch,yaw);
				//ROS_INFO("End-effector angle %f %f %f \n",roll*180.0/M_PI,
				//				pitch*180.0/M_PI, yaw*180.0/M_PI);

				// send everything to moveIt !!!!! //
				//moveitToPose(desiredPose, group, my_plan);

				//==========================================================================
				group.setPoseTarget(desiredPose);
				bool success = group.plan(my_plan);
				ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
				//Sleep to give Rviz time to visualize the plan.
				//sleep(5.0);

				//rotQoculus.setRPY(oculusPose.orientation.x,M_PI/2,0);

				//rotate the oculus pose by 180 degree around y
	            //rotQoculus = rotQoculus *= rotQ180;
				//rotQoculus = rotQoculus *= rotQ90_x;

	        	if (success == true)
	        	{
	        		group.move();
	        		ROS_INFO("Move End-effector now");
	        	}
	        	else ROS_INFO("Can not move End-effector");


        rate.sleep() ;

        messageRecieved = false; //reset message flag
    }


    return 0;


}



