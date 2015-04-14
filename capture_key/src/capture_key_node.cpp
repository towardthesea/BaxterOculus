// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <stdlib.h> // For rand() and RAND_MAX
#include <termios.h>
#include <fcntl.h>
#include "std_msgs/Int32.h"

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	return ch;
}



int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "capture_key_dinosaur");
  ROS_INFO("Node Capture Key");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<std_msgs::Int32>("/key_typed", 1);

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);
  while(ros::ok()) {


  //for(int i = 0; i < argc; i++)
  //{
  //    msg.ascii. = argv[i];

  //}
    std_msgs::Int32 msg;

    int data = kbhit();

    if(data > 0)
    {
       msg.data=data;

    // Publish the message.
    pub.publish(msg);
    }
    ros::spinOnce();
    // Wait until it's time for another iteration.
    rate.sleep();
  }
}
