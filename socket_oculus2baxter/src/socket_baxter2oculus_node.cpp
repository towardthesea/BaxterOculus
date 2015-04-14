#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <string>
#include <cstring>
#include <algorithm>

// Socket include
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <time.h>

//Includes for hangling images
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;

// ROS variables
ros::Publisher pub_pose_delta;
sensor_msgs::JointState joint_state;
geometry_msgs::Pose delta_pose;

// Image variables
static cv::Mat image(640,480,CV_8UC3); //Global image object

// Socket variables
char sentBuff[1024];
int sockfd = 0, n = 0;

string limb_name = "right";  //Name of the controlled limb (either "right" or "left")
double cmd;
int cmdMode=1;
string joint_name;
int incrKey, decrKey;


void joint_states_Callback(sensor_msgs::JointState join_state_cu)
{
  sensor_msgs::JointState q  = join_state_cu;
  joint_state = q;

}

//Subscriber Callback for image
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);

	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}

  ROS_INFO("Image extracted");
  //image = (cv_ptr->image).clone();
  //cv::imshow("client",image);

  cv::Mat frame = (cv_ptr->image).clone();; //colour image
  //cv::imshow("client1",frame);
  int imgSize = frame.total()*frame.elemSize();
  int bytes = send(sockfd, frame.data, imgSize, 0);//write to the socket

  //Send image to server through socket connection
  //image = (image.reshape(0,1)); // to make it continuous
  //ROS_INFO("check2");
  //int  imgSize = image.total()*image.elemSize();
  //ROS_INFO("check3");
  // Send data here
  //bytes = send(clientSock, frame.data, imgSize, 0))
  //n = write(sockfd,sentBuff,sizeof(sentBuff));
  //n = send (sockfd,"alo test",100,0);
  //n = send(sockfd,image.data,imgSize,0);
  //n = write(sockfd,image.data,imgSize);
  if (bytes < 0)
  {
       perror("ERROR writing to socket");
       ROS_INFO("Fail sending image");
       //exit(1);
  }
  else ROS_INFO("Image sent");
}

// ***MAIN***
int main(int argc, char **argv)
{

  ros::init(argc, argv, "socket");
  ROS_INFO("Node socket_oculus2baxter");

  ros::NodeHandle nh_("~");

  //Subscribing
  image_transport::ImageTransport it(nh_);
  image_transport::Subscriber cam_sub = it.subscribe("/cameras/" + limb_name + "_hand_camera/image", 1, imageCallback);
  //image_transport::Subscriber cam_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  ros::Subscriber joint_state_sub = nh_.subscribe <sensor_msgs::JointState> ("/robot/joint_states",1,joint_states_Callback);

  //Publishing
  pub_pose_delta = nh_.advertise <geometry_msgs::Pose> ("/delta_pose", 1);

  ros::Rate loop_rate(500);

  //Socket client

  char recvBuff[1024];
  struct sockaddr_in serv_addr;

  if(argc != 2)
  {
      printf("\n Usage: %s <ip of server> \n",argv[0]);
      return 1;
  }

  memset(recvBuff, '0',sizeof(recvBuff));
  if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
      printf("\n Error : Could not create socket \n");
      return 1;
  }

  memset(&serv_addr, '0', sizeof(serv_addr));

  serv_addr.sin_family = AF_INET;
  //serv_addr.sin_port = htons(1111);
  serv_addr.sin_port = htons(1111);

  if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
  {
      printf("\n inet_pton error occured\n");
      return 1;
  }

  if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
  {
     printf("\n Error : Connect Failed \n");
     ROS_INFO("\n Error : Connect Failed \n");
     return 1;
  }
  else ROS_INFO("\n Connect Succeed \n");


/*
  while ( (n = read(sockfd, recvBuff, sizeof(recvBuff)-1)) > 0)
  {
	  recvBuff[n] = 0;
	  if(fputs(recvBuff, stdout) == EOF)
	  {
		  printf("\n Error : Fputs error\n");
	  }
		float x,y,z,w;
		//sscanf(recvBuff, "%f %f %f %f", &x,&y,&z,&w);
		sscanf(recvBuff, "%f %f %f", &x,&y,&z);
		//dprintf("values: %f %f %f \n",x,y,z);
		//ROS_INFO("values: %d %d %d %d\n",x,y,z,w);
		ROS_INFO("received value %f %f %f\n",x,y,z);
		//Publish the
		delta_pose.orientation.x = x;
		delta_pose.orientation.y = y;
		delta_pose.orientation.z = z;
		//delta_pose.orientation.w = w;
		pub_pose_delta.publish(delta_pose);
  }

  if(n < 0)
  {
      printf("\n Read error \n");
  }
*/

  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
      /*n = read(sockfd, recvBuff, sizeof(recvBuff)-1);
      if (n>0)
      {
      	  recvBuff[n] = 0;
      	  if(fputs(recvBuff, stdout) == EOF)
      	  {
      		    printf("\n Error : Fputs error\n");
      	  }
				float x,y,z,w;
				//sscanf(recvBuff, "%f %f %f %f", &x,&y,&z,&w);
				sscanf(recvBuff, "%f %f %f", &x,&y,&z);
				//dprintf("values: %f %f %f \n",x,y,z);
				//ROS_INFO("values: %d %d %d %d\n",x,y,z,w);
				ROS_INFO("received value %f %f %f\n",x,y,z);
				//Publish the
				delta_pose.orientation.x = x;
				delta_pose.orientation.y = y;
				delta_pose.orientation.z = z;
				//delta_pose.orientation.w = w;
				pub_pose_delta.publish(delta_pose);
      }
      else
      {
            printf("\n Read error \n");
      }*/
  }
  return 0;
}
