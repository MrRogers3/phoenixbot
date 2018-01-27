#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "trajectory_msgs/JointTrajectory.h"

#include <sstream>

ros::Publisher simon_state_pub;
ros::Publisher joint_trajectory_pub;

//GET CURRENT POSITIONS TO DEAL WITH FLOAT A AND B

void targetPoint(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	float x = msg->point.x;
	float y = msg->point.y;

	float A = x + y;
	float B = x - y;

	ROS_INFO("A: [%f], B:[%f]", A, B);
	
	trajectory_msgs::JointTrajectory pos;
	pos.header.stamp = ros::Time::now();
	pos.joint_names = {"X_JOINT", "Y_JOINT"};
	pos.points[0].positions = {A, B};

	joint_trajectory_pub.publish(pos);
}

int calculateHeadPosition()
{
	int pos = 0;

	return pos;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "gantry_control_node");

  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  simon_state_pub = n.advertise<geometry_msgs::PointStamped>("simon/state", 1);
  joint_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("simon/state", 1);
  ros::Subscriber target_point_sub = n.subscribe("simon/target_point", 1, targetPoint);
  ros::Rate loop_rate(10);

  
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    ros::spin();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}


