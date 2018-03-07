#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport feed(nh);
  image_transport::Publisher pubF = feed.advertise("camera/front", 1);
  image_transport::Publisher pubR = feed.advertise("camera/rear", 1);
/*
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
*/
  sensor_msgs::ImagePtr msgF = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  sensor_msgs::ImagePtr msgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pubF.publish(msgF);
    pubR.publish(msgR);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
