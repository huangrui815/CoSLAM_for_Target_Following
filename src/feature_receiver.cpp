#include "ros/ros.h"
#include "coslam_gs/features.h"
#include "opencv2/opencv.hpp"
#include <vector>
using namespace cv;

std::vector<cv::Point2f> points;
VideoCapture cap;


void subCB(const coslam_gs::featuresConstPtr& msg)
{
//for (int i=0; i<msg->images.size; ++i)
//{
//  const my_pkg::imgData &data = msg->images[i];
//  ROS_INFO_STREAM("UL: " << data.upperleft << "UR: " << data.upperright <<
//				  "color: " << data.color << "ID: " << data.cameraID);
//}
	points.clear();
	for (int i = 0; i < msg->points.size(); i = i + 2){
		Point2f p(msg->points[i], msg->points[i+1]);
		points.push_back(p);
	}

	cv::Mat image;
	int frameId = msg->frameId;
//	cap.set(CV_CAP_PROP_POS_FRAMES, frameId);
//cap >> image;
//for (int i = 0; i < points.size(); i++){
//	circle( image, points[i], 3, Scalar(0,255,0), -1, 8);
//}
ROS_INFO("Receive %d feature points of frame %d\n", msg->points.size(), frameId);
//imshow("LK Demo", image);
//waitKey(10);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "feature_receiver");
cap.open("/home/autolab/rosbuild_ws/videos/imgs02.avi");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("features", 1000, subCB);
ros::spin();
}
