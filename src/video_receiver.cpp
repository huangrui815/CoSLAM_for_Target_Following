/*
 * video_receiver.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: rui
 */

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdio.h>
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

cv::VideoWriter vw[2];
bool opened = false;

vector<double> tm1;
vector<double> tm2;

void subCB_kitt_video(const sensor_msgs::ImageConstPtr &img){
	int sec = img->header.stamp.sec;
	int nsec = img->header.stamp.nsec;
	ROS_INFO("Getting image at time stamp %d sec and %d nsec\n",
			sec, nsec);

	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

//	cv::imshow("view", cv_ptr->image);
//	cv::waitKey(3);
//
	if (!opened){
		vw[0].open("/media/rui/Data/Chatterbox_Data/ros_data/calib_data/2014-12-12-11-55-41.avi", CV_FOURCC('D', 'I', 'V', 'X'), 30,
				cv::Size(cv_ptr->image.cols, cv_ptr->image.rows), 1);
		opened = true;
	}
	if (vw[0].isOpened()){
		vw[0]<<cv_ptr->image;
	}
}

void subCB_two_videos(const sensor_msgs::ImageConstPtr &img1,  const sensor_msgs::ImageConstPtr &img2){
	int sec = img1->header.stamp.sec;
	int nsec = img1->header.stamp.nsec;
//	ROS_INFO("Getting image at time stamp %d sec and %d nsec\n",
//			sec, nsec);

	tm1.push_back((double)(sec + nsec * 10e-9));

	sec = img2->header.stamp.sec;
	nsec = img2->header.stamp.nsec;
//	ROS_INFO("Getting image at time stamp %d sec and %d nsec\n",
//			sec, nsec);
	tm2.push_back((double)(sec + nsec * 10e-9));

	cv_bridge::CvImagePtr cv_ptr1, cv_ptr2;
	try{
		cv_ptr1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::BGR8);
		cv_ptr2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

//	ROS_INFO("tm1(%d x %d): %lf  tm2(%d x %d): %lf\n",
//			cv_ptr1->image.cols, cv_ptr1->image.rows,
//			cv_ptr2->image.cols, cv_ptr2->image.rows,
//			tm1.back(), tm2.back());
//
//	cv::imshow("view1", cv_ptr1->image);
//	cv::imshow("view2", cv_ptr2->image);

	if (!opened){
		vw[0].open("/media/rui/Data/Chatterbox_Data/test/video0.avi", CV_FOURCC('D', 'I', 'V', 'X'), 30,
				cv::Size(cv_ptr1->image.cols, cv_ptr1->image.rows), 1);
		vw[1].open("/media/rui/Data/Chatterbox_Data/test/video1.avi", CV_FOURCC('D', 'I', 'V', 'X'), 30,
						cv::Size(cv_ptr2->image.cols, cv_ptr2->image.rows), 1);
		opened = true;
	}
	if (vw[0].isOpened()){
		vw[0]<<cv_ptr1->image;
		vw[1]<<cv_ptr2->image;
	}

	cv::waitKey(3);
//
//	if (!opened){
//		vw.open("/media/rui/Data/Chatterbox_Data/ros_data/video_write_out.avi", CV_FOURCC('D', 'I', 'V', 'X'), 30,
//				cv::Size(cv_ptr->image.cols, cv_ptr->image.rows), 1);
//		opened = true;
//	}
//	if (vw.isOpened()){
//		vw<<cv_ptr->image;
//	}
}

int main (int argc, char **argv){

	ros::init(argc, argv, "video_receiver");
	ros::NodeHandle nh;
//	image_transport::Subscriber sub_kitt = image_transport::ImageTransport(nh).subscribe("/cbodroid_usb_cam/image_raw",
//			1000, subCB_kitt_video);
	image_transport::ImageTransport it_(nh);
	image_transport::SubscriberFilter sub_gonk(it_, "/cbodroid01/coslam_feature_tracker/image_raw", 100);
	image_transport::SubscriberFilter sub_kitt(it_, "/cbodroid02/coslam_feature_tracker/image_raw", 100);

	typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_kitt, sub_gonk);
	sync.registerCallback(boost::bind(&subCB_two_videos, _1, _2));

	while (nh.ok()){
	ros::spinOnce();
	}
	vw[0].release();
	vw[1].release();
	cout << "Video writer released\n";
	return 0;
}



