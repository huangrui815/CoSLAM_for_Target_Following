/*
 * MyApp.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef MYAPP_H_
#define MYAPP_H_

//#include "redis/PosVelKF.h"

#include "app/SL_CoSLAM.h"
#include "app/SL_GlobParam.h"

#include "GLImageWnd.h"
#include "GLSceneWnd.h"
#include "MainDialog.h"
#include "CoSLAMThread.h"

#include "videoReader/VR_AVIReader.h"
#include "videoReader/VR_USBCamReader.h"
#include "videoReader/VR_ROSReader.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
//#include "std_msgs/Bool.h"

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <coslam_feature_tracker/features.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include "redis/cbredisclient.h"

#define RUN_MODE_OFFLINE 0
#define RUN_MODE_ONLINE_ARDRONE 1
#define RUN_MODE_ONLINE_USB 2
#define RUN_MODE_ONLINE_MINI_CAM 3
#define RUN_MODE_ROS 4
#define RUN_MODE_ROS_FEATURES 5

#define MAX_NUM_MARKERS 18

class MyApp: public wxApp {
protected:
	void initSyncVars();
	bool initOffline();
	bool initUSBCam();
	bool initROS();
	bool initROS_features();
	virtual int OnExit();
	virtual bool OnInit();

public:
	int _camNum;
	ros::NodeHandle _nh;
	image_transport::ImageTransport* _it;
	static pthread_t _nodeThread;

	static ros::Publisher pub_mapInit;
	static ros::Publisher pub_triggerClients;

	static std::list<coslam_feature_tracker::features> featuresList[SLAM_MAX_NUM];
	static std::list<ar_track_alvar_msgs::AlvarMarkers> markerList[SLAM_MAX_NUM];

	image_transport::SubscriberFilter sub_video[SLAM_MAX_NUM];
	message_filters::Subscriber<coslam_feature_tracker::features> sub_features[SLAM_MAX_NUM];
	message_filters::Subscriber<ar_track_alvar_msgs::AlvarMarkers>
	sub_ar_marker_pose[SLAM_MAX_NUM];
//	ros::Subscriber sub_features[SLAM_MAX_NUM];
	static ros::Publisher pub_features[SLAM_MAX_NUM];

//	typedef message_filters::sync_policies::ApproximateTime<
//		sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> videoSyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<
		sensor_msgs::Image, sensor_msgs::Image> videoSyncPolicy;
	message_filters::Synchronizer< videoSyncPolicy > *videoSync;

//	typedef message_filters::sync_policies::ApproximateTime<
//			coslam_feature_tracker::features, coslam_feature_tracker::features,coslam_feature_tracker::features> featuresSyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<
				coslam_feature_tracker::features,coslam_feature_tracker::features> featuresSyncPolicy;
	message_filters::Synchronizer< featuresSyncPolicy > *featuresSync;

	typedef message_filters::sync_policies::ApproximateTime<
				ar_track_alvar_msgs::AlvarMarkers,
				ar_track_alvar_msgs::AlvarMarkers> markerPoseSyncPolicy;
	message_filters::Synchronizer< markerPoseSyncPolicy > *markerPoseSync;

	static cv::Point3f markerPoseGlobal[MAX_NUM_MARKERS];

	static pthread_mutex_t _mutexImg;
	static pthread_cond_t _condImg;
	static pthread_mutex_t _mutexFeatures;
	static pthread_cond_t _condFeatures;

	static cv::VideoCapture _cap[SLAM_MAX_NUM];

	static void publishMapInit();
	static void triggerClients();
	void videoNodeInit();
	void featureNodeInit();
	void arMarkerNodeInit();
	static void markerPose2ImageLoc(geometry_msgs::PoseStamped& poses,
			cv::Point2f& imgLocs, double K[9]);

	static void *threadProc(void*);
	void loop();
	static void end();
	void subCB_two_videos(const sensor_msgs::ImageConstPtr &img1,  const sensor_msgs::ImageConstPtr &img2);
	void subCB_two_features(const coslam_feature_tracker::featuresConstPtr &features0,  const coslam_feature_tracker::featuresConstPtr &features1);
	void subCB_3_videos(const sensor_msgs::ImageConstPtr &img1,  const sensor_msgs::ImageConstPtr &img2,
			const sensor_msgs::ImageConstPtr &img3);
	void subCB_3_features(const coslam_feature_tracker::featuresConstPtr &features0,
			const coslam_feature_tracker::featuresConstPtr &features1,
			const coslam_feature_tracker::featuresConstPtr &features2);
	void subCB_2_marker_pose(const ar_track_alvar_msgs::AlvarMarkersConstPtr &pose0,
			const ar_track_alvar_msgs::AlvarMarkersConstPtr &pose1);


	// Create redis clients to send command
	static CBRedisClient* redis[SLAM_MAX_NUM];
	static CBRedisClient* redis_start;

//	static PosVelKF posVelKF[SLAM_MAX_NUM][3];

	static int _numMarkers;

public:
	static GLImageWnd* videoWnd[SLAM_MAX_NUM];
	static GLSceneWnd* modelWnd1;
	static GLSceneWnd* modelWnd2;
	static MainDlg* mainDlg;
	static double videoWndScale;

	static AVIReader aviReader[SLAM_MAX_NUM];
	static USBCamReader usbReader[SLAM_MAX_NUM];
	static ROSReader rosReader[SLAM_MAX_NUM];
	static CoSLAM coSLAM;

	static pthread_mutex_t s_mutexCreateGUI;
	static pthread_cond_t s_condCreateGUI;
	static pthread_mutex_t s_mutexBA;

	static bool bInitSucc;
	static bool bBusyDrawingVideo[SLAM_MAX_NUM];
	static bool bBusyDrawingModel;

	/*flag for busy running bundle adjustment*/
	static bool bBusyBAing;
	static bool bCancelBA;	//when group merge happens 
	static MyApp* app;

	static CoSLAMThread *coSlamThread;

	bool render_loop_on;

	static void preWaitCreateGUI();
	static void waitCreateGUI();
	static void broadcastCreateGUI();
	DECLARE_EVENT_TABLE();
public:
	static void redrawViews();
	void onUpdateViews(wxCommandEvent& evt);
	void onIdle(wxIdleEvent& event);
	void activateRenderLoop(bool on);
	void onClose(wxCommandEvent& evt);
	static void exitProgram();
public:
	static int runMode;
	static bool bStop;
	static bool bExit;
	static bool bStartInit;
	static bool bStartMove;
	static bool bSingleStep;
	static char timeStr[256];

	// For debug
	static vector<vector<float> > s_reprojErrStatic;
	static vector<vector<float> > s_reprojErrDynamic;
	static vector<vector<int> > s_frameNumber;
};

void getCurTimeString(char* timeStr);
#endif /* MYAPP_H_ */
