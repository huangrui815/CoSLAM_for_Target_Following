/*
 * MyApp.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef MYAPP_H_
#define MYAPP_H_

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
#include <coslam_gs/features.h>

#define RUN_MODE_OFFLINE 0
#define RUN_MODE_ONLINE_ARDRONE 1
#define RUN_MODE_ONLINE_USB 2
#define RUN_MODE_ONLINE_MINI_CAM 3
#define RUN_MODE_ROS 4
#define RUN_MODE_ROS_FEATURES 5

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

	static std::list<coslam_gs::features> featuresList[SLAM_MAX_NUM];

	image_transport::SubscriberFilter sub_video[SLAM_MAX_NUM];
	message_filters::Subscriber<coslam_gs::features> sub_features[SLAM_MAX_NUM];
//	ros::Subscriber sub_features[SLAM_MAX_NUM];
	static ros::Publisher pub_features[SLAM_MAX_NUM];

	typedef message_filters::sync_policies::ApproximateTime<
		sensor_msgs::Image, sensor_msgs::Image> videoSyncPolicy;
	message_filters::Synchronizer< videoSyncPolicy > *videoSync;

	typedef message_filters::sync_policies::ApproximateTime<
		coslam_gs::features, coslam_gs::features> featuresSyncPolicy;
	message_filters::Synchronizer< featuresSyncPolicy > *featuresSync;

	static pthread_mutex_t _mutexImg;
	static pthread_cond_t _condImg;
	static pthread_mutex_t _mutexFeatures;
	static pthread_cond_t _condFeatures;

	static cv::VideoCapture _cap[SLAM_MAX_NUM];

	static void publishMapInit();
	static void triggerClients();
	void videoNodeInit();
	void featureNodeInit();

	static void *threadProc(void*);
	void loop();
	static void end();
	void subCB_two_videos(const sensor_msgs::ImageConstPtr &img1,  const sensor_msgs::ImageConstPtr &img2);
	void subCB_two_features(const coslam_gs::featuresConstPtr &features0,  const coslam_gs::featuresConstPtr &features1);

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
	static bool bSingleStep;
	static char timeStr[256];

	// For debug
	static vector<vector<float> > s_reprojErrStatic;
	static vector<vector<float> > s_reprojErrDynamic;
	static vector<vector<int> > s_frameNumber;
};

void getCurTimeString(char* timeStr);
#endif /* MYAPP_H_ */
