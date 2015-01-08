/*
 * main.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#include "MyApp.h"
#include "tracking/CGKLT/v3d_gpuklt.h"
#include "CoSLAMThread.h"
#include "tools/SL_Timing.h"
#include "tools/SL_Tictoc.h"

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Byte.h>


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

GLImageWnd* MyApp::videoWnd[SLAM_MAX_NUM];
GLSceneWnd* MyApp::modelWnd1;
GLSceneWnd* MyApp::modelWnd2;
MainDlg* MyApp::mainDlg;

double MyApp::videoWndScale = 1;
AVIReader MyApp::aviReader[SLAM_MAX_NUM];
USBCamReader MyApp::usbReader[SLAM_MAX_NUM];
ROSReader MyApp::rosReader[SLAM_MAX_NUM];
CoSLAM MyApp::coSLAM;

pthread_mutex_t MyApp::s_mutexCreateGUI;
pthread_cond_t MyApp::s_condCreateGUI;
pthread_mutex_t MyApp::s_mutexBA;

bool MyApp::bInitSucc = false;
bool MyApp::bBusyDrawingModel = false;
bool MyApp::bBusyDrawingVideo[SLAM_MAX_NUM];
bool MyApp::bBusyBAing = false;
bool MyApp::bCancelBA = false;

MyApp* MyApp::app = 0;

int MyApp::runMode = RUN_MODE_OFFLINE;
bool MyApp::bStop = false;
bool MyApp::bExit = false;
bool MyApp::bStartInit = false;
bool MyApp::bSingleStep = false;
char MyApp::timeStr[256];

ros::Publisher MyApp::pub_mapInit;
ros::Publisher MyApp::pub_triggerClients;

cv::VideoCapture MyApp::_cap[SLAM_MAX_NUM];

vector<vector<float> > MyApp::s_reprojErrStatic;
vector<vector<float> > MyApp::s_reprojErrDynamic;
vector<vector<int> > MyApp::s_frameNumber;

list<coslam_gs::features> MyApp::featuresList[SLAM_MAX_NUM];
ros::Publisher MyApp::pub_features[SLAM_MAX_NUM];
pthread_mutex_t MyApp::_mutexImg;
pthread_cond_t MyApp::_condImg;
pthread_mutex_t MyApp::_mutexFeatures;
pthread_cond_t MyApp::_condFeatures;

CoSLAMThread* MyApp::coSlamThread;

pthread_t MyApp::_nodeThread;

DEFINE_EVENT_TYPE(CloseApp);
BEGIN_EVENT_TABLE(MyApp, wxApp)
EVT_COMMAND(wxID_ANY, EventUpdateViews, MyApp::onUpdateViews)
EVT_COMMAND(wxID_ANY, CloseApp, MyApp::onClose)
END_EVENT_TABLE()
//IMPLEMENT_APP(MyApp)
void MyApp::initSyncVars() {
	pthread_mutex_init(&s_mutexCreateGUI, 0);
	pthread_mutex_init(&s_mutexBA, 0);
	pthread_cond_init(&s_condCreateGUI, 0);
}
void MyApp::onClose(wxCommandEvent& evt) {
	cout << "close app\n";
//	activateRenderLoop(false);
//	evt.Skip(); // don't stop event, we still want window to close
//	destroySynObjs();
//
//	delete moSLAM;
//	delete _estimationNode;
	exit(0);
}

int MyApp::OnExit() {
	pthread_mutex_destroy(&s_mutexCreateGUI);
	pthread_mutex_destroy(&s_mutexBA);
	pthread_cond_destroy(&s_condCreateGUI);
	return 0;
}

void MyApp::exitProgram() {
	wxCommandEvent evt(CloseApp, wxID_ANY);
	evt.SetInt(0);
	MyApp::app->AddPendingEvent(evt);
	end();
}
void MyApp::preWaitCreateGUI() {
	pthread_mutex_lock(&s_mutexCreateGUI);
}
void MyApp::waitCreateGUI() {
	pthread_cond_wait(&s_condCreateGUI, &s_mutexCreateGUI);
#ifndef WIN32
	pthread_mutex_unlock(&s_mutexCreateGUI);
#endif
}
void MyApp::broadcastCreateGUI() {
#ifndef WIN32
	pthread_mutex_lock(&s_mutexCreateGUI);
#endif
	pthread_cond_signal(&s_condCreateGUI);
#ifndef WIN32
	pthread_mutex_unlock(&s_mutexCreateGUI);
#endif
}
///////////////////////////////////////////////////////////////////////////////////
void MyApp::redrawViews() {
	wxCommandEvent evt(EventUpdateViews, wxID_ANY);
	evt.SetInt(0);
	MyApp::app->AddPendingEvent(evt);
}
#define SAVE_VIEW_IMAGE
void MyApp::onUpdateViews(wxCommandEvent& evt) {
	for (int i = 0; i < coSLAM.numCams; i++) {
		videoWnd[i]->redraw();
	}
	modelWnd1->redraw();
	modelWnd2->redraw();
#ifdef SAVE_VIEW_IMAGE
	static bool bFirst = true;
	char dirPath[1024];
	//sprintf(dirPath, "/home/tsou/slam_results_video/%s", MyApp::timeStr);
	sprintf(dirPath,"/media/VIDEO_DATA_/slam_result_video2/%s",MyApp::timeStr);
#ifdef WIN32
	mkdir(dirPath);
#else
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

	if (coSLAM.curFrame > 0) {
		static int frame = coSLAM.curFrame;
		char filePath[256];
		for (int i = 0; i < coSLAM.numCams; i++) {
			sprintf(filePath, "%s/%dcam_%04d.ppm", dirPath, i, frame);
			videoWnd[i]->save(filePath);
		}
		sprintf(filePath, "%s/1map_%04d.ppm", dirPath, frame);
		modelWnd1->save(filePath);
		sprintf(filePath, "%s/2map_%04d.ppm", dirPath, frame);
		modelWnd2->save(filePath);
		frame++;
	}
#endif
}
void getCurTimeString(char* timeStr) {
	time_t rawtime;
	time(&rawtime);

	struct tm * timeinfo;
	timeinfo = localtime(&rawtime);
	strftime(timeStr, 256, "%y-%m-%d=%H-%M", timeinfo);
}

bool MyApp::initOffline() {
	initSyncVars();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glEnable(GL_MULTISAMPLE);
	app = this;
	MyApp::runMode = RUN_MODE_OFFLINE;

//	mainDlg = new MainDlg(0);
//	if (mainDlg->ShowModal() != wxID_OK)
//		return false;

	getCurTimeString(timeStr);

//	Param::videoFilePath.push_back("/home/rui/workspace/calibrations/logiCam01.avi");
//	Param::videoFilePath.push_back("/home/rui/workspace/calibrations/logiCam02.avi");
////	Param::videoFilePath.push_back("/home/rui/workspace/motion_capture_data/0595_out.mp4");
//
//	Param::camFilePath.push_back(
//			"/home/rui/workspace/calibrations/logiCam.txt");
//	Param::camFilePath.push_back(
//			"/home/rui/workspace/calibrations/logiCam.txt");

//	Param::videoFilePath.push_back(
////			"/home/rui/workspace/calibrations/logiCam01.avi");
//			"/home/rui/workspace/motion_capture_data/0578_out.mp4");
//	Param::videoFilePath.push_back(
////			"/home/rui/workspace/calibrations/logiCam02.avi");
//			"/home/rui/workspace/motion_capture_data/0587_out.mp4");
//	Param::videoFilePath.push_back(
////			"/home/rui/workspace/calibrations/logiCam02.avi");
//			"/home/rui/workspace/motion_capture_data/0595_out.mp4");
	//Param::videoFilePath.push_back("/media/WIND_DATA/CoSLAM_video/indoor_01/056_syn.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/SFM_data/Kaimo data/187/Camera01/960/01.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/SFM_data/Kaimo data/187/Camera02/960/02.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/SFM_data/Kaimo data/187/Camera03/960/03.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/SFM_data/Kaimo data/187/Camera04/960/04.avi");

//	Param::videoFilePath.push_back("/media/rui/Data/SFM_data/ARDrone/flight_data_pc/data79_square/video.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/SFM_data/ARDrone/flight_data_pc/data80_square/video.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/SFM_data/ARDrone/flight_data_pc/data77/video.avi");

//	Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/camfeeds/v1_selected.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/camfeeds/v2_selected.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/camfeeds/v3_selected.avi");

	Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/test3/v1_4.avi");
	Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/test3/v1_5.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/test3/v1_6.avi");
//
//	Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/ros_data/test3/2014-12-11-16-16-56/video0.avi");
//	Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/ros_data/test3/2014-12-11-16-16-56/video1.avi");


//	Param::camFilePath.push_back(
//			"/home/rui/workspace/calibrations/0578_cal_new.txt");
//	Param::camFilePath.push_back(
//			"/home/rui/workspace/calibrations/0578_cal_new.txt");
//	Param::camFilePath.push_back(
//			"/home/rui/workspace/calibrations/0578_cal_new.txt");
//	Param::camFilePath.push_back(
//			"/media/rui/Data/SFM_data/Kaimo data/187/Camera01/960/intrinsics.txt");
//	Param::camFilePath.push_back(
//			"/media/rui/Data/SFM_data/Kaimo data/187/Camera02/960/intrinsics.txt");
//	Param::camFilePath.push_back(
//				"/media/rui/Data/SFM_data/Kaimo data/187/Camera03/960/intrinsics.txt");
//	Param::camFilePath.push_back(
//				"/media/rui/Data/SFM_data/Kaimo data/187/Camera04/960/intrinsics.txt");
	
//	Param::camFilePath.push_back("/home/rui/workspace/calibrations/ardrone_cam3.txt");
//	Param::camFilePath.push_back("/home/rui/workspace/calibrations/ardrone_cam3.txt");
//	Param::camFilePath.push_back("/home/rui/workspace/calibrations/ardrone_cam3.txt");

//	Param::camFilePath.push_back("/home/rui/workspace/calibrations/chatterbox.txt");
//	Param::camFilePath.push_back("/home/rui/workspace/calibrations/chatterbox.txt");
//	Param::camFilePath.push_back("/home/rui/workspace/calibrations/chatterbox.txt");

//	Param::camFilePath.push_back("/home/rui/workspace/calibrations/nexus5.txt");
	Param::camFilePath.push_back("/home/rui/workspace/calibrations/nexus5.txt");
	Param::camFilePath.push_back("/home/rui/workspace/calibrations/nexus5.txt");

//	Param::camFilePath.push_back(
//			"/home/rui/workspace/calibrations/odroid.txt");
//	Param::camFilePath.push_back(
//			"/home/rui/workspace/calibrations/odroid.txt");

	vector<int> startFrame;
//	startFrame.push_back(150);
//	startFrame.push_back(200);
//	startFrame.push_back(200);

//	startFrame.push_back(640);
//	startFrame.push_back(640);
	startFrame.push_back(0);
	startFrame.push_back(0);


	Param::nInitFrame = 0;
	Param::SSD_Threshold = 2000; //20000;
	Param::minCornerness = 800; //2500; //800;
	
	

	for (size_t i = 0; i < Param::videoFilePath.size(); ++i) {
		coSLAM.addVideo(Param::videoFilePath[i].c_str(),
				Param::camFilePath[i].c_str(), startFrame[i]);
	}

//	coSLAM.addVideo(Param::videoFilePath[2].c_str(),
//			Param::camFilePath[0].c_str(),
//			Param::nSkipFrame);
//
//	coSLAM.addVideo(GlobParam::Ref().m_videoFilePath[3].c_str(),
//			GlobParam::Ref().m_camFilePath[0].c_str(),
//			GlobParam::Ref().m_nSkipFrame);

	coSLAM.setInitFrame(Param::nInitFrame);

	preWaitCreateGUI();

	//create main thread
	coSlamThread = new CoSLAMThread();
	coSlamThread->Create();
	coSlamThread->Run();

	waitCreateGUI();

	if (!bInitSucc) {
		wxMessageBox("Initialization failed!");
		return false;
	}
	modelWnd1 = new GLSceneWnd("model window1", 0, 0, 800, 450);
	if (modelWnd1->glPane->load("scene1_display.txt"))
		modelWnd1->glPane->m_autoScale = false;
	else
		modelWnd1->glPane->m_autoScale = true;
	modelWnd1->glPane->m_pointSize = 0.5;
//	modelWnd1->Show();

	modelWnd2 = new GLSceneWnd("model window2", 600, 0, 800, 450);
	modelWnd2->glPane->load("scene2_display.txt");
	modelWnd2->glPane->m_autoScale = false;
	modelWnd2->glPane->m_camView = true;
	modelWnd2->glPane->m_followCamId = 0;
//	modelWnd2->Show();

	char wndName[256];
	for (int i = 0; i < coSLAM.numCams; i++) {
		int W = coSLAM.slam[i].videoReader->_w;
		int H = coSLAM.slam[i].videoReader->_h;
		int Ws = W * videoWndScale;
		int Hs = H * videoWndScale;
		sprintf(wndName, "video %d", i);
		videoWnd[i] = new GLImageWnd(wndName, 300 + i * Ws, 600, Ws, Hs);
		videoWnd[i]->initVideoFrame(W, H);
//		videoWnd[i]->Show();
	}
	//finish creating GUIs, broadcast the CoSLAMThread to start SLAM
	broadcastCreateGUI();
	waitCreateGUI();
	modelWnd1->Show();
	modelWnd2->Show();
	for (int i = 0; i < coSLAM.numCams; i++){
		videoWnd[i]->Show();
	}
	modelWnd1->SetFocus();
	return true;
}

bool MyApp::initUSBCam() {
	initSyncVars();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glEnable(GL_MULTISAMPLE);
	app = this;
	MyApp::runMode = RUN_MODE_ONLINE_USB;

	getCurTimeString(timeStr);

#ifdef WIN32
	Param::camFilePath.push_back(
				"/home/rui/workspace/calibrations/0578_cal.txt");
	Param::camFilePath.push_back(
					"/home/rui/workspace/calibrations/0578_cal.txt");
#else
//	Param::camFilePath.push_back(
//				"/home/rui/workspace/calibrations/asusCam.txt");
	Param::camFilePath.push_back(
			"/home/rui/workspace/calibrations/logiCam.txt");
	Param::camFilePath.push_back(
			"/home/rui/workspace/calibrations/logiCam.txt");
#endif

	Param::SSD_Threshold = 20000; //20000;
	Param::minCornerness = 550; //800;

	for (size_t i = 0; i < Param::camFilePath.size(); ++i) {
		coSLAM.addCam(Param::camFilePath[i].c_str());
		int camId = i + 1;
		MyApp::usbReader[i]._camid = camId;
		coSLAM.slam[i].videoReader = &MyApp::usbReader[i];
		coSLAM.slam[i].videoReader->open();
	}

	preWaitCreateGUI();

	//create main thread
	coSlamThread = new CoSLAMThread();
	coSlamThread->Create();
	coSlamThread->Run();

	waitCreateGUI();

	if (!bInitSucc) {
		wxMessageBox("Initialization failed!");
		return false;
	}
	modelWnd1 = new GLSceneWnd("model window1", 0, 0, 800, 450);
	if (modelWnd1->glPane->load("scene1_display.txt"))
		modelWnd1->glPane->m_autoScale = false;
	else
		modelWnd1->glPane->m_autoScale = true;
	modelWnd1->glPane->m_pointSize = 0.5;
	modelWnd1->Show();

	modelWnd2 = new GLSceneWnd("model window2", 600, 0, 800, 450);
	modelWnd2->glPane->load("scene2_display.txt");
	modelWnd2->glPane->m_autoScale = false;
	modelWnd2->glPane->m_camView = true;
	modelWnd2->glPane->m_followCamId = 0;
	modelWnd2->Show();

	char wndName[256];
	for (int i = 0; i < coSLAM.numCams; i++) {
		int W = coSLAM.slam[i].videoReader->_w;
		int H = coSLAM.slam[i].videoReader->_h;
		std::printf("camera %d size: W (%d) x H (%d)\n", i, W, H);
		int Ws = W * videoWndScale;
		int Hs = H * videoWndScale;
		sprintf(wndName, "video %d", i);
		videoWnd[i] = new GLImageWnd(wndName, 300 + i * Ws, 600, Ws, Hs);
		videoWnd[i]->initVideoFrame(W, H);
		videoWnd[i]->Show();
	}
	//finish creating GUIs, broadcast the CoSLAMThread to start SLAM
	broadcastCreateGUI();
	modelWnd1->SetFocus();
	return true;
}

bool MyApp::initROS() {
	initSyncVars();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glEnable(GL_MULTISAMPLE);
	app = this;
	MyApp::runMode = RUN_MODE_ROS;

	getCurTimeString(timeStr);

#ifdef WIN32
	Param::camFilePath.push_back(
				"/home/rui/workspace/calibrations/0578_cal.txt");
	Param::camFilePath.push_back(
					"/home/rui/workspace/calibrations/0578_cal.txt");
#else
//	Param::camFilePath.push_back(
//				"/home/rui/workspace/calibrations/asusCam.txt");
	Param::camFilePath.push_back(
			"/home/rui/workspace/calibrations/odroid.txt");
	Param::camFilePath.push_back(
			"/home/rui/workspace/calibrations/odroid.txt");
#endif

	Param::SSD_Threshold = 20000; //20000;
	Param::minCornerness = 550; //800;

	for (size_t i = 0; i < Param::camFilePath.size(); ++i) {
		coSLAM.addCam(Param::camFilePath[i].c_str());
		int camId = i + 1;
		MyApp::rosReader[i]._camid = camId;
		coSLAM.slam[i].videoReader = &MyApp::rosReader[i];
//		coSLAM.slam[i].videoReader->open();
	}

	// Initialize the ROS node
	videoNodeInit();

	preWaitCreateGUI();

	//create main thread
	coSlamThread = new CoSLAMThread();
	coSlamThread->Create();
	coSlamThread->Run();

	waitCreateGUI();

	if (!bInitSucc) {
		wxMessageBox("Initialization failed!");
		return false;
	}
	modelWnd1 = new GLSceneWnd("model window1", 0, 0, 800, 450);
	if (modelWnd1->glPane->load("scene1_display.txt"))
		modelWnd1->glPane->m_autoScale = false;
	else
		modelWnd1->glPane->m_autoScale = true;
	modelWnd1->glPane->m_pointSize = 0.5;
//	modelWnd1->Show();

	modelWnd2 = new GLSceneWnd("model window2", 600, 0, 800, 450);
	modelWnd2->glPane->load("scene2_display.txt");
	modelWnd2->glPane->m_autoScale = false;
	modelWnd2->glPane->m_camView = true;
	modelWnd2->glPane->m_followCamId = 0;
//	modelWnd2->Show();

	char wndName[256];
	for (int i = 0; i < coSLAM.numCams; i++) {
		int W = coSLAM.slam[i].videoReader->_w;
		int H = coSLAM.slam[i].videoReader->_h;
		int Ws = W * videoWndScale;
		int Hs = H * videoWndScale;
		sprintf(wndName, "video %d", i);
		videoWnd[i] = new GLImageWnd(wndName, 300 + i * Ws, 600, Ws, Hs);
		videoWnd[i]->initVideoFrame(W, H);
//		videoWnd[i]->Show();
	}
	//finish creating GUIs, broadcast the CoSLAMThread to start SLAM
	broadcastCreateGUI();
	waitCreateGUI();
	modelWnd1->Show();
	modelWnd2->Show();
	for (int i = 0; i < coSLAM.numCams; i++){
		videoWnd[i]->Show();
	}
	modelWnd1->SetFocus();
	return true;
}

bool MyApp::initROS_features() {
	initSyncVars();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glEnable(GL_MULTISAMPLE);
	app = this;
	MyApp::runMode = RUN_MODE_ROS_FEATURES;

	getCurTimeString(timeStr);

#ifdef WIN32
	Param::camFilePath.push_back(
				"/home/rui/workspace/calibrations/0578_cal.txt");
	Param::camFilePath.push_back(
					"/home/rui/workspace/calibrations/0578_cal.txt");
#else
//	Param::camFilePath.push_back(
//				"/home/rui/workspace/calibrations/asusCam.txt");
	Param::camFilePath.push_back(
			"/home/rui/workspace/calibrations/odroid.txt");
	Param::camFilePath.push_back(
			"/home/rui/workspace/calibrations/odroid.txt");

//		Param::camFilePath.push_back("/home/rui/workspace/calibrations/nexus5.txt");
//		Param::camFilePath.push_back("/home/rui/workspace/calibrations/nexus5.txt");
#endif
//		Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/test3/v1_4.avi");
//		Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/test3/v1_5.avi");
		Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/ros_data/test3/video0.avi");
		Param::videoFilePath.push_back("/media/rui/Data/Chatterbox_Data/ros_data/test3/video1.avi");

		_camNum = Param::camFilePath.size();

	Param::SSD_Threshold = 20000; //20000;
	Param::minCornerness = 550; //800;

	for (size_t i = 0; i < Param::camFilePath.size(); ++i) {
		coSLAM.addCam(Param::camFilePath[i].c_str());
//		coSLAM.addVideo(Param::videoFilePath[i].c_str(),
//					Param::camFilePath[i].c_str());
		_cap[i].open(Param::videoFilePath[i]);
		int camId = i + 1;
		MyApp::rosReader[i]._camid = camId;
		coSLAM.slam[i].videoReader = &MyApp::rosReader[i];
//		coSLAM.slam[i].videoReader->open();
	}

	// Initialize the ROS node
	videoNodeInit();
	featureNodeInit();
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&_nodeThread, &attr, threadProc, this);

	preWaitCreateGUI();

	//create main thread
	coSlamThread = new CoSLAMThread();
	coSlamThread->Create();
	coSlamThread->Run();

	waitCreateGUI();

	if (!bInitSucc) {
		wxMessageBox("Initialization failed!");
		return false;
	}
	modelWnd1 = new GLSceneWnd("model window1", 0, 0, 800, 450);
	if (modelWnd1->glPane->load("scene1_display.txt"))
		modelWnd1->glPane->m_autoScale = false;
	else
		modelWnd1->glPane->m_autoScale = true;
	modelWnd1->glPane->m_pointSize = 0.5;
//	modelWnd1->Show();

	modelWnd2 = new GLSceneWnd("model window2", 600, 0, 800, 450);
	modelWnd2->glPane->load("scene2_display.txt");
	modelWnd2->glPane->m_autoScale = false;
	modelWnd2->glPane->m_camView = true;
	modelWnd2->glPane->m_followCamId = 0;
//	modelWnd2->Show();

	char wndName[256];
	for (int i = 0; i < coSLAM.numCams; i++) {
		int W = coSLAM.slam[i].videoReader->_w;
		int H = coSLAM.slam[i].videoReader->_h;
		int Ws = W * videoWndScale;
		int Hs = H * videoWndScale;
		sprintf(wndName, "video %d", i);
		videoWnd[i] = new GLImageWnd(wndName, 300 + i * Ws, 600, Ws, Hs);
		videoWnd[i]->initVideoFrame(W, H);
//		videoWnd[i]->Show();
	}
	//finish creating GUIs, broadcast the CoSLAMThread to start SLAM
	broadcastCreateGUI();
	waitCreateGUI();
	modelWnd1->Show();
	modelWnd2->Show();
	for (int i = 0; i < coSLAM.numCams; i++){
		videoWnd[i]->Show();
	}
	modelWnd1->SetFocus();
	return true;
}


void MyApp::onIdle(wxIdleEvent& event) {
	if (render_loop_on) {
		for (int i = 0; i < coSLAM.numCams; i++){
		if (MyApp::videoWnd[i])
			MyApp::videoWnd[i]->redraw();
		}
		if (MyApp::modelWnd1)
			MyApp::modelWnd1->redraw();
		if (MyApp::modelWnd2)
			MyApp::modelWnd2->redraw();
//		if (MyApp::mapViewWnd)
//			MyApp::mapViewWnd->redraw();

		event.RequestMore(); // render continuously, not only once on idle
	}
}

void MyApp::activateRenderLoop(bool on) {
	if (on && !render_loop_on) {
		Connect(wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(MyApp::onIdle));
		render_loop_on = true;
	} else if (!on && render_loop_on) {
		Disconnect(wxEVT_IDLE, wxIdleEventHandler(MyApp::onIdle));
		render_loop_on = false;
	}
}

void MyApp::publishMapInit(){
	pub_mapInit.publish(std_msgs::Byte());
}

void MyApp::triggerClients(){
	pub_triggerClients.publish(std_msgs::Byte());
}

void MyApp::subCB_two_videos(const sensor_msgs::ImageConstPtr &img1,  const sensor_msgs::ImageConstPtr &img2){
//	int sec = img1->header.stamp.sec;
//	int nsec = img1->header.stamp.nsec;

//	ROS_INFO("Getting image at time stamp %d sec and %d nsec\n",
//			sec, nsec);
//
//	tm1.push_back((double)(sec + nsec * 10e-9));
//
//	sec = img2->header.stamp.sec;
//	nsec = img2->header.stamp.nsec;
////	ROS_INFO("Getting image at time stamp %d sec and %d nsec\n",
////			sec, nsec);

	cv_bridge::CvImagePtr cv_ptr1, cv_ptr2;
	try{
		cv_ptr1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::MONO8);
		cv_ptr2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//pthread_mutex_lock(&_mutexImg);
	if (MyApp::rosReader[0]._imgs.size() > 10)
	{
		MyApp::rosReader[0]._imgs.pop_front();
		MyApp::rosReader[1]._imgs.pop_front();
		MyApp::rosReader[0]._ts.pop_front();
		MyApp::rosReader[1]._ts.pop_front();
		MyApp::rosReader[0]._seqs.pop_front();
		MyApp::rosReader[1]._seqs.pop_front();
	}

	MyApp::rosReader[0]._imgs.push_back(cv_ptr1->image);
	MyApp::rosReader[1]._imgs.push_back(cv_ptr2->image);
	MyApp::rosReader[0]._ts.push_back(img1->header.stamp.toSec());
	MyApp::rosReader[1]._ts.push_back(img2->header.stamp.toSec());
	MyApp::rosReader[0]._seqs.push_back(img1->header.seq);
	MyApp::rosReader[1]._seqs.push_back(img2->header.seq);
	//pthread_mutex_unlock(&_mutexImg);
	pthread_cond_signal(&MyApp::_condImg);

	ROS_INFO("img1: %f, img2: %f\n", img1->header.stamp.toSec(), img2->header.stamp.toSec());

//	ROS_INFO("tm1(%d x %d): %lf  tm2(%d x %d): %lf\n",
//			cv_ptr1->image.cols, cv_ptr1->image.rows,
//			cv_ptr2->image.cols, cv_ptr2->image.rows,
//			tm1.back(), tm2.back());

//	cv::imshow("view1", cv_ptr1->image);
//	cv::imshow("view2", cv_ptr2->image);
//	cv::waitKey(3);
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

void MyApp::subCB_two_features(const coslam_gs::featuresConstPtr &features0,  const coslam_gs::featuresConstPtr &features1){
	coslam_gs::features fts0 = *features0;
	coslam_gs::features fts1 = *features1;

	cout << "subCB_two_features\n";

//	if (MyApp::featuresList[0].size() > 10){
//		MyApp::featuresList[0].pop_front();
//		MyApp::featuresList[1].pop_front();
//	}
	pthread_mutex_lock(&MyApp::_mutexFeatures);
	MyApp::featuresList[0].push_back(fts0);
	MyApp::featuresList[1].push_back(fts1);
	pthread_cond_signal(&MyApp::_condFeatures);
	pthread_mutex_unlock(&MyApp::_mutexFeatures);

	ROS_INFO("features0: %f, features1: %f\n", features0->header.stamp.toSec(), features1->header.stamp.toSec());
}

void MyApp::videoNodeInit(){
	_it = new image_transport::ImageTransport(_nh);
//	sub_video[0].subscribe(*_it, "/gonk_usb_cam/image_raw", 100);
//	sub_video[1].subscribe(*_it, "/kitt_usb_cam/image_raw", 100);

	sub_video[0].subscribe(*_it, "/coslam_feature_tracker1/image_raw", 10);
	sub_video[1].subscribe(*_it, "/coslam_feature_tracker2/image_raw", 10);


	videoSync = new message_filters::Synchronizer< videoSyncPolicy >(videoSyncPolicy(10), sub_video[0], sub_video[1]);
	videoSync->registerCallback(boost::bind(&MyApp::subCB_two_videos, this, _1, _2));
}

void MyApp::featureNodeInit(){
	pub_mapInit = _nh.advertise<std_msgs::Byte>("/mapInit", 1);
	pub_triggerClients = _nh.advertise<std_msgs::Byte>("/trigger",1);

//	for (int i = 0; i <_camNum; i++){
		pub_features[0] = _nh.advertise<coslam_gs::features>("/coslam_feature_tracker1/init_features",10);
		pub_features[1] = _nh.advertise<coslam_gs::features>("/coslam_feature_tracker2/init_features",10);

		sub_features[0].subscribe(_nh, "/coslam_feature_tracker1/features", 10);
		sub_features[1].subscribe(_nh, "/coslam_feature_tracker2/features", 10);
//	}

	featuresSync = new message_filters::Synchronizer< featuresSyncPolicy >(featuresSyncPolicy(10), sub_features[0], sub_features[1]);
	featuresSync->registerCallback(boost::bind(&MyApp::subCB_two_features, this, _1, _2));
}

void* MyApp::threadProc(void* data) {
	MyApp* thread = (MyApp*) data;
	thread->loop();
	return 0;
}

void MyApp::loop(){
	ros::Rate loop_rate(50);
	while (_nh.ok()){
//		ROS_INFO("Node running\n");
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void MyApp::end()
{
	ROS_INFO("waiting the gs ros thread to quit...\n");
	pthread_join(_nodeThread, NULL);
	ROS_INFO("ROS thread quits!\n");
}

bool MyApp::OnInit() {
	render_loop_on = false;
	//For displaying icon
//	wxPNGHandler* png = new wxPNGHandler;
//	wxImage::AddHandler(png);

//  	if (!initOffline())
//  		return false;

//  	if (!initROS())
//  		return false;

  	if (!initROS_features())
  		return false;

//
// 	if (!initUSBCam())
// 		return false;

//	if(!initARDrone())
//	  return false;

//	if(!initRosARDrone())
//		return false;

	activateRenderLoop(true);
	return true;
}

int main(int argc, char** argv) {
	argc = 2;
	argv[1] = new char[100];
	string a = "_image_transport:=compressed";
	strcpy(argv[1], a.c_str());
	ros::init(argc, argv, "coslam_gs");

	MyApp* pApp = new MyApp();

	wxApp::SetInstance(pApp);
	wxEntry(argc, argv);
	return 0;
}
