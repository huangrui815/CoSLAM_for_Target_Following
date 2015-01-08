#include "gui/CoSLAMThread.h"
#include "app/SL_GlobParam.h"
#include "app/SL_CoSLAM.h"
#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"

#include "gui/MyApp.h"

bool usbCamMain() {
	CoSLAM& coSLAM = MyApp::coSLAM;
	/////////////////////////1.GPU initilization/////////////////////////
	//initialization for CG;
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(" ");
	glutHideWindow();

	GLenum err = glewInit();
	if (GLEW_OK != err)
			cout << "ERROR: could not initialize GLEW!" << endl;

	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "OpenGL vendor: " << glGetString(GL_VENDOR) << endl;
	cout << "OpenGL renderer: " << glGetString(GL_RENDERER) << endl << endl;


	V3D_GPU::Cg_ProgramBase::initializeCg();

	//////////////////////////2.read video information//////////////////
	try {
		coSLAM.init(false);
		MyApp::bInitSucc = true;
		logInfo("Open cameras.. OK!\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		return 0;
	}

	//notify the GUI thread to create GUIs
	MyApp::broadcastCreateGUI();

	//wait for the accomplishment of creating GUIs
	MyApp::waitCreateGUI();

	for (int i = 0; i < coSLAM.numCams; i++){
		MyApp::videoWnd[i]->setSLAMData(i, &coSLAM);
		vector<float> reprojErrStatic, reprojErrDynamic;
		vector<int> frameNumber;
		MyApp::s_reprojErrDynamic.push_back(reprojErrStatic);
		MyApp::s_reprojErrStatic.push_back(reprojErrDynamic);
		MyApp::s_frameNumber.push_back(frameNumber);
	}


	MyApp::modelWnd1->setSLAMData(&coSLAM);
	MyApp::modelWnd2->setSLAMData(&coSLAM);

	//for measuring the timings
	Timings timingsPerStep;
	Timings timingsReadFrame;
	Timings timingsNewMapPonits;

	/* start the SLAM process*/
	try {
		coSLAM.readFrame();
		while (!MyApp::bExit){
			coSLAM.grabReadFrame();
			//copy the data to buffers for display
			updateDisplayData();
			//printf("current frame: %d\n", coSLAM.curFrame);
			if (MyApp::bStartInit){
				printf("Start initializing map...\n");
				coSLAM.curFrame = 0;
				//initialise the map points
				if (coSLAM.initMap()){
					for (int i = 0; i < coSLAM.numCams; i++)
						coSLAM.state[i] = SLAM_STATE_NORMAL;
					printf("Init map success!\n");
					break;
				}
				else
					MyApp::bStartInit = false;
			}
		}
		updateDisplayData();
		coSLAM.pause();

		int i = 0;
		// init estimation flag
		bool bEstPose[SLAM_MAX_NUM];
		for (int i = 0; i < SLAM_MAX_NUM; i++){
			bEstPose[i] = false;
		}

		while (!MyApp::bExit) {
			while (MyApp::bStop) {/*stop*/
			}
			i++;
			TimeMeasurer tmPerStep;
			tmPerStep.tic();

			coSLAM.grabReadFrame();

			//coSLAM.processOneFrame();

			coSLAM.featureTracking();

			coSLAM.poseUpdate(bEstPose);

			for (int i = 0; i < coSLAM.numCams; i++){
				if (coSLAM.state[i] == SLAM_STATE_RELOCATE)
					if (coSLAM.doRelocalization(i))
						coSLAM.endRelocalization(i);
			}

			int nGroup = coSLAM.cameraGrouping();
			printf("Number of current groups %d\n", nGroup);
			//existing 3D to 2D points robust
			int nReg = coSLAM.activeMapPointsRegister(Const::PIXEL_ERR_VAR);
			printf("Registered active map points number %d\n", nReg);

			TimeMeasurer tmNewMapPoints;
			tmNewMapPoints.tic();

			coSLAM.genNewMapPoints();
			coSLAM.m_tmNewMapPoints = tmNewMapPoints.toc();

			for (int i = 0; i < coSLAM.numCams; i++){
				if(coSLAM.state[i] == SLAM_STATE_NORMAL)
					coSLAM.m_relocalizer[i].cacheNewKeyPose(coSLAM.slam[i]);
			}

			//point registration
			coSLAM.currentMapPointsRegister(Const::PIXEL_ERR_VAR,
					i % 50 == 0 ? true : false);

			coSLAM.storeDynamicPoints();

			updateDisplayData();
			redrawAllViews();

			coSLAM.m_tmPerStep = tmPerStep.toc();
			Sleep(50);

			if (i % 500 == 0) {
				//coSLAM.releaseFeatPts(coSLAM.curFrame - 500);
				coSLAM.releaseKeyPoseImgs(coSLAM.curFrame - 500);
				coSLAM.m_lastReleaseFrm = coSLAM.curFrame;
			}

			for (int i = 0; i < coSLAM.numCams; i++)
				if ( coSLAM.state[i] == SLAM_STATE_RECOVERED)
					coSLAM.state[i] = SLAM_STATE_NORMAL;
		}
		cout << " the result is saved at " << MyApp::timeStr << endl;
		coSLAM.exportResults(MyApp::timeStr);
		logInfo("slam finished\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
	} catch (std::exception& e) {
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		logInfo("%s\n", e.what());
		logInfo("slam failed!\n");
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	}

	logInfo("\nslam exits!\n");
	return true;
}
