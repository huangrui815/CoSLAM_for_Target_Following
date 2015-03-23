cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#OPENCV and wxWidgets support
find_package( OpenCV REQUIRED )
SET(wxWidgets_USE_LIBS base core gl net)
find_package( wxWidgets )
include( "${wxWidgets_USE_FILE}" )
include_directories(
/home/rui/rosbuild_ws/myROS/libvisualslam/src
/home/rui/rosbuild_ws/myROS/coslam_gs/src
/home/rui/rosbuild_ws/myROS/hiredis-master
)

set(COSLAM_APP
src/app/APP_OfflineMain_Reloc.cpp
src/app/APP_OfflineMain.cpp
src/app/APP_USBCamMain.cpp
src/app/APP_ROSMain.cpp
src/app/APP_ROSMain_features.cpp
src/app/SL_CoSLAM.cpp
src/app/SL_CoSLAM.h
src/app/SL_CoSLAMBA.cpp
src/app/SL_CoSLAMBA.h
src/app/SL_CoSLAMRobustBA.cpp
src/app/SL_CoSLAMRobustBA.h
src/app/SL_GlobParam.cpp
src/app/SL_GlobParam.h
src/app/SL_InitMap.cpp
src/app/SL_InitMap.h
src/app/SL_InterCamPoseEstimator.cpp
src/app/SL_InterCamPoseEstimator.h
src/app/SL_MergeCameraGroup.cpp
src/app/SL_MergeCameraGroup.h
src/app/SL_NewMapPointsInterCam.cpp
src/app/SL_NewMapPointsInterCam.h
src/app/SL_Relocalization.cpp
src/app/SL_Relocalization.h
src/app/SL_SingleSLAM.cpp
src/app/SL_SingleSLAM.h

src/redis/cbredisclient.h
src/redis/cbredisclient.cpp
src/redis/PosVelKF.h
src/redis/PosVelKF.cpp
)

set(COSLAM_GUI
src/gui/CoSLAMThread.cpp
src/gui/CoSLAMThread.h
src/gui/GLHelper.cpp
src/gui/GLHelper.h
src/gui/GLImagePane.cpp
src/gui/GLImagePane.h
src/gui/GLImageWnd.cpp
src/gui/GLImageWnd.h
src/gui/GLScenePane.cpp
src/gui/GLScenePane.h
src/gui/GLScenePaneHelper.cpp
src/gui/GLScenePaneHelper.h
src/gui/GLSceneWnd.cpp
src/gui/GLSceneWnd.h
src/gui/GLTrackballPane.cpp
src/gui/GLTrackballPane.h
src/gui/MainDialog.cpp
src/gui/MainDialog.h
src/gui/MyApp.cpp
src/gui/MyApp.h
src/gui/trackball.cpp
src/gui/trackball.h
)

set(COSLAM_SLAM
src/slam/SL_Camera.cpp
src/slam/SL_Camera.h
src/slam/SL_CameraGroup.h
src/slam/SL_CoSLAMHelper.cpp
src/slam/SL_CoSLAMHelper.h
src/slam/SL_Define.h
src/slam/SL_FeatureMatching.cpp
src/slam/SL_FeatureMatching.h
src/slam/SL_FeaturePoint.cpp
src/slam/SL_FeaturePoint.h
src/slam/SL_FeaturePoints.cpp
src/slam/SL_FeaturePoints.h
src/slam/SL_GlobalPoseEstimation.cpp
src/slam/SL_GlobalPoseEstimation.h
src/slam/SL_InitMapHelper.cpp
src/slam/SL_InitMapHelper.h
src/slam/SL_IntraCamPose.cpp
src/slam/SL_IntraCamPose.h
src/slam/SL_IntraCamPoseEstimator.cpp
src/slam/SL_IntraCamPoseEstimator.h
src/slam/SL_KeyPoseList.cpp
src/slam/SL_KeyPoseList.h
src/slam/SL_MapPoint.cpp
src/slam/SL_MapPoint.h
src/slam/SL_MapPointList.cpp
src/slam/SL_MapPointList.h
src/slam/SL_NCCBlock.cpp
src/slam/SL_NCCBlock.h
src/slam/SL_PtrVec.h
src/slam/SL_SLAMHelper.cpp
src/slam/SL_SLAMHelper.h
src/slam/SL_SolvePnP.cpp
src/slam/SL_SolvePnP.h
)

set(COSLAM_TRACKING
    src/tracking/BriefExtractor.h
    src/tracking/BriefExtractor.cpp
	src/tracking/CoSL_BaseKLTTracker.h
	src/tracking/SL_Track2D.h
	src/tracking/SL_Track2D.cpp
	src/tracking/CoSL_FeatureTracker.cpp
	src/tracking/CoSL_FeatureTracker.h
	src/tracking/CVKLTTracker.h
	src/tracking/CVKLTTracker.cpp
	src/tracking/GPUKLT.h
	src/tracking/GPUKLT.cpp
	src/tracking/CGKLT/v3d_exception.h
	src/tracking/CGKLT/v3d_gpubase.h
	src/tracking/CGKLT/v3d_gpuflow.h
	src/tracking/CGKLT/v3d_gpuklt.h
	src/tracking/CGKLT/v3d_gpupyramid.h
	src/tracking/CGKLT/v3d_gpuundistort.h
	src/tracking/CGKLT/v3d_image.h
	src/tracking/CGKLT/v3d_linear.h
	src/tracking/CGKLT/v3d_linearbase.h
	src/tracking/CGKLT/v3d_serialization.h
	src/tracking/CGKLT/v3d_timer.h
	src/tracking/CGKLT/v3d_utilities.h
	src/tracking/CGKLT/v3d_gpubase.cpp
	src/tracking/CGKLT/v3d_gpuflow.cpp
	src/tracking/CGKLT/v3d_gpuklt.cpp
	src/tracking/CGKLT/v3d_gpupyramid.cpp
	src/tracking/CGKLT/v3d_gpuundistort.cpp
	src/tracking/CGKLT/v3d_image.cpp
)

set(COSLAM_VIDEO_READER
    src/videoReader/VR_AVIReader.h
    src/videoReader/VR_AVIReader.cpp
    src/videoReader/VR_ROSReader.h
    src/videoReader/VR_ROSReader.cpp
    src/videoReader/VR_USBCamReader.h
    src/videoReader/VR_USBCamReader.cpp
)

set(COSLAM_LIB
	/home/rui/rosbuild_ws/myROS/libvisualslam/Debug/libVisualSLAM.a
	lapack
	blas
	swscale
	avformat
	avutil
	avcodec
	Cg	
	GL
	GLEW
	CgGL
	glut 
	GLU
	hiredis
)

add_definitions(-DV3DLIB_GPGPU_ENABLE_CG -DV3DLIB_ENABLE_GPGPU )

rosbuild_add_executable(coslam_gs    ${COSLAM_APP}
									  ${COSLAM_GUI}
									  ${COSLAM_SLAM}
									  ${COSLAM_TRACKING}
									  ${COSLAM_VIDEO_READER}									
)
target_link_libraries(coslam_gs ${COSLAM_LIB} ${OpenCV_LIBS} ${wxWidgets_LIBRARIES})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(feature_receiver src/feature_receiver.cpp)
target_link_libraries(feature_receiver ${OpenCV_LIBS})

rosbuild_add_executable(video_receiver src/video_receiver.cpp)
target_link_libraries(video_receiver ${OpenCV_LIBS})
