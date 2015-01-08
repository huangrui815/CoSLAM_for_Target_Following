#ifndef RELOCALIZATION_H
#define RELOCALIZATION_H
#include "slam/SL_KeyPoseList.h"
#include <deque>
using namespace std;

/* register the current image to the given key frame
 * output: 1.camera pose of the current image
 *         2.corresponding 2D feature points of the 3D map points in the key frame
 */
bool registerToKeyFrame(int camId, const KeyFrame* keyFrame, const ImgG& curImg,
		vector<MapPoint*>& matchedMapPts, Mat_d& matched2DPts);

#define RELOC_MAX_NUM_STD 30.0
#define RELOC_FRM_NUM 10
#define RELOC_MAX_CACHE_FRAME 10

class CoSLAM;
class Relocalizer {
public:
	CoSLAM* _coSLAM;
	deque<int> numTrackedFeatPts;
	int _camId;
	KeyPoseList _pastNKeyPoses; // The past N poses for camera _camId
	pthread_t _threadReloc;

public:
	Relocalizer(CoSLAM* slam_, int camId);
	Relocalizer();
	void setCoSLAM(CoSLAM* slam_);
	void setCamId(int camId);
	void reset();
	bool isNumChangeLittle(const deque<int>& nums);
	bool isCameraSteady();
	bool tryToRecover();

	void cacheNewKeyPose(SingleSLAM& slam);

	KeyPose* searchKeyPosebyThumbImage(double maxcost);
};
#endif
