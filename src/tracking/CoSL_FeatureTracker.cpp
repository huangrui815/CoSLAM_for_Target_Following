/*
 * GPUKLT.cpp
 *
 *  Created on: Mar 24, 2011
 *      Author: Danping Zou
 */

#include "CoSL_FeatureTracker.h"
#include "SL_error.h"
#include "slam/SL_Define.h"
#include "geometry/SL_Distortion.h"
#include "tools/SL_TypeConversion.h"
#include "slam/SL_MapPoint.h"
#include <fstream>
#include <sstream>

using namespace cv;
bool comparator ( const pair<float, int>& l, const pair<float, int>& r)
   { return l.first < r.first; }

void FeatureTracker::undistorPointNew(double* K, double* kc, double* x, double* x_undist){
	double invK[9];
	double xn[2];
	getInvK(K, invK);
	normPoint(invK, x[0], x[1], xn[0], xn[1]);
    /*
	k1 = k(1);
    k2 = k(2);
    k3 = k(5);
    p1 = k(3);
    p2 = k(4);

    x = xd; 				% initial guess

    for kk=1:20,

        r_2 = sum(x.^2);
        k_radial =  1 + k1 * r_2 + k2 * r_2.^2 + k3 * r_2.^3;
        delta_x = [2*p1*x(1,:).*x(2,:) + p2*(r_2 + 2*x(1,:).^2);
        p1 * (r_2 + 2*x(2,:).^2)+2*p2*x(1,:).*x(2,:)];
        x = (xd - delta_x)./(ones(2,1)*k_radial);

    end;
    */
	double k1 = kc[0];
	double k2 = kc[1];
	double k3 = kc[4];
	double p1 = kc[2];
	double p2 = kc[3];
	double x_temp[2];
	x_temp[0] = xn[0];
	x_temp[1] = xn[1];
	for (int i = 0; i < 20; i++){
		double r_2 = pow(x_temp[0], 2) + pow(x_temp[1], 2);
		double k_radial =  1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * pow(r_2, 3);
		double delta_x[2];
		delta_x[0] = 2*p1*x_temp[0]*x_temp[1] + p2*(r_2 + 2*x_temp[0] * x_temp[0]);
		delta_x[1] = p1* (r_2 + 2*x_temp[1] * x_temp[1])+2*p2*x_temp[0]*x_temp[1];
		x_temp[0] = (xn[0] - delta_x[0]) / k_radial;
		x_temp[1] = (xn[1] - delta_x[1]) / k_radial;
	}
	imagePoint(K, x_temp, x_undist);
}

bool FeatureTracker::readFMatrix(string path)
{
//  ifstream file(path.c_str());
//  if (!file.is_open())
//    return false;
//
//  int state = 0; // for states see comment above
//  string line;
//  int id0, id1;
//  string file0, file1;
//  int fileId = 0;
//  int numMathcing= 0;
//  FeaturePoint pt0, pt1;
//  int frmId = 1;
//  bool valid = true;
//  PointsInImage pts;
//  ImagePair pair;
//
//  while (file.good()) {
//    getline(file, line);
//    stringstream sline(line);
//    string filename;
//    //printf("%s\n", line.c_str());
//
//    switch (state) {
//      case 0: // header
//        if (line == "\r") {
//          state = 1;
//        }
//        break;
//      case 1: // file ID and file path
//    	  if (fileId == 0)
//		  sline >> id0 >> file0;
//		  else if (fileId == 1)
//		  {
//			sline >> id1 >> file1;
//			  if (ptsInImage.empty()){
//				  ptsInImage.push_back(PointsInImage());
//				  ptsInImage.push_back(PointsInImage());
//			  }
//			  else {
//				  ptsInImage.push_back(PointsInImage());
//			  }
//  //        	if (id0 + 1 == id1 && id0 == frmId){
//  //        		valid = true;
//  //        		frmId++;
//  //        	}
//  //        	else
//  //        		valid = false;
//			fileId = 0;
//			state = 2;
//			break;
//		  }
//		  fileId++;
//		  break;
//      case 2: // Number of matching pairs
//    	  if (valid){
//			  sline >> numMathcing;
//			  std::cout << numMathcing << endl;
//    	  }
//          state = 3;
//        break;
//      case 3: // Matchings
//
//    	  if (line != "\r" && valid){
//    		  sline >> pt0.id >> pt0.x >> pt0.y >> pt1.id >> pt1.x >>pt1.y;
//    		  pair.add(pt0.id, pt1.id);
//    		  // if frame id0 and id1 point list is created
////    		  if (ptsInImage.size() == id0 - 1)
////    			  ptsInImage.push_back(PointsInImage());
////    		  if (ptsInImage.size() == id1 - 1)
////    			  ptsInImage.push_back(PointsInImage());
//    		  // if pt0 and pt1 have not been added yet
//    		  if (ptsInImage[ptsInImage.size()-2].ptMap.count(pt0.id) == 0)
//    		  {
//    			  FeaturePoint *pPt0 = new FeaturePoint(ptsInImage.size()-2, pt0.x, pt0.y, pt0.x, pt0.y);
//    			  pPt0->id = pt0.id;
//    			  ptsInImage[ptsInImage.size()-2].ptMap[pt0.id] = pPt0;
//    			  ptsInImage[ptsInImage.size()-2].pts.push_back(pPt0);
//    			  ptsInImage[ptsInImage.size()-2].ptsId.push_back(pt0.id);
//    		  }
//    		  if (ptsInImage.back().ptMap.count(pt1.id) == 0)
//			  {
//    			  FeaturePoint *pPt1 = new FeaturePoint(ptsInImage.size()-1, pt1.x, pt1.y, pt1.x, pt1.y);
//    			  pPt1->id = pt1.id;
//				  ptsInImage.back().ptMap[pt1.id] = pPt1;
//				  ptsInImage.back().pts.push_back(pPt1);
//				  ptsInImage.back().ptsId.push_back(pt1.id);
//			  }
//    	  }
//    	  else if (line == "\r"){
//    		  state = 1;
//    		  if (valid){
//    			  imgPairs.push_back(pair);
//    			  pair.clear();
//    		  }
//    	  }
//        break;
//      default:
//    	  break;
//    }
//  }
//	printf("imgPair size: %d\n", imgPairs.size());
//	printf("Points list size: %d\n", ptsInImage.size());
//	file.close();
//	return true;
}

bool FeatureTracker::readFMatrix01(string path)
{
//	int imgNum = 133;
//	int step = 3;
//	int pairNum = 0;
//	for (int i = 1; i <= step; i++){
//		pairNum += imgNum - i;
//	}
//
//	imgPairs.resize(pairNum);
//	ptsInImage.resize(imgNum);
//
//  ifstream file(path.c_str());
//  if (!file.is_open())
//    return false;
//
//  int state = 0; // for states see comment above
//  string line;
//  int id0, id1;
//  string file0, file1;
//  int fileId = 0;
//  int numMathcing= 0;
//  FeaturePoint pt0, pt1;
//  int frmId = 1;
//  bool valid = true;
//  PointsInImage pts;
//  int pairId = 0;
//
//  while (file.good()) {
//    getline(file, line);
//    stringstream sline(line);
//    string filename;
//    //printf("%s\n", line.c_str());
//
//    switch (state) {
//      case 0: // header
//        if (line == "\r") {
//          state = 1;
//        }
//        break;
//      case 1: // file ID and file path
//        if (fileId == 0)
//    	  sline >> id0 >> file0;
//        else if (fileId == 1)
//        {
//        	sline >> id1 >> file1;
////  		  if (ptsInImage.empty()){
////  			  ptsInImage.push_back(PointsInImage());
////  			  ptsInImage.push_back(PointsInImage());
////  		  }
////  		  else {
////  			  ptsInImage.push_back(PointsInImage());
////  		  }
////        	if (id0 + 1 == id1 && id0 == frmId){
////        		valid = true;
////        		frmId++;
////        	}
////        	else
////        		valid = false;
//        	fileId = 0;
//        	state = 2;
//        	break;
//        }
//        fileId++;
//        break;
//      case 2: // Number of matching pairs
//    	  if (valid){
//			  sline >> numMathcing;
//			  std::cout << numMathcing << endl;
//    	  }
//          state = 3;
//        break;
//      case 3: // Matchings
//
//    	  if (line != "\r" && valid){
//    		  sline >> pt0.id >> pt0.x >> pt0.y >> pt1.id >> pt1.x >>pt1.y;
//    		  //pairId = (id0 + id1) - 1;
//    		  if (id1 - id0 == 1){
//    			  pairId = id0;
//    		  }
//    		  else if (id1 - id0 == 2){
//    			  pairId = id0 + imgNum - 1;
//    		  }
//    		  else if (id1 - id0 == 3){
//    			  pairId = id0 + imgNum - 1 + imgNum - 2;
//    		  }
//    		  imgPairs[pairId].add(pt0.id, pt1.id);
//    		  imgPairs[pairId].frmId0 = id0;
//    		  imgPairs[pairId].frmId1 = id1;
//
//    		  // if frame id0 and id1 point list is created
////    		  if (ptsInImage.size() == id0 - 1)
////    			  ptsInImage.push_back(PointsInImage());
////    		  if (ptsInImage.size() == id1 - 1)
////    			  ptsInImage.push_back(PointsInImage());
//
//    		  // if pt0 and pt1 have not been added yet
//    		  if (ptsInImage[id0].ptMap.count(pt0.id) == 0)
//    		  {
//    			  double in[2], out[2];
//    			  in[0] = pt0.x;
//    			  in[1] = pt0.y;
//    			  undistorPointNew(K_.data, _kc, in, out);
//    			  FeaturePoint *pPt0 = new FeaturePoint(id0, pt0.x, pt0.y, out[0], out[1]);
////    			  FeaturePoint *pPt0 = new FeaturePoint(id0, pt0.x, pt0.y, pt0.x, pt0.y);
//    			  pPt0->id = pt0.id;
//    			  ptsInImage[id0].ptMap[pt0.id] = pPt0;
//    			  ptsInImage[id0].pts.push_back(pPt0);
//    			  ptsInImage[id0].ptsId.push_back(pt0.id);
//    		  }
//    		  if (ptsInImage[id1].ptMap.count(pt1.id) == 0)
//			  {
//    			  double in[2], out[2];
//    			  in[0] = pt1.x;
//    			  in[1] = pt1.y;
//    			  undistorPointNew(K_.data, _kc, in, out);
//    			  FeaturePoint *pPt1 = new FeaturePoint(id1, pt1.x, pt1.y, out[0], out[1]);
////    			  FeaturePoint *pPt1 = new FeaturePoint(id1, pt1.x, pt1.y, pt1.x, pt1.y);
//    			  pPt1->id = pt1.id;
//				  ptsInImage[id1].ptMap[pt1.id] = pPt1;
//				  ptsInImage[id1].pts.push_back(pPt1);
//				  ptsInImage[id1].ptsId.push_back(pt1.id);
//			  }
//    	  }
//    	  else if (line == "\r"){
//    		  state = 1;
////    		  if (valid){
////    			  imgPairs.push_back(pair);
////    			  pair.clear();
////    		  }
//    	  }
//        break;
//      default:
//    	  break;
//    }
//  }
//  file.close();
//  return true;
}
FeatureTracker::FeatureTracker() :
		frame_(0), W_(0), H_(0), m_nMaxCorners(0), trackedFeatureNum_(0), detectedFeatureNum_(
				0), trackRatio(1.0) {
	firstTracking = true;
	_bRetrack = false;
	_bLost = false;
}
FeatureTracker::~FeatureTracker() {
}
//
//void FeatureTracker::updateTracks(const Mat_d& pts, const Mat_i& flag) {
////	assert(pts.m == flag.m && pts.m == maxTrackNum_);
//	int nOldFeatures = trackedFeatureNum_ + detectedFeatureNum_;
//
//	trackedFeatureNum_ = 0;
//	detectedFeatureNum_ = 0;
//
////	double in[2], out[2];
////	for (int i = 0; i < maxTrackNum_; i++) {
////		if (flag[i] >= 0) {
////			//remove the image distortion first
////			in[0] = pts[2 * i];
////			in[1] = pts[2 * i + 1];
////			undistorPoint(K_.data, kUnDist_.data, in, out);
////			if (out[0] < 0 || out[1] < 0 || out[0] >= W_ || out[1] >= H_) {
////				tracks[i].clear();
////				continue;
////			}
////
////			FeaturePoint p(frame_, in[0], in[1], out[0], out[1]);
////			if (flag[i] == KLT_TRACKED) {
////				tracks[i].add(p);
////				//tracked
////				trackedFeatureNum_++;
////			} else {
////				//newly detected;
////				tracks[i].clear();
////				tracks[i].add(p);
////				detectedFeatureNum_++;
////			}
////		} else
////			tracks[i].clear();
////	}
//
//	vector<FeaturePoint*> newDetectedPts;
//	int imgNum = ptsInImage.size();
//
//		if (frame_ == 0){
//			for (int ii = 0; ii < ptsInImage[frame_].pts.size(); ii++){
//				ptsInImage[frame_].pts[ii]->tracked = true;
//				tracks[ii].add(*(ptsInImage[frame_].pts[ii]));
//				detectedFeatureNum_++;
//			}
//		}
//		else {
//			for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++){
//				if (!tracks[i].empty()){
//					FeaturePoint& pt = tracks[i].tail->pt;
//					int id = tracks[i].tail->pt.id;
//
//					if (id != -1){
//						if (frame_ < imgNum && imgPairs[frame_ - 1].pointPair.count(id)){
//							ImagePair& pair01 = imgPairs[frame_ - 1];
//							int trackedId = pair01.pointPair[id];
//							FeaturePoint* pt = ptsInImage[frame_].ptMap[trackedId];
//							pt->tracked = true;
//							tracks[i].add(*pt);
////							trackedFeatureNum_++;
//						}
//						else if (frame_ + 1 < imgNum && imgPairs[imgNum -1 + frame_ - 1].pointPair.count(id)){
//							ImagePair& pair02 = imgPairs[imgNum -1 + frame_ - 1];
//							FeaturePoint* pt = new FeaturePoint(frame_, -1, -1, -1, -1);
//							pt->id = -1;
//							pt->tracked = true;
//							pt->virtualNum = 1;
//							tracks[i].add(*pt);
//						}
//						else if (frame_ + 2 < imgNum && imgPairs[imgNum -1 + imgNum - 2 + frame_ - 1].pointPair.count(id)){
//							ImagePair& pair03 = imgPairs[imgNum -1 + imgNum - 2 + frame_ - 1];
//							FeaturePoint* pt = new FeaturePoint(frame_, -1, -1, -1, -1);
//							pt->id = -1;
//							pt->tracked = true;
//							pt->virtualNum = 2;
//							tracks[i].add(*pt);
//						}
//						else{
//							tracks[i].clear();
//						}
//					}
//					else{
//						if (pt.virtualNum == 1){
//							ImagePair& pair02 = imgPairs[imgNum -1 + frame_ - 2];
//							int trackedId = pair02.pointPair[tracks[i].tail->pre->pt.id];
//							FeaturePoint* pt = ptsInImage[frame_].ptMap[trackedId];
//							pt->tracked = true;
//							tracks[i].add(*pt);
////							trackedFeatureNum_++;
//						}
//						else if (pt.virtualNum == 2 && tracks[i].tail->pre->pt.id != -1){
//							FeaturePoint* pt = new FeaturePoint(frame_, -1, -1, -1, -1);
//							pt->id = -1;
//							pt->virtualNum = 2;
//							pt->tracked = true;
//							tracks[i].add(*pt);
//						}
//						else if (pt.virtualNum == 2 && tracks[i].tail->pre->pt.id == -1){
//							ImagePair& pair03 = imgPairs[imgNum -1 + imgNum - 2 + frame_ - 3];
//							int trackedId = pair03.pointPair[tracks[i].tail->pre->pre->pt.id];
//							FeaturePoint* pt = ptsInImage[frame_].ptMap[trackedId];
//							pt->tracked = true;
//							tracks[i].add(*pt);
////							trackedFeatureNum_++;
//						}
//					}
//				}
//			}
//			vector<FeaturePoint*> newDetectedPts;
//			for (int i = 0; i < ptsInImage[frame_].ptMap.size(); i++){
//				if (ptsInImage[frame_].pts[i]->tracked == false){
//					newDetectedPts.push_back(ptsInImage[frame_].pts[i]);
//				}
//				else
//					trackedFeatureNum_++;
//			}
//			detectedFeatureNum_ = newDetectedPts.size();
//
//			if (detectedFeatureNum_ > 0){
//				for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++){
//					if (!newDetectedPts.empty()){
//						if (tracks[i].empty()){
//							tracks[i].add(*(newDetectedPts.back()));
//							newDetectedPts.pop_back();
//						}
//					}
//				}
//			}
//		}
//		newDetectedPts.clear();
//
//	//	if (frmId == 2)
//	//		return;
//	//trackRatio = (trackedFeatureNum_ * 1.0) / double(nOldFeatures);
//	trackRatio = (trackedFeatureNum_ * 1.0) / (double)ptsInImage[frame_].ptMap.size();
//}


void FeatureTracker::updateTracks01
(const Mat_d& pts, const Mat_i& flag, const ImgG& img, FeaturePoints& ips) {
//	assert(pts.m == flag.m && pts.m == maxTrackNum_);
//	int nOldFeatures = trackedFeatureNum_ + detectedFeatureNum_;
//
	trackedFeatureNum_ = 0;
	detectedFeatureNum_ = 0;
	int& nTracked = trackedFeatureNum_;
	int& nDetected = detectedFeatureNum_;
	int nVirtualTracked = 0;
	int nReTracked = 0;

	double in[2], out[2];
	vector<int> lostTracksID;
	vector<int> detectedId;

	if (firstTracking){
		for (int i = 0; i < m_nMaxCorners; i++) {
			if (flag[i] == KLT_NEWLY_DETECTED){
				in[0] = pts[2 * i];
				in[1] = pts[2 * i + 1];
				undistorPointNew(K_.data, _kc, in, out);
				FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
				m_tks[i].add(p);
				nDetected++;
			}
		}
		// Cache buffered images
		bufferImg.resize(W_, H_);
		memcpy(bufferImg.data, img.data, sizeof(uchar) * img.w * img.h);
	}
	else{
	for (int i = 0; i < m_nMaxCorners; i++) {
		if (flag[i] == KLT_TRACKED){
				double in[2], out[2];
				in[0] = pts[2 * i];
				in[1] = pts[2 * i + 1];
				undistorPointNew(K_.data, _kc, in, out);
				FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
				m_tks[i].add(p);
				nTracked++;
		}
		else if (flag[i] == KLT_NEWLY_DETECTED){
//				if(!tracks[i].empty()){
//					lostTracksID.push_back(i);
//				}
//
//				detectedId.push_back(i);
//				nDetected++;
			if (!_bRetrack){
				m_tks[i].clear();
				double in[2], out[2];
				in[0] = pts[2 * i];
				in[1] = pts[2 * i + 1];
				undistorPointNew(K_.data, _kc, in, out);
				FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
				m_tks[i].add(p);
				nDetected++;
			}

		}
		else if (flag[i] == KLT_INVALID){
			if (!m_tks[i].empty()){
				lostTracksID.push_back(i);
			}
			if (!_bRetrack)
				m_tks[i].clear();
		}
	}

	if(_bRetrack && frame_ % 5 != 0){
		for (int i = 0; i < lostTracksID.size(); i++){
			FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
			p->id = -1;
			m_tks[lostTracksID[i]].add(p);
			nVirtualTracked++;
		}
	}
	else if (_bRetrack && frame_ % 5 ==0 && lostTracksID.size() > 0)
	{
		printf("lost tracks size: %d\n", lostTracksID.size());

		// Find matches in previous buffered frame
		int npts = lostTracksID.size();
		Mat cvImg1(bufferImg.h, bufferImg.w, CV_8UC1, bufferImg.data);
		Mat cvImg2(img.h, img.w, CV_8UC1, img.data);

//
		cv::Mat preFeat(lostTracksID.size(), 2, CV_32F);
		cv::Mat nextFeat(lostTracksID.size(), 2, CV_32F);
		vector<uchar> cvStatus;
		vector<float> cvErr;
//
//		FILE* pFile = fopen("pts1.txt", "w");
		for (int i = 0; i < lostTracksID.size(); i++){
			int id = lostTracksID[i];
			for (Track2DNode* pt = m_tks[id].tail; pt; pt = pt->pre){
				if (pt->pt->f == frame_ - 5){
					preFeat.at<float>(i,0)= pt->pt->xo;
					preFeat.at<float>(i,1)= pt->pt->yo;
//					cv::circle(cvImg1, cv::Point2f(preFeat.at<float>(i,0), preFeat.at<float>(i,1)),3, cv::Scalar(255,0,0),2,CV_AA);
				}
			}
//			fprintf(pFile, "%f %f\n", preFeat.at<float>(i,0), preFeat.at<float>(i,1));
		}
//		fclose(pFile);
//		cout << preFeat << endl;

//
//
		int _blkWidth = 31;
		int _pyLevel = 3;
		double _stopEPS = 0.01;
		cv::calcOpticalFlowPyrLK(cvImg1, cvImg2, preFeat, nextFeat, cvStatus,
				cvErr, cv::Size(_blkWidth, _blkWidth), _pyLevel,
				cv::TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30,
						_stopEPS));

		vector<pair<int, int> > pairs;
		for (int i = 0; i < npts; i++){
			pair<float, int> pair;
			pair.first = cvErr[i];
			pair.second = i;
			pairs.push_back(pair);
		}

		std::sort (pairs.begin(), pairs.end(), comparator);

//		pFile = fopen("pts2.txt", "w");

		for (int i = 0; i < npts; i++){
			int ii = pairs[i].second;
			int trackId = lostTracksID[ii];
			if (i >= 50){
				m_tks[trackId].clear();
				continue;
			}
			if (cvStatus[ii] == 1){
				double in[2], out[2];
				in[0] = nextFeat.at<float>(ii,0);
				in[1] = nextFeat.at<float>(ii,1);
				undistorPointNew(K_.data, _kc, in, out);
				FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
				m_tks[trackId].add(p);
//				cv::circle(cvImg2, cv::Point2f(in[0], in[1]),3, cv::Scalar(255,0,0),2,CV_AA);
				nReTracked++;
			}
			else{
				m_tks[trackId].clear();
			}

//			fprintf(pFile, "%f %u %f %f\n", cvErr[i], cvStatus[i], nextFeat.at<float>(i,0), nextFeat.at<float>(i,1));
		}
//		fclose(pFile);
//		cv::imshow("Buffered video", cvImg1);
//		cv::imshow("current image", cvImg2);
//		cv::waitKey(10);

		/*
		FILE* pFile = fopen("pts1.txt", "w");
		Mat_f fpts;
		fpts.resize((int) lostTracksID.size(), 2);
		for (int i = 0; i < lostTracksID.size(); i++){
			int id = lostTracksID[i];
			for (TrackedFeaturePoint* pt = tracks[id].tail; pt; pt = pt->pre){
				if (pt->pt.f == _frame - 5){
					fpts[2 * i] = (float)pt->pt.xo;
					fpts[2*i+1] = (float)pt->pt.yo;
					printf("%f %f\n", pt->pt.xo, pt->pt.yo);
					//cv::circle(cvImg1, cv::Point2f(pt->pt.xo, pt->pt.yo),3, cv::Scalar(255,0,0),2,CV_AA);
				}
			}
			fprintf(pFile, "%f %f\n", fpts[2*i], fpts[2*i + 1]);
		}
		fclose(pFile);

		klt_.reset(bufferImg, fpts);
		klt_.track(img);
		Mat_d pts;
		Mat_i flag;
		klt_.readFeatPoints(pts, flag);

		printf("\n");
		pFile = fopen("pts2.txt", "w");
		for (int i = 0; i < npts; i++){
			int id = lostTracksID[i];
			if (flag[i] == 0){
				double in[2], out[2];
				in[0] = pts[2 * i];
				in[1] = pts[2*i + 1];
				undistorPointNew(_K, _kc, in, out);
				FeaturePoint p(_frame, in[0], in[1], out[0], out[1]);
				tracks[id].add(p);
				printf("%f %f\n", in[0], in[1]);
				//cv::circle(cvImg2, cv::Point2f(in[0], in[1]),3, cv::Scalar(255,0,0),2,CV_AA);
				nReTracked++;
			}
			else{
				tracks[id].clear();
			}
			fprintf(pFile, "%f %f\n", pts[2*i], pts[2*i + 1]);
		}
		fclose(pFile);
		*/

//		cv::imshow("Buffered video", cvImg1);
//		cv::imshow("current image", cvImg2);
//		cv::waitKey(-1);

		// Cache buffered images
		memcpy(bufferImg.data, img.data, sizeof(uchar) * img.w * img.h);
	}

	// Add new detected features to empty tracks
	if(_bRetrack){
		int k = 0;
		for (int i = 0; i < m_nMaxCorners && k < detectedId.size(); i++) {
			if (m_tks[i].empty())
			{
				double in[2], out[2];
				in[0] = pts[2 * detectedId[k]];
				in[1] = pts[2 * detectedId[k] + 1];
				undistorPointNew(K_.data, _kc, in, out);
				FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
				m_tks[i].add(p);
				k++;
			}
		}
	}
	}
	printf("frame: %d, Detected: %d, Tracked: %d, ReTracked: %d, VirtualTracked: %d\n", frame_, nDetected, nTracked, nReTracked, nVirtualTracked);
//	vector<FeaturePoint*> newDetectedPts;
//	int imgNum = ptsInImage.size();
//
//		if (frame_ == 0){
//			for (int ii = 0; ii < ptsInImage[frame_].pts.size(); ii++){
//				ptsInImage[frame_].pts[ii]->tracked = true;
//				tracks[ii].add(*(ptsInImage[frame_].pts[ii]));
//				detectedFeatureNum_++;
//			}
//		}
//		else {
//			for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++){
//				if (!tracks[i].empty()){
//					FeaturePoint& pt = tracks[i].tail->pt;
//					int id = tracks[i].tail->pt.id;
//
//					if (id != -1){
//						if (frame_ < imgNum && imgPairs[frame_ - 1].pointPair.count(id)){
//							ImagePair& pair01 = imgPairs[frame_ - 1];
//							int trackedId = pair01.pointPair[id];
//							FeaturePoint* pt = ptsInImage[frame_].ptMap[trackedId];
//							pt->tracked = true;
//							tracks[i].add(*pt);
////							trackedFeatureNum_++;
//						}
//						else if (frame_ + 1 < imgNum && imgPairs[imgNum -1 + frame_ - 1].pointPair.count(id)){
//							ImagePair& pair02 = imgPairs[imgNum -1 + frame_ - 1];
//							FeaturePoint* pt = new FeaturePoint(frame_, -1, -1, -1, -1);
//							pt->id = -1;
//							pt->tracked = true;
//							pt->virtualNum = 1;
//							tracks[i].add(*pt);
//						}
//						else if (frame_ + 2 < imgNum && imgPairs[imgNum -1 + imgNum - 2 + frame_ - 1].pointPair.count(id)){
//							ImagePair& pair03 = imgPairs[imgNum -1 + imgNum - 2 + frame_ - 1];
//							FeaturePoint* pt = new FeaturePoint(frame_, -1, -1, -1, -1);
//							pt->id = -1;
//							pt->tracked = true;
//							pt->virtualNum = 2;
//							tracks[i].add(*pt);
//						}
//						else{
//							tracks[i].clear();
//						}
//					}
//					else{
//						if (pt.virtualNum == 1){
//							ImagePair& pair02 = imgPairs[imgNum -1 + frame_ - 2];
//							int trackedId = pair02.pointPair[tracks[i].tail->pre->pt.id];
//							FeaturePoint* pt = ptsInImage[frame_].ptMap[trackedId];
//							pt->tracked = true;
//							tracks[i].add(*pt);
////							trackedFeatureNum_++;
//						}
//						else if (pt.virtualNum == 2 && tracks[i].tail->pre->pt.id != -1){
//							FeaturePoint* pt = new FeaturePoint(frame_, -1, -1, -1, -1);
//							pt->id = -1;
//							pt->virtualNum = 2;
//							pt->tracked = true;
//							tracks[i].add(*pt);
//						}
//						else if (pt.virtualNum == 2 && tracks[i].tail->pre->pt.id == -1){
//							ImagePair& pair03 = imgPairs[imgNum -1 + imgNum - 2 + frame_ - 3];
//							int trackedId = pair03.pointPair[tracks[i].tail->pre->pre->pt.id];
//							FeaturePoint* pt = ptsInImage[frame_].ptMap[trackedId];
//							pt->tracked = true;
//							tracks[i].add(*pt);
////							trackedFeatureNum_++;
//						}
//					}
//				}
//			}
//			vector<FeaturePoint*> newDetectedPts;
//			for (int i = 0; i < ptsInImage[frame_].ptMap.size(); i++){
//				if (ptsInImage[frame_].pts[i]->tracked == false){
//					newDetectedPts.push_back(ptsInImage[frame_].pts[i]);
//				}
//				else
//					trackedFeatureNum_++;
//			}
//			detectedFeatureNum_ = newDetectedPts.size();
//
//			if (detectedFeatureNum_ > 0){
//				for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++){
//					if (!newDetectedPts.empty()){
//						if (tracks[i].empty()){
//							tracks[i].add(*(newDetectedPts.back()));
//							newDetectedPts.pop_back();
//						}
//					}
//				}
//			}
//		}
//		newDetectedPts.clear();

	//	if (frmId == 2)
	//		return;
	//trackRatio = (trackedFeatureNum_ * 1.0) / double(nOldFeatures);
//	trackRatio = (trackedFeatureNum_ * 1.0) / (double)ptsInImage[frame_].ptMap.size();
}

int FeatureTracker::currentFrame(){
	return frame_;
}

void FeatureTracker::openGPU(int W, int H,
		V3D_GPU::KLT_SequenceTrackerConfig* pCfg) {
	W_ = W;
	H_ = H;
//	klt_.open(W, H, pCfg);
//	maxTrackNum_ = klt_._nMaxTrack;
	klt_.open(W,H);
	m_nMaxCorners = KLT_MAX_FEATURE_NUM;

	m_tks = new Track2D[m_nMaxCorners];
	_corners = new float[3 * m_nMaxCorners];
//	readFMatrix01("/home/rui/SFM_data/ARDrone/data01/imgs01_FMatrix03.txt");
}

void FeatureTracker::init(int camid, int W, int H,
		V3D_GPU::KLT_SequenceTrackerConfig* pCfg) {
	W_ = W;
	H_ = H;
	camid_ = camid;
//	klt_.open(W, H, pCfg);
//	maxTrackNum_ = klt_._nMaxTrack;
	klt_.open(W,H);
	m_nMaxCorners = KLT_MAX_FEATURE_NUM;

	m_tks = new Track2D[m_nMaxCorners];
	_corners = new float[3 * m_nMaxCorners];
//	readFMatrix01("/home/rui/SFM_data/ARDrone/data01/imgs01_FMatrix03.txt");
}

void FeatureTracker::getCurrFeatures(){
	for (int i = 0; i < m_nMaxCorners; i++) {
		if (!m_tks[i].empty()) {
			_corners[3 * i] = m_tks[i].tail->pt->xo;
			_corners[3 * i + 1] = m_tks[i].tail->pt->yo;
			_corners[3 * i + 2] = 1.0f;
		}
		else{
			_corners[3*i] = -1.0f;
			_corners[3*i+1] = -1.0f;
			_corners[3*i+2] = 1.0f;
		}
	}
}

void FeatureTracker::getMappedFlags(){
	klt_._numMappedTracks = 0;
	for (size_t i = 0; i < KLT_MAX_FEATURE_NUM; i++){
		if (!m_tks[i].empty() && m_tks[i].tail->pt->mpt && m_tks[i].tail->pt->id != -1){
			klt_._flagMapped[i] = 1;
			klt_._numMappedTracks++;
		}
		else
			klt_._flagMapped[i] = 0;
	}

	printf("Mapped tracks number: %d\n", klt_._numMappedTracks);
}
int FeatureTracker::first(int frame, const unsigned char* imgData, FeaturePoints& ips) {
	ImgG img;
	img.resize(W_, H_);
	memcpy(img.data, imgData, sizeof(uchar)*W_*H_);
	frame_ = frame;
	klt_.detect(img);
//
	Mat_d featPts;
	Mat_i flag;
	klt_.readFeatPoints(featPts, flag);
	updateTracks01(featPts, flag, img, ips);
	oldImg.resize(img.w, img.h);
	memcpy(oldImg.data, img.data, sizeof(uchar) * img.w * img.h);
	return detectedFeatureNum_;
}

//maybe problem
int FeatureTracker::first(int f, const unsigned char* imgData,
				std::vector<FeaturePoint*>& pExistFeat, FeaturePoints& ips){
	ImgG img;
	img.resize(W_, H_);
	memcpy(img.data, imgData, sizeof(uchar)*W_*H_);
	frame_ = f;
	reset(img, pExistFeat, ips);
	firstTracking = false;
//
//	Mat_d featPts;
//	Mat_i flag;
//	klt_.readFeatPoints(featPts, flag);
//	updateTracks01(featPts, flag, img, ips);
//	oldImg.resize(img.w, img.h);
//	memcpy(oldImg.data, img.data, sizeof(uchar) * img.w * img.h);
	return detectedFeatureNum_;
}

int FeatureTracker::feedExternFeatPoints(std::vector<FeaturePoint*>& externFeatPts){

}

int FeatureTracker::getNumDynamicFeats(){
	int num = 0;
	for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++){
		if (m_tks[i].empty())
			continue;

		FeaturePoint* fp = m_tks[i].tail->pt;
		if (fp && fp->mpt && fp->mpt->isCertainDynamic()){
			num++;
		}
	}
	printf("getNumDynamicFeats %d\n", num);
	return num;
}

int FeatureTracker::next(const unsigned char* imgData, FeaturePoints& ips) {
	ImgG img;
	img.resize(W_, H_);
	memcpy(img.data, imgData, sizeof(uchar)*W_*H_);

	if(_bRetrack){
	getCurrFeatures();
	klt_.provideCurrFeatures(oldImg, _corners);
	}

	klt_._numDynamicFeatPts = getNumDynamicFeats();

	frame_++;
	Mat_d pts;
	Mat_i flag;
	const int nTrackedFrames = 10;

	int trackRes = 0;
	if (frame_ % nTrackedFrames == 0)
		//re-detect feature points
		trackRes = klt_.trackRedetect(img);
	else
		trackRes = klt_.track(img);
	if (trackRes < 0)
		return trackRes;

	klt_.readFeatPoints(pts, flag);
	updateTracks01(pts, flag, img, ips);
//	memcpy(oldImg.data, img.data, sizeof(uchar) * img.w * img.h);
	return detectedFeatureNum_ + trackedFeatureNum_;
}

int FeatureTracker::next(FeaturePoints& ips,
		vector<unsigned int>& featureId,
		vector<float>& features){
	ImgG img;
	img.resize(W_, H_);

	frame_++;

	updateTracks02(ips, featureId, features);
//	memcpy(oldImg.data, img.data, sizeof(uchar) * img.w * img.h);
	return detectedFeatureNum_ + trackedFeatureNum_;
}

void FeatureTracker::updateTracks02(FeaturePoints& ips,
		vector<unsigned int>& featureId,
		vector<float>& pts){
	trackedFeatureNum_ = 0;
	detectedFeatureNum_ = 0;
	int& nTracked = trackedFeatureNum_;
	int& nDetected = detectedFeatureNum_;
	int nInvalid = 0;
	int nVirtualTracked = 0;
	int nReTracked = 0;

	double in[2], out[2];
	vector<int> lostTracksID;
	vector<int> detectedId;

	vector<int> flag_(m_nMaxCorners, -1);

	std::map<unsigned int, int> _featureId2TrackId_copy;
	_featureId2TrackId_copy.insert(_featureId2TrackId.begin(), _featureId2TrackId.end());
	_featureId2TrackId.clear();
	vector<int> newFeatureId;

//	cout << "featureId size: " << featureId.size() << endl;

	for (int i = 0; i < featureId.size(); i++){
		unsigned int id = featureId[i];
		if (_featureId2TrackId_copy.count(id)){
			// the feature appeared before
			double in[2], out[2];
			in[0] = pts[2 * i];
			in[1] = pts[2 * i + 1];
			undistorPointNew(K_.data, _kc, in, out);
			FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
			m_tks[_featureId2TrackId_copy[id]].add(p);
			flag_[_featureId2TrackId_copy[id]] = KLT_TRACKED;
			_featureId2TrackId[id] = _featureId2TrackId_copy[id];
			nTracked++;
		}
		else{
			newFeatureId.push_back(i);
		}
	}

//	cout << "new Feature Id size: " << newFeatureId.size() << endl;

	int k = 0;
	for (int i = 0; i < m_nMaxCorners; i++){
		//Find available tracks
		if (flag_[i] == -1 && k < newFeatureId.size()){
			// if track i is available
			int ii = newFeatureId[k];
			// Put new feature point ii in
			double in[2], out[2];
			in[0] = pts[2 * ii];
			in[1] = pts[2 * ii + 1];
			undistorPointNew(K_.data, _kc, in, out);
			FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
			m_tks[i].clear();
			m_tks[i].add(p);
			flag_[i] = KLT_NEWLY_DETECTED;
			_featureId2TrackId[featureId[ii]] = i;
			nDetected++;
			k++;
		}
	}

	for (int i = 0; i < m_nMaxCorners; i++){
		if (flag_[i] == -1){
			m_tks[i].clear();
			nInvalid++;
		}
	}

//		if (flag[i] == KLT_TRACKED){
//				double in[2], out[2];
//				in[0] = pts[2 * i];
//				in[1] = pts[2 * i + 1];
//				undistorPointNew(K_.data, _kc, in, out);
//				FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
//				m_tks[i].add(p);
//				nTracked++;
//		}
//		else if (flag[i] == KLT_NEWLY_DETECTED){
//				m_tks[i].clear();
//				double in[2], out[2];
//				in[0] = pts[2 * i];
//				in[1] = pts[2 * i + 1];
//				undistorPointNew(K_.data, _kc, in, out);
//				FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
//				m_tks[i].add(p);
//				nDetected++;
//		}
//		else if (flag[i] == KLT_INVALID){
//			if (!m_tks[i].empty()){
//				lostTracksID.push_back(i);
//			}
//			if (!_bRetrack)
//				m_tks[i].clear();
//		}

	printf("frame: %d, Detected: %d, Tracked: %d, Invalid: %d\n",
			frame_, nDetected, nTracked, nInvalid);

}

void FeatureTracker::rollBack(){
	klt_.rollBack();
}

void FeatureTracker::getFeatPoints(vector<Track2DNode*>& featPts) {
	featPts.clear();
	for (int i = 0; i < m_nMaxCorners; i++) {
		if (!m_tks[i].empty()) {
			//assert(tracks[i].tail->pt.f == frame_);
			featPts.push_back(m_tks[i].tail);
		}
	}
}

void FeatureTracker::reset(const ImgG& img,
		const vector<FeaturePoint*>& vecPts, FeaturePoints& ips) {
	Mat_f fpts;
	fpts.resize((int) vecPts.size(), 2);

	for (int i = 0; i < fpts.m; i++) {
		fpts[2 * i] = vecPts[i]->xo;
		fpts[2 * i + 1] = vecPts[i]->yo;
	}

	klt_.reset(img, fpts);

	for (int i = 0; i < m_nMaxCorners; i++)
		m_tks[i].clear();

	Mat_d pts;
	Mat_i flag;
	klt_.readFeatPoints(pts, flag);

	unsigned int featId = 0;
	double in[2], out[2];
	for (int i = 0; i < m_nMaxCorners; i++) {
		if (flag[i] == KLT_NEWLY_DETECTED){
			if (i < vecPts.size()){
				m_tks[i].add(vecPts[i]);
				_featureId2TrackId[featId] = i;
				featId++;
			}
			else{
				in[0] = pts[2 * i];
				in[1] = pts[2 * i + 1];
				undistorPointNew(K_.data, _kc, in, out);
				FeaturePoint *p = ips.add(frame_, camid_, in[0], in[1], out[0], out[1]);
				m_tks[i].add(p);
				_featureId2TrackId[featId] = i;
				featId++;
			}
		}
	}
}
