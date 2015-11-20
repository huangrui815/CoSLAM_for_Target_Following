#ifndef _BRIEFEXTRACTOR
#define _BRIEFEXTRACTOR
#include "opencv2/core/core.hpp"
#include "opencv2/core/internal.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
using namespace cv;

class BriefExtractor{
public:
	cv::Mat mSum;

	BriefExtractor();
	~BriefExtractor();

	void computeIntegralImg(const Mat& image);

	void compute(const Mat& image, cv::KeyPoint& keyPt, cv::Mat& desc);
	void compute(const Mat &image,vector<KeyPoint> &kpts,Mat &descriptors);

	static void pixelTests32_single(const Mat& sum, const KeyPoint& kpt, Mat& descriptor);
	static void pixelTests32(const Mat& sum, const std::vector<KeyPoint>& keypoints, Mat& descriptors);
};
#endif
