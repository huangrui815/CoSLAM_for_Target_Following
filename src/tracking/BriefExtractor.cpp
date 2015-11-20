#include "BriefExtractor.h"

BriefExtractor::BriefExtractor(){

}

BriefExtractor::~BriefExtractor(){

}

void BriefExtractor::computeIntegralImg(const Mat& image){
	// Construct integral image for fast smoothing (box filter)
	  Mat sum;
	  if(image.type() == CV_32S)
		sum = image;
	  else
		integral(image, sum, CV_32S);
	  sum.copyTo(mSum);
}

void BriefExtractor::compute(const Mat& image, cv::KeyPoint& keyPt, cv::Mat& desc){
//	// Construct integral image for fast smoothing (box filter)
//	  Mat sum;
//	  if(image.type() == CV_32S)
//		sum = image;
//	  else
//		integral(image, sum, CV_32S);

	  desc = cv::Mat::zeros(1, 32, CV_8UC1);
	  pixelTests32_single(mSum,keyPt,desc);
}

void BriefExtractor::compute(const Mat &image,vector<KeyPoint> &kpts,Mat &descriptors){
//	  Mat sum;
//	  if(image.type() == CV_32S)
//		sum = image;
//	  else
//		integral(image, sum, CV_32S);

	  descriptors = Mat::zeros((int)kpts.size(),32,CV_8UC1);
	  pixelTests32(mSum,kpts,descriptors);
}

inline int smoothedSum(const Mat& sum, const KeyPoint& pt, int y, int x)
{
    static const int HALF_KERNEL = BriefDescriptorExtractor::KERNEL_SIZE / 2;

    int img_y = (int)(pt.pt.y + 0.5) + y;
    int img_x = (int)(pt.pt.x + 0.5) + x;
    return   sum.at<int>(img_y + HALF_KERNEL + 1, img_x + HALF_KERNEL + 1)
    - sum.at<int>(img_y + HALF_KERNEL + 1, img_x - HALF_KERNEL)
    - sum.at<int>(img_y - HALF_KERNEL, img_x + HALF_KERNEL + 1)
    + sum.at<int>(img_y - HALF_KERNEL, img_x - HALF_KERNEL);
}

void BriefExtractor::pixelTests32_single(const Mat& sum, const KeyPoint& kpt, Mat& descriptor)
{
		uchar* desc = descriptor.data;
		const KeyPoint& pt = kpt;
		#include  "generated_32.i"
}

void BriefExtractor::pixelTests32(const Mat& sum, const std::vector<KeyPoint>& keypoints, Mat& descriptors)
{
    for (int i = 0; i < (int)keypoints.size(); ++i)
    {
        uchar* desc = descriptors.ptr(i);
        const KeyPoint& pt = keypoints[i];
		#include  "generated_32.i"
    }
}
