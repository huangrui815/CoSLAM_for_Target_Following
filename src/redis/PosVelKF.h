#ifndef _POS_VEL_FILTER_H_
#define _POS_VEL_FILTER_H_
#include "opencv2/opencv.hpp"
#include <vector>
using namespace cv;

class PosVelKF{
public:
	cv::Mat _F; // process function
	cv::Mat _state; // state vector including position velocity and scale
	cv::Mat _statePred;
	cv::Mat _P; // The covariance matrix
	cv::Mat _P_pred;
	cv::Mat _Q; // process noise
	cv::Mat _R; // Noise Covariance for measurements
	cv::Mat _H;
	double ts_last_update;

	std::deque<double> _state_ts_vec;
	std::deque<cv::Mat> _state_vec;
	std::deque<cv::Mat> _P_vec;

	std::deque<double> _navMeas_vec;

	cv::Mat _state_last;
	cv::Mat _P_last;

	PosVelKF();
	~PosVelKF();

	bool predict(double curr_ts);
	void update(cv::Mat& meas, double curr_ts);
	void setStartTs(double ts);

	double getPos();
	double getVel();
};


#endif /* DRONE_SCALEFILTER_H_ */
