#include "PosVelKF.h"
#include <stdio.h>

PosVelKF::PosVelKF(){
	_state = Mat::zeros(2,1,CV_32F);
	_statePred = Mat::zeros(2,1,CV_32F);
	_P = Mat::zeros(2,2,CV_32F);
	_F = Mat::eye(2,2,CV_32F);
	_Q = Mat::zeros(2,2,CV_32F);
	_H = Mat::eye(2,2, CV_32F);

	_P.at<float>(0,0) = 10;
	_P.at<float>(1,1) = 10;

	_R.at<float>(0,0) = 0.01;
	_R.at<float>(1,1) = 10e-6;

	ts_last_update = 0;
}

PosVelKF::~PosVelKF(){

}

void PosVelKF::setStartTs(double ts){
	ts_last_update = ts;
}

bool PosVelKF::predict(double curr_ts){
	if (ts_last_update == 0){
		ts_last_update = curr_ts;
		return false;
	}
	double delta_t = curr_ts - ts_last_update;
	_F.at<float>(0,1) = delta_t;
	_Q.at<double>(0,0) = pow(delta_t,4) / 4;
	_Q.at<double>(0,1) = pow(delta_t,3) / 2;
	_Q.at<double>(1,0) = pow(delta_t,3) / 2;
	_Q.at<double>(1,1) = pow(delta_t,2);
	_statePred = _F * _state;
	_P_pred = _F * _P * _F.t() + _Q;
	ts_last_update = curr_ts;
	return true;
}

void PosVelKF::update(cv::Mat& meas, double curr_ts){
	if(!predict(curr_ts))
		return;

	cv::Mat y = meas - _statePred;
	cv::Mat S = _H * _P_pred * _H.t() + _R;
	cv::Mat K = _P_pred * _H.t() * S.inv(DECOMP_LU);
	_state = _statePred + K * y;
	_P = (Mat::eye(K.rows, K.rows, CV_32F) - K * _H) * _P_pred;
}

double PosVelKF::getPos(){
	return _state.at<float>(0,0);
}

double PosVelKF::getVel(){
	return _state.at<float>(0,0);
}

