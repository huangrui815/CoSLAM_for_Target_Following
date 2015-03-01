#include "redis/cbredisclient.h"
#include <stdlib.h>
#include "string.h"
#include <string>

CBRedisClient::CBRedisClient(std::string name, std::string hostname, int port) {
  mName = name;
  mRedis = redisConnect(hostname.c_str(), port);
  if (mRedis == NULL) printf("Can't connect to redis server!\n");

  
}
CBRedisClient::~CBRedisClient() {
  redisFree(mRedis);
}

bool CBRedisClient::setPose(double ts, double ts_now, double x, double x_vel, double y, double y_vel, double yaw) {
  bool res;
  redisReply* reply = (redisReply*) redisCommand(mRedis, "SET %s %f:%f:%f:%f:%f:%f:%f",
		  mName.c_str(), ts, ts_now, x, x_vel, y, y_vel, yaw);
  if (reply && strcmp(reply->str, "OK") == 0)
    res = true; // success
  else
    res = false;

  freeReplyObject(reply);
  return res;
}

bool CBRedisClient::setPoseTarget(double ts, int present, double theta, double robot_x, double robot_y, double robot_yaw, double target_x, double target_y,
		double H){
	  bool res;
	  redisReply* reply = (redisReply*) redisCommand(mRedis, "SET %s %f:%d:%f:%f:%f:%f:%f:%f:%f:%f",
			  mName.c_str(), ts, present, theta, robot_x, robot_y, robot_yaw, target_x, target_y, H, ts);
	  if (reply && strcmp(reply->str, "OK") == 0)
	    res = true; // success
	  else
	    res = false;

	  freeReplyObject(reply);
	  return res;
}

bool CBRedisClient::setDynObj(double x, double y, double z){
  bool res;
  redisReply* reply = (redisReply*) redisCommand(mRedis, "SET %s %f:%f:%f",
		  mName.c_str(), x,y,z);
  if (reply && strcmp(reply->str, "OK") == 0)
	res = true; // success
  else
	res = false;

  freeReplyObject(reply);
  return res;
}

bool CBRedisClient::setCommand(std::string command) {
  bool res;
  redisReply* reply = (redisReply*) redisCommand(mRedis, "SET %s %s", mName.c_str(), command.c_str());
  if (reply && strcmp(reply->str, "OK") == 0)
    res = true; // success
  else
    res = false;

  freeReplyObject(reply);
  return res;
}

bool CBRedisClient::getOdometryPose(float&  x_vel){
	bool res;
	redisReply* reply = (redisReply*) redisCommand(mRedis, "GET %s", "vel");
	if (reply){
		res = true; // success
	  x_vel = atof (reply->str);
	}
	else
		res = false;

	freeReplyObject(reply);
	return res;
}

bool CBRedisClient::getStart(std::string& cmd){
	bool res;
	redisReply* reply = (redisReply*) redisCommand(mRedis, "GET %s", "start");
	if (reply && reply->str && strcmp(reply->str, "go") == 0){
		res = true; // success
		cmd = reply->str;
	}
	else
		res = false;

	freeReplyObject(reply);
	return res;
}
