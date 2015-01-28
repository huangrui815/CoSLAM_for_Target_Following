#include "redis/cbredisclient.h"
#include "string.h"

CBRedisClient::CBRedisClient(std::string name, std::string hostname, int port) {
  mName = name;
  mRedis = redisConnect(hostname.c_str(), port);
  if (mRedis == NULL) printf("Can't connect to redis server!\n");

  
}
CBRedisClient::~CBRedisClient() {
  redisFree(mRedis);
}

bool CBRedisClient::setPose(double x, double y, double yaw) {
  bool res;
  redisReply* reply = (redisReply*) redisCommand(mRedis, "SET %s %f:%f:%f", mName.c_str(), x, y, yaw);
  if (!reply && strcmp(reply->str, "OK") == 0)
    res = true; // success
  else
    res = false;

  freeReplyObject(reply);
  return res;
}

bool CBRedisClient::setCommand(std::string command) {
  bool res;
  redisReply* reply = (redisReply*) redisCommand(mRedis, "SET %s %s", mName.c_str(), command.c_str());
  if (!reply && strcmp(reply->str, "OK") == 0)
    res = true; // success
  else
    res = false;

  freeReplyObject(reply);
  return res;
}
