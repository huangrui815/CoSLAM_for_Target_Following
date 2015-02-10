#include "hiredis.h"
#include <string>

class CBRedisClient {

  public:
    CBRedisClient(const std::string robotName, const std::string hostname, int port=6379);
    ~CBRedisClient();
    bool setPose(double ts, double ts_now, double x, double x_vel, double y, double y_vel, double yaw);
    bool setCommand(std::string command);
    bool getOdometryPose(float& x_vel);
    bool getStart(std::string& cmd);

  private:
    std::string mName;
    redisContext* mRedis;
};
