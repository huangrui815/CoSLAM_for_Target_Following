#include "hiredis.h"
#include <string>

class CBRedisClient {

  public:
    CBRedisClient(const std::string robotName, const std::string hostname, int port=6379);
    ~CBRedisClient();
    bool setPose(double ts, double ts_now, double x, double x_vel, double y, double y_vel, double yaw);
    bool setPoseTarget(double ts, int present, double theta, double robot_x, double robot_y, double robot_yaw, double target_x, double target_y,
    		double H);
    bool setDynObj(double ts, double x, double y, double z);
    bool setCommand(std::string command);
    bool getOdometryPose(float& x_vel);
    bool getStart(std::string& cmd);

  private:
    std::string mName;
    redisContext* mRedis;
};
