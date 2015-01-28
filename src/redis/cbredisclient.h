#include "hiredis.h"
#include <string>

class CBRedisClient {

  public:
    CBRedisClient(const std::string robotName, const std::string hostname, int port=6379);
    ~CBRedisClient();
    bool setPose(double x, double y, double yaw);
    bool setCommand(std::string command);

  private:
    std::string mName;
    redisContext* mRedis;
};
