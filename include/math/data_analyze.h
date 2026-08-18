#ifndef DATA_ANALYZE
#define DATA_ANALYZE
#include <curl/curl.h>
#include <sstream>
#include <string>
#include "math/mathtool.h"

class data_analyze
{
public:
    data_analyze(/* args */);
    void sendComPos(double sim_time, const Eigen::Vector3d& pcom);

    ~data_analyze();
private:

};




#endif