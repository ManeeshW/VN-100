#ifndef VN100_H
#define VN100_H

#include <string>
#include <map>
#include <fstream>
#include <vn/sensors.h>
#include <vn/compositedata.h>
#include <vn/ezasyncdata.h>
#include <vn/thread.h>
#include <iostream>

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace std;

class vn100
{
public:
    vn100();
    ~vn100();

    std::string port;
    uint32_t baud_rate;
    uint32_t frequency; 
    bool is_open;

    void load_config(const std::string& config_file);
    void open();
    void read(std::string port, int baud_rate);
    void loop();
    void close();

    // Method to access the latest sensor data
    bool get_latest_data(vec3f& ypr, vec3f& ang_rate, vec4f& quat, vec3f& accel);

private:
    VnSensor vs;
    EzAsyncData* ez;
    std::map<std::string, std::string> config_params;

    // Store the latest sensor data
    vec3f latest_ypr;
    vec3f latest_ang_rate;
    vec4f latest_quat;
    vec3f latest_accel;
    bool data_available;
};

#endif