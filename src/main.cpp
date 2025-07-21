#include "vn100.h"
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    vn100 imu;

    try
    {
        imu.load_config("../config.cfg");
        cout << "Loaded config: port=" << imu.port << ", baud_rate=" << imu.baud_rate << ", frequency=" << imu.frequency << endl;
        imu.open();
        cout << "Connected to VN100 sensor" << endl;

        // Calculate target duration based on frequency (in Hz)
        const auto target_duration = milliseconds(1000 / imu.frequency);

        // Run for 10 seconds
        const int cycles = imu.frequency * 10;
        for (int i = 0; i < cycles; i++)
        {
            auto loop_start = steady_clock::now();

            imu.loop();

            vec3f ypr, ang_rate, accel;
            vec4f quat;
            if (imu.get_latest_data(ypr, ang_rate, quat, accel))
            {
                cout << "YPR: " << ypr << " | Gyro: " << ang_rate 
                     << " | Accel: " << accel << " | Quaternion: " << quat << endl;
            }

            // Calculate elapsed time and sleep for the remaining duration
            auto loop_end = steady_clock::now();
            auto elapsed = duration_cast<milliseconds>(loop_end - loop_start);
            auto sleep_time = target_duration - elapsed;

            if (sleep_time > milliseconds(0))
            {
                this_thread::sleep_for(sleep_time);
            }
        }

        imu.close();
    }
    catch (const std::exception& e)
    {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}