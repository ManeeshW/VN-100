#include "vn100.h"
#include <stdexcept>
#include <iostream>
#include <algorithm>

vn100::vn100() : frequency(200), is_open(false), ez(nullptr), data_available(false)
{
}

vn100::~vn100()
{
    if (is_open)
    {
        close();
    }
}

void vn100::load_config(const std::string& config_file)
{
    std::ifstream file(config_file);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open config file: " + config_file);
    }

    std::string line;
    std::string section;

    while (std::getline(file, line))
    {
        // Preserve original line for parsing, but remove trailing whitespace
        std::string trimmed_line = line;
        trimmed_line.erase(trimmed_line.find_last_not_of(" \t") + 1);

        if (trimmed_line.empty() || trimmed_line[0] == '#')
            continue;

        // Check for section header (e.g., "vn100:" or "[vn100]")
        if (trimmed_line.back() == ':' || (trimmed_line[0] == '[' && trimmed_line.back() == ']'))
        {
            section = trimmed_line[0] == '[' ? trimmed_line.substr(1, trimmed_line.size() - 2) : trimmed_line.substr(0, trimmed_line.size() - 1);
            section.erase(std::remove_if(section.begin(), section.end(), ::isspace), section.end());
            continue;
        }

        if (section == "vn100")
        {
            // Handle indented lines
            trimmed_line.erase(0, trimmed_line.find_first_not_of(" \t"));
            if (trimmed_line.empty())
                continue;

            // Support both "key=value" and "key: value" formats
            size_t pos = trimmed_line.find_first_of("=:");
            if (pos != std::string::npos)
            {
                std::string key = trimmed_line.substr(0, pos);
                key.erase(std::remove_if(key.begin(), key.end(), ::isspace), key.end());
                std::string value = trimmed_line.substr(pos + 1);
                value.erase(0, value.find_first_not_of(" \t\""));
                value.erase(value.find_last_not_of(" \t\"") + 1);
                if (!key.empty() && !value.empty())
                {
                    config_params[key] = value;
                }
            }
        }
    }

    file.close();

    if (config_params.find("port") != config_params.end())
        port = config_params["port"];
    else
        throw std::runtime_error("Config file missing 'port' parameter");

    if (config_params.find("baud_rate") != config_params.end())
        baud_rate = std::stoi(config_params["baud_rate"]);
    else
        throw std::runtime_error("Config file missing 'baud_rate' parameter");

    if (config_params.find("frequency") != config_params.end())
        frequency = std::stoi(config_params["frequency"]);
    else
        throw std::runtime_error("Config file missing 'frequency' parameter");
}

void vn100::open()
{
    read(port, baud_rate);
}

void vn100::read(std::string port, int baud_rate)
{
    try
    {
        vs.connect(port, baud_rate);
        string mn = vs.readModelNumber();
        cout << "Model Number: " << mn << endl;

        cout << "Setting up binary output register .." << endl;
        // Disable ASCII output
        vs.writeAsyncDataOutputType(VNOFF);

        // Default port number to 2
        int port_number = 2;

        if (port_number == 1)
        {
            BinaryOutputRegister bor(
                ASYNCMODE_PORT1, // Output to port 1
                static_cast<uint16_t>(800 / frequency), // Rate divisor (800 Hz / desired frequency)
                COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL | COMMONGROUP_QUATERNION, // Data fields
                TIMEGROUP_NONE,  // No time group data
                IMUGROUP_NONE,   // No IMU group data
                GPSGROUP_NONE,   // No GPS group data
                ATTITUDEGROUP_NONE, // No attitude group data
                INSGROUP_NONE,   // No INS group data
                GPSGROUP_NONE    // No GPS2 group data
            );
            vs.writeBinaryOutput1(bor); // Configure Binary Output Register 1
        }
        else if (port_number == 2)
        {
            BinaryOutputRegister bor(
                ASYNCMODE_PORT2, // Output to port 2
                static_cast<uint16_t>(800 / frequency), // Rate divisor (800 Hz / desired frequency)
                COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE| COMMONGROUP_ACCEL | COMMONGROUP_QUATERNION, // Data fields
                TIMEGROUP_NONE,  // No time group data
                IMUGROUP_NONE,   // No IMU group data
                GPSGROUP_NONE,   // No GPS group data
                ATTITUDEGROUP_NONE, // Linear acceleration in body frame
                INSGROUP_NONE,   // No INS group data
                GPSGROUP_NONE    // No GPS2 group data
            );
            vs.writeBinaryOutput2(bor); // Configure Binary Output Register 2
        }
        else
        {
            std::cout << "Error: port number is incorrectly set at " << port_number << std::endl;
            throw std::runtime_error("Invalid port number: " + std::to_string(port_number));
        }

        // Ensure the binary output frequency is set
        vs.writeAsyncDataOutputFrequency(frequency);

        ez = EzAsyncData::connect(port, baud_rate);
        is_open = true;
    }
    catch (const vn::not_found& e)
    {
        throw std::runtime_error("Failed to connect to VN100 sensor at " + port +
                                 " with baud rate " + std::to_string(baud_rate) +
                                 ": " + e.what());
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error connecting to VN100 sensor: " + std::string(e.what()));
    }
}

void vn100::loop()
{
    if (!is_open || !ez)
        return;

    try
    {
        CompositeData data = ez->getNextData();
        if (data.hasYawPitchRoll() && data.hasAngularRate() && 
            data.hasAcceleration())
        {
            latest_ypr = data.yawPitchRoll();
            latest_ang_rate = data.angularRate();
            latest_accel = data.acceleration();
            data_available = true;
        }
        else
        {
            data_available = false;
            std::cerr << "Incomplete data received from VN100 sensor" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        data_available = false;
        std::cerr << "Data retrieval error: " << e.what() << std::endl;
    }
}

bool vn100::get_latest_data(vec3f& ypr, vec3f& ang_rate, vec4f& quat, vec3f& accel)
{
    if (!data_available)
        return false;

    ypr = latest_ypr;
    ang_rate = latest_ang_rate;
    quat = latest_quat;
    accel = latest_accel;
    return true;
}

void vn100::close()
{
    if (ez)
    {
        ez->disconnect();
        delete ez;
        ez = nullptr;
    }
    vs.disconnect();
    is_open = false;
}