/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_attitude.h>

#include <matrix/math.hpp>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <sstream>
#include <termios.h>
#include <unistd.h>

using namespace time_literals;

namespace
{
constexpr speed_t default_baudrate = B115200;
constexpr float deg_to_rad = M_PI / 180.f;
}

class PC104SensorHub : public ModuleBase<PC104SensorHub>, public ModuleParams
{
public:
        PC104SensorHub(const char *device, speed_t baud);
        ~PC104SensorHub() override;

        /** @see ModuleBase */
        static int task_spawn(int argc, char *argv[]);
        /** @see ModuleBase */
        static PC104SensorHub *instantiate(int argc, char *argv[]);
        /** @see ModuleBase */
        static int custom_command(int argc, char *argv[]);
        /** @see ModuleBase */
        static int print_usage(const char *reason = nullptr);

        void run() override;
        int print_status() override;

private:
        int init();
        bool configure_port();
        void handle_line(const std::string &line);

        int _fd{-1};
        std::string _device_path;
        speed_t _baudrate{default_baudrate};
        std::string _buffer;

        uORB::PublicationMulti<sensor_gps_s> _gps_pub{ORB_ID(sensor_gps)};
        uORB::Publication<vehicle_attitude_s> _att_pub{ORB_ID(vehicle_attitude)};
        uORB::Publication<vehicle_attitude_s> _att_gt_pub{ORB_ID(vehicle_attitude_groundtruth)};
};

PC104SensorHub::PC104SensorHub(const char *device, speed_t baud)
        : ModuleParams(nullptr),
          _device_path(device),
          _baudrate(baud)
{
}

PC104SensorHub::~PC104SensorHub()
{
        if (_fd >= 0) {
                ::close(_fd);
                _fd = -1;
        }
}

int PC104SensorHub::print_status()
{
        PX4_INFO("reading from %s", _device_path.c_str());
        return 0;
}

int PC104SensorHub::init()
{
        _fd = ::open(_device_path.c_str(), O_RDONLY | O_NOCTTY);

        if (_fd < 0) {
                PX4_ERR("failed to open %s: %d", _device_path.c_str(), errno);
                return PX4_ERROR;
        }

        if (!configure_port()) {
                PX4_ERR("failed to configure %s", _device_path.c_str());
                ::close(_fd);
                _fd = -1;
                return PX4_ERROR;
        }

        return PX4_OK;
}

bool PC104SensorHub::configure_port()
{
        termios config{};

        if (tcgetattr(_fd, &config) != 0) {
                PX4_ERR("tcgetattr failed: %d", errno);
                return false;
        }

        cfmakeraw(&config);
        cfsetispeed(&config, _baudrate);
        cfsetospeed(&config, _baudrate);
        config.c_cflag |= (CLOCAL | CREAD);
        config.c_cflag &= ~CSTOPB;
        config.c_cflag &= ~CRTSCTS;

        config.c_cc[VMIN] = 0;
        config.c_cc[VTIME] = 1; // 100 ms read timeout

        if (tcsetattr(_fd, TCSANOW, &config) != 0) {
                PX4_ERR("tcsetattr failed: %d", errno);
                return false;
        }

        return true;
}

void PC104SensorHub::handle_line(const std::string &line)
{
        std::istringstream iss(line);

        float yaw_deg{};
        float pitch_deg{};
        float roll_deg{};
        float altitude_m{};
        float heading_deg{};
        double latitude_deg{};
        double longitude_deg{};

        if (!(iss >> yaw_deg >> pitch_deg >> roll_deg >> altitude_m >> heading_deg >> latitude_deg >> longitude_deg)) {
                PX4_DEBUG("malformed line: %s", line.c_str());
                return;
        }

        const float yaw_rad = yaw_deg * deg_to_rad;
        const float pitch_rad = pitch_deg * deg_to_rad;
        const float roll_rad = roll_deg * deg_to_rad;
        const float heading_rad = heading_deg * deg_to_rad;

        const hrt_abstime timestamp_sample = hrt_absolute_time();

        matrix::Eulerf euler{roll_rad, pitch_rad, yaw_rad};
        matrix::Quatf q{euler};

        vehicle_attitude_s attitude{};
        attitude.timestamp_sample = timestamp_sample;
        attitude.timestamp = hrt_absolute_time();
        attitude.q[0] = q(0);
        attitude.q[1] = q(1);
        attitude.q[2] = q(2);
        attitude.q[3] = q(3);
        attitude.delta_q_reset[0] = 1.f;
        attitude.delta_q_reset[1] = 0.f;
        attitude.delta_q_reset[2] = 0.f;
        attitude.delta_q_reset[3] = 0.f;
        attitude.quat_reset_counter = 0;

        _att_pub.publish(attitude);
        _att_gt_pub.publish(attitude);

        sensor_gps_s gps{};
        gps.timestamp = hrt_absolute_time();
        gps.lat = static_cast<int32_t>(latitude_deg * 1e7);
        gps.lon = static_cast<int32_t>(longitude_deg * 1e7);
        gps.alt = static_cast<int32_t>(altitude_m * 1e3f);
        gps.alt_ellipsoid = gps.alt;
        gps.fix_type = 3;
        gps.eph = 1.0f;
        gps.epv = 1.0f;
        gps.hdop = NAN;
        gps.vdop = NAN;
        gps.vel_m_s = 0.f;
        gps.vel_n_m_s = 0.f;
        gps.vel_e_m_s = 0.f;
        gps.vel_d_m_s = 0.f;
        gps.cog_rad = heading_rad;
        gps.vel_ned_valid = false;
        gps.timestamp_time_relative = 0;
        gps.time_utc_usec = 0;
        gps.satellites_used = 0;
        gps.heading = heading_rad;
        gps.heading_offset = NAN;
        gps.heading_accuracy = NAN;

        _gps_pub.publish(gps);
}

void PC104SensorHub::run()
{
        if (init() != PX4_OK) {
                PX4_ERR("initialization failed");
                return;
        }

        px4_pollfd_struct_t fds{};
        fds.fd = _fd;
        fds.events = POLLIN;

        constexpr int timeout_ms = 200;
        char read_buffer[128];

        while (!should_exit()) {
                int pret = px4_poll(&fds, 1, timeout_ms);

                if (pret < 0) {
                        PX4_ERR("poll error: %d", errno);
                        px4_usleep(100_ms);
                        continue;

                } else if (pret == 0) {
                        // timeout
                        continue;
                }

                if (fds.revents & POLLIN) {
                        const ssize_t len = ::read(_fd, read_buffer, sizeof(read_buffer));

                        if (len > 0) {
                                _buffer.append(read_buffer, len);

                                size_t newline_pos{0};

                                while ((newline_pos = _buffer.find('\n')) != std::string::npos) {
                                        std::string line = _buffer.substr(0, newline_pos);

                                        if (!line.empty() && line.back() == '\r') {
                                                line.pop_back();
                                        }

                                        _buffer.erase(0, newline_pos + 1);

                                        if (!line.empty()) {
                                                handle_line(line);
                                        }
                                }

                                if (_buffer.size() > 256) {
                                        // Prevent unbounded growth in case of missing newline
                                        _buffer.clear();
                                }

                        } else if (len == 0) {
                                px4_usleep(50_ms);

                        } else {
                                PX4_ERR("read error: %d", errno);
                                px4_usleep(100_ms);
                        }
                }
        }
}

int PC104SensorHub::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("pc104_sensorhub",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_DEFAULT,
                                      PX4_STACK_ADJUSTED(2048),
                                      (px4_main_t)&run_trampoline,
                                      (char *const *)argv);

        if (_task_id < 0) {
                _task_id = -1;
                return -errno;
        }

        return PX4_OK;
}

PC104SensorHub *PC104SensorHub::instantiate(int argc, char *argv[])
{
        const char *device = "/dev/ttyS3";
        speed_t baud = default_baudrate;
        bool error = false;

        int myoptind = 1;
        int ch;
        const char *myoptarg = nullptr;

        while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
                switch (ch) {
                case 'd':
                        device = myoptarg;
                        break;

                case 'b': {
                        const long baudrate = strtol(myoptarg, nullptr, 10);

                        switch (baudrate) {
                        case 9600:
                                baud = B9600;
                                break;

                        case 19200:
                                baud = B19200;
                                break;

                        case 38400:
                                baud = B38400;
                                break;

                        case 57600:
                                baud = B57600;
                                break;

                        case 115200:
                                baud = B115200;
                                break;

                        default:
                                PX4_ERR("unsupported baudrate %ld", baudrate);
                                error = true;
                                break;
                        }
                        break;
                }

                case '?':
                default:
                        error = true;
                        break;
                }
        }

        if (error) {
                        return nullptr;
        }

        PC104SensorHub *instance = new PC104SensorHub(device, baud);

        if (instance == nullptr) {
                PX4_ERR("alloc failed");
        }

        return instance;
}

int PC104SensorHub::custom_command(int argc, char *argv[])
{
        return print_usage("unknown command");
}

int PC104SensorHub::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description
Reads fused navigation data from the PC104 sensor hub over a serial connection.

The hub is expected to stream lines containing
```
yaw pitch roll altitude heading latitude longitude
```
using degrees for angular terms and metres for altitude, separated by whitespace.
)DESCR_STR"
        );

        PRINT_MODULE_USAGE_NAME("pc104_sensorhub", "driver");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", nullptr, "Serial device path", true);
        PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 9600, 115200, "Baudrate", true);

        return 0;
}

extern "C" __EXPORT int pc104_sensorhub_main(int argc, char *argv[])
{
        return PC104SensorHub::main(argc, argv);
}
