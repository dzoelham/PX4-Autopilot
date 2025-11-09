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

#include <mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>

using namespace time_literals;

namespace
{
constexpr speed_t default_baudrate = B115200;
}

class PC104SerialServo : public ModuleBase<PC104SerialServo>, public ModuleParams
{
public:
        PC104SerialServo(const char *device, speed_t baud);
        ~PC104SerialServo() override;

        /** @see ModuleBase */
        static int task_spawn(int argc, char *argv[]);
        /** @see ModuleBase */
        static PC104SerialServo *instantiate(int argc, char *argv[]);
        /** @see ModuleBase */
        static int custom_command(int argc, char *argv[]);
        /** @see ModuleBase */
        static int print_usage(const char *reason = nullptr);

        void run() override;
        int print_status() override;

private:
        bool init();
        bool configure_port();
        void send_outputs(const actuator_servos_s &data);

        int _fd{-1};
        std::string _device_path;
        speed_t _baudrate{default_baudrate};
        uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
};

PC104SerialServo::PC104SerialServo(const char *device, speed_t baud)
        : ModuleParams(nullptr),
          _device_path(device),
          _baudrate(baud)
{
}

PC104SerialServo::~PC104SerialServo()
{
        if (_fd >= 0) {
                ::close(_fd);
                _fd = -1;
        }
}

int PC104SerialServo::print_status()
{
        PX4_INFO("publishing servo commands to %s", _device_path.c_str());
        return 0;
}

bool PC104SerialServo::init()
{
        _fd = ::open(_device_path.c_str(), O_WRONLY | O_NOCTTY);

        if (_fd < 0) {
                PX4_ERR("failed to open %s: %d", _device_path.c_str(), errno);
                return false;
        }

        if (!configure_port()) {
                PX4_ERR("failed to configure %s", _device_path.c_str());
                ::close(_fd);
                _fd = -1;
                return false;
        }

        return true;
}

bool PC104SerialServo::configure_port()
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
        config.c_cc[VTIME] = 1;

        if (tcsetattr(_fd, TCSANOW, &config) != 0) {
                PX4_ERR("tcsetattr failed: %d", errno);
                return false;
        }

        return true;
}

void PC104SerialServo::send_outputs(const actuator_servos_s &data)
{
        for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
                if (!PX4_ISFINITE(data.control[i])) {
                        continue;
                }

                const float constrained = math::constrain(data.control[i], 0.f, 1.f);
                const uint16_t pulse_us = static_cast<uint16_t>(1000.f + constrained * 1000.f);

                char buffer[32];
                const int len = snprintf(buffer, sizeof(buffer), "SERVO%u %u\n", i + 1, pulse_us);

                if (len > 0) {
                        const ssize_t written = ::write(_fd, buffer, len);

                        if (written != len) {
                                PX4_DEBUG("partial write (%d/%d)", (int)written, len);
                        }
                }
        }
}

void PC104SerialServo::run()
{
        if (!init()) {
                PX4_ERR("initialization failed");
                return;
        }

        actuator_servos_s servos{};

        while (!should_exit()) {
                if (_actuator_servos_sub.updated() && _actuator_servos_sub.copy(&servos)) {
                        send_outputs(servos);
                }

                px4_usleep(20_ms);
        }
}

int PC104SerialServo::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("pc104_servo",
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

PC104SerialServo *PC104SerialServo::instantiate(int argc, char *argv[])
{
        const char *device = "/dev/ttyS4";
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

        PC104SerialServo *instance = new PC104SerialServo(device, baud);

        if (instance == nullptr) {
                PX4_ERR("alloc failed");
        }

        return instance;
}

int PC104SerialServo::custom_command(int argc, char *argv[])
{
        return print_usage("unknown command");
}

int PC104SerialServo::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description
Serialises actuator servo commands to a dedicated controller connected to /dev/ttyS4.

Outputs are transmitted as ASCII commands of the form `SERVO<index> <pulse_us>` for each valid servo channel.
)DESCR_STR"
        );

        PRINT_MODULE_USAGE_NAME("pc104_serial_servo", "driver");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS4", nullptr, "Serial device path", true);
        PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 9600, 115200, "Baudrate", true);

        return 0;
}

extern "C" __EXPORT int pc104_serial_servo_main(int argc, char *argv[])
{
        return PC104SerialServo::main(argc, argv);
}
