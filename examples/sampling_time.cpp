// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

bool running = true;

void signal_handler(int signal) {
    std::cout << "Cought signal : " << signal << std::endl;
    if (signal == SIGTERM || signal == SIGINT) {
        std::cout << "Ctrl C pressed !!!" << std::endl;
        running = false;
    }
}

int main() {
    try {
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        std::cout << "=== OpenArm CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the OpenArm API functionality" << std::endl;

        // Initialize OpenArm with CAN interface and enable CAN-FD
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm("can0", true);  // Use CAN-FD on can0 interface

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM8009, openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340, openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310};

        std::vector<uint32_t> send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Enable all motors
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        openarm.enable_all();
        // Allow time (2ms) for the motors to respond for slow operations like enabling
        openarm.recv_all(2000);

        // Set device mode to param and query motor id
        std::cout << "\n=== Querying Motor Recv IDs ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::MST_ID));
        // Allow time (2ms) for the motors to respond for slow operations like querying
        // parameter from register
        openarm.recv_all(2000);

        // Access motors through components
        for (const auto& motor : openarm.get_arm().get_motors()) {
            std::cout << "Arm Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }

        // Set device mode to state and control motor
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Control arm motors with position control
        std::vector<openarm::damiao_motor::MITParam> params(8, {0, 0, 0, 0, 0});
        openarm.get_arm().mit_control_all(params);
        openarm.recv_all(500);

        int count = 0;
        while (running) {
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // openarm.refresh_all();

            auto start = std::chrono::high_resolution_clock::now();
            openarm.recv_all(100);
            openarm.get_arm().mit_control_all(params);
            // Display arm motor states
            for (const auto& motor : openarm.get_arm().get_motors()) {
                // std::cout << "Arm Motor: " << motor.get_send_can_id()
                //           << " position: " << motor.get_position() << std::endl;
            }
            auto end = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

            if (count++ % 1000 == 0) {
                std::cout << "\n==============================================" << std::endl;
                std::cout << "excution time : " << 1e6 / duration.count() << " : hz" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.disable_all();
        openarm.recv_all(1000);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
