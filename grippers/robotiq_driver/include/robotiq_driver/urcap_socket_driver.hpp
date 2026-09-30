#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <Robotiq/gripper.hpp>

namespace robotiq_driver
{

/// Driver for the Robotiq URCap's ASCII socket server (default TCP 63352).
///
/// Use this instead of DefaultDriver when the Robotiq URCap is installed: the
/// URCap owns the tool-flange RS485 line exclusively, so no Modbus RTU master
/// slot is available to us.
class UrcapSocketDriver : public Robotiq::Gripper
{
public:
    UrcapSocketDriver(std::string host, uint16_t port);

    ~UrcapSocketDriver() override;

    void set_slave_address(uint8_t) override {}  // not applicable over the socket

    bool connect() override;

    void disconnect() override;

    void activate() override;

    void deactivate() override;

    void set_gripper_position(uint8_t pos) override;

    uint8_t get_gripper_position() override;

    bool gripper_is_moving() override;

    void set_speed(uint8_t speed) override;

    void set_force(uint8_t force) override;

private:
    void set_var(const std::string& name, int value);

    int get_var(const std::string& name);

    std::string transact(const std::string& request);

    std::string host_;

    uint16_t port_;

    int fd_{ -1 };

    std::mutex io_mutex_;
};

}  // namespace robotiq_driver