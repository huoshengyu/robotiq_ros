//! \brief A Robotiq::Serial that reaches the gripper through the Robotiq
//! URCap's ASCII socket (TCP 63352 on the UR controller) instead of a
//! serial port.
//! The URCap owns the tool-flange RS485 line, so neither a serial port nor UR
//! tool communication can reach the gripper while it is installed. Its socket
//! server can: it takes `GET <VAR>` / `SET <VAR> <value> ...` lines, the same
//! interface GELLO drives. This transport answers the SDK's Modbus RTU
//! requests by translating them into those lines, so Robotiq::Gripper and
//! everything above it run unchanged.

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <Robotiq/gripper/command.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/serial.hpp>
#include <Robotiq/gripper/status.hpp>

namespace robotiq_driver {
    
inline constexpr auto kUrcapTimeoutDefault = std::chrono::milliseconds{500};

//! Synchronous, like the SDK's fake transport: write() takes one whole RTU
//! request (nanoMODBUS sends each frame in a single write), runs the URCap
//! round trips it implies, and queues the RTU reply for read() to hand back.
//!
//! Only the three requests Robotiq::Gripper issues are understood: FC 0x03 on
//! the status block, FC 0x10 on the command block, and FC 0x17 across both.
//! Anything else gets a Modbus exception reply.
//!
//! URCap failures surface as SerialIOException from write(), which the SDK
//! counts as a failed exchange. The socket is dropped on every failure, so a
//! late reply can never be read as the answer to the next request, and is
//! reopened by the next write().
class UrcapSerial : public Robotiq::Serial
{
public:
    //! \param host The UR controller's address; an IP or a resolvable name.
    //! \param port The URCap socket server's port, 63352 on a stock install.
    //! \param timeout Budget for connecting and for each URCap reply.
    //! \param logger Sink for reconnect notices; may be null.
    UrcapSerial(std::string host,
                uint16_t port,
                std::chrono::milliseconds timeout = kUrcapTimeoutDefault,
                std::shared_ptr<Robotiq::Logger> logger = nullptr);

    ~UrcapSerial() override;

    UrcapSerial(const UrcapSerial&) = delete;

    UrcapSerial& operator=(const UrcapSerial&) = delete;

    void open() override;

    [[nodiscard]] bool isOpen() const override;

    void close() override;

    [[nodiscard]] std::vector<uint8_t> read(size_t size, std::chrono::milliseconds timeout) override;

    void write(const std::vector<uint8_t>& data) override;

    [[nodiscard]] std::chrono::milliseconds getTimeout() const override;

private:
    //! The URCap variables the command block maps onto, in the order one SET
    //! line carries them. ACT goes last so a line that activates the gripper
    //! has already set everything else.
    enum Field : std::size_t
    {
    kPos,
    kSpe,
    kFor,
    kAtr,
    kAdr,
    kGto,
    kAct,
    kFieldCount
    };
    using FieldValues = std::array<std::optional<int>, kFieldCount>;
    
    std::vector<uint8_t> handleRequest(const std::vector<uint8_t>& request);
    void applyCommand(const Robotiq::GripperCommand& command);
    Robotiq::GripperStatus readStatus();
    void connectSocket();
    void closeSocket();
    void sendLine(const std::string& line);
    std::string receiveReply();
    std::string transact(const std::string& line);
    int get(const std::string& variable);
    void set(const std::vector<std::pair<const char, int>>& values);
    std::string host_;
    uint16_t port_;
    std::chrono::milliseconds timeout_;
    std::shared_ptr<Robotiq::Logger> logger_;
    bool open_ = false;
    int fd_ = -1;
    std::chrono::steady_clock::time_point nextConnectAttempt_{};

    //! What the URCap was last told, per variable; empty where unknown. Seeded
    //! from the first status read, so connecting does not re-send a command to
    //! a gripper another client (GELLO) already set up.
    FieldValues lastSent_{};
    bool seeded_ = false;
    std::vector<uint8_t> pendingReply_;
};

} // namespace robotiq_driver