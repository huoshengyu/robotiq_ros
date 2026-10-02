

#include <robotiq_driver/urcap_serial.hpp>

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <Robotiq/detail/modbus_constants.hpp>
#include <Robotiq/gripper/serial_io_exception.hpp>

namespace robotiq_driver {
namespace {
namespace modbus_constants = Robotiq::detail::modbus_constants;
namespace register_map = Robotiq::detail::register_map;

// After a failed connect, writes fail fast for this long instead of each
// spending a full connect timeout on an unreachable controller.
constexpr auto kReconnectBackoff = std::chrono::seconds{1};
// GELLO reads each reply with a single recv() of this size; the URCap answers
// every request with one short line in one segment.
constexpr std::size_t kReplyBufferSize = 1024;
constexpr uint8_t kReadHoldingRegisters = 0x03;
constexpr uint8_t kWriteMultipleRegisters = 0x10;
constexpr uint8_t kReadWriteMultipleRegisters = 0x17;
constexpr uint8_t kExceptionFlag = 0x80;
constexpr uint8_t kIllegalFunction = 0x01;
constexpr uint8_t kIllegalDataAddress = 0x02;
constexpr std::size_t kCommandBytes = register_map::kCommandDocumentedBytes;
constexpr std::size_t kStatusBytes = register_map::kStatusDocumentedBytes;

// Thrown inside this file for anything that should drop the URCap connection
// and fail the current request; write() turns it into a SerialIOException.
class UrcapError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

uint16_t crc16(const uint8_t* data, std::size_t length)
{
    uint16_t crc = 0xFFFF;
    for(std::size_t i = 0; i < length; ++i)
        {
        crc ^= data[i];
        for(int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1) != 0 ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

void appendCrc(std::vector<uint8_t>& frame)
{
    const uint16_t crc = crc16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc >> 8));
}

uint16_t readBigEndian(const std::vector<uint8_t>& frame, std::size_t offset)
{
    return static_cast<uint16_t>((frame[offset] << 8) | frame[offset + 1]);
}

std::vector<uint8_t> exceptionReply(uint8_t slave, uint8_t function, uint8_t code)
{
    std::vector<uint8_t> reply{slave, static_cast<uint8_t>(function | kExceptionFlag), code};
    appendCrc(reply);
    return reply;
}

std::vector<uint8_t> statusReply(uint8_t slave, uint8_t function, const Robotiq::GripperStatus& status)
{
    std::vector<uint8_t> reply{slave, function, static_cast<uint8_t>(kStatusBytes)};
    reply.insert(reply.end(), status.data(), status.data() + kStatusBytes);
    appendCrc(reply);
    return reply;
}

Robotiq::GripperCommand commandFromFrame(const std::vector<uint8_t>& frame, std::size_t offset)
{
    Robotiq::GripperCommand command;
    std::copy_n(frame.begin() + static_cast<std::ptrdiff_t>(offset), kCommandBytes, command.data());
    return command;
}

std::string trimTrailingWhitespace(std::string text)
{
    while(!text.empty() && std::isspace(static_cast<unsigned char>(text.back())) != 0)
    {
        text.pop_back();
    }
    return text;
}

std::optional<int> parseWhole(const std::string& text, int base)
{
    try
    {
        std::size_t consumed = 0;
        const int value = std::stoi(text, &consumed, base);
        if(consumed == text.size())
        {
            return value;
        }
    }
    catch(const std::logic_error&)
    {
    }
    return std::nullopt;
}

timeval toTimeval(std::chrono::milliseconds duration)
{
    timeval tv{};
    tv.tv_sec = static_cast<decltype(tv.tv_sec)>(duration.count() / 1000);
    tv.tv_usec = static_cast<decltype(tv.tv_usec)>((duration.count() % 1000) * 1000);
    return tv;
}

std::string errnoText(const std::string& what)
{
    return what + ": " + std::strerror(errno);
}

} // namespace

UrcapSerial::UrcapSerial(std::string host,
                         uint16_t port,
                         std::chrono::milliseconds timeout,
                         std::shared_ptr<Robotiq::Logger> logger)
    : host_(std::move(host))
    , port_(port)
    , timeout_(timeout)
    , logger_(std::move(logger))
{
}

UrcapSerial::~UrcapSerial()
{
    closeSocket();
}

void UrcapSerial::open()
{
    try
    {
        connectSocket();
    }
    catch(const UrcapError& e)
    {
        throw Robotiq::SerialIOException(e.what());
    }
    open_ = true;
}

bool UrcapSerial::isOpen() const
{
    return open_;
}

void UrcapSerial::close()
{
    closeSocket();
    open_ = false;
    pendingReply_.clear();
}

std::vector<uint8_t> UrcapSerial::read(size_t size, std::chrono::milliseconds /*timeout*/)
{
    // write() has already produced the whole reply, so there is nothing to
    // wait for: a short or empty answer is what nanoMODBUS reads as a timeout.
    const std::size_t count = std::min(size, pendingReply_.size());
    std::vector<uint8_t> chunk(pendingReply_.begin(), pendingReply_.begin() + static_cast<std::ptrdiff_t>(count));
    pendingReply_.erase(pendingReply_.begin(), pendingReply_.begin() + static_cast<std::ptrdiff_t>(count));
    return chunk;
}

void UrcapSerial::write(const std::vector<uint8_t>& data)
{
    if(!open_)
    {
        throw Robotiq::SerialIOException("the URCap link is not open");
    }
    pendingReply_.clear();
    try
    {
        pendingReply_ = handleRequest(data);
    }
    catch(const std::exception& e)
    {
        if(fd_ >= 0 && logger_)
        {
            logger_->log(Robotiq::Logger::Level::Warn,
            "dropping the URCap connection to " + host_ + ":" + std::to_string(port_) + ": " + e.what());
        }
        closeSocket();
        throw Robotiq::SerialIOException(e.what());
    }
}

std::chrono::milliseconds UrcapSerial::getTimeout() const
{
    return timeout_;
}

std::vector<uint8_t> UrcapSerial::handleRequest(const std::vector<uint8_t>& request)
{
    if(request.size() < 4)
    {
        throw UrcapError("Modbus request of " + std::to_string(request.size()) + " bytes is too short");
    }
    const std::size_t body = request.size() - 2;
    const uint16_t crc = static_cast<uint16_t>(request[body] | (request[body + 1] << 8));
    if(crc16(request.data(), body) != crc)
    {
        throw UrcapError("Modbus request failed its CRC check");
    }
    const uint8_t slave = request[0];
    const uint8_t function = request[1];
    switch(function)
    {
        case kReadHoldingRegisters:
        {
            if(request.size() != 8)
            {
                throw UrcapError("malformed FC 0x03 request");
            }
            if(readBigEndian(request, 2) != modbus_constants::kStatusAddress
            || readBigEndian(request, 4) != modbus_constants::kStatusRegisterCount)
            {
                return exceptionReply(slave, function, kIllegalDataAddress);
            }
            return statusReply(slave, function, readStatus());
        }
        case kWriteMultipleRegisters:
        {
            if(request.size() < 9 || request.size() != 9 + std::size_t{request[6]})
            {
                throw UrcapError("malformed FC 0x10 request");
            }
            if(readBigEndian(request, 2) != modbus_constants::kCommandAddress
            || readBigEndian(request, 4) != modbus_constants::kCommandRegisterCount || request[6] != kCommandBytes)
            {
                return exceptionReply(slave, function, kIllegalDataAddress);
            }
            applyCommand(commandFromFrame(request, 7));
            std::vector<uint8_t> reply(request.begin(), request.begin() + 6);
            appendCrc(reply);
            return reply;
        }
        case kReadWriteMultipleRegisters:
        {
            if(request.size() < 13 || request.size() != 13 + std::size_t{request[10]})
            {
                throw UrcapError("malformed FC 0x17 request");
            }
            if(readBigEndian(request, 2) != modbus_constants::kStatusAddress
            || readBigEndian(request, 4) != modbus_constants::kStatusRegisterCount
            || readBigEndian(request, 6) != modbus_constants::kCommandAddress
            || readBigEndian(request, 8) != modbus_constants::kCommandRegisterCount || request[10] != kCommandBytes)
            {
                return exceptionReply(slave, function, kIllegalDataAddress);
            }
            applyCommand(commandFromFrame(request, 11));
            return statusReply(slave, function, readStatus());
        }
        default:
            return exceptionReply(slave, function, kIllegalFunction);
    }
}

void UrcapSerial::applyCommand(const Robotiq::GripperCommand& command)
{
    static constexpr std::array<const char*, kFieldCount> kNames{"POS", "SPE", "FOR", "ATR", "ADR", "GTO", "ACT"};
    using Robotiq::ActionRequestBit;
    if(!command.action.get(ActionRequestBit::Activate))
    {
        // rACT low resets the gripper and it ignores the rest of the block, so
        // only the falling edge goes out, alone, as GELLO's reset does it.
        if(lastSent_[kAct] != 0)
        {
            set({{kNames[kAct], 0}});
        }
        // Re-send the motion fields with the next activation rather than trust
        // that they survived the reset. ADR is kept: it is only ever sent when
        // it changes.
        for(const Field field : {kPos, kSpe, kFor, kAtr, kGto})
        {
            lastSent_[field].reset();
        }
        lastSent_[kAct] = 0;
        return;
    }
    FieldValues wanted{};
    wanted[kPos] = command.positionRequest;
    wanted[kSpe] = command.speed;
    wanted[kFor] = command.force;
    wanted[kAtr] = command.action.get(ActionRequestBit::AutoRelease) ? 1 : 0;
    wanted[kAdr] = command.action.get(ActionRequestBit::AutoReleaseOpenDirection) ? 1 : 0;
    wanted[kGto] = command.action.get(ActionRequestBit::GoTo) ? 1 : 0;
    wanted[kAct] = 1;
    std::vector<std::pair<const char*, int>> changes;
    for(std::size_t field = 0; field < kFieldCount; ++field)
    {
        if(wanted[field] and lastSent_[field] != wanted[field])
        {
            changes.emplace_back(kNames[field], wanted[field].value());
        }
    }
    if(changes.empty())
    {
        return;
    }
    set(changes);
    lastSent_ = wanted;
}

Robotiq::GripperStatus UrcapSerial::readStatus()
{
    const int sta = get("STA");
    const int act = get("ACT");
    const int gto = get("GTO");
    const int obj = get("OBJ");
    const int flt = get("FLT");
    const int pre = get("PRE");
    const int pos = get("POS");
    Robotiq::GripperStatus status;
    status.gripperStatus = Robotiq::GripperStatusFlags::fromRaw(static_cast<uint8_t>(
        (act != 0 ? register_map::kActivationStatusMask : 0) | (gto != 0 ? register_map::kGoToEchoMask : 0)
        | ((sta << register_map::kActivationStateShift) & register_map::kActivationStateMask)
        | ((obj << register_map::kObjectDetectionShift) & register_map::kObjectDetectionMask)));
    status.data()[2] = static_cast<uint8_t>(flt);
    status.positionRequestEcho = static_cast<uint8_t>(pre);
    status.position = static_cast<uint8_t>(pos);
    // The URCap does not report motor current; gCU stays 0.
    if(!seeded_)
    {
        // Mirror how the SDK seeds its own command image from this first read
        // (GripperState::initializeImage), so its first exchange sends nothing.
        lastSent_[kPos] = pre;
        lastSent_[kSpe] = 0xFF;
        lastSent_[kFor] = 0xFF;
        lastSent_[kAtr] = 0;
        lastSent_[kAdr] = 0;
        lastSent_[kGto] = gto != 0 ? 1 : 0;
        lastSent_[kAct] = act != 0 ? 1 : 0;
        seeded_ = true;
    }
    return status;
}

void UrcapSerial::connectSocket()
{
    if(fd_ >= 0)
    {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    if(now < nextConnectAttempt_)
    {
        throw UrcapError("waiting to reconnect to the URCap at " + host_ + ":" + std::to_string(port_));
    }
    nextConnectAttempt_ = now + kReconnectBackoff;
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* addresses = nullptr;
    if(const int err = ::getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &addresses); err != 0)
    {
        throw UrcapError("cannot resolve " + host_ + ": " + ::gai_strerror(err));
    }
    const std::unique_ptr<addrinfo, decltype(&::freeaddrinfo)> guard(addresses, &::freeaddrinfo);
    const timeval tv = toTimeval(timeout_);
    const int one = 1;
    std::string lastError = "no usable address";
    for(const addrinfo* address = addresses; address != nullptr; address = address->ai_next)
    {
        const int fd = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if(fd < 0)
        {
            lastError = errnoText("socket");
            continue;
        }
        // Linux applies SO_SNDTIMEO to connect() as well, which bounds it.
        if(::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0
        || ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) != 0
        || ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0)
        {
            lastError = errnoText("setsockopt");
            ::close(fd);
            continue;
        }
        if(::connect(fd, address->ai_addr, address->ai_addrlen) != 0)
        {
            lastError = errnoText("connect");
            ::close(fd);
            continue;
        }
        fd_ = fd;
        if(logger_)
        {
            logger_->log(Robotiq::Logger::Level::Info, "connected to the URCap at " + host_ + ":" + std::to_string(port_));
        }
        return;
    }
    throw UrcapError("cannot connect to the URCap at " + host_ + ":" + std::to_string(port_) + ": " + lastError);
}

void UrcapSerial::closeSocket()
{
    if(fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

void UrcapSerial::sendLine(const std::string& line)
{
    std::size_t sent = 0;
    while(sent < line.size())
    {
        const ssize_t n = ::send(fd_, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
        if(n < 0)
        {
            if(errno == EINTR)
            {
                continue;
            }
            throw UrcapError(errnoText("send"));
        }
        sent += static_cast<std::size_t>(n);
    }
}

std::string UrcapSerial::receiveReply()
{
    std::array<char, kReplyBufferSize> buffer{};
    for(;;)
    {
        const ssize_t n = ::recv(fd_, buffer.data(), buffer.size(), 0);
        if(n > 0)
        {
            return trimTrailingWhitespace(std::string(buffer.data(), static_cast<std::size_t>(n)));
        }
        if(n == 0)
        {
            throw UrcapError("the URCap closed the connection");
        }
        if(errno == EINTR)
        {
            continue;
        }
        if(errno == EAGAIN || errno == EWOULDBLOCK)
        {
            throw UrcapError("no reply from the URCap within " + std::to_string(timeout_.count()) + " ms");
        }
        throw UrcapError(errnoText("recv"));
    }
}

std::string UrcapSerial::transact(const std::string& line)
{
    connectSocket();
    sendLine(line);
    return receiveReply();
}

int UrcapSerial::get(const std::string& variable)
{
    const std::string reply = transact("GET " + variable + "\n");
    const std::size_t space = reply.find(' ');
    if(space == std::string::npos || reply.compare(0, space, variable) != 0)
    {
        throw UrcapError("unexpected reply '" + reply + "' to GET " + variable);
    }
    const std::string text = reply.substr(space + 1);
    std::optional<int> value = parseWhole(text, 10);
    // GELLO notes FLT may come back as two characters rather than an integer;
    // read those as the hex FAULT STATUS byte (kFLT, gFLT).
    if(!value && variable == "FLT")
    {
        value = parseWhole(text, 16);
    }
    if(!value || value < 0 || value > 0xFF)
    {
        throw UrcapError("unexpected reply '" + reply + "' to GET " + variable);
    }
    return value.value();
}
void UrcapSerial::set(const std::vector<std::pair<const char*, int>>& values)
{
    std::string line = "SET";
    for(const auto& [variable, value] : values)
    {
        line += ' ';
        line += variable;
        line += ' ';
        line += std::to_string(value);
    }
    const std::string reply = transact(line + "\n");
    if(reply != "ack")
    {
        throw UrcapError("unexpected reply '" + reply + "' to " + line);
    }

}

} // namespace robotiq_driver