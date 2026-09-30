#include "robotiq_driver/urcap_socket_driver.hpp"
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace robotiq_driver
{

namespace
{
constexpr auto kReadTimeout = std::chrono::milliseconds(200);
constexpr auto kActivationTimeout = std::chrono::seconds(10);
}  // namespace

UrcapSocketDriver::UrcapSocketDriver(std::string host, uint16_t port)
 : host_(std::move(host)), port_(port)
{

}

UrcapSocketDriver::~UrcapSocketDriver()
{
 disconnect();
}

bool UrcapSocketDriver::connect()

{
 std::lock_guard<std::mutex> lock(io_mutex_);
 fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
 if (fd_ < 0)
 {
   return false;
 }

 int one = 1;
 ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
 timeval tv{};
 tv.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(kReadTimeout).count();
 ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
 ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
 sockaddr_in addr{};
 addr.sin_family = AF_INET;
 addr.sin_port = htons(port_);

 if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1 ||
     ::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
 {
   ::close(fd_);
   fd_ = -1;
   return false;
 }
 return true;
}

void UrcapSocketDriver::disconnect()
{
 std::lock_guard<std::mutex> lock(io_mutex_);
 if (fd_ >= 0)
 {
   ::close(fd_);
   fd_ = -1;
 }
}

std::string UrcapSocketDriver::transact(const std::string& request)
{
 std::lock_guard<std::mutex> lock(io_mutex_);

 if (fd_ < 0)
 {
   throw std::runtime_error("URCap socket is not connected");
 }
 if (::send(fd_, request.data(), request.size(), 0) != static_cast<ssize_t>(request.size()))
 {
   throw std::runtime_error("failed writing to URCap socket: " + std::string(strerror(errno)));
 }

 // The URCap terminates every reply with '\n' and never pipelines, so one
 // read-to-newline is a complete response.
 std::string reply;
 char c = 0;
 while (true)
 {
   const ssize_t n = ::recv(fd_, &c, 1, 0);
   if (n <= 0)
   {
     throw std::runtime_error("timed out reading from URCap socket");
   }
   if (c == '\n')
   {
     return reply;
   }
   reply.push_back(c);
 }
}

void UrcapSocketDriver::set_var(const std::string& name, int value)
{
 const auto reply = transact("SET " + name + " " + std::to_string(value) + "\n");

 if (reply.rfind("ack", 0) != 0)
 {
   throw std::runtime_error("URCap rejected SET " + name + ": " + reply);
 }
}

int UrcapSocketDriver::get_var(const std::string& name)
{
 const auto reply = transact("GET " + name + "\n");

 const auto space = reply.find(' ');

 if (space == std::string::npos)
 {
   throw std::runtime_error("malformed URCap reply for GET " + name + ": " + reply);
 }
 return std::stoi(reply.substr(space + 1));
}

void UrcapSocketDriver::activate()
{
 // A gripper already activated by the pendant or by another client must not be
 // re-activated: ACT 0 -> 1 re-runs the calibration stroke and drops the jaws.
 if (get_var("STA") != 3)
 {
   set_var("ACT", 0);

   set_var("ACT", 1);

   const auto deadline = std::chrono::steady_clock::now() + kActivationTimeout;

   while (get_var("STA") != 3)
   {
     if (std::chrono::steady_clock::now() > deadline)
     {
       throw std::runtime_error("gripper failed to activate within timeout");
     }
     std::this_thread::sleep_for(std::chrono::milliseconds(100));
   }
 }
 set_var("GTO", 1);
}

void UrcapSocketDriver::deactivate()
{
 set_var("GTO", 0);
}

void UrcapSocketDriver::set_gripper_position(uint8_t pos)
{
 set_var("POS", pos);
}

uint8_t UrcapSocketDriver::get_gripper_position()
{
 return static_cast<uint8_t>(get_var("POS"));
}

bool UrcapSocketDriver::gripper_is_moving()
{
 return get_var("OBJ") == 0;
}

void UrcapSocketDriver::set_speed(uint8_t speed)
{
 set_var("SPE", speed);
}

void UrcapSocketDriver::set_force(uint8_t force)
{
 set_var("FOR", force);
}

}  // namespace robotiq_driver