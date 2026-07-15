// safety_io_node.cpp — PILZ PNOZ m B0.1 + PNOZ m ES ETH Safety I/O driver
//
// The node reads the physical I/O process data of the PNOZmulti 2 over
// Modbus/TCP and publishes it to ROS.
//
// Register map source: Pilz "PNOZmulti 2 Communication Interfaces"
// (1002971-EN), Process data addressing on base unit.
//   physical inputs  i0..i31: Discrete Inputs 16384..16415
//   physical outputs o0..o31: Discrete Inputs 16416..16447

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/file.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string deviceLockPath(const std::string& host, int port)
{
  std::string endpoint = host + "_" + std::to_string(port);
  std::transform(endpoint.begin(), endpoint.end(), endpoint.begin(), [](unsigned char character) {
    return std::isalnum(character) ? static_cast<char>(character) : '_';
  });
  return "/tmp/safety_io_driver_" + endpoint + ".lock";
}

class DeviceLock
{
public:
  DeviceLock(const std::string& host, int port) : path_(deviceLockPath(host, port))
  {
    fd_ = ::open(path_.c_str(), O_CREAT | O_RDWR, 0660);
    if (fd_ < 0)
      throw std::runtime_error("cannot open device lock " + path_ + ": " + std::strerror(errno));

    if (::flock(fd_, LOCK_EX | LOCK_NB) < 0)
    {
      const std::string error = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error(
        "another local process is using " + host + ":" + std::to_string(port) +
        " (lock " + path_ + "): " + error);
    }
  }

  ~DeviceLock()
  {
    if (fd_ >= 0)
    {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

  DeviceLock(const DeviceLock&) = delete;
  DeviceLock& operator=(const DeviceLock&) = delete;

private:
  std::string path_;
  int fd_ = -1;
};

class ModbusTcpClient
{
public:
  ModbusTcpClient(const std::string& host, int port, uint8_t unit_id, double timeout_s)
    : host_(host), port_(port), unit_id_(unit_id), fd_(-1), transaction_id_(0)
  {
    const double bounded_timeout = std::max(timeout_s, 0.01);
    timeout_.tv_sec = static_cast<long>(bounded_timeout);
    timeout_.tv_usec = static_cast<long>((bounded_timeout - timeout_.tv_sec) * 1e6);
  }

  ~ModbusTcpClient() { closeSocket(); }

  bool isOpen() const { return fd_ >= 0; }
  const std::string& lastError() const { return last_error_; }

  bool openSocket()
  {
    closeSocket();

    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
      setErrnoError("socket");
      return false;
    }

    const int original_flags = ::fcntl(fd, F_GETFL, 0);
    if (original_flags < 0 || ::fcntl(fd, F_SETFL, original_flags | O_NONBLOCK) < 0)
    {
      setErrnoError("fcntl");
      ::close(fd);
      return false;
    }

    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1)
    {
      last_error_ = "invalid IPv4 address: " + host_;
      ::close(fd);
      return false;
    }

    int result = ::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    if (result < 0 && errno != EINPROGRESS)
    {
      setErrnoError("connect");
      ::close(fd);
      return false;
    }

    if (result < 0)
    {
      fd_set write_set;
      FD_ZERO(&write_set);
      FD_SET(fd, &write_set);
      timeval connect_timeout = timeout_;
      result = ::select(fd + 1, nullptr, &write_set, nullptr, &connect_timeout);
      if (result <= 0)
      {
        last_error_ = result == 0 ? "connect timeout" : std::string("select: ") + std::strerror(errno);
        ::close(fd);
        return false;
      }

      int socket_error = 0;
      socklen_t socket_error_length = sizeof(socket_error);
      if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_length) < 0 ||
          socket_error != 0)
      {
        last_error_ = std::string("connect: ") +
                      std::strerror(socket_error != 0 ? socket_error : errno);
        ::close(fd);
        return false;
      }
    }

    if (::fcntl(fd, F_SETFL, original_flags) < 0)
    {
      setErrnoError("fcntl");
      ::close(fd);
      return false;
    }

    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout_, sizeof(timeout_));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout_, sizeof(timeout_));
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    fd_ = fd;
    last_error_.clear();
    return true;
  }

  void closeSocket()
  {
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // FC02: read discrete inputs. Values are unpacked LSB-first per Modbus.
  bool readDiscreteInputs(uint16_t address, uint16_t quantity, std::vector<uint8_t>& values)
  {
    if (quantity == 0 || quantity > 2000)
    {
      last_error_ = "invalid FC02 quantity";
      return false;
    }

    const uint8_t request[] = {
      0x02,
      static_cast<uint8_t>(address >> 8),
      static_cast<uint8_t>(address & 0xFF),
      static_cast<uint8_t>(quantity >> 8),
      static_cast<uint8_t>(quantity & 0xFF)
    };

    std::vector<uint8_t> response;
    if (!transact(request, sizeof(request), response)) return false;

    const size_t expected_bytes = (quantity + 7U) / 8U;
    if (response.size() < 2 || response[0] != 0x02 ||
        response[1] != expected_bytes || response.size() != expected_bytes + 2U)
    {
      last_error_ = "malformed FC02 response";
      closeSocket();
      return false;
    }

    values.assign(quantity, 0);
    for (uint16_t bit = 0; bit < quantity; ++bit)
      values[bit] = (response[2 + bit / 8] >> (bit % 8)) & 0x01;

    last_error_.clear();
    return true;
  }

private:
  bool transact(const uint8_t* pdu, uint16_t pdu_length, std::vector<uint8_t>& response)
  {
    if (!isOpen() && !openSocket()) return false;

    const uint16_t transaction_id = ++transaction_id_;
    const uint16_t mbap_length = pdu_length + 1;  // Unit identifier + PDU.
    uint8_t request[260];
    request[0] = static_cast<uint8_t>(transaction_id >> 8);
    request[1] = static_cast<uint8_t>(transaction_id & 0xFF);
    request[2] = 0;
    request[3] = 0;
    request[4] = static_cast<uint8_t>(mbap_length >> 8);
    request[5] = static_cast<uint8_t>(mbap_length & 0xFF);
    request[6] = unit_id_;
    std::memcpy(request + 7, pdu, pdu_length);

    if (!sendAll(request, 7 + pdu_length))
    {
      closeSocket();
      return false;
    }

    uint8_t header[7];
    if (!receiveAll(header, sizeof(header)))
    {
      closeSocket();
      return false;
    }

    const uint16_t received_transaction_id =
      static_cast<uint16_t>((header[0] << 8) | header[1]);
    const uint16_t protocol_id = static_cast<uint16_t>((header[2] << 8) | header[3]);
    const uint16_t response_length = static_cast<uint16_t>((header[4] << 8) | header[5]);
    if (received_transaction_id != transaction_id || protocol_id != 0 ||
        header[6] != unit_id_ || response_length < 2 || response_length > 253)
    {
      last_error_ = "invalid Modbus/TCP MBAP header";
      closeSocket();
      return false;
    }

    const size_t pdu_bytes = response_length - 1;
    std::vector<uint8_t> body(pdu_bytes);
    if (!receiveAll(body.data(), body.size()))
    {
      closeSocket();
      return false;
    }

    if (!body.empty() && (body[0] & 0x80) != 0)
    {
      char message[64];
      const unsigned int exception_code = body.size() > 1 ? body[1] : 0;
      std::snprintf(message, sizeof(message), "Modbus exception 0x%02X", exception_code);
      last_error_ = message;
      return false;
    }

    response.swap(body);
    return true;
  }

  bool sendAll(const uint8_t* data, size_t length)
  {
    size_t sent = 0;
    while (sent < length)
    {
      const ssize_t result = ::send(fd_, data + sent, length - sent, MSG_NOSIGNAL);
      if (result < 0 && errno == EINTR) continue;
      if (result <= 0)
      {
        setErrnoError("send");
        return false;
      }
      sent += static_cast<size_t>(result);
    }
    return true;
  }

  bool receiveAll(uint8_t* data, size_t length)
  {
    size_t received = 0;
    while (received < length)
    {
      const ssize_t result = ::recv(fd_, data + received, length - received, 0);
      if (result < 0 && errno == EINTR) continue;
      if (result <= 0)
      {
        last_error_ = result == 0 ? "peer closed connection" :
                                    std::string("recv: ") + std::strerror(errno);
        return false;
      }
      received += static_cast<size_t>(result);
    }
    return true;
  }

  void setErrnoError(const char* operation)
  {
    last_error_ = std::string(operation) + ": " + std::strerror(errno);
  }

  std::string host_;
  int port_;
  uint8_t unit_id_;
  int fd_;
  uint16_t transaction_id_;
  timeval timeout_;
  std::string last_error_;
};

struct Signal
{
  std::string name;
  int bit;
};

std::string normaliseTopicPrefix(std::string prefix)
{
  if (prefix.empty()) prefix = "/safety";
  if (prefix.front() != '/') prefix.insert(prefix.begin(), '/');
  while (prefix.size() > 1 && prefix.back() == '/') prefix.pop_back();
  return prefix;
}

}  // namespace

class SafetyIONode
{
public:
  SafetyIONode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  {
    private_nh.param<std::string>("ip", ip_, "192.168.100.103");
    private_nh.param("port", port_, 502);
    int unit_id;
    private_nh.param("unit_id", unit_id, 1);
    private_nh.param("timeout", timeout_, 1.0);
    private_nh.param("publish_rate", publish_rate_, 10.0);
    private_nh.param("input_discrete_base", input_discrete_base_, 16384);
    private_nh.param("output_discrete_base", output_discrete_base_, 16416);
    // IM16 is an auxiliary output, but its configurable-I/O status is exposed
    // in the base-unit input process image at bit 16 (FC02 address 0x4010).
    private_nh.param("input_count", input_count_, 17);
    private_nh.param("output_count", output_count_, 4);
    private_nh.param<std::string>("topic_prefix", topic_prefix_, "/safety");

    if (unit_id < 0 || unit_id > 255) throw std::runtime_error("unit_id must be in [0, 255]");
    unit_id_ = static_cast<uint8_t>(unit_id);
    validateAddressRange("input", input_discrete_base_, input_count_);
    validateAddressRange("output", output_discrete_base_, output_count_);
    if (port_ <= 0 || port_ > 65535) throw std::runtime_error("port must be in [1, 65535]");
    if (timeout_ <= 0.0) throw std::runtime_error("timeout must be greater than zero");
    if (publish_rate_ <= 0.0) throw std::runtime_error("publish_rate must be greater than zero");

    topic_prefix_ = normaliseTopicPrefix(topic_prefix_);
    loadSignals(private_nh, "inputs", defaultInputs(), input_count_, inputs_);
    loadSignals(private_nh, "outputs", defaultOutputs(), output_count_, outputs_);
    loadSignals(private_nh, "config_io_outputs", defaultConfigIoOutputs(), input_count_,
                config_io_outputs_);

    for (const Signal& config_io_output : config_io_outputs_)
    {
      const bool duplicate = std::any_of(outputs_.begin(), outputs_.end(),
        [&config_io_output](const Signal& output) {
          return output.name == config_io_output.name;
        });
      if (duplicate)
        throw std::runtime_error("duplicate output signal name '" + config_io_output.name + "'");
    }

    device_lock_.reset(new DeviceLock(ip_, port_));
    client_.reset(new ModbusTcpClient(ip_, port_, unit_id_, timeout_));

    connected_publisher_ = nh.advertise<std_msgs::Bool>(topic_prefix_ + "/connected", 1, true);
    state_publisher_ = nh.advertise<std_msgs::String>(topic_prefix_ + "/state_all", 1, true);

    for (const Signal& signal : inputs_)
      input_publishers_[signal.name] =
        nh.advertise<std_msgs::Bool>(topic_prefix_ + "/input/" + signal.name, 1, true);
    for (const Signal& signal : outputs_)
      output_publishers_[signal.name] =
        nh.advertise<std_msgs::Bool>(topic_prefix_ + "/output/" + signal.name, 1, true);
    for (const Signal& signal : config_io_outputs_)
      output_publishers_[signal.name] =
        nh.advertise<std_msgs::Bool>(topic_prefix_ + "/output/" + signal.name, 1, true);

    publishConnected(false);

    ROS_INFO("[safety_io] %s:%d unit=%u, input DI=%d..%d, output DI=%d..%d",
             ip_.c_str(), port_, static_cast<unsigned int>(unit_id_),
             input_discrete_base_, input_discrete_base_ + input_count_ - 1,
             output_discrete_base_, output_discrete_base_ + output_count_ - 1);

    timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_), &SafetyIONode::poll, this);
  }

  ~SafetyIONode()
  {
    timer_.stop();
    std::lock_guard<std::mutex> lock(client_mutex_);
    client_->closeSocket();
  }

private:
  static std::vector<Signal> defaultInputs()
  {
    return {
      {"safety_emergency_1b", 0},
      {"safety_emergency_2b", 1},
      {"safety_auto_mode", 4},
      {"safety_manual_mode", 5},
      {"safety_reset_switch", 6},
      {"safety_brake_release_sw", 7},
      {"safety_bumper_front", 9},
      {"safety_bumper_rear", 10}
    };
  }

  static std::vector<Signal> defaultOutputs()
  {
    return {
      {"motor_sto_01sr", 0},
      {"motor_sto_02sr", 1},
      {"traction_motor_power_on", 3}
    };
  }

  static std::vector<Signal> defaultConfigIoOutputs()
  {
    return {
      {"brake_release", 16}
    };
  }

  static void validateAddressRange(const char* label, int base, int count)
  {
    if (base < 0 || base > 65535 || count <= 0 || count > 2000 || base + count > 65536)
      throw std::runtime_error(std::string(label) + " Modbus address/count is invalid");
  }

  static void loadSignals(ros::NodeHandle& private_nh, const std::string& parameter,
                          const std::vector<Signal>& defaults, int available_bits,
                          std::vector<Signal>& destination)
  {
    XmlRpc::XmlRpcValue values;
    if (private_nh.getParam(parameter, values) &&
        values.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int index = 0; index < values.size(); ++index)
      {
        if (values[index].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !values[index].hasMember("name") || !values[index].hasMember("bit"))
        {
          ROS_WARN("[safety_io] ignoring malformed ~%s[%d]", parameter.c_str(), index);
          continue;
        }

        const std::string name = static_cast<std::string>(values[index]["name"]);
        const int bit = static_cast<int>(values[index]["bit"]);
        if (name.empty() || name.find('/') != std::string::npos || bit < 0 || bit >= available_bits)
        {
          ROS_WARN("[safety_io] ignoring invalid ~%s[%d] (name='%s', bit=%d)",
                   parameter.c_str(), index, name.c_str(), bit);
          continue;
        }

        const bool duplicate = std::any_of(destination.begin(), destination.end(),
          [&name](const Signal& signal) { return signal.name == name; });
        if (duplicate)
        {
          ROS_WARN("[safety_io] ignoring duplicate signal name '%s'", name.c_str());
          continue;
        }
        destination.push_back({name, bit});
      }
    }

    if (destination.empty()) destination = defaults;
  }

  void poll(const ros::TimerEvent&)
  {
    std::vector<uint8_t> input_values;
    std::vector<uint8_t> output_values;
    bool input_ok = false;
    bool output_ok = false;
    std::string error;

    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      input_ok = client_->readDiscreteInputs(static_cast<uint16_t>(input_discrete_base_),
                                             static_cast<uint16_t>(input_count_), input_values);
      if (input_ok)
        output_ok = client_->readDiscreteInputs(static_cast<uint16_t>(output_discrete_base_),
                                                static_cast<uint16_t>(output_count_), output_values);
      error = client_->lastError();
      if (!input_ok || !output_ok) client_->closeSocket();
    }

    const bool connected = input_ok && output_ok;
    publishConnected(connected);
    reportConnectionState(connected, error);
    if (!connected) return;

    std::ostringstream state;
    state << "inputs";
    for (const Signal& signal : inputs_)
    {
      const bool value = input_values[signal.bit] != 0;
      std_msgs::Bool message;
      message.data = value;
      input_publishers_[signal.name].publish(message);
      state << ' ' << signal.name << '=' << (value ? 1 : 0);
    }
    state << "; outputs";
    for (const Signal& signal : outputs_)
    {
      const bool value = output_values[signal.bit] != 0;
      std_msgs::Bool message;
      message.data = value;
      output_publishers_[signal.name].publish(message);
      state << ' ' << signal.name << '=' << (value ? 1 : 0);
    }
    for (const Signal& signal : config_io_outputs_)
    {
      const bool value = input_values[signal.bit] != 0;
      std_msgs::Bool message;
      message.data = value;
      output_publishers_[signal.name].publish(message);
      state << ' ' << signal.name << '=' << (value ? 1 : 0);
    }
    std_msgs::String state_message;
    state_message.data = state.str();
    state_publisher_.publish(state_message);
  }

  void publishConnected(bool connected)
  {
    std_msgs::Bool message;
    message.data = connected;
    connected_publisher_.publish(message);
  }

  void reportConnectionState(bool connected, const std::string& error)
  {
    if (!connection_state_known_ || connected != last_connected_)
    {
      if (connected)
        ROS_INFO("[safety_io] connected to %s:%d", ip_.c_str(), port_);
      else
        ROS_WARN("[safety_io] disconnected from %s:%d (%s)",
                 ip_.c_str(), port_, error.empty() ? "unknown error" : error.c_str());
      last_connected_ = connected;
      connection_state_known_ = true;
    }
    else if (!connected)
    {
      ROS_WARN_THROTTLE(5.0, "[safety_io] read failed: %s",
                        error.empty() ? "unknown error" : error.c_str());
    }
  }

  std::string ip_;
  int port_;
  uint8_t unit_id_;
  double timeout_;
  double publish_rate_;
  int input_discrete_base_;
  int output_discrete_base_;
  int input_count_;
  int output_count_;
  std::string topic_prefix_;

  std::vector<Signal> inputs_;
  std::vector<Signal> outputs_;
  std::vector<Signal> config_io_outputs_;
  std::unique_ptr<DeviceLock> device_lock_;
  std::unique_ptr<ModbusTcpClient> client_;
  std::mutex client_mutex_;

  std::map<std::string, ros::Publisher> input_publishers_;
  std::map<std::string, ros::Publisher> output_publishers_;
  ros::Publisher connected_publisher_;
  ros::Publisher state_publisher_;
  ros::Timer timer_;

  bool connection_state_known_ = false;
  bool last_connected_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_io_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  try
  {
    SafetyIONode node(node_handle, private_node_handle);
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL("[safety_io] configuration error: %s", exception.what());
    return 1;
  }
  return 0;
}
