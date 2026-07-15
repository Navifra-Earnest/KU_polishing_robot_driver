// crevis_io_node.cpp — Crevis GN-9289 (MODBUS TCP) I/O 드라이버 (ROS1 Noetic, C++)
//
// LED 6개와 충전 릴레이를 개별 토픽으로 ON/OFF 하고, Crevis 입력/출력 상태를 폴링하여 퍼블리시한다.
// Modbus TCP 는 외부 라이브러리(libmodbus 등) 없이 순수 소켓으로 구현 → 납품 시 추가 의존성 없음.
//
// [토픽]
//   구독(쓰기) — LED 개별 제어  std_msgs/Bool  (True=ON, False=OFF)
//     /crevis/led/vision        Y01.00 VISION LED
//     /crevis/led/front         Y01.01 FRONT LED
//     /crevis/led/side          Y01.02 SIDE LED
//     /crevis/led/status_red    Y01.03 STATUS LED RED
//     /crevis/led/status_green  Y01.04 STATUS LED GREEN
//     /crevis/led/status_blue   Y01.05 STATUS LED BLUE
//     /crevis/charging          Y01.06 CHARGING RELAY
//   퍼블리시(읽기)
//     /crevis/led_state/<name>  std_msgs/Bool             출력 코일 readback (실제 상태, 개별)
//     /crevis/led_state_all     std_msgs/String           LED 6개 통합 상태 한 줄 (name=0/1 ... mask=0x..)
//     /crevis/charge_port_on    std_msgs/Bool             충전 릴레이 출력 코일 readback
//     /crevis/di                std_msgs/Int32MultiArray  입력 레지스터 raw (16bit 워드)
//     /crevis/connected         std_msgs/Bool             Modbus 연결 상태
//
// [레지스터 맵 근거] GN-9289 User Manual REV1.06 p.28 "MODBUS Interface Register/Bit Map"
//   - 출력 image BIT (coil)  : 0x1000~  (func 1/5/15)  ← LED 개별 제어에 사용
//   - 출력 image REGISTER    : 0x0800~  (func 3/16)    ← 16bit 묶음(register 모드, 토비카식)
//   - 입력 image REGISTER    : 0x0000~  (func 3/4)
//   IO 맵(출력 IO.png / GT-225F OUT01 SLOT#01): Y01.00~Y01.05 = 위 LED 6개,
//   Y01.06 = 충전 릴레이.
//   → 첫 출력슬롯이면 coil 0x1000 = Y01.00. 앞에 다른 출력슬롯이 있으면
//     output_coil_base 를 16(=한 슬롯 16DO)씩 가산해서 맞춘다.

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <cstdint>

// ------------------------------------------------------------------
// 순수 소켓 Modbus TCP 클라이언트 (스레드-안전 아님 → 호출측에서 lock)
// ------------------------------------------------------------------
class ModbusTcp
{
public:
  ModbusTcp(const std::string& host, int port, uint8_t unit_id, double timeout_s)
    : host_(host), port_(port), unit_(unit_id), fd_(-1), tid_(0)
  {
    tv_.tv_sec  = static_cast<long>(timeout_s);
    tv_.tv_usec = static_cast<long>((timeout_s - tv_.tv_sec) * 1e6);
  }
  ~ModbusTcp() { closeSock(); }

  bool isOpen() const { return fd_ >= 0; }

  bool openSock()
  {
    closeSock();
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;

    int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);   // 논블로킹 connect (timeout 적용용)

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port_));
    if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) { ::close(fd); return false; }

    int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) { ::close(fd); return false; }
    if (rc < 0)
    {
      fd_set wset; FD_ZERO(&wset); FD_SET(fd, &wset);
      timeval tv = tv_;
      rc = ::select(fd + 1, nullptr, &wset, nullptr, &tv);
      if (rc <= 0) { ::close(fd); return false; }
      int err = 0; socklen_t len = sizeof(err);
      ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
      if (err != 0) { ::close(fd); return false; }
    }
    ::fcntl(fd, F_SETFL, flags);                // 다시 블로킹
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv_, sizeof(tv_));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv_, sizeof(tv_));
    int one = 1; ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    fd_ = fd;
    return true;
  }

  void closeSock() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

  // 코일 상태를 0/1 바이트로 out 에 채움. 실패(소켓/예외) 시 false.
  bool readCoils(uint16_t addr, uint16_t qty, std::vector<uint8_t>& out)
  {
    uint8_t pdu[5] = { 0x01, uint8_t(addr >> 8), uint8_t(addr & 0xFF),
                       uint8_t(qty >> 8),  uint8_t(qty & 0xFF) };
    std::vector<uint8_t> r;
    if (!transact(pdu, 5, r)) return false;
    if (r.size() < 2 || r[0] != 0x01) return false;
    uint8_t bc = r[1];
    if (r.size() < static_cast<size_t>(2 + bc)) return false;
    out.assign(qty, 0);
    for (uint16_t i = 0; i < qty; ++i) out[i] = (r[2 + i / 8] >> (i % 8)) & 0x01;
    return true;
  }

  bool readInputRegisters(uint16_t addr, uint16_t qty, std::vector<uint16_t>& out)
  {
    uint8_t pdu[5] = { 0x04, uint8_t(addr >> 8), uint8_t(addr & 0xFF),
                       uint8_t(qty >> 8),  uint8_t(qty & 0xFF) };
    std::vector<uint8_t> r;
    if (!transact(pdu, 5, r)) return false;
    if (r.size() < 2 || r[0] != 0x04) return false;
    uint8_t bc = r[1];
    if (r.size() < static_cast<size_t>(2 + bc)) return false;
    out.clear();
    for (int i = 0; i < bc / 2; ++i)
      out.push_back(static_cast<uint16_t>((r[2 + i * 2] << 8) | r[2 + i * 2 + 1]));
    return true;
  }

  bool writeSingleCoil(uint16_t addr, bool on)
  {
    uint8_t pdu[5] = { 0x05, uint8_t(addr >> 8), uint8_t(addr & 0xFF),
                       uint8_t(on ? 0xFF : 0x00), 0x00 };
    std::vector<uint8_t> r;
    if (!transact(pdu, 5, r)) return false;
    return !r.empty() && r[0] == 0x05;
  }

  bool writeSingleRegister(uint16_t addr, uint16_t val)
  {
    uint8_t pdu[5] = { 0x06, uint8_t(addr >> 8), uint8_t(addr & 0xFF),
                       uint8_t(val >> 8), uint8_t(val & 0xFF) };
    std::vector<uint8_t> r;
    if (!transact(pdu, 5, r)) return false;
    return !r.empty() && r[0] == 0x06;
  }

private:
  // MBAP 붙여 요청 전송 후 응답 PDU(function code 이후) 를 resp 에. 소켓오류면 소켓닫음+false,
  // Modbus 예외응답이면 소켓유지+false.
  bool transact(const uint8_t* pdu, uint16_t pdu_len, std::vector<uint8_t>& resp)
  {
    if (fd_ < 0 && !openSock()) return false;

    uint16_t tid = ++tid_;
    uint16_t len = pdu_len + 1;                 // unit + PDU
    uint8_t req[260];
    req[0] = tid >> 8; req[1] = tid & 0xFF;
    req[2] = 0;        req[3] = 0;
    req[4] = len >> 8; req[5] = len & 0xFF;
    req[6] = unit_;
    std::memcpy(req + 7, pdu, pdu_len);
    if (!sendAll(req, 7 + pdu_len)) { closeSock(); return false; }

    uint8_t hdr[7];
    if (!recvAll(hdr, 7)) { closeSock(); return false; }
    uint16_t rlen = (hdr[4] << 8) | hdr[5];      // unit + PDU
    if (rlen < 2 || rlen > 253) { closeSock(); return false; }
    int pdu_bytes = rlen - 1;
    std::vector<uint8_t> body(pdu_bytes);
    if (!recvAll(body.data(), pdu_bytes)) { closeSock(); return false; }

    if (!body.empty() && (body[0] & 0x80)) return false;  // Modbus 예외 → 소켓 유지
    resp.swap(body);
    return true;
  }

  bool sendAll(const uint8_t* b, int n)
  { int s = 0; while (s < n) { int w = ::send(fd_, b + s, n - s, MSG_NOSIGNAL); if (w <= 0) return false; s += w; } return true; }

  bool recvAll(uint8_t* b, int n)
  { int r = 0; while (r < n) { int q = ::recv(fd_, b + r, n - r, 0); if (q <= 0) return false; r += q; } return true; }

  std::string host_;
  int         port_;
  uint8_t     unit_;
  int         fd_;
  uint16_t    tid_;
  timeval     tv_;
};

// ------------------------------------------------------------------
// ROS 노드
// ------------------------------------------------------------------
struct Led { std::string name; int bit; };

class CrevisIONode
{
public:
  CrevisIONode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("ip", ip_, "192.168.100.104");
    pnh.param("port",    port_, 502);
    int uid; pnh.param("unit_id", uid, 1); unit_ = static_cast<uint8_t>(uid);
    pnh.param("timeout", timeout_, 1.0);
    pnh.param("publish_rate", rate_hz_, 10.0);
    pnh.param<std::string>("write_mode", write_mode_, std::string("coil"));
    pnh.param("output_coil_base",     coil_base_,   0x1000);
    pnh.param("output_register_base", reg_base_,    0x0800);
    pnh.param("input_register_base",  in_reg_base_, 0x0000);
    pnh.param("input_register_count", in_reg_count_, 1);
    pnh.param("leds_off_on_shutdown", leds_off_on_shutdown_, false);
    pnh.param<std::string>("charging_topic", charging_topic_, std::string("/crevis/charging"));
    pnh.param("charging_bit", charging_bit_, 6);
    pnh.param("charging_off_on_shutdown", charging_off_on_shutdown_, true);

    loadLeds(pnh);
    readback_nb_ = 1;
    for (const auto& l : leds_) readback_nb_ = std::max(readback_nb_, l.bit + 1);
    readback_nb_ = std::max(readback_nb_, charging_bit_ + 1);
    if (write_mode_ == "register") readback_nb_ = std::max(readback_nb_, 16);

    client_.reset(new ModbusTcp(ip_, port_, unit_, timeout_));
    out_shadow_ = 0;
    out_shadow_valid_ = false;

    // 퍼블리셔
    for (const auto& l : leds_)
      pub_state_[l.name] = nh.advertise<std_msgs::Bool>("/crevis/led_state/" + l.name, 1, true /*latch*/);
    pub_state_all_ = nh.advertise<std_msgs::String>("/crevis/led_state_all", 1, true /*latch*/);
    pub_charge_port_on_ = nh.advertise<std_msgs::Bool>("/crevis/charge_port_on", 1, true /*latch*/);
    pub_di_        = nh.advertise<std_msgs::Int32MultiArray>("/crevis/di", 1);
    pub_conn_      = nh.advertise<std_msgs::Bool>("/crevis/connected", 1, true /*latch*/);

    // 구독 (LED 개별 제어)
    for (const auto& l : leds_)
    {
      std::string name = l.name; int bit = l.bit;
      subs_.push_back(nh.subscribe<std_msgs::Bool>(
        "/crevis/led/" + name, 10,
        boost::bind(&CrevisIONode::ledCb, this, name, bit, _1)));
    }
    charging_sub_ = nh.subscribe<std_msgs::Bool>(charging_topic_, 10,
                                                  &CrevisIONode::chargingCb, this);

    std::string leds_desc;
    for (size_t i = 0; i < leds_.size(); ++i)
      leds_desc += (i ? ", " : "") + leds_[i].name + "@" + std::to_string(leds_[i].bit);
    ROS_INFO("[crevis_io] %s:%d unit=%d mode=%s coil_base=0x%X leds=[%s] charging=%s@%d",
             ip_.c_str(), port_, unit_, write_mode_.c_str(), coil_base_, leds_desc.c_str(),
             charging_topic_.c_str(), charging_bit_);

    timer_ = nh.createTimer(ros::Duration(1.0 / std::max(rate_hz_, 0.1)),
                            &CrevisIONode::poll, this);
  }

  ~CrevisIONode() { onShutdown(); }

private:
  void loadLeds(ros::NodeHandle& pnh)
  {
    XmlRpc::XmlRpcValue arr;
    if (pnh.getParam("leds", arr) && arr.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < arr.size(); ++i)
      {
        if (arr[i].hasMember("name") && arr[i].hasMember("bit"))
          leds_.push_back({ static_cast<std::string>(arr[i]["name"]),
                            static_cast<int>(arr[i]["bit"]) });
      }
    }
    if (leds_.empty())   // 파라미터 없을 때 기본 매핑
      leds_ = { {"vision",0},{"front",1},{"side",2},
                {"status_red",3},{"status_green",4},{"status_blue",5} };
  }

  void ledCb(const std::string& name, int bit, const std_msgs::Bool::ConstPtr& msg)
  {
    setOutput("LED " + name, bit, msg->data);
  }

  void chargingCb(const std_msgs::Bool::ConstPtr& msg)
  {
    setOutput("charging relay", charging_bit_, msg->data);
  }

  bool syncOutputShadowLocked()
  {
    std::vector<uint8_t> coils;
    if (!client_->readCoils(static_cast<uint16_t>(coil_base_), 16, coils)) return false;

    out_shadow_ = 0;
    for (size_t bit = 0; bit < coils.size() && bit < 16; ++bit)
      if (coils[bit]) out_shadow_ |= (1 << bit);
    out_shadow_valid_ = true;
    return true;
  }

  bool writeOutputLocked(int bit, bool on)
  {
    if (bit < 0 || bit >= 16) return false;
    if (write_mode_ == "register")
    {
      if (!out_shadow_valid_ && !syncOutputShadowLocked()) return false;
      if (on) out_shadow_ |=  (1 << bit);
      else    out_shadow_ &= ~(1 << bit);
      const bool ok = client_->writeSingleRegister(static_cast<uint16_t>(reg_base_),
                                                    static_cast<uint16_t>(out_shadow_ & 0xFFFF));
      if (!ok) out_shadow_valid_ = false;
      return ok;
    }
    return client_->writeSingleCoil(static_cast<uint16_t>(coil_base_ + bit), on);
  }

  void setOutput(const std::string& name, int bit, bool on)
  {
    bool ok = false;
    {
      std::lock_guard<std::mutex> lk(lock_);
      ok = writeOutputLocked(bit, on);
    }
    if (ok) ROS_INFO("[crevis_io] %s -> %s", name.c_str(), on ? "ON" : "OFF");
    else    ROS_WARN("[crevis_io] %s write FAILED (연결/주소 확인)", name.c_str());
  }

  void poll(const ros::TimerEvent&)
  {
    std::vector<uint8_t>  coils;
    std::vector<uint16_t> regs;
    bool have_coils = false, have_regs = false, connected = false;
    {
      std::lock_guard<std::mutex> lk(lock_);
      if (!client_->isOpen()) client_->openSock();
      have_coils = client_->readCoils(static_cast<uint16_t>(coil_base_),
                                      static_cast<uint16_t>(readback_nb_), coils);
      if (have_coils && write_mode_ == "register")
      {
        out_shadow_ = 0;
        for (size_t bit = 0; bit < coils.size() && bit < 16; ++bit)
          if (coils[bit]) out_shadow_ |= (1 << bit);
        out_shadow_valid_ = true;
      }
      if (in_reg_count_ > 0)
        have_regs = client_->readInputRegisters(static_cast<uint16_t>(in_reg_base_),
                                                static_cast<uint16_t>(in_reg_count_), regs);
      connected = client_->isOpen();
    }

    std_msgs::Bool cmsg; cmsg.data = connected; pub_conn_.publish(cmsg);

    if (have_coils)
    {
      int mask = 0;
      std::string s;
      for (size_t i = 0; i < leds_.size(); ++i)
      {
        const Led& l = leds_[i];
        bool on = (l.bit < static_cast<int>(coils.size())) && coils[l.bit];
        std_msgs::Bool b; b.data = on; pub_state_[l.name].publish(b);
        if (on) mask |= (1 << l.bit);
        char buf[64]; std::snprintf(buf, sizeof(buf), "%s%s=%d", i ? " " : "", l.name.c_str(), on ? 1 : 0);
        s += buf;
      }
      char m[24]; std::snprintf(m, sizeof(m), " mask=0x%02X", mask);
      std_msgs::String all; all.data = s + m; pub_state_all_.publish(all);

      std_msgs::Bool charging;
      charging.data = charging_bit_ < static_cast<int>(coils.size()) && coils[charging_bit_];
      pub_charge_port_on_.publish(charging);
    }
    if (have_regs)
    {
      std_msgs::Int32MultiArray di;
      for (uint16_t v : regs) di.data.push_back(static_cast<int>(v));
      pub_di_.publish(di);
    }
  }

  void onShutdown()
  {
    std::lock_guard<std::mutex> lk(lock_);
    if (write_mode_ == "register" && (leds_off_on_shutdown_ || charging_off_on_shutdown_))
    {
      if (out_shadow_valid_ || syncOutputShadowLocked())
      {
        if (leds_off_on_shutdown_)
          for (const auto& l : leds_) out_shadow_ &= ~(1 << l.bit);
        if (charging_off_on_shutdown_) out_shadow_ &= ~(1 << charging_bit_);
        client_->writeSingleRegister(static_cast<uint16_t>(reg_base_),
                                     static_cast<uint16_t>(out_shadow_ & 0xFFFF));
      }
    }
    else if (write_mode_ != "register")
    {
      if (leds_off_on_shutdown_)
        for (const auto& l : leds_)
          client_->writeSingleCoil(static_cast<uint16_t>(coil_base_ + l.bit), false);
      if (charging_off_on_shutdown_)
        client_->writeSingleCoil(static_cast<uint16_t>(coil_base_ + charging_bit_), false);
    }
    client_->closeSock();
  }

  // params
  std::string ip_, write_mode_, charging_topic_;
  int  port_, coil_base_, reg_base_, in_reg_base_, in_reg_count_, charging_bit_;
  uint8_t unit_;
  double timeout_, rate_hz_;
  bool leds_off_on_shutdown_, charging_off_on_shutdown_;
  std::vector<Led> leds_;
  int readback_nb_;
  int out_shadow_;
  bool out_shadow_valid_;

  std::unique_ptr<ModbusTcp> client_;
  std::mutex lock_;

  std::map<std::string, ros::Publisher> pub_state_;
  ros::Publisher pub_state_all_, pub_charge_port_on_, pub_di_, pub_conn_;
  ros::Subscriber charging_sub_;
  std::vector<ros::Subscriber> subs_;
  ros::Timer timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crevis_io_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  CrevisIONode node(nh, pnh);
  ros::spin();
  return 0;
}
