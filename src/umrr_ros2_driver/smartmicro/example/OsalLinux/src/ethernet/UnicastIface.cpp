/************************************************************************
 * Copyright
 *
 * s.m.s. smart microwave sensors GmbH
 * In den Waashainen 1
 * 38108 Braunschweig - Germany
 *
 * Register Court: Amtsgericht Braunschweig / Register Number: HRB 4525
 * VAT ID: DE 193324798 / Tax Number: 13/207/02357
 ************************************************************************/

#include <arpa/inet.h>
#include <ethernet/UnicastIface.h>
#include <ifaddrs.h>
#include <linux/if_link.h>
#include <netdb.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>

osal::UnicastIface::UnicastIface(const std::string &ethIfaceName, uint16_t port,
                                 const std::string &ip)
    : EthIface(ethIfaceName, ip, port), _ethIfaceName(ethIfaceName) {}

osal::UnicastIface::~UnicastIface() {
  Stop();
  close(_socket);
}

bool osal::UnicastIface::Init() {
  if (!_ip.empty()) {
    return true;
  }
  bool retVal = false;
  struct ifaddrs *ifAddrs;
  // Get all linux ethernet interfaces
  if (-1 == getifaddrs(&ifAddrs)) {
    printf("Error osal::UnicastIface: Failed to retreive ip address \n");
    return retVal;
  } else {
    // Search for required ethernet interface
    for (auto iface = ifAddrs; iface != nullptr; iface = iface->ifa_next) {
      if (iface->ifa_addr == nullptr) {
        continue;
      }
      char host[NI_MAXHOST];
      auto family = iface->ifa_addr->sa_family;
      if (family == AF_INET) {
        if (0 != getnameinfo(iface->ifa_addr, sizeof(struct sockaddr_in), host,
                             NI_MAXHOST, nullptr, 0, NI_NUMERICHOST)) {
          continue;
        }
      } else {
        continue;
      }
      // Compare found ethernet interface from list with configured EthIface
      // from registry
      if (_ethIfaceName == iface->ifa_name) {
        // Set the rx ip address
        SetRxIpAddr(host);
        retVal = init();
        break;
      }
    }
  }
  freeifaddrs(ifAddrs);
  return retVal;
}

bool osal::UnicastIface::RegisterReceiver(IN const std::string &ip,
                                          IN ReceiverCallback callback) {
  bool retVal = false;
  std::lock_guard<std::mutex> lock(_callback_mutex);
  auto it = _callbacks.find(ip);
  if (it == _callbacks.end()) {
    _callbacks.insert(std::pair<std::string, ReceiverCallback>(ip, callback));
    retVal = true;
  }
  return retVal;
}

bool osal::UnicastIface::isRun() { return _isRun.load(); }

bool osal::UnicastIface::UnRegisterReceiver(IN const std::string &ip) {
  bool retVal = false;
  std::lock_guard<std::mutex> lock(_callback_mutex);
  auto it = _callbacks.find(ip);
  if (it != _callbacks.end()) {
    _callbacks.erase(it);
    retVal = true;
  }
  return retVal;
}

bool osal::UnicastIface::Start() {
  _isRun.store(true);
  _isStopped.store(false);
  _receiver_thread = std::thread(&UnicastIface::receiver, this);
  return true;
}

void osal::UnicastIface::Stop() {
  _isRun.store(false);
  if (_receiver_thread.joinable()) {
    _receiver_thread.join();
  }
  setIsStopped();
}

void osal::UnicastIface::setIsStopped() { _isStopped.store(true); }

bool osal::UnicastIface::getCallback(IN const std::string &ip,
                                     OUT ReceiverCallback &callback) {
  bool retVal = false;
  std::lock_guard<std::mutex> lock(_callback_mutex);
  auto it = _callbacks.find(ip);
  if (it != _callbacks.end()) {
    callback = it->second;
    retVal = true;
  }
  return retVal;
}

bool osal::UnicastIface::SendData(IN const std::string &ip, IN uint16_t port,
                                  IN osal::types::BufferDescriptor desc) {
  bool retVal = true;
  sockaddr_in addr;

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr(ip.c_str());
  addr.sin_port = htons(port);

  auto sent = sendto(_socket, desc.GetBufferPtr(), desc.GetSize(), 0,
                     (struct sockaddr *)&addr, sizeof(addr));
  if (sent != desc.GetSize()) {
    printf("Only [%zu] bytes were sent", sent);
    printf("Error sending: %i\n", errno);
    retVal = false;
  }
  return retVal;
}
