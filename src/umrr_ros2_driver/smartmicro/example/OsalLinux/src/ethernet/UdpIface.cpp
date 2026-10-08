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
#include <ethernet/UdpIface.h>
#include <ifaddrs.h>
#include <linux/if_link.h>
#include <netdb.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <string>
#include <utility>

#include "OSALTypes.h"

namespace {

template <size_t N>
uint8_t *BufferCopy(const std::array<uint8_t, N> buffer, ssize_t received) {
  auto *copy = new uint8_t[received];  // NOLINT
  std::memcpy(copy, buffer.data(), static_cast<size_t>(received));
  return copy;
}

}  // namespace

osal::UdpIface::UdpIface(const std::string &ethIfaceName, uint16_t port,
                         const std::string &ip)
    : UnicastIface(ethIfaceName, port, ip) {}

osal::UdpIface::~UdpIface() {
  Stop();

  if (!stop_requested_) {
    stop_requested_ = true;
    callback_condition_.notify_one();
  }
  if (callback_thread_.joinable()) {
    callback_thread_.join();
  }
  ClearQueuedBuffers();
}

bool osal::UdpIface::init() {
  bool retVal = true;
  _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (0 > _socket) {
    printf("Error: Failed to create socket \n");
    retVal = false;
  }
  if (retVal) {
    // Set timeout-Option for socket with timeval = 100 msec
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100 msex = 100000 usec
    if (0 > setsockopt(_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv))) {
      printf(
          "Error in osal::UdpIface: Failed to set timeout option for socket\n");
      retVal = false;
    }
  }
  {
    // We want 4MB of buffer size.
    // Even if configuration does not allow 4MB, we will get whatever is
    // possible...
    const int32_t kBufferSize = 4 * 1024 * 1024;
    if (setsockopt(_socket, SOL_SOCKET, SO_RCVBUF, &kBufferSize,
                   sizeof(kBufferSize)) != 0) {
      printf("Failed to set receive buffer size.\n");  // NOLINT(*vararg)
    }
  }
  {
    int32_t size{-1};
    uint32_t sizelen{sizeof(size)};
    if (getsockopt(_socket, SOL_SOCKET, SO_RCVBUF, &size, &sizelen) == 0) {
      printf("Using %d bytes for receive buffer.\n", size);  // NOLINT(*vararg)
    } else {
      printf("Failed to retrieve receive buffer size.\n");  // NOLINT(*vararg)
    }
  }
  if (retVal) {
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(GetRxPort());
    inet_pton(AF_INET, GetRxIpAddr().c_str(), &addr.sin_addr.s_addr);
    if (0 > bind(_socket, (struct sockaddr *)&addr, sizeof(addr))) {
      printf("Failed to bind socket");
      retVal = false;
    }
  }

  callback_thread_ = std::thread(&UdpIface::ProcessBufferQueue, this);
#ifdef __linux
  pthread_setname_np(callback_thread_.native_handle(), "UDP-callbacks");
#endif

  return retVal;
}

void osal::UdpIface::receiver() {
#ifdef __linux
  pthread_setname_np(pthread_self(), "UDP-receiver");
#endif

  sockaddr_in source_ip{};
  socklen_t srcLen = sizeof(source_ip);
  auto *source_ip_pointer =
      reinterpret_cast<sockaddr *>(&source_ip);  // NOLINT(*reinterpret*)

  constexpr size_t kBufferSize = 65535u;
  std::array<uint8_t, kBufferSize> buffer = {};

  while (isRun()) {
    const auto received_bytes = recvfrom(_socket, buffer.data(), kBufferSize, 0,
                                         source_ip_pointer, &srcLen);
    if (received_bytes > 0) {
      CallThreaded(QueuedBuffer{source_ip, BufferCopy(buffer, received_bytes),
                                static_cast<std::size_t>(received_bytes)});
    }
  }

  setIsStopped();
}

bool osal::UdpIface::createSocket(IN const std::string &ip, IN uint16_t port,
                                  OUT int &result) {
  bool retVal = true;
  int soc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (0 > soc) {
    retVal = false;
  } else {
    result = soc;
  }
  return retVal;
}

bool osal::UdpIface::createSocketAddr(IN const std::string &ip,
                                      IN uint16_t port, OUT sockaddr_in &addr) {
  bool retVal = false;
  if (1 == inet_pton(AF_INET, ip.c_str(), &addr.sin_addr.s_addr)) {
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    retVal = true;
  }
  return retVal;
}

bool osal::UdpIface::send(IN int socket, IN const std::string &ip,
                          IN uint16_t port,
                          IN osal::types::BufferDescriptor &desc) {
  bool retVal = true;
  sockaddr_in addr;
  if (createSocketAddr(ip, port, addr)) {
    auto sent = sendto(socket, desc.GetBufferPtr(), desc.GetSize(), 0,
                       (struct sockaddr *)&addr, sizeof(addr));
    if (sent != desc.GetSize()) {
      printf("Error: Only [%zu] bytes were sent \n", sent);
      retVal = false;
    }

  } else {
    printf("Error: Failed to cretae socketaddr \n");
    retVal = false;
  }
  return retVal;
}

void osal::UdpIface::CallThreaded(osal::UdpIface::QueuedBuffer &&buffer) {
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    queued_buffers_.push(buffer);
  }
  callback_condition_.notify_one();
}

void osal::UdpIface::ClearQueuedBuffers() {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  while (!queued_buffers_.empty()) {
    auto &entry = queued_buffers_.front();
    delete[] entry.buffer;  // NOLINT
    queued_buffers_.pop();
  }
}

void osal::UdpIface::ProcessBufferQueue() {
  while (true) {
    QueuedBuffer queued_entry;
    {
      std::unique_lock<std::mutex> lock(buffer_mutex_);
      callback_condition_.wait(
          lock, [&]() { return stop_requested_ || !queued_buffers_.empty(); });
      if (stop_requested_) {
        return;
      }

      queued_entry = queued_buffers_.front();
      queued_buffers_.pop();
    }

    const auto source_ip = [&queued_entry]() {
      static constexpr size_t kAddressLength = 16;
      std::array<char, kAddressLength> tmp = {};
      inet_ntop(AF_INET, &queued_entry.source_ip.sin_addr, tmp.data(),
                sizeof(tmp));
      return std::string(tmp.data());
    }();
    ReceiverCallback callback;
    if (!getCallback(source_ip, callback)) {
      delete[] queued_entry.buffer;  // NOLINT(*owning-memory)
      continue;
    }

    osal::types::BufferDescriptor desc(queued_entry.buffer,
                                       queued_entry.buffer_size);
    callback(desc);
    delete[] queued_entry.buffer;  // NOLINT(*owning-memory)
  }
}
