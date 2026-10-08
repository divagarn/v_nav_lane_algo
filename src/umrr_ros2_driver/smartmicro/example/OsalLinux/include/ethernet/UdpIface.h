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

#ifndef ETHERNET__UDPIFACE_H_
#define ETHERNET__UDPIFACE_H_

#include <ethernet/UnicastIface.h>
#include <netinet/ip.h>

#include <condition_variable>
#include <queue>
#include <string>
#include <thread>

#include "OSALIface.h"
#include "OSALTypes.h"

namespace osal {
class UdpIface : public UnicastIface {
 public:
  UdpIface(const std::string &ethIfaceName, uint16_t port,
           const std::string &ip);
  virtual ~UdpIface();

 protected:
  virtual bool init();

  virtual void receiver();
  virtual bool createSocket(IN const std::string &ip, IN uint16_t port,
                            OUT int &result);
  virtual bool send(IN int socket, IN const std::string &ip, IN uint16_t port,
                    IN osal::types::BufferDescriptor &desc);

 private:
  bool createSocketAddr(IN const std::string &ip, IN uint16_t port,
                        OUT sockaddr_in &addr);

  struct QueuedBuffer {
    sockaddr_in source_ip{};
    uint8_t *buffer{nullptr};
    size_t buffer_size{};
  };
  std::queue<QueuedBuffer> queued_buffers_;
  std::mutex buffer_mutex_;  /// Guarding `queued_buffers_`

  std::thread callback_thread_;
  std::atomic<bool> stop_requested_{false};
  std::condition_variable callback_condition_;

  void CallThreaded(QueuedBuffer &&buffer);
  void ClearQueuedBuffers();
  void ProcessBufferQueue();
};

}  // namespace osal

#endif  // ETHERNET__UDPIFACE_H_
