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

#ifndef ETHERNET__UNICASTIFACE_H_
#define ETHERNET__UNICASTIFACE_H_

#include <OSALIface.h>
#include <OSALTypes.h>
#include <ethernet/EthIface.h>
#include <termios.h>

#include <atomic>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace osal {
class UnicastIface : public EthIface {
 public:
  UnicastIface(const std::string &ethIfaceName, uint16_t port,
               const std::string &ip);
  virtual ~UnicastIface();

  virtual bool Init();

  virtual bool Start();

  virtual void Stop();

  virtual bool RegisterReceiver(IN const std::string &, IN ReceiverCallback);

  virtual bool UnRegisterReceiver(IN const std::string &);

  virtual bool SendData(IN const std::string &ip, IN uint16_t port,
                        IN osal::types::BufferDescriptor desc);

 protected:
  virtual bool init() = 0;

  virtual void receiver() = 0;

  virtual bool createSocket(IN const std::string &ip, IN uint16_t port,
                            OUT int &result) = 0;

  virtual bool send(IN int socket, IN const std::string &ip, IN uint16_t port,
                    IN osal::types::BufferDescriptor &desc) = 0;

  bool getCallback(IN const std::string &ip, OUT ReceiverCallback &);

  bool isRun();

  void setIsStopped();

  int _socket{-1};

 private:
  using CallbackMap = std::map<std::string, ReceiverCallback>;

  std::mutex _callback_mutex;  // guarding _callbacks
  std::thread _receiver_thread;
  std::string _ethIfaceName;
  CallbackMap _callbacks;
  std::atomic_bool _isRun{false};
  std::atomic_bool _isStopped{false};
};

}  // namespace osal

#endif  // ETHERNET__UNICASTIFACE_H_
