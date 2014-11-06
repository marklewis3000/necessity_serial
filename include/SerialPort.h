#pragma once

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>
#include <vector>
#include <queue>

//#include "Wireless.h"

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

#define SERIAL_PORT_READ_BUF_SIZE 256
#define SAMPLING_RATE_ACC 256.0

class SerialPort
{
protected:
  boost::asio::io_service io_service_;
  serial_port_ptr port_;
  boost::mutex mutex_;

  unsigned char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
  std::string read_buf_str_;

  char end_of_line_char_;

private:
  SerialPort(const SerialPort &p);
  SerialPort &operator=(const SerialPort &p);
  //NecessityBluetooth wireless_;

public:
  SerialPort(void);
  virtual ~SerialPort(void);

  char end_of_line_char() const;
  void end_of_line_char(const char &c);

  virtual bool start(const char *com_port_name, int baud_rate=9600);
  virtual void stop();

  int write_some(const std::string &buf);
  int write_some(const char *buf, const int &size);

  size_t read_some();

  static int get_port_number();
  static std::vector<std::string> get_port_names();
  static void print_devices();

protected:
  virtual void async_read_some_();
  virtual void on_receive_sync(const boost::system::error_code& ec, const size_t bytes_transferred);
  virtual void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);
  virtual void on_receive_(const std::string &data);
  void time_out(const boost::system::error_code& error);
};
