#include "SerialPort.h"

static FILE* fp;
SerialPort::SerialPort(void)
  : end_of_line_char_('\n')
{
}

SerialPort::~SerialPort(void)
{
  //fclose(fp);
  stop();
}

char SerialPort::end_of_line_char() const
{
  return this->end_of_line_char_;
}

void SerialPort::end_of_line_char(const char &c)
{
  this->end_of_line_char_ = c;
}

bool SerialPort::start(const char *com_port_name, int baud_rate)
{
  boost::system::error_code ec;

  if (port_) {
    std::cout << "error : port is already opened..." << std::endl;
    return false;
  }

  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(com_port_name, ec);
  if (ec) {
    std::cout << "error : port_->open() failed...com_port_name="
      << com_port_name << ", e=" << ec.message().c_str() << std::endl;
    return false;
  }

  // option settings...
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  //boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));

  //async_read_some_();

  //boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));

  return true;
}

void SerialPort::stop()
{
  boost::mutex::scoped_lock look(mutex_);

  std::cout << "Stopping the port" << std::endl;
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
}

int SerialPort::write_some(const std::string &buf)
{
  //std::cout << "writing: " << buf << std::endl;
  return write_some(buf.c_str(), (int)buf.size());
}

int SerialPort::write_some(const char *buf, const int &size)
{
  boost::system::error_code ec;

  if (!port_) return -1;
  if (size == 0) return 0;

  return (int)(port_->write_some(boost::asio::buffer(buf, size), ec));
}

void SerialPort::async_read_some_()
{
  if (port_.get() == NULL || !port_->is_open()) return;


  port_->async_read_some(
    boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
    boost::bind(
    &SerialPort::on_receive_,
    this, boost::asio::placeholders::error,
    boost::asio::placeholders::bytes_transferred));


	//timer.expires_from_now(boost::posix_time::millisec(100));
	//timer.async_wait(boost::bind(&SerialPort::time_out,
         //   this, boost::asio::placeholders::error));

}

size_t SerialPort::read_some()
{
  boost::system::error_code ec;
  size_t bytes_transferred=port_->read_some(
    boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE));

  on_receive_sync(ec, bytes_transferred);


  return bytes_transferred;
}


void SerialPort::on_receive_sync(const boost::system::error_code& ec, const size_t bytes_transferred)
{
  if (port_.get() == NULL || !port_->is_open()) return;
  if (ec) {
    //read_some_();
    return;
  }

  for (unsigned int i = 0; i < bytes_transferred; ++i) {
    unsigned char c = static_cast<unsigned char>(read_buf_raw_[i]);
    //wireless_.processNewChar(c);
    read_buf_str_ += c;
  }

  this->on_receive_(read_buf_str_);
  read_buf_str_.clear();

}


void SerialPort::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
{
  boost::mutex::scoped_lock look(mutex_);

  if (port_.get() == NULL || !port_->is_open()) return;
  if (ec) {
    async_read_some_();
    return;
  }

  for (unsigned int i = 0; i < bytes_transferred; ++i) {
    unsigned char c = static_cast<unsigned char>(read_buf_raw_[i]);
    //wireless_.processNewChar(c);
    if (c & end_of_line_char_) {
      this->on_receive_(read_buf_str_);
      read_buf_str_.clear();
    }
    else {
      read_buf_str_ += c;
    }
  }

  async_read_some_();
}


void SerialPort::on_receive_(const std::string &data)
{
  //std::cout << "SerialPort::on_receive_() : " << data << std::endl;
}
