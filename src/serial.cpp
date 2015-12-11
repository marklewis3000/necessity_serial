

//------------------------------------------------------------------------------
// Includes

#include "serial.h"
#include "ros/ros.h"

//------------------------------------------------------------------------------
// Definitions

//------------------------------------------------------------------------------
// Variables

// Serial stream decoding
unsigned char buf_[256];
unsigned char bufIndex_ = 0;
const uint8_t FULL_SCALE=0xFF;

const uint8_t ACC_ZERO=0x80;
const double ACC_RATIO=1.0/44.0;
//------------------------------------------------------------------------------
// Methods

NecessitySerial::NecessitySerial():
  rollover_(false),
  dataReady_(false),
  inited_(false),
  rollover_direction_(0),
  offset_(0),
  is_vibrating_(false)
{
}

NecessitySerial::~NecessitySerial() {
  ROS_INFO("Sleeping...");
  makeSleep();
}

void NecessitySerial::init(void) {
  if (!inited_) {
    readSignal();
    if (serial_data_.signal_raw>FULL_SCALE/2)
      offset_=serial_data_.signal_raw;
    inited_=true;
  }
}

void NecessitySerial::makeSleep(void) {

}

void NecessitySerial::vibrate(bool is_on) {
  if (is_on)
    write_some("1p\r");
  else
    write_some("0p\r");

  is_vibrating_ = is_on;
}

void NecessitySerial::readAllData(void){
  readPressureOnly();
  //readSignal();
}

void NecessitySerial::readAcc(void){
  write_some("dd");
  read_some();
  //serial_data_.analog=read_buf_raw_[0];
}


void NecessitySerial::readPressureOnly(void) {
  write_some("MM\r");
  int len = read_some();
  if (! read_buf_raw_[0]==0xFD)
    return;
  if (len==SIZE_RAW_PRESSURE_PACKET)
  {
    serial_data_.signal=(read_buf_raw_[1]*256+read_buf_raw_[2]);
    serial_data_.fall_count=read_buf_raw_[3]; // continuous fall_count
    serial_data_.finger_clutch = read_buf_raw_[4] > 50 ? true : false;
  }
  else if (len==SIZE_RAW_PRESSURE_PACKET-1) // no fall detection
  {
    serial_data_.signal=(read_buf_raw_[1]*256+read_buf_raw_[2]);
    serial_data_.fall_count=0; // continuous fall_count
    serial_data_.finger_clutch=-1;
  }
}

void NecessitySerial::readSignal(void){
  write_some("mm\r");
  prev_signal_=serial_data_.signal_raw;
  int len = read_some();
  if (read_buf_raw_[0] == 0xFE)
  {
    if (len == SIZE_SUMMARY_PACKET || len == SIZE_SUMMARY_PACKET-1)
    {
      serial_data_.signal_raw = read_buf_raw_[4];
      serial_data_.acc_abs = 0.0;
      for (int i = 1; i < 4; i++)
      {
        double acc = ACC_RATIO * (read_buf_raw_[i] - ACC_ZERO);
        serial_data_.acc[i - 1] = read_buf_raw_[i];
        serial_data_.acc_abs += acc * acc;
      }
      serial_data_.acc_abs = sqrt(serial_data_.acc_abs);
      if (len == SIZE_SUMMARY_PACKET)
        serial_data_.fall_count = read_buf_raw_[5]; // continuous fall_count
      else
        serial_data_.fall_count = 0; // older UID firmware
    }
  }

  if (inited_ && rollover_)
    rollOver();
  else
    serial_data_.signal=serial_data_.signal_raw;
  serial_data_.signal=std::max(0, serial_data_.signal);
}


ErrorCode NecessitySerial::processNewChar(unsigned char c) {

    // Add new byte to buffer
    buf_[bufIndex_++] = c;

    // Process receive buffer if framing char received
    if(c & 0x80) {

        bufIndex_ = 0;   //reset index

    }
    return ERR_NO_ERROR;
}

float NecessitySerial::fixedToFloat(const short fixed, const unsigned char q) const {
    return (float)fixed / (float)(1 << q);
}

unsigned short NecessitySerial::concat(const unsigned char msb, const unsigned char lsb) const {
    return ((unsigned short)msb << 8) | (unsigned short)lsb;
}

bool NecessitySerial::isDataGetReady(void) const {
  return dataReady_;
}

const pressure_serial::pressure_serial_msg&
  NecessitySerial::getData(void) {

  dataReady_ = false;
  return serial_data_;
}

void NecessitySerial::rollOver(void) {
  if (prev_signal_>0.9*FULL_SCALE && serial_data_.signal_raw<0.1*FULL_SCALE) {
    //ROS_INFO("roll over +");
    rollover_direction_++;
  }
  else if (prev_signal_<0.1*FULL_SCALE && serial_data_.signal_raw>0.9*FULL_SCALE) {
    //ROS_INFO("roll over -");
    rollover_direction_--;
  }

  serial_data_.signal=serial_data_.signal_raw+rollover_direction_*FULL_SCALE-offset_;
}

void  NecessitySerial::printData(void) {
  std::cout << "pressure: " << serial_data_.signal
       << "  acc_abs: " << serial_data_.acc_abs
       << "  acc[0]: " << (int)serial_data_.acc[0]
       << "  acc[1]: " << (int)serial_data_.acc[1]
       << "  acc[2]: " << (int)serial_data_.acc[2]
       << "  fallCnt: " << (int)serial_data_.fall_count
       << std::endl;
}

//------------------------------------------------------------------------------
// End of file
