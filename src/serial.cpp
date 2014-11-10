

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

//------------------------------------------------------------------------------
// Methods

NecessitySerial::NecessitySerial() {
  // Data ready flags
  dataReady_ = false;
}

NecessitySerial::~NecessitySerial() {
  ROS_INFO("Sleeping...");
  makeSleep();
}

void NecessitySerial::makeSleep(void) {

}

void NecessitySerial::readAllData(void){
  //readAcc();
  readSignal();
}

void NecessitySerial::readAcc(void){
  write_some("dd");
  read_some();
  //serial_data_.analog=read_buf_raw_[0];
}

void NecessitySerial::readSignal(void){
  write_some("mm\r");
  if (read_some()==PACKET_SIZE && read_buf_raw_[0]==0xFE) {
    serial_data_.signal = read_buf_raw_[4];
    for (int i=1; i<4; i++)
      serial_data_.acc[i-1]= read_buf_raw_[i];
  }
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

const necessity_serial::necessity_serial_msg&
  NecessitySerial::getData(void) {

  dataReady_ = false;
  return serial_data_;
}

void  NecessitySerial::printData(void) {
  std::cout << "pressure: " << (unsigned int)serial_data_.signal
       << "  acc[0]: " << (int)serial_data_.acc[0]
       << "  acc[1]: " << (int)serial_data_.acc[1]
       << "  acc[2]: " << (int)serial_data_.acc[2] << std::endl;
}

//------------------------------------------------------------------------------
// End of file
