#ifndef NECESSOTY_WIRELESS_MODULE_H
#define NECESSOTY_WIRELESS_MODULE_H

#include "SerialPort.h"
#include "necessity_serial/necessity_serial_msg.h"
//------------------------------------------------------------------------------
// Definitions

//------------------------------------------------------------------------------
// Class declaration
typedef enum {
    ERR_NO_ERROR,
    ERR_FACTORY_RESET_FAILED,
    ERR_LOW_BATTERY,
    ERR_USB_RECEIVE_BUFFER_OVERRUN,
    ERR_USB_TRANSMIT_BUFFER_OVERRUN,
    ERR_TOO_FEW_BYTES_IN_PACKET,
    ERR_TOO_MANY_BYTES_IN_PACKET,
    ERR_INVALID_CHECKSUM,
    ERR_INVALID_REGSITER_VALUE,
    ERR_INVALID_COMMAND,
    ERR_INCORRECT_AUXILIARY_PORT_MODE,
    ERR_UART_RECEIVE_BUFFER_OVERRUN,
    ERR_UART_TRANSMIT_BUFFER_OVERRUN,
} ErrorCode;

class NecessitySerial: public SerialPort {
  public:
    NecessitySerial(void);
    virtual ~NecessitySerial(void);

    void readAllData(void);
    void readAcc(void);
    void readSignal(void);

    void makeSleep(void);

    void printData(void);

    ErrorCode processNewChar(unsigned char c);
    bool isDataGetReady(void) const;
    const necessity_serial::necessity_serial_msg& getData(void);

  private:
    unsigned char buf_[256];
    unsigned char bufIndex_;
    necessity_serial::necessity_serial_msg serial_data_;
    bool dataReady_;
    float fixedToFloat(const short fixed, const unsigned char q) const;
    unsigned short concat(const unsigned char msb, const unsigned char lsb) const;
};

#endif

//------------------------------------------------------------------------------
// End of file
