#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <string>
#include <sstream>
#include <vector>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoMsg
{
  public:
    uint32_t hdr;
    uint32_t id;
    uint32_t millis;
    double batt_voltage;
    double charger_voltage;
    double charger_current;
    double wheel_l_position;
    double wheel_r_position;
    uint32_t chksum;
};

class ArduinoComms
{

public:

  const uint32_t ARDUINTERFACE_HDR = 0xF0AAAA0F;

  static const int ARDUINTERFACE_CMD_LEN = 8;
  typedef uint32_t ArdumowerCmd[ARDUINTERFACE_CMD_LEN];

  static const int ARDUINTERFACE_MSG_LEN = 16;
  typedef uint32_t ArdumowerMsg[ARDUINTERFACE_MSG_LEN];

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case

    // --- SRS ---
    serial_conn_.Write(msg_to_send);
    serial_conn_.Write("AT+R\r\n");

    std::string response = "";

    // --- SRS ---
    //return response;


    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r\n");
  }

  int read_encoder_values(ArduinoMsg &msg)
  {
    // --- SRS ---
    //val_1 = val_1 + 1;
    //val_2 = val_2 + 1;
    //return;

    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write("XS\r");

    unsigned char buf[ARDUINTERFACE_MSG_LEN * 4];

    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      do{
        serial_conn_.ReadByte(buf[0], timeout_ms_);
      } while (buf[0] != 0x0F);

      for( int i=1; i<(ARDUINTERFACE_MSG_LEN * 4); i++)
      {
        serial_conn_.ReadByte( buf[i], timeout_ms_);
      }

    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    uint8_t chksum1 = 0;
    uint8_t chksum2 = 0;

    // Fletcher-16 Checksum
    for (int i=0; i<(ARDUINTERFACE_MSG_LEN)*4-2; i++ )
    {
      chksum1 = (chksum1 + buf[i]) & 0xFF;
      chksum2 = (chksum2 + chksum1) & 0xFF;
    }

    // chksum is stored just for debug analysis:
    // bit32..16 checksum of message
    // bit15..0  checksum calculated
    msg.chksum = (*((uint32_t*)(&buf[15*4]))) & 0xFFFF0000;
    msg.chksum |= chksum1 << 8; // second last byte
    msg.chksum |= chksum2 << 0; // last byte
    if( (chksum1 != buf[(ARDUINTERFACE_MSG_LEN)*4-2]) || (chksum2 != buf[(ARDUINTERFACE_MSG_LEN)*4-1]))
    {
        return -1;
    }

    msg.hdr              = *((uint32_t*)(&buf[0*4]));
    msg.id               = *((uint32_t*)(&buf[1*4]));
    msg.millis           = *((uint32_t*)(&buf[2*4]));

    msg.batt_voltage     = *((float*)(&buf[4*4]));
    msg.charger_voltage  = *((float*)(&buf[5*4]));
    msg.charger_current  = *((float*)(&buf[6*4]));
    msg.wheel_l_position = *((float*)(&buf[7*4]));
    msg.wheel_r_position = *((float*)(&buf[8*4]));

    /*
    std::string delimiter = " ";
    size_t start = 0;
    size_t end = response.find_first_of(delimiter, start);
    std::string token_1 = response.substr(start, end-start);
    start = end + delimiter.length();
    end = response.find_first_of(delimiter, start);
    std::string token_2 = response.substr(start, end-start);
    start = end + delimiter.length();
    end = response.find_first_of(delimiter, start);
    std::string token_3 = response.substr(start, end-start);
    start = end + delimiter.length();
    end = response.find_first_of(delimiter, start);
    std::string token_4 = response.substr(start, end-start);
    start = end + delimiter.length();
    end = response.find_first_of(delimiter, start);
    std::string token_5 = response.substr(start, end-start);
    val_1 = std::atof(token_2.c_str());
    val_2 = std::atof(token_4.c_str());
    */
    return 0;
  }

  void set_motor_values(float val_1, float val_2)
  {
    std::stringstream ss;

    // --- SRS ---
    ss << "XM " << val_1 << " " << val_2 << " \r\n";
    //send_msg(ss.str());
    serial_conn_.Write(ss.str());

  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP