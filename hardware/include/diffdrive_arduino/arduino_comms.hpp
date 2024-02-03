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

class ArduinoComms
{

public:

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

  void read_encoder_values(double &val_1, double &val_2)
  {
    // --- SRS ---
    //val_1 = val_1 + 1;
    //val_2 = val_2 + 1;
    //return;

    std::string response = send_msg("XO\r\n");

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

    /*
    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);

    std::string token_2 = response.substr(del_pos + delimiter.length());
    std::string token_3 = response.substr(del_pos + delimiter.length());
    std::string token_4 = response.substr(del_pos + delimiter.length());
    std::string token_5 = response.substr(del_pos + delimiter.length());
    */

    val_1 = std::atof(token_2.c_str());
    val_2 = std::atof(token_4.c_str());
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