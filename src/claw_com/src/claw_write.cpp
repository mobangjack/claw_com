/**
 * Copyright (c) 2017, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>

#include <string>
#include <sstream>

#include "crc8.h"
#include "uart.h"

#define BUF_LEN 4

int uart_fd = -1;

int cmd_claw(uint8_t c)
{
    uint8_t buf[BUF_LEN];
    buf[0] = 0xa5;
    buf[1] = c;
    buf[2] = 0xfe;
    CRC8Append(buf, BUF_LEN, 0xff);
    return write(uart_fd, buf, BUF_LEN);
}

void claw_cmd_callback(const std_msgs::UInt8& msg)
{
    cmd_claw(msg.data);
}

bool claw_cmd_srv_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  int write_len = cmd_claw(request.data);
  std::stringstream ss;
  if (write_len == BUF_LEN)
  {
    response.success = true;
    ss << "Claw cmd sent, " << write_len << " bytes writen\n";
    response.message = ss.str();
  }
  else
  {
    response.success = false;
    ss << "Claw cmd packet corrupted, " << write_len << " bytes writen (expect " << BUF_LEN << ")";
    response.message = ss.str();
  }
  
  return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "claw_read");

    std::string serial_port = "/dev/ttyTHS2";
    int serial_baudrate = 115200;
    int spin_rate = 30;

    ros::NodeHandle np("~");
    np.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    np.param<int>("serial_baudrate", serial_baudrate, 115200);
    np.param<int>("spin_rate", spin_rate, 30);

    int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_WR);
    if (ret < 0) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , serial_port.c_str());
        return -1;
    }

    ros::NodeHandle nh;

    ros::Subscriber claw_cmd_sub = nh.subscribe("claw/cmd", 5, claw_cmd_callback);
    ros::ServiceServer claw_cmd_srv = nh.advertiseService("claw/cmd", claw_cmd_srv_callback);
    
    std::cout << "\n----------------------------claw_write node started----------------------------" << std::endl;
    std::cout << "                                                                         \n       111                      111                                      \n       111                      111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111  1111111  111111111  11111111   1111111  111111111   11111111 \n       111 1111 1111   1111     1111 111  1111 111  11111 111  111 111   \n       111 111   111   1111     111  1111 11    111 1111  111  111  111  \n 111   111 111111111   1111     111   111    111111 111   111  111 1111  \n 111   111 111         1111     111   111 111111111 111   111  1111111   \n 111  1111 111         1111     111   111 111   111 111   111  111111    \n 111  111  1111  111    111     111  1111 111  1111 111   111  111       \n  1111111   11111111    111111  11111111  111111111 111   111  11111111  \n   11111     11111       11111  1111111    11111111 111   111  111  1111 \n                                                              111    111 \n                                                               11111111  " << std::endl;
 
    std::cout << "\n----------------------------Copyright 2017. Jetbang----------------------------" << std::endl;   
    ros::Rate rate(spin_rate);
    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
    
    uart_close(uart_fd);
    uart_fd = -1;

    return 0;
}


 
