/*
 * Copyright (c) 2014, SRI International
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SRI International nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Author: Sean Seungkook Yun <seungkook.yun@sri.com>
 */

#include "serial.h"
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include <signal.h>
#include "necessity_serial/necessity_serial_msg.h"

bool quit = false;    // signal flag

void got_signal(int)
{
    quit = true;
}


int main(int argc, char* argv[])
{

  struct sigaction sa;
  memset( &sa, 0, sizeof(sa) );
  sa.sa_handler = got_signal;
  sigfillset(&sa.sa_mask);
  sigaction(SIGINT,&sa,NULL);

  ros::init(argc, argv, "necessity_serial", ros::init_options::NoSigintHandler);
  ros::NodeHandle node_handle, n_private("~");

  std::string port;
  n_private.param<std::string>("port", port, "/dev/ttyUSB0");
  bool print_data;
  n_private.param<bool>("print", print_data, false);

  bool rv;
  NecessitySerial serial;
  ROS_INFO("opening a serial-usb module at %s", port.c_str());
  rv = serial.start(port.c_str(), 115200);
  if (rv == false) {
    ROS_ERROR("failed to start the serial port");
    return -1;
  }
  ROS_INFO("Seial port opened at %s", port.c_str());

  std::string publish_name="necessity_serial";
  publish_name+=port;

  ros::Publisher serial_pub = node_handle.advertise<necessity_serial::necessity_serial_msg>(publish_name, 1);
  necessity_serial::necessity_serial_msg serial_msg;

  ros::Rate loop_rate(20);

  serial_msg.serial=port;

  while(node_handle.ok()) {


    try {
      serial.readAllData();
    }
    catch (boost::exception const&  ex) {
      std::cout << "ABNORMAL ENDING";
    }

    const necessity_serial::necessity_serial_msg&
      serial_data=serial.getData();

    if (print_data)
      serial.printData();

    serial_msg.signal=serial_data.signal;
    for (int i=0; i<3; i++)
      serial_msg.acc[i]=serial_data.acc[i];
    serial_msg.header.stamp = ros::Time::now();

    serial_pub.publish(serial_msg);
    loop_rate.sleep();

    ros::spinOnce();

    if( quit ) break;    // exit normally after SIGINT
  }

  //make it sleep
  ROS_INFO("Make it Sleep");
  //serial.makeSleep();

  ros::shutdown();

  return 0;
}

