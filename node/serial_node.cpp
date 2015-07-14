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
#include "pressure_serial/pressure_serial_msg.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

static double prev_time;     // previous time for vibration
static bool quit = false;    // signal flag
const static double MIN_TIME=1.0; // minimum time between vibrations
const static double VIB_TIME=0.5;
const static int LOOP_RATE=20; // main loop rate Hz. may affect vibration effects
static NecessitySerial serial;
static int vibEffect; // which vibration effect style to use. 0=off, see effects below
static int MAX_EFFECT=8; // highest array index in effect table below
int vibPattern[10][10] =
  {                        // on/off patterns at Loop Rate:
    {0,0,0,0,0,0,0,0,0,0}, // off
    {1,1,1,1,1,1,1,1,1,0}, // long on
    {1,0,0,0,0,0,0,0,0,0}, // bump
    {1,0,1,0,0,0,0,0,0,0}, // bump bump
    {1,0,1,0,1,0,0,0,0,0}, // bump bump bump
    {1,0,0,1,0,0,0,0,0,0}, // bump bump slower
    {1,0,0,0,1,0,0,0,1,0}, // bump bump even slower
    {1,1,0,0,1,1,0,0,0,0}, // dash dash
    {1,1,0,0,0,1,1,0,0,0}  // dash dash slower (effect 8)
  };
int vibStep; // index into vibEffect pattern, changes vibe on/off during each loop

void got_signal(int)
{
  serial.vibrate(false);
  quit = true;
}

void vibrationCallback(const std_msgs::Int8::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();
  vibEffect = msg->data;

  if (vibEffect == 0 || vibEffect > MAX_EFFECT)
  {
    serial.vibrate(false);
    return;
  }
  if (now.toSec()-prev_time < MIN_TIME)
    return;

  ROS_INFO("vibration effect# %d", msg->data);
  serial.vibrate(true);
  vibStep = 1;
  prev_time = now.toSec();
}

void vibration_step()
{
  ros::Time now = ros::Time::now();
  if (now.toSec()-prev_time> VIB_TIME && serial.getVibrating()
      || vibEffect > MAX_EFFECT || vibStep >=10)
  {
    serial.vibrate(false);  //turn off vibration
    return;
  }

  // update vibration according to effect pattern
  bool vibe = vibPattern[vibEffect][vibStep];
  serial.vibrate(vibe);
  vibStep++;
}

int main(int argc, char* argv[])
{
  struct sigaction sa;
  memset( &sa, 0, sizeof(sa) );
  sa.sa_handler = got_signal;
  sigfillset(&sa.sa_mask);
  sigaction(SIGINT,&sa,NULL);

  ros::init(argc, argv, "pressure_serial", ros::init_options::NoSigintHandler);
  ros::NodeHandle node_handle, n_private("~");

  std::string port;
  n_private.param<std::string>("port", port, "/dev/ttyUSB0");
  bool print_data;
  n_private.param<bool>("print", print_data, false);

  bool rv;

  ROS_INFO("opening a serial-usb module at %s", port.c_str());
  rv = serial.start(port.c_str(), 115200);
  if (rv == false) {
    ROS_ERROR("failed to start the serial port");
    return -1;
  }
  ROS_INFO("Serial port opened at %s", port.c_str());

  std::string publish_name="pressure_serial";
  publish_name+=port;

  ros::Publisher serial_pub = node_handle.advertise<pressure_serial::pressure_serial_msg>(publish_name, 1);
  pressure_serial::pressure_serial_msg serial_msg;

  std::string subscribe_name="vibration";
  subscribe_name+=port;
  ros::Subscriber vibration_sub = node_handle.subscribe(
                                               subscribe_name.c_str(),
                                               1000,
                                               vibrationCallback);

  ros::Rate loop_rate(LOOP_RATE);

  serial_msg.serial=port;

  serial.init();

  while(node_handle.ok()) {
    loop_rate.sleep();

    try {
      serial.readAllData();
    }
    catch (boost::exception const&  ex) {
      std::cout << "ABNORMAL ENDING";
    }

    const pressure_serial::pressure_serial_msg&
      serial_data=serial.getData();

    if (print_data)
      serial.printData();

    serial_msg.signal=serial_data.signal;
    for (int i=0; i<3; i++)
      serial_msg.acc[i]=serial_data.acc[i];
    serial_msg.acc_abs=serial_data.acc_abs;
    serial_msg.fall_count=serial_data.fall_count;
    serial_msg.header.stamp = ros::Time::now();

    serial_pub.publish(serial_msg);

    vibration_step();

    ros::spinOnce();

    if( quit ) break;    // exit normally after SIGINT
  }

  //make it sleep
  ROS_INFO("Make it Sleep");
  //serial.makeSleep();

  ros::shutdown();

  return 0;
}

